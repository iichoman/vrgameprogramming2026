# choi 병합 이후 lyw 작업 인수인계

기준일: 2026-11-11  
기준 브랜치: `codex/lyw-factory-integration`  
기준 커밋: `cb165d1`  
변경 범위: `1311d46`(choi 내용 병합 및 휠 센서 연결) **이후**의 네 커밋 `0df9121` → `63edda8` → `ceef0ff` → `cb165d1`.

이 문서는 팀원 GPT가 변경 내용을 읽고 자신의 작업에 연결하기 위한 요약이다. `FACTORY_TASK_CONTRACT.md`와 `FACTORY_ARCHITECTURE.md`가 공통 작업·로봇·부품 ID 및 편집 규약이다. 아래 구현은 그 규약 위에 추가한 기능이며 모든 센서/물리 계획이 완료되었다는 뜻은 아니다.

## 1. 한눈에 보는 변경

| 영역 | 현재 구현 | 팀원 작업 시 확인할 점 |
|---|---|---|
| 작업 할당·편집 | 로봇 클릭 → 차체 초록색 목표 → 실제 부품 클릭. 휠/문 공급 부품을 수평 이동하고 작업·오프셋을 JSON v5에 저장 | `FFactoryTask`의 로봇·부품·목표 포트 ID를 유지. Actor 포인터나 메시 이름으로 저장하지 않기 |
| 조립 맵 | `factory_p1`에서 휠 4개 다음 문 4개. 양측 도어 셀/모듈과 레일 시작·끝 배치 보정 | 맵은 바이너리 `.umap`이므로 양쪽 변경 시 단순 텍스트 병합 불가. 현재 맵과 설치 스크립트의 결과를 비교하기 |
| 왼쪽 앞 휠 물리 파지 | 기본 18kg·과중량 시연 90kg 점질량, 중력, 패드 힘 1200N, 마찰 한계로 운반 위치·미끄러짐 계산. 정상/저마찰/과중량 시연 | Chaos 강체 접촉, 차체 충돌, 체결 토크가 아님. UI 수치를 실제 측정치로 표기하지 않기 |
| 왼쪽 앞 휠 레이저 | 차량 도착 허브 → 장착 직전 허브 → 가상 체결·그리퍼 해제 후 휠 중심을 각각 스캔·판정 | 현재는 반지름 8cm 구형 충돌 프록시. 실제 허브/휠 메시나 축 방향 측정으로 오해하지 않기 |
| 시연 UI | 레이저 활성화 체크박스, 3단계 판정·시각·명중 수·잔차·오차, 광선/중심 표식, 물리 힘·미끄러짐 수치 | 상세 에디터는 아래 개별 필드에서 읽기. 현재 표시의 값과 단위를 유지 |

## 2. 코드와 데이터 연결 지점

- **작업/부품**: `unreal projects/test1/Source/test1/FactoryTaskModel.h/.cpp`, `FactoryTaskAssignment.cpp`, `FactoryPartEditing.cpp`, `FactoryLayoutModel.h/.cpp`. 현재 계약은 `FACTORY_TASK_CONTRACT.md` 참조. `ObservationProvider`는 `UWheelRobotSensorBank`의 부품 관측용이며 레이저 허브 측정과 별도다.
- **통합 공정**: `IntegratedAssemblyLine.cpp/.h`의 `bFourDoorMode`, `DoorModules`, `MountedDoorCount`. 문 4개는 `Door_FL`, `Door_RL`, `Door_FR`, `Door_RR`를 사용한다. `DoorAssemblyCell`과 `DoorPoseComponent`에도 양측 문 작업 조정이 있다.
- **휠 레이저**: `WheelAssemblyCell.h/.cpp`의 `bLaserScanFrontLeftHub`, `ScanFrontLeftHub()`, `FrontLeftDockScan`, `FrontLeftPreMountScan`, `FrontLeftFinalWheelScan`. 세 측정값은 `FWheelHubScanReading`이며 `ResetCycle()`에서 초기화한다. `GetMountTargetWorld()`가 유효한 두 번째 허브 측정을 장착 경로에 사용한다.
- **휠 규격 불량 시연(추가 작업)**: `SetFactoryCheck(9/10)`은 왼쪽 앞 휠을 각각 기준 크기의 112%/88%로 표시한다. `ScanFrontLeftWheelSize()`는 공급 휠의 구형 검사 프록시를 양쪽에서 레이저로 조사해 표면 명중점 사이 지름을 측정한다. 기준은 시연용 66±1cm이며 범위 밖이면 파지 전에 중단한다. 화면에는 두 명중점·측정선·측정 지름을 표시한다. 실제 타이어 메시 표면 스캔이나 제조사 규격 검사는 아니다.
- **차문 규격 불량 시연(추가 작업)**: `SetFactoryCheck(11/12)`은 왼쪽 앞문 시각 메시를 112%/88%로 바꾼다. `DoorAssemblyCell::ScanDoorSize()`는 공급 문을 둘러싼 축정렬 충돌 프록시를 양쪽에서 조사해 폭을 계산한다. 같은 문 메시의 원래 경계 폭을 기준으로 ±1cm를 벗어나면 파지 전에 중단한다. 이는 실제 도어 패널 표면, 차체 개구부와의 간극, OEM 공차 측정이 아니다.
- **차체 도킹 초과 시연(추가 작업)**: `SetFactoryCheck(13)`은 `ASensorAssemblyLine`의 실제 반입 운반대 목표를 정지점보다 35cm 연장한다. 차체가 지나간 뒤 운반대를 후진시켜 위치·속도 센서의 도킹 조건을 다시 통과해야 휠 작업을 시작한다. `MaxDockOvershootCm`와 `bDockOvershootRecovered`가 검증 근거다.

### 2026-11-12 검증 상태

`BuildEditor.ps1`과 `RunFactoryModularTest.ps1`에서 과중량 낙하, 휠/차문 과대·과소 파지 전 거부, 도킹 초과·후진 후 전체 4륜·4문 완료가 통과했다. `RunFactoryAssemblyTest.ps1 -Mode Capture`는 기본 맵의 시작·완료 화면을 저장하고 통합 UI와 4륜·4문 완료 화면을 확인했다. 이 캡처는 돌발상황 선택 화면을 촬영하지 않으므로 각 돌발상황의 **실제 화면 가시성은 아직 수동 확인 대상**이다. 규격 검사는 충돌 프록시 기반 시연이며 실제 휠/문 메시 표면 또는 OEM 공차의 검증 결과가 아니다.
- **레이저 판정**: 5광선 중 3개 이상 명중, 중심 후보 RMS 잔차 1mm 이하, 도착→장착 직전 허브 이동 5mm 이하, 최종 휠 중심 오차 `QualityToleranceMm` 이하(기본 2mm). 스캔 중심은 충돌 명중점과 표면 법선으로 복원한다. 자세 회전은 차체 기준이며 레이저 축 추정값이 아니다.
- **물리 파지**: `WheelGripPhysics.h`의 `StepWheelGrip()`을 통합 휠 셀과 단일 휠 기능 실행기가 사용한다. `AutonomousAssemblyLine.cpp`의 `SetFactoryCheck(6/7/8)`은 각각 정상/저마찰/과중량이다. 과중량은 정상 마찰계수를 유지하면서 왼쪽 앞 휠 질량을 18kg에서 90kg으로 바꾼다. 질량, 필요 힘, 실제 전달 힘, 마찰 한계, 미끄러짐이 표시된다. 미끄러짐 3mm 이상이면 파지를 해제하고 중단한다.
- **UI/표식**: `FactoryRuntimeUI.cpp`가 레이저 체크박스와 3단계 요약을 표시한다. `SensorAssemblyLine.cpp::DrawSensorVisualization()`이 검사 중심과 최근 광선을 그린다. `WheelAssemblyCell.cpp::GetLaserInspectionSummary()`가 수치 문구를 만든다.

레이저 필드의 의미·단위·오차 주입법은 `WHEEL_LASER_EDITOR_HANDOFF.md`에 상세히 적었다. 팀원 에디터에서는 `ScanOriginWorld`, `RayEndsWorld`, `HitPointsWorld`, `HitCount`, `ResidualMm`, `MeasuredAt`, `MeasuredWorld`, `FailureReason`을 직접 사용하면 된다. `RayEndsWorld`의 명중 광선은 실제 명중점이며 미명중 광선은 최대 사거리 끝점이다.

## 3. 팀원 브랜치에 적용할 때

1. 기준 커밋과 차이를 먼저 본다: `git log --oneline 1311d46..cb165d1` 및 `git diff --stat 1311d46..cb165d1`. 이 문서의 범위는 그 이후 네 커밋이며 choi 쪽 후속 변경은 별도로 비교해야 한다.
2. `FACTORY_TASK_CONTRACT.md`의 작업/부품/포트 ID와 JSON v5 규약을 우선한다. 팀원 브랜치의 동일 파일이 더 진행됐다면 관련 함수별로 통합한다. 특히 `FactoryTaskModel`, `FactoryLayoutModel`, `FactoryPartEditing`, `IntegratedAssemblyLine`, `WheelAssemblyCell`은 서로 연결된다.
3. 맵을 가져올 때 `Content/DigitalTwin/Maps/factory_p1.umap`의 레일 보정과 양측 도어 로봇/모듈 배치를 확인한다. `Scripts/repair_factory_rail.py`, `Scripts/install_four_door_station.py`는 그 작업의 재현 자료다. 이미 수정한 맵에 설치 스크립트를 무조건 다시 실행하지 않는다.
4. 레이저는 기본 꺼짐이다. 통합 테스트 화면의 **이식된 센서 기능 확인 → 왼쪽 앞 휠 레이저 3단계 검사 켜기**를 선택한다. **센서·휠 중심·장착점 표시**를 켜면 표식과 최근 광선이 보인다. 화면 수치와 추후 상세 에디터는 같은 `FWheelHubScanReading`을 읽어야 한다.
5. 테스트를 분리해서 실행한다. 레이저 기본 성공/2cm 허브 이동/10mm 거리 오차/광선 차단, 물리 파지 정상 운반/저마찰 중단, 4륜·4문 전체 공정을 확인한다. `Scripts/BuildEditor.ps1`, `Scripts/RunFactoryModularTest.ps1`을 사용한다. 마지막 확인에서는 Editor 빌드와 모듈 검사 전체가 통과했다. 자동 검사 구현은 `FactoryModularChecks.cpp`에 있다.

## 4. 현재 한계와 후속 작업

- 레이저 시연은 **왼쪽 앞 휠만** 적용한다. 실제 메시 표면, 축 방향, 센서 지연·노이즈, 재스캔 복구, 다른 휠/문은 아직 없다. 3단계 측정값은 실행 중 메모리에 있으며 장기 기록 저장은 별도 작업이다.
- 물리 파지는 운반 중 병진 점질량 모델이다. 저마찰 한계 초과 시 그리퍼 명령을 해제하고 휠을 아래·바깥쪽으로 2초간 이동시켜 실패 상태를 화면에 남긴다. 이는 시연용 낙하 모델이며 Chaos 바닥 충돌은 아니다. 휠 회전 관성, 실제 충돌·접촉, 그리퍼 물리 구속, 차체 장착력, 볼트 토크는 아직 없다. 정상/저마찰 자동 검사는 통과했지만 실제 제조 물성 검증은 아니다.
- 앞유리/창문 압력 센서와 장착 공정은 `SENSOR_PHYSICS_EXTENSION_PLAN.md`의 **계획**이다. 구현 완료 목록에 넣지 않는다.
- 팀원 에디터가 추가할 상세 수치/시각화는 `WHEEL_LASER_EDITOR_HANDOFF.md`의 개별 필드를 사용하면 된다. 화면에 통과 표시만 복제하지 말고 각 판정의 명중 수·잔차·이동량·오차와 기준값을 함께 보여준다.

관련 문서: `FACTORY_TASK_CONTRACT.md`, `FACTORY_ARCHITECTURE.md`, `WHEEL_LASER_EDITOR_HANDOFF.md`, `SENSOR_PHYSICS_EXTENSION_PLAN.md`.
