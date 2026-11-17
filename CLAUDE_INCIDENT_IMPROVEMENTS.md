# 돌발상황 개선 인수인계 (claude/incident-improvements)

작성: 2026-11-17 · 기준 커밋: `a5b456d` · 작성자: Claude

**상태: 컴파일·테스트 미실행.** 이 작업은 빌드 환경 없이 소스만 수정했다. 병합 전에 `Scripts/BuildEditor.ps1`과 `Scripts/RunFactoryModularTest.ps1`을 반드시 실행한다. 아래 자동 검사 14개는 통과 여부가 확인되지 않았다.

UI 파일(`FactoryRuntimeUI.cpp`, `FactoryRuntimeEditor.cpp`)은 수정하지 않았다. 새 기능은 모두 기본값이 꺼져 있어 기존 동작과 기존 검사 결과를 바꾸지 않도록 했다.

## 기능 단위 변경

### 1. 그립 마찰계수 수치화 + 운반 중 마찰 저하

- `WheelGripPhysics.h`: `StepWheelGrip`의 `bool LowFriction` 인자를 `double Friction`으로 변경. `WheelGrip::NormalFriction`(0.65), `LowFriction`(0.08), `TotalPadForceN`(1200), `DegradedFriction()` 추가.
- `FactoryProcessRunner.cpp`: 호출부 1곳을 새 인자에 맞춤. 동작 동일.
- `WheelAssemblyCell.h/.cpp`: `bGripDegradesInTransit`, `GripDegradeSeconds`(기본 1.5s) 추가. 왼쪽 앞 휠이 `Transfer` 단계에 들어간 시점부터 마찰계수가 0.65 → 0.08로 선형 감소한다. 들어 올릴 때는 버티고 운반 도중에 미끄러진 뒤, 기존 낙하·작업자 회수·대체 휠 복구 경로를 그대로 탄다.
- `bPhysicalLeftGrip`이 켜져 있어야 동작한다. 복구 후 대체 휠은 같은 사이클에서 정상 마찰로 잡는다.
- 기록 필드: `LeftGripFriction`, `bSlipFailureGripDegraded`, `SlipFailureStage`, `SlipFailureTravelCm`, `GetSlipFailureLabel()`.
- `SensorAssemblyLine.cpp`: 낙하 표식 문구 1곳을 `GetSlipFailureLabel()`로 교체(3종 구분).
- 한계: 패드 오염을 가정한 시연용 선형 감소다. 측정된 그리퍼 특성이 아니다.

### 2. 레이저 스캔 제한 횟수 재시도

- `ScanFrontLeftHub()`가 최대 `MaxLaserScanAttempts`(기본 3)회 재측정한다. 기존 단일 측정은 `ScanFrontLeftHubOnce()`로 옮겼다.
- `LaserTransientBlockScans`: 사이클의 처음 N회 측정을 가리는 일시 가림 주입. N이 시도 한도보다 작으면 재스캔으로 통과하고, 크거나 같으면 기존처럼 중단한다.
- `FWheelHubScanReading::Attempts`, `LaserScanRetries` 추가. 최종 실패 사유에 시도 횟수가 붙는다.
- 영구 고장(`bBlockFrontLeftLaser`, `LaserOutlierMm`)은 3회 모두 실패하므로 기존 검사 결과는 같다.
- 한계: 재시도는 한 고정 스텝 안에서 연속 실행된다. 스캐너 주기·지연은 모델링하지 않았다.

### 3. 시드 난수 사건 풀 확장

- `AutonomousAssemblyLine.h/.cpp`: `bExtendedRandomEvents`(기본 꺼짐). 검사 5(기존 난수 사건)에서만 동작한다.
- 켜면 실행마다 시드로 라인 수준 사건 하나를 추가로 뽑는다: 없음 / 과중량 휠 / 운반 중 마찰 저하 / 도킹 초과. 결과는 `ExtendedEvent`(0~3)와 로그 `RANDOM_LINE_EVENT`에 남는다.
- 기존 휠별 사건을 뽑은 뒤에 추가로 뽑으므로 기존 시드의 휠 사건 순서는 바뀌지 않는다.
- 과중량·마찰 저하가 뽑히면 왼쪽 앞 휠의 기존 난수 사건은 건너뛴다(휠 하나에 사건 하나). 뒤 휠 사건은 유지된다.
- 복구가 구현된 사건만 풀에 넣었다. 규격 불량은 복구 없이 라인이 멈추므로 제외했다.
- UI에 켜는 버튼이 없다. 에디터 Details 패널에서 켜거나 UI 토글을 추가해야 한다.

### 4. 규격 불합격 휠 교체 후 재개 (P0-2)

- `WheelAssemblyCell.h/.cpp`: `bReplaceRejectedWheel`(기본 꺼짐). 센서 구동 왼쪽 레인에서만 동작한다.
- 켜면 왼쪽 앞 휠이 규격 불합격일 때 라인을 멈추지 않는다. 불합격 휠을 임시 보관 지점으로 옮기고(1.8s), 대체 휠이 예비 위치에서 공급 위치로 이동한다(1.8s). 그동안 왼쪽 레인은 `WaitSupply`에서 대기하고 다른 로봇은 계속 작업한다.
- 재개 조건은 기존 `WaitSupply` 경로 그대로다: 공급 센서가 8cm 이내 도착을 확인하고 규격 재스캔이 통과해야 한다. 대체 휠도 불합격이면 중단한다. 교체는 사이클당 1회다.
- 이동은 `AdvanceSimulation`의 고정 스텝에서 `AdvanceWheelReplacement()`가 수행한다. 이 동안 Tick 기반 `AdvanceFrontLeftWheelReject()`는 건너뛴다.
- 기록 필드: `WheelReplacePhase`(1 격리, 2 대체품 이송), `WheelReplacements`, `RejectedWheelDiameterCm`, `WheelSpareStartWorld`.
- 꺼져 있으면 기존 동작(격리 후 중단)과 기존 검사 2개가 그대로다.
- 한계: 대체 휠은 같은 휠 오브젝트를 재사용한다. 보관 지점의 불합격 휠은 낙하 휠 표시용 메시(`DroppedTireVisual`)를 빌려 쓴 대역이다. 별도 부품 ID·재고 수량은 없다. 교체 후 `FrontLeftWheelSizeScale`이 1.0으로 바뀌므로, 같은 불량을 다시 보려면 검사를 다시 선택해야 한다(과중량 복구와 같은 방식). 차문 규격 불량은 여전히 중단만 한다.

### 5. 사건 기록 (UI 증거 패널용 데이터)

- `WheelAssemblyCell.h/.cpp`: `FWheelIncidentRecord`와 `AWheelAssemblyCell::Incidents` 추가. 셀이 판단을 내리는 시점에 한 줄씩 기록한다. 동작은 바꾸지 않는다.
- 필드: `Kind`, `PartId`, `Stage`, `DetectedAt`, `Quantity`, `Unit`, `Measured`, `Limit`, `Response`, `Outcome`(`pending` / `recovered` / `stopped`), `ResolvedAt`. `Kind`와 `Outcome`은 영문 고정 ID, 나머지 문구는 한국어다.
- 기록되는 사건: `supply_missing`, `pickup_pose_stale`, `pickup_displaced`, `grip_unconfirmed`, `grip_slip`, `wheel_size_reject`, `laser_rescan`, `laser_scan_failed`, `safety_stop`.
- `Fail()`은 진행 중인 기록을 모두 `stopped`로 닫고 중단 사유를 `safety_stop` 한 줄로 남긴다.
- 읽기용 함수: `GetIncidentSummary(MaxRows)`(한국어 요약), `GetIncidentCsv()`, `CountIncidents(Kind, Outcome)`. UI는 판정을 다시 계산하지 말고 이 기록을 표시하면 된다.
- `ResetCycle()`에서 비운다. 사이클당 최대 64줄이다. 파일 저장은 하지 않으므로 필요하면 라인의 `Record()`에서 `GetIncidentCsv()`를 쓰면 된다.
- 한계: `Measured`는 판단에 쓴 값(센서·레이저 측정, 횟수, 파지 모델의 미끄러짐)이다. 시뮬레이션 실제값은 따로 담지 않았다. 차문 셀과 차체 도킹 사건은 아직 기록하지 않는다.

## 추가한 자동 검사 (`FactoryModularChecks.cpp`)

| 검사 이름 | 확인 내용 |
|---|---|
| `laser_transient_block_rescans_then_completes` | 2회 가림 후 3번째 측정 통과, 4륜 완료 |
| `laser_rescan_limit_blocks_pickup` | 3회 가림이면 파지 전 중단 |
| `incident_log_records_laser_rescan_as_recovered` | 재스캔이 복구 완료로 1줄 기록 |
| `incident_log_case_1..4_*`, `incident_log_case_9_wheel_size_reject` (5개) | 누락·센서 끊김·파지 실패·위치 변화는 복구 완료로, 교체 없는 규격 불량은 안전 중단으로 닫힘 |
| `incident_log_closes_replaced_size_reject_as_recovered` | 교체한 규격 불량이 복구 완료로 닫힘 |
| `incident_log_closes_transit_slip_as_recovered` | 운반 중 미끄러짐이 복구 완료로 닫힘 |
| `oversized_wheel_replaced_rescanned_and_cycle_completes` | 과대 휠 격리, 대체품 이송 중 센서 미검출·대기, 재스캔 통과, 4륜 완료 |
| `undersized_wheel_replaced_rescanned_and_cycle_completes` | 과소 휠에 대해 같은 내용 |
| `grip_degrades_in_transit_slips_after_lift_and_recovers` | Transfer에서 휠을 든 상태가 관측되고, 그 뒤 미끄러짐·복구·4륜 완료 |
| `extended_random_line_events_match_plan_and_recover` | 4회 연속 실행에서 뽑힌 사건이 실제로 일어나고 모두 완료 |

## 검증 시 볼 곳

- 마찰 저하 검사가 실패하면 `SlipFailureStage`를 먼저 본다. `Transfer` 구간이 1.3초보다 짧으면 미끄러짐이 `ApproachMount`에서 날 수 있고 이는 검사 통과 조건 안이다. `Lift`에서 나면 설계와 다르다.
- 확장 난수 검사는 검사 5의 전체 사이클 완료를 전제로 한다. 기존에 검사 5 완료를 확인하는 자동 검사가 없어서, 이 검사가 실패하면 확장 사건이 아닌 기존 난수 경로 문제일 수도 있다.
- 과중량·마찰 저하와 뒤 휠 난수 사건이 한 실행에 함께 나오는 조합은 이번에 처음 생긴 경로다.
