# Camry 차체·앞문 확인

## C++ 조립 Actor (2026-09-30)

현재 작업용 맵은 `Content/DigitalTwin/Maps/CamryAssemblyPreview`입니다.
네이티브 `ACamryAssembly` 인스턴스 하나로 차량을 관리합니다. 구현은 `Source/test1/CamryAssembly.h`, `Source/test1/CamryAssembly.cpp`에 있으며 블루프린트를 사용하지 않습니다.

```text
VehicleRoot
├─ Body       : 차체 5개 메시
├─ LeftDoor   : 좌측 앞문 3개 메시
└─ RightDoor  : 우측 앞문 3개 메시
```

전체 이동: 맵의 `CamryAssembly` 액터를 선택해서 옮깁니다.
문 위치 조정: 맵의 Actor를 선택한 뒤 Details의 컴포넌트 목록에서 `LeftDoor` 또는 `RightDoor`를 선택해서 상대 Location을 조정합니다. 하위 판재 3개가 함께 움직입니다. C++에서는 `GetLeftDoor()`와 `GetRightDoor()`로 해당 Scene Component에 접근합니다. 위치 단위는 cm이며 기준점은 차량 루트에 상대적입니다.

문 그룹의 기준점은 판재 경계상자의 중심이며 실제 힌지 회전축이 아닙니다. 현재 단계는 조립 위치 조정용 묶음이고, 문 개폐·물리 힌지는 아직 구현하지 않았습니다.

### 문 기준 자세·이동·복원

`CamryAssembly` Actor 자체를 선택하고 Details에서 `Door Control`을 검색합니다. Play를 누르지 않아도 사용할 수 있습니다.

1. `Selected Door`에서 Left 또는 Right를 선택합니다.
2. `Preview Offset Mm`에 차체 축 기준 X/Y/Z 이동량을 mm 단위로 입력합니다. 왼쪽 문은 `(0, -100, 0)`, 오른쪽 문은 `(0, 100, 0)`으로 바깥쪽 10cm 이동을 확인할 수 있습니다.
3. `2. Apply Offset (mm)`를 누릅니다. 같은 값을 반복 적용해도 이동량은 누적되지 않습니다.
4. `3. Restore Reference Pose`를 누르면 저장된 위치와 회전으로 복원됩니다.

`1. Save Current Pose as Reference`는 현재 선택한 문의 자세로 기준을 **덮어쓰는** 버튼입니다. 정상 장착 자세를 새로 정할 때만 사용하고, 맵을 저장해야 다음 실행에도 유지됩니다. 시작할 때 자동으로 현재 자세를 기준으로 덮어쓰지 않습니다. 새로 배치한 Actor에는 기준이 없으므로 먼저 명시적으로 저장해야 합니다. 에디터 버튼 조작은 Undo 트랜잭션에 포함됩니다. 실행 중 기준 변경은 레벨 파일에 자동 저장되지 않습니다.

기준은 월드 좌표가 아닌 `Body` 컴포넌트 좌표계에 위치·회전으로 저장합니다. 차체를 이동·회전시켜도 기준이 따라가며, 문을 다른 운반 객체에 붙여도 복원 목표는 차체 기준입니다. 이 단계는 즉시 자세 변경이며 이동 경로·충돌 회피·체결·힌지 물리는 수행하지 않습니다. mm와 cm의 관계를 보장하기 위해 차체·문·운반 부모는 스케일 1인 강체 변환을 사용합니다.

### 왼쪽 앞문 검사점

`LeftDoorInspection` C++ 컴포넌트에서 앞쪽 3곳·뒤쪽 3곳의 검사 정의와 계산 결과를 관리합니다. 문 쪽 위치는 원본 외판 OBJ의 가장자리 정점이며, 차체 쪽 위치는 차체 좌표계에 고정한 가상 계측 기준점입니다. 기준점은 정상 외판 위치에서 틈 방향으로 3mm 떨어뜨려 정의했습니다. **3mm는 표시용 교육 가정값이며 실차 정상 간격이 아닙니다.**

표시하는 값은 정상 기준 대비 부호 있는 변화량입니다. 곡면 전체의 최소 간격·실제 도어 공차·센서 측정값으로 해석하지 않습니다. `dG`는 틈 변화, `dF`는 외판 단차 변화이며 단위는 mm입니다. 앞쪽 틈 방향은 차체 +X, 뒤쪽은 -X, 왼쪽 외판 바깥 방향은 -Y로 정의합니다. +X 1mm 이동 시 앞쪽 dG=-1, 뒤쪽 dG=+1이며, -Y 1mm 이동 시 모든 dF=+1입니다. 이 첫 정의에서는 수직 이동에 대한 틈 변화는 0입니다. 이후 위·아래 가장자리 검사점이나 경사 방향을 추가해 수직 정렬을 평가할 수 있습니다.

뷰포트에는 녹색 차체 기준점, 노란색 문 검사점, 흰색 대응선, 노란색 틈 방향선, 하늘색 단차 방향선을 표시합니다. P1~P6 글자는 검사점 순서이고 `Points` 배열의 Name으로 Front/Rear 및 높이를 확인할 수 있습니다. 선·점은 가림 없이 보이는 계측 오버레이이고 표시 길이 4cm는 실제 오차 크기가 아닙니다. 실제 수치는 dG/dF에 표시합니다.

`CamryAssemblyPreview`에서 Actor를 선택해 기존 `Apply Offset (mm)`로 `(1,0,0)` 또는 `(0,-1,0)`을 적용하고 `Restore Reference Pose`로 복원합니다. `LeftDoorInspection`을 선택하면 Details의 `Measurements` 배열에서도 값 6개를 확인할 수 있습니다. `Show Inspection`으로 표시를 끕니다. Realtime을 꺼도 C++ 자세 명령 이벤트가 결과를 갱신합니다. 컴포넌트를 직접 옮겼다면 Actor의 `Refresh Door Inspection` 버튼으로 갱신하며, Realtime이 켜져 있으면 자동 추적합니다.

문 기준 자세 저장과 검사 기준은 별개입니다. 문 기준을 덮어써도 `Points.ReferenceDoorPointBodyCm`는 자동 변경되지 않습니다. 검사 기준 재정의는 명시적으로 설정 데이터를 수정해야 합니다. 로봇·지그는 자세를 명령하고, 검사기는 실제 자세에서 결과를 계산하며, 향후 센서 모델은 이 결과에서 편차·잡음을 별도로 생성합니다.

`tools/select_inspection_points.py`가 원본 정점 6개를 선택하고 `Scripts/inspection_points.json`에 출처와 축 정의를 저장합니다. `Scripts/setup_door_inspection.py`는 작업 맵에 정의를 등록하고 계산을 검증합니다. 결과는 `Saved/DoorInspectionReport.json`에 기록하며, `-InspectionVerifyOnly` 실행은 저장된 정의를 새 프로세스에서 검증하고 맵을 변경하지 않습니다.

2026-10-06 등록·빌드·계산 검증·새 프로세스 재로드가 통과했습니다. 실제 뷰포트 렌더링과 텍스트 가독성은 육안으로 확인하지 않았습니다. 문은 정상 기준 자세로 복원한 상태로 저장했습니다.

### 내판과 지그 접촉 후보 분석

2026-10-06: 현재 왼쪽 내판 `2000358`(앞쪽, 원본 셸 두께 1.5mm)과 `2000359`(뒤쪽, 0.7mm)의 형상을 분석했다. 앞쪽 위·아래와 뒤쪽 상단의 자세 결정 접촉 후보 3개, 아래쪽 받침 구역 2개, 힌지 위치를 저장했다. 뒤쪽 중앙에는 큰 개구부가 있어 초기 패드 위치로 사용하지 않는다. 하부 받침은 접힌 테두리에 맞는 형상이 필요하다.

분석 결과는 저장소의 `car models/Camry_Converted/InnerDoorAnalysis/README.md`, 좌표 데이터는 같은 폴더의 `contact_candidates.json`에 있다. 후보의 도어 로컬 좌표는 현재 C++ 기본 피벗과 임포트 배치를 기준으로 하며 단위는 cm다. 실제 작업 맵에 등록한 지그 접촉점이나 승인된 실차 지그 위치가 아니다.

원본에 보강재·브래킷·창틀·힌지 부품이 더 있으며 현재 LeftDoor의 3개 메시에는 포함되지 않는다. 원본 셸 두께는 OBJ/Unreal에 입체 두께로 적용되지 않았다. 이 분석은 내장 트림이 없는 골격 기준이며 클램프 힘·변형·전체 접근 경로는 별도 설계가 필요하다.

같은 폴더의 `part_properties.json`에 원본에서 추출한 부품별 두께·재질 물성과 셸 질량 추정값을 저장했다. 앞쪽/뒤쪽 내판과 외판의 탄성계수는 모두 210GPa, 포아송비는 0.3, 밀도는 7,890kg/m³이며 초기 항복강도는 앞쪽 내판 360MPa, 뒤쪽 내판/외판 250MPa다. 강종명은 확정되지 않았다. 선정 셸의 질량 추정은 완성 도어 질량이 아니다. 이 데이터는 아직 Unreal 계산에 연결하지 않았으며, 개발 계획 v1.3에서 접촉부 1개 영역의 국소 탄성 근사와 시각화·검사 연동을 후속 구현으로 정했다.

### 지그 접촉 후보 Actor

`ADoorFixture`는 왼쪽 도어의 저장된 정상 자세를 기준으로 접촉 후보를 배치하는 네이티브 C++ Actor다. 위치 결정 A1~A3, 하부 받침 B1~B2, 뒤쪽 측면 스토퍼 C1을 배치 검토용으로 사용한다. 패드·봉·화살표·라벨은 단순 형상이며 가공 치수 또는 완성 지그가 아니다. 반대쪽 클램프 접촉은 외판 간섭 때문에 아직 선정하지 않았다.

맵에 배치된 `DoorFixture_Left`를 선택하면 Details에서 다음을 사용할 수 있다.

- `Fixture > Layout`: 연결할 `Assembly`, 접촉 배열 `Contacts`, 좌표 데이터 출처.
- `Fixture > Actions > 1. Align Fixture to Saved Door Reference`: 저장된 왼쪽 도어 기준 자세로 지그를 배치하고 차체 `Body`에 연결한다. 현재 문이 이동 중이어도 기준 자세를 사용하며 문을 움직이지 않는다.
- `Fixture > Actions > 2. Refresh Fixture Preview`: 좌표·표시·결과를 다시 갱신한다.
- `Fixture > Display`: `Show Fixture`, `Show Labels`, `Preview Retraction Mm`. Retraction은 각 접촉 법선 방향으로 지그 표시를 물려 배치를 확인하는 설정이며 실제 개폐 스트로크가 아니다.
- `Contacts`의 `Offset Mm`은 개별 패드 배치 조절용이다. `Preview Size Mm`은 가로·세로·깊이의 전체 크기이고, `Preview Stem Length Mm`는 표시용 봉 길이다. 변경이 도어에 힘을 가하거나 안착 자세를 계산하지 않는다.

화살표는 도어에서 지그 쪽으로 향한다. 라벨의 `CANDIDATE`는 형상 후보, `REGION`은 접촉 형상·방향을 더 정해야 하는 구역이다. `N`은 문 대응점과 지그 기준점 사이 법선 방향의 부호 있는 변위, `T`는 접선 방향 변위 크기(mm)다. 셸 중간면을 기준으로 한 좌표 비교이며 실제 접촉 간극·관통 깊이·센서 신호가 아니다. 패널 두께를 반영한 물리 접촉점은 후속 계산에서 정의한다.

지그는 차체에 붙고 현재 문에는 붙지 않는다. 문에 `Apply Offset`을 적용하면 지그는 남아 있고 N/T가 바뀐다. 차량 전체 이동·회전은 문과 지그에 함께 적용된다. 기준 자세를 새로 저장했을 때 지그가 자동으로 재정렬되지는 않으므로 의도적으로 Align을 호출한다. 배치 비교용 지그 Actor이며 클램프 힘·안착·변형 계산은 후속 구현이다.

`Scripts/setup_door_fixture.py`가 분석 데이터를 읽어 작업 맵에 배치하고 변환·문 독립성·잘못된 입력 처리·저장 재로드를 검증한다. `-FixtureVerifyOnly`는 맵을 저장하지 않는다. 실제 실행·검증 상태는 `Saved/DoorFixtureReport.json`을 확인한다.

### 로봇·지그 자세 제어 API

문마다 `UDoorPoseComponent`가 하나씩 있습니다. `ACamryAssembly::GetDoorPoseController(ECamryDoorSide::Left/Right)`로 접근합니다. 로봇·지그·상태 머신은 에디터 버튼 대신 이 컴포넌트의 함수를 호출합니다.

| 함수/이벤트 | 역할 |
|---|---|
| `CaptureReferencePose()` | 현재 자세를 차체 기준으로 저장 |
| `ApplyReferenceOffsetMm(OffsetMm)` | 저장 기준에서 차체 축 방향으로 지정 거리 이동 |
| `RestoreReferencePose()` | 저장된 위치·회전 복원 |
| `SetDoorWorldPose(WorldPose)` | 운반 장치 등이 계산한 월드 자세 적용. Unreal 단위 cm |
| `HasReferencePose()` | 기준 등록 여부 확인 |
| `OnDoorPoseChanged` | 자세 적용 후 C++ 구독자에 통지. 물리적 체결 완료 신호가 아님 |

명령 함수는 성공 여부를 bool로 반환합니다. 실패한 명령은 문을 이동시키지 않습니다. 기준 자세와 외부 장치가 명령한 현재 자세를 분리하므로 운반 명령이 정상 기준을 덮어쓰지 않습니다. 컴포넌트는 부모 연결을 변경하지 않습니다. 이후 공정 제어기가 운반·지그 간 소유권과 연결 전환을 관리하고, 한 시점에 한 제어 주체만 자세를 명령하게 구성해야 합니다. 센서값은 별도 계층에서 이 실제 자세를 읽어 생성합니다.

`Scripts/setup_door_pose.py`가 기준 초기 등록과 기능 검증을 수행합니다. 검증 결과는 `Saved/DoorPoseReport.json`, 변경 전 맵은 `Saved/Backups/DoorPose_*`에 기록합니다. `-CamryPoseVerifyOnly` 옵션은 새 Unreal 프로세스에서 저장된 기준을 읽고 이동·복원을 재검증하며 맵은 저장하지 않습니다.

2026-09-30 양쪽 문 기준 등록과 검증을 완료했습니다. mm 변환, 반복 명령 비누적, 반대 문·차체 유지, 위치·회전 복원, 외부 월드 자세 명령, 차체 이동·90도 회전, 운반 부모 연결 중 복원, 잘못된 입력 거부, 새 프로세스에서의 기준 유지가 통과했습니다. 기본 자세로 복원한 상태로 맵을 저장했습니다. 에디터 버튼의 화면 배치와 Undo 조작은 육안으로 확인하지 않았습니다.

전환·검증 스크립트는 `Scripts/migrate_camry_to_native.py`입니다. 성공 시 `Saved/CamryNativeMigrationReport.json`에 메시 11개의 변환·재질·계층 유지, 좌우 문 개별 10cm 이동·복원, 차량 전체 이동, 저장된 맵 재로드 결과를 기록합니다. `Saved/Backups/CamryNative_*`에 전환 전 맵과 BP를 보관하고, 참조가 없는 이전 BP 에셋은 Content에서 제거합니다. 원본 형상 확인 맵들은 유지합니다.

2026-09-30 전환과 위 검증을 완료했습니다. `Scripts/verify_camry_native.py`를 새 Unreal 프로세스에서 실행해 BP 제거 후에도 네이티브 Actor의 형상·배치·재질이 유지되는 것을 확인했습니다. 실제 뷰포트의 육안 확인은 별도입니다. 기존 `create_camry_assembly.py`, `finish_camry_assembly.py`, `inspect_camry_assembly.py`는 이전 BP 작업 기록이며 현재 작업에 실행하지 않습니다.

### C++ 빌드와 에디터 작업

2026-10-07 경로 이동 완료: 프로젝트는 `C:/Users/user/Documents/git/vrprogramming2026/unreal projects/test1`에 있다. 이동 전 Intermediate/Binaries에는 이전 한글 절대 경로가 남아 있어 Live Coding이 C1083으로 실패했다. 기존 캐시와 실행 스크립트는 `Saved/Backups/PathMigration_20261007_181728`에 보존하고, 새 위치에서 캐시를 재생성한 정규 C++ 빌드가 성공했다. `BuildEditor.ps1`과 `OpenEditor.ps1`는 이제 현재 프로젝트 경로를 직접 사용한다. 이전 TEMP 경로 연결을 사용하지 않는다. 새 에디터 세션의 Live Coding 컴파일·패치는 아직 확인 전이다.

일상적인 C++ 수정은 **Unreal Editor를 열어 둔 채 Live Coding으로 적용**합니다. 새 클래스나 헤더 변경이라는 이유만으로 먼저 종료하지 않습니다.

1. `Editor Preferences > General > Live Coding`에서 `Enable Live Coding`과 `Enable Reinstancing`을 켭니다. `Automatically Compile Newly Added C++ Classes`는 새 클래스 추가 시 자동 컴파일을 요청하는 선택 항목입니다.
2. 소스를 저장하고 에디터에서 `Ctrl+Alt+F11`을 누릅니다. Live Coding 콘솔의 컴파일·패치 결과를 확인한 뒤 새 클래스와 속성·동작을 확인합니다. 단축키 입력이나 클래스 파일 생성만으로 적용 성공을 판단하지 않습니다.
3. Object Reinstancing은 `UCLASS`, `USTRUCT`, `UFUNCTION` 등의 구조 변경을 반영합니다. 새 C++ 클래스도 실행 중 추가할 수 있습니다. 재생성되는 객체를 가리키는 포인터·캐시는 갱신하거나 무효화하도록 구현합니다.
4. `.cpp` 생성자에서 바꾼 기본값은 기존 인스턴스에 자동 반영되지 않습니다. 새 인스턴스와 기존 인스턴스를 구분해 확인하고, 기존 맵 설정을 바꿀 때는 명시적인 갱신 절차를 사용합니다.

이 동작과 설정은 [Epic의 UE 5.8 Live Coding 문서](https://dev.epicgames.com/documentation/en-us/unreal-engine/using-live-coding-to-recompile-unreal-engine-applications-at-runtime)를 기준으로 합니다. 현재 작업의 실제 컴파일·패치·배치 검증 결과는 별도로 기록합니다.

열린 작업 맵의 Actor 배치·수정·저장은 **같은 실행 중인 에디터**에서 수행합니다. 맵 저장에는 에디터 종료가 필요하지 않습니다. 다른 에디터 프로세스나 commandlet에서 그 맵을 동시에 수정·저장하지 않습니다.

컴파일 오류는 먼저 소스를 수정해 다시 시도합니다. 실제 패치·로딩 제한이나 변경 미반영이 확인되어 재시작이 필요할 때는 그 원인을 설명하고 저장 후 종료합니다. 별도로, 정규 빌드·새 프로세스 재로드·패키지 검증 시점을 정해 Live Coding 세션 밖에서도 동작하는지 확인합니다. 정규 Editor 모듈 빌드가 필요할 때는 에디터를 종료한 뒤 프로젝트 폴더에서 다음을 실행합니다.

```powershell
& .\Scripts\BuildEditor.ps1
```

현재 영문 프로젝트 경로에서 직접 빌드한다. BuildEditor.ps1는 동시 컴파일 수를 2개로 제한한다. 정규 빌드용이며 열린 에디터에서는 Live Coding을 사용한다.

## 원본 형상 확인 맵

언리얼 콘텐츠 브라우저에서 `Content/DigitalTwin/Maps/CamryBodyDoorPreview`를 엽니다.

- 차체: 293개 쉘 부품을 측면·펜더, 바닥·후방 구조, 전방 구조, 기둥·문턱, 지붕의 5개 Static Mesh로 묶었습니다.
- 앞문: 기존 6개 메시를 재사용합니다. 원본 좌표와 동일한 배치 변환을 적용해 차체와 상대 위치를 유지합니다.
- 아웃라이너의 `Camry/Body`, `Camry/FrontDoors` 폴더로 구분했습니다. 폴더별 숨김으로 문이 들어갈 자리를 확인할 수 있습니다.
- 회색 계열은 차체, 파란 계열은 앞문입니다. 판재 양면이 보이도록 확인용 재질을 적용했습니다.
- 기존 `CamryDoorPreview` 맵은 그대로 둡니다.

## 범위

완성차 전체가 아닌 선택된 차체 판재와 기존 앞문 표면입니다. 유리·본넷·트렁크 덮개·뒷문·구동계·좌석은 포함하지 않았습니다. 앞문도 모든 부품을 갖춘 조립체는 아닙니다.

형상만 가져왔으며 판 두께, 물리 충돌, 힌지 구속, 용접·조립 로직은 아직 구현하지 않았습니다. 차체 그룹은 미리보기 용도이며, 개별 원본 부품 ID와 이름은 `car models/Camry_Converted/BodyGroups/manifest.json`에 남아 있습니다.

## 검증

`Saved/CamryBodyImportReport.json`에서 그룹별 크기, 저장된 차체 5개·앞문 6개 액터, 같은 배치 좌표 적용 여부 및 메시 빌드 검사 결과를 확인할 수 있습니다. 명령줄 검사이며 실제 뷰포트 렌더링 검증은 별도입니다.

UV 없는 판재에 맞춰 MikkTSpace를 끄고 노멀·탄젠트를 다시 계산했습니다. 빌드는 완료됐으나 거의 0인 탄젠트·바이노멀 메시지가 남아 있어 표면 음영은 뷰포트에서 추가 확인해야 합니다. 현재 재질은 노멀 맵을 사용하지 않습니다.

### 2026-10-07 지그 구현 검증 완료

새 경로에서 C++ 빌드, DoorFixture_Left 배치, 접촉 6곳 등록, 문 이동 시 지그 고정과 상대 변위, 차체 이동·회전, 표시·리트랙션, 잘못된 법선 처리, 원래 자세 복원, 맵 저장 후 재로드 및 별도 프로세스 재검증을 통과했다. 실행 맵은 CamryAssemblyPreview다. 에디터에서 맵을 열고 DoorFixture_Left를 선택해 Fixture 설정을 확인한다. CamryAssembly의 왼쪽 Door Control에서 Apply Offset/Restore를 사용해 표시 변화를 확인한다. Play는 필요하지 않다. 검증 결과는 Saved/DoorFixtureReport.json, 로그는 Saved/Logs/DoorFixtureColdLoad.log에 있다. 뷰포트 육안 검증과 Live Coding 패치는 미확인이며, 안착·클램프 힘·응력·변형 계산은 후속 구현이다.

