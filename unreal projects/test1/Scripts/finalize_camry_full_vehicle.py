"""Document the new test map only after successful import and cold-load checks."""
import json
from pathlib import Path

project = Path(__file__).resolve().parents[1]
report = json.loads((project / 'Saved/CAR_TESTColdLoadReport.json').read_text(encoding='utf-8'))
assert report['success'] and report['cold_load_verified']
assert report['mesh_count'] == 54 and report['source_parts'] == 1068
assert all(x['unreal_triangles'] > 0 for x in report['meshes'])
render_report = json.loads((project / 'Saved/CAR_TESTRenderReport.json').read_text(encoding='utf-8'))
assert render_report['success'] and len(render_report['captures']) == 2
assert all(Path(x['file']).is_file() for x in render_report['captures'])
doc = project / 'CAMRY_IMPORT.md'
marker = '## 2026-10-14 CAR_TEST 전체 차량 확인 맵'
text = '''

## 2026-10-14 CAR_TEST 전체 차량 확인 맵

`Content/DigitalTwin/Maps/CAR_TEST`는 원본 Camry 차량의 전체 표면 형상을 확인하는 별도 맵이다. 기존 CamryAssemblyPreview와 CamryBodyDoorPreview는 변경하지 않았다. 앞서 준비한 FullVehicleTest 메시를 재사용하며, 이전 CamryFullVehicle_Test는 작업 도중의 시험 맵이다. 전체 차량 확인에는 CAR_TEST를 사용한다.

- 원본의 999개 쉘 부품과 69개 솔리드 부품 외피를 54개 Static Mesh로 임포트했다. 앞·뒤 시트 쿠션, 등받이, 헤드레스트, 앞·뒤·측면 유리, 도어, 휠, 차체 및 구동계 등 원본의 표면 형상이 포함된다.
- 아웃라이너 `CamryFull` 아래에 Seats, Glass, Doors, Wheels, Roof, Body, Interior 등의 폴더가 있다. 분류는 원본 이름에 기초한 확인용 분류이며 완성 조립체의 엄밀한 부품 구성 정의는 아니다.
- 시트 주요 쿠션과 유리는 별도 메시다. 유리 12개 원본 레이어 중 전·후면 유리 내부 레이어 4개는 중복 표시를 줄이기 위해 숨겨져 있고 `Glass/HiddenSourceLayers`에 보존되어 있다.
- 맵을 열고 Play를 누른 후 화면을 클릭하면 기본 자유 비행 Pawn으로 둘러볼 수 있다. WASD와 마우스로 이동·시점 조작, E/Space로 상승, Q/C로 하강한다. 에디터에서는 우클릭을 누른 채 WASD로 이동할 수 있다.
- `ReviewCameras/View_01_WholeVehicle`, `View_02_InteriorSeats`는 전체·실내 관찰 카메라다. 시트만 보고 싶으면 Seats의 메시를 선택해 F로 초점을 맞추고 주변 Roof/Doors/Glass 폴더를 일시적으로 숨길 수 있다.
- Blueprint 없이 엔진의 네이티브 StaticMeshActor, CameraActor, GameModeBase를 사용한다. 신규 C++ 컴파일은 필요 없다.
- 전체 원본 표면은 약 427만 삼각형으로 무겁다. 차량 Actor의 NoCollision 프로필과 충돌 해제를 저장하고, 레이 트레이싱·거리장·Lumen 카드 생성이 꺼진 기존 메시를 재사용한다. 테스트 맵의 Lumen·모션 블러를 끈 확인용 설정이며 프로젝트 전역 렌더 설정은 변경하지 않는다.
- OBJ는 미터, Unreal은 센티미터다. FBX/OBJ 임포터의 좌표계 변환 `(x,y,z) -> (100*x,-100*y,100*z)`와 공통 배치 이동 `(250,0,0)cm`를 검증한다. 기존 Camry 임포트와 같은 좌표 변환이다.
- 재질은 구분용이며 원본 실차 색상·텍스처가 아니다. 쉘 두께는 입체화하지 않았고, 용접·스프링 빔·이산 요소·점질량, 원본 접촉·재질·물리 시뮬레이션은 가져오지 않았다. 원본이 포함하지 않는 세부 장식까지 새로 만들지는 않는다.

변환은 `tools/extract_camry_full_vehicle.py`, 임포트는 `Scripts/import_camry_full_vehicle.py`를 사용한다. `Scripts/full_vehicle_manifest.json`에는 포함 부품과 좌표·개수·제외 사항을 기록했다. 원본 KEY/OBJ는 Git 공유 대상이 아니지만, 생성된 Unreal 에셋과 맵은 기존 프로젝트 공유 규칙을 따른다.

검증 기록: `Saved/CAR_TESTReport.json`, `Saved/CAR_TESTColdLoadReport.json`. 54개 메시, 1,068개 부품의 포함 목록, 메시별 실제 크기·좌표 변환·배치·가시성·충돌 프로필과 저장된 충돌 설정 및 새 프로세스 재로드를 확인한다. 전체·실내 렌더 캡처 결과는 `Saved/CAR_TESTRenderReport.json`과 실행별 `Saved/CAR_TESTPreview` 하위 폴더에 기록한다. Play 자유 이동은 엔진 기본 GameModeBase/DefaultPawn 구성이며 자동 플레이 입력 시험과 조립 공정 검증은 포함하지 않는다.

원본 출처: GMU Center for Collision Safety and Analysis(CCSA), FHWA 지원, 2012 Toyota Camry Detailed FE Model v5a(2016). 충돌 해석용 모델이며 제조 공차·조립 정확도를 보장하는 제작 CAD로 해석하지 않는다.
'''
existing = doc.read_text(encoding='utf-8')
if marker not in existing:
    doc.write_text(existing.rstrip() + text, encoding='utf-8')
print('CAMRY_FULL_DOCUMENTATION_UPDATED')
