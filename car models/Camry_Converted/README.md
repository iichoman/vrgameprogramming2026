# Camry v5a 판재 형상 변환

원본 `camry-detailed-v5a.key`의 `*NODE`, `*ELEMENT_SHELL`, `*PART`를 읽어 만든 OBJ입니다. 원본 파일은 변경하지 않았습니다.

## 파일

- `camry_shells_m.obj`: 전체 쉘 요소를 부품별 OBJ 오브젝트로 분리한 모델. 차체 외에 실내, 유리 등 쉘로 정의된 부품도 포함합니다.
- `camry_frontdoor_named_parts_m.obj`: 이름에 `frontdoor`가 포함된 부품만 모은 확인용 모델. 양쪽 앞문 관련 부품이 섞여 있으며, 완전한 도어 조립체로 검증된 선택은 아닙니다. 다른 이름의 힌지·유리·트림 등이 빠질 수 있습니다.
- `parts.csv`: 내보낸 부품 ID, 원본 이름, 정점·면 개수.
- `conversion_report.json`: 변환 범위와 통계.
- `validation.json`: 생성 OBJ를 다시 읽어 확인한 정점 참조·개수·크기 검사 결과.
- `preview_shells.png`, `preview_frontdoors.png`: 일부 면의 모서리를 투영한 확인 이미지. 전체 면 렌더링은 아닙니다.

## Blender에서 열기

1. OBJ를 가져오고 오브젝트 분리를 유지합니다. 오브젝트 이름은 `p부품ID_원본이름`입니다.
2. 좌표 수치는 **미터**입니다. mm → m 변환이 이미 적용됐으므로 다시 0.001배 하지 마세요.
3. 원본 좌표축과 원점을 유지했습니다. 차량이 화면 중심에서 벗어나면 전체 선택 후 선택 항목으로 화면을 맞춥니다.
4. 먼저 앞문 확인용 OBJ로 형상을 살펴본 뒤 전체 모델에서 필요한 차체·도어 부품을 선택합니다.
5. 두 OBJ에는 중복되는 부품이 있으므로 같은 장면에 겹쳐 넣지 마세요.
6. 재질, 표면 방향, 충돌용 형상, 필요한 두께를 정리한 뒤 FBX로 내보낼 수 있습니다. Unreal로 가져올 때 차체와 움직일 도어를 합치지 않습니다.

## 변환의 한계

- 쉘의 기준면만 추출했습니다. 해석에 설정된 두께는 입체로 만들지 않았습니다.
- 솔리드·빔·이산 요소·질량 요소는 이번 출력에서 제외했습니다.
- UV, 원본 렌더링 재질, 용접 연결, 힌지 구속, 강도, 변형 및 충돌해석 동작은 포함하지 않습니다.
- OBJ 형식 검사와 간단한 투영 이미지 확인은 Unreal/Blender 실제 임포트 검증을 대체하지 않습니다.
- 연구용 충돌해석 모델이므로 실제 제조 공차나 1mm 수준의 조립 간극을 검증한 형상으로 간주하지 않습니다.

## 출처

2012 Toyota Camry Detailed Finite Element Model v5a, CCSA / George Mason University, FHWA 지원.
공식 페이지: https://www.ccsa.gmu.edu/models/2012-toyota-camry/
원본 README의 사용·출처 안내를 함께 보관하세요. 원본은 논문 및 출판물에서 CCSA at GMU와 FHWA를 명시하도록 요청합니다.

## 재실행

프로젝트 루트에서:

```powershell
python tools/convert_camry_key.py "car models/2012-toyota-camry-detailed-v5a/2012-toyota-camry-detailed-v5a/camry-detailed-v5a.key" "car models/Camry_Converted"
python tools/validate_camry_obj.py "car models/Camry_Converted"
```
