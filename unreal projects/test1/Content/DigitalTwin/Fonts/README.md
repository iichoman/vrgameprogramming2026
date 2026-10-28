# 한국어 표시용 글꼴

`NanumGothic.ttf`는 Unreal Engine 5.8에 포함된 나눔고딕 원본을 변경하지 않고 프로젝트에 복사한 것이다. UI와 공장 구역 이름을 한국어로 표시하기 위해 사용한다.

- 원본: `Engine/Content/Editor/Slate/Fonts/NanumGothic.ttf`
- 저작권 및 예약 글꼴명: `NanumGothic_License.txt`의 엔진 동봉 고지를 그대로 보존한다.
- 라이선스 전문: `NanumGothic_OFL-1.1.txt`. 엔진 고지의 저작권·예약 글꼴명을 유지하고 SIL OFL 1.1 전문을 함께 제공한다.
- 배포 안내: [네이버 나눔글꼴](https://hangeul.naver.com/download)
- 라이선스 원문: [SIL Open Font License 1.1](https://openfontlicense.org/open-font-license-official-text/)

글꼴 파일과 고지문을 함께 유지한다. 이 글꼴의 라이선스가 프로젝트의 C++ 코드나 UI 문구 자체를 OFL로 바꾸는 것은 아니다.

Slate는 프로젝트의 `Content/DigitalTwin/Fonts/NanumGothic.ttf`를 읽는다. 에디터 전용 엔진 폰트 경로에 의존하지 않는다. `DefaultGame.ini`의 `DirectoriesToAlwaysStageAsUFS`에 `DigitalTwin/Fonts`를 추가하여 실행 파일 패키징 시 글꼴과 고지문도 포함한다. 실제 패키지 실행 확인은 별도 검증 항목이다.
