## 변경 내용

<!-- 무엇을 왜 바꿨는지 2~3줄. 커밋 메시지 body 와 겹쳐도 된다. -->

## 관련 명세 항목

<!-- 예: 4.2 자율 주행 — DWA 핵심 로직 직접 구현 / 7. 제약 사항 — colcon build 무경고 -->

## 검증 방법·결과

<!-- 실행한 명령과 실제 출력 요약. 시뮬레이션 검증은 시나리오·횟수·결과(충돌 0건 등)를 적는다. -->

```
scripts/build.sh
scripts/test.sh
```

## 체크리스트

- [ ] `colcon build` 경고/에러 0건 (CI build-test 통과)
- [ ] 테스트 통과 (`colcon test` + 신규 코드에 단위 테스트 추가)
- [ ] 문서 갱신 (docs/, README, 설정 파일 주석 — 해당 시)
- [ ] 커밋 메시지가 Conventional Commits 형식 (CI commit-lint 통과)
- [ ] base 브랜치가 `develop` (release PR 만 `main`)
