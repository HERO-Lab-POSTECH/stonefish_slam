<!-- PR 제목: type(scope): 설명  — Conventional Commits 형식 (CONTRIBUTING.md §3) -->

## 요약

<!-- 무엇을·왜. 이슈가 있으면 "Closes #N" -->

## 변경 유형

- [ ] feat (새 기능)
- [ ] fix (버그 수정)
- [ ] docs / refactor / test / chore
- [ ] BREAKING CHANGE 포함 (본문에 설명 필수)

## 테스트 증빙 (필수)

<!-- 실행한 명령과 결과를 그대로 붙일 것. 예:
python3 -m pytest → 84 passed
시뮬 실기 확인: ros2 launch ... (RTX4070, corner e_y < 0.5 m) -->

## 체크리스트

- [ ] 해당 repo `python3 -m pytest` 통과 (slam은 `.so` 스테이징 후)
- [ ] `CHANGELOG.md` [Unreleased]에 항목 추가 (사용자에게 보이는 변경일 때)
- [ ] 문서 갱신 (README·docs — 동작/인터페이스가 바뀐 경우)
- [ ] `stonefish_msgs` 등 인터페이스 변경 시 상대 repo 대응 PR 링크
