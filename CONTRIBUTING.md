# Contributing Guide — stonefish-ws (HERO Lab)

stonefish_sim · stonefish_slam · stonefish(fork) 세 repo와 이 워크스페이스에 공통 적용되는
협업 규칙입니다. **정본은 워크스페이스 repo의 `CONTRIBUTING.md`**이며, 각 repo의 사본은
정본과 동일하게 유지합니다(사본만 고치지 말 것).

> **발효 시점**: 2026-07-23. 그 이전 이력(브랜치명·직접 커밋 등)에는 소급 적용하지 않습니다.

## 1. 브랜치 규칙 (GitHub Flow)

소규모 연구팀 표준인 GitHub Flow를 따릅니다. 장수 브랜치는 `main` 하나뿐입니다.

- **`main` 직접 커밋 금지.** 모든 변경은 브랜치 → PR → 리뷰 → merge 경로로만 들어갑니다.
- 브랜치는 **짧게** 유지하고(수일 단위), merge 후 삭제합니다.
- 브랜치 이름: `<type>/<짧은-설명>` (소문자, 하이픈 구분). 이슈가 있으면 번호를 붙입니다.

| type | 용도 | 예시 |
|:--|:--|:--|
| `feat/` | 새 기능 | `feat/fls-noise-model` |
| `fix/` | 버그 수정 | `fix/12-fft-mask-erosion` |
| `docs/` | 문서만 | `docs/install-guide` |
| `refactor/` | 동작 불변 구조 변경 | `refactor/split-mapping-node` |
| `test/` | 테스트만 | `test/golden-octree` |
| `chore/` | 빌드·설정·잡무 | `chore/docker-base-bump` |
| `exp/` | 실험용(merge 전제 아님) | `exp/cascade-gain-sweep` |

`exp/` 브랜치는 결과가 채택될 때만 정식 `feat/`·`fix/` PR로 정리해 올립니다.

## 2. 커밋 규칙 (Conventional Commits v1.0.0)

커밋 메시지는 [Conventional Commits](https://www.conventionalcommits.org/en/v1.0.0/)를 따릅니다.
(기존 커밋 이력이 이미 이 형식을 사용 중입니다 — 그대로 유지.)

```text
<type>(<scope>): <제목 — 명령형, 72자 이내>

<본문 — 무엇을(What)보다 왜(Why)를. 측정·검증 결과가 있으면 요약>
```

- **type**: `feat` `fix` `docs` `style` `refactor` `perf` `test` `build` `ci` `chore` `revert`
- **scope**: 패키지/모듈명 — 예: `feat(albc_bridge):`, `fix(slam.core):`, `docs(control):`
- **호환성 깨는 변경**: type 뒤 `!` + 본문에 `BREAKING CHANGE:` 설명 — SemVer MAJOR 대상
- 커밋은 **원자적으로**(한 커밋 = 한 논리 변경). 서로 무관한 변경을 한 커밋에 섞지 않습니다.

## 3. PR 규칙

- **PR 제목도 커밋과 동일한 `type(scope): 설명` 형식**을 사용합니다(squash 시 제목이 커밋이 됨).
- `.github/PULL_REQUEST_TEMPLATE.md` 템플릿을 채웁니다 — 특히 **테스트 증빙**(실행한 명령과
  결과)은 생략 불가.
- **승인 1인 이상** 필수. 본인 PR은 본인이 승인하지 않습니다.
- 올리기 전 자체 게이트: 해당 repo `python3 -m pytest` 통과 확인
  (slam은 [.so 스테이징](#6-테스트--품질-게이트) 선행). 결과를 템플릿에 기록합니다.
- **merge 방식**: 커밋 이력이 지저분하면 squash merge, 커밋들이 각각 의미 있는 원자
  단위면 merge commit으로 보존(리뷰어 재량 — PX4 방식).
- PR은 **작게**. 논리적으로 독립인 변경은 PR을 나눕니다(수천 줄 PR 금지).
- 리뷰가 3일 이상 없으면 리뷰어에게 ping. merge된 브랜치는 삭제합니다.
- **PR과 main의 정합 유지**: 브랜치 내용을 main에 직접 merge/push 해버리고 PR을 열어두는
  것 금지 — merge는 반드시 GitHub PR 버튼으로 (이력 추적·리뷰 기록 보존).

## 4. 버전·릴리스 규칙

- **SemVer** `MAJOR.MINOR.PATCH` — `fix`→PATCH, `feat`→MINOR, BREAKING→MAJOR.
- `CHANGELOG.md`는 [Keep a Changelog](https://keepachangelog.com/en/1.1.0/) 형식:
  `[Unreleased]` 최상단, 릴리스는 `## [x.y.z] - YYYY-MM-DD`, 변경 유형은
  Added/Changed/Deprecated/Removed/Fixed/Security 여섯 가지만.
- 릴리스 시: repo 내 **모든 패키지의 `package.xml` 버전 동기** (sim은 멀티패키지) →
  CHANGELOG 정리 → annotated tag `git tag -a vX.Y.Z -m "..."` → push.
- 신규 패키지도 placeholder 금지: `package.xml`의 description/maintainer/license를
  실제 값으로 채우고 버전은 repo 통일 버전을 따릅니다.

## 5. 멀티-repo 조정 (stonefish.repos)

- 워크스페이스 repo의 `stonefish.repos`가 소스 구성의 기준입니다 (`vcs import src < stonefish.repos`).
- **일상 개발**: 세 repo 모두 `version: main` 추적.
- **재현 스냅샷**(논문·데모·릴리스): 태그/커밋으로 핀한 `.repos` 사본을 만들어 커밋합니다.
- **인터페이스 변경 주의**: `stonefish_msgs`(sim repo)를 바꾸면 slam이 깨질 수 있습니다.
  msg/srv 변경 PR에는 영향받는 상대 repo의 대응 PR을 본문에 상호 링크합니다.

## 6. 테스트 · 품질 게이트

- 각 repo에서 `python3 -m pytest` 가 로컬 게이트입니다 (sim 84·slam 37 기준, 2026-07-23).
- **slam 사전 단계**: pybind11 확장을 소스 트리에 스테이징해야 수집이 됩니다.

  ```bash
  colcon build --merge-install --packages-select stonefish_slam
  cp build/stonefish_slam/*.so src/stonefish_slam/stonefish_slam/
  ```

- colcon 빌드는 **`--merge-install`로 통일**합니다(기존 install 레이아웃·문서·테스트 경로가
  merge 기준).
- 명명 규칙(워크스페이스 `.omp/NAMING.md`가 기준): 패키지 `stonefish_<name>`,
  Python 모듈 `snake_case.py`, msg/srv `PascalCase`, config `snake_case.yaml`
  (전대문자 약어 예외: `TAM.yaml`), 씬 파일 `snake_case.scn`.

## 7. 하지 말 것 (요약)

- `main` 직접 커밋 / PR 우회 merge / 열린 PR 방치(내용이 이미 main에 있는 stale PR)
- 커밋 없이 장기간 로컬 보관(작업 유실 위험 — WIP도 브랜치에 push)
- `build/`·`install/`·`log/`·`*.so` 커밋 (gitignore 준수)
- 시뮬 physics 파라미터(.scn)와 알고리즘 변경을 한 PR에 섞기
- placeholder(`TODO: Package description` 등)가 남은 채 merge
