# 3D Mapping Parameter Tuning Log

## 목표
Stonefish SLAM 3D 매핑의 품질을 정량적/정성적으로 개선하기 위한 파라미터 최적화 기록

---

## 파라미터 구조 (config/slam.yaml)

### 1. 소나 하드웨어 파라미터 (변경 불가)
- `max_range`: 20.0m (소나 최대 거리)
- `min_range`: 0.5m (소나 최소 거리)
- `horizontal_fov`: 130° (수평 시야각)
- `vertical_aperture`: 20° (수직 빔 폭)
- `image_width`: 918 (bearing 개수)
- `image_height`: 512 (range bin 개수)
- `sonar_tilt_deg`: 30° (하향 틸트)

### 2. 3D Octree 파라미터 (mapping_3d)
- `map_3d_voxel_size`: 0.5m (복셀 해상도) **[조정 가능]**
- `min_probability`: 0.6 (occupied 판정 확률 임계값) **[조정 가능]**
- `log_odds_occupied`: 1.5 (occupied voxel 증가량) **[조정 가능]**
- `log_odds_free`: -2.0 (free space 감소량) **[조정 가능]**
- `log_odds_min`: -10.0 (클램핑 최소값)
- `log_odds_max`: 10.0 (클램핑 최대값)

### 3. Bayesian 업데이트 파라미터
- `adaptive_update`: true (adaptive protection 사용) **[조정 가능]**
- `adaptive_threshold`: 0.5 (보호 임계값) **[조정 가능]**
- `adaptive_max_ratio`: 0.5 (보호된 voxel의 최대 업데이트 비율) **[조정 가능]**

### 4. Range Weighting 파라미터
- `use_range_weighting`: true (거리 기반 가중치) **[조정 가능]**
- `lambda_decay`: 0.1 (지수 감쇠 계수) **[조정 가능]**

### 5. SLAM 노이즈 모델 파라미터
- `slam_prior_noise`: [0.1, 0.1, 0.01]
- `slam_odom_noise`: [0.2, 0.2, 0.02]
- `slam_icp_noise`: [0.1, 0.1, 0.01]

### 6. 키프레임 파라미터
- `keyframe_translation`: 3.0m (키프레임 간격)
- `keyframe_rotation`: 0.5236 rad (30°)

---

## 파라미터 간 관계 분석

### A. Voxel Resolution vs Map Quality
**관계**: `map_3d_voxel_size` ↔ 맵 디테일, 메모리, 처리 속도
- **작은 값 (0.1-0.3m)**: 세밀한 장애물 표현, 높은 메모리/연산
- **큰 값 (0.5-1.0m)**: 거친 표현, 낮은 메모리/연산, robust

**예상**: 0.5m는 BlueROV2의 6-8 thruster (직경 ~0.1m) 대비 적절한 밸런스

### B. Log-Odds Balance
**관계**: `log_odds_occupied` vs `log_odds_free`
- **비율 (occupied/free)**: 현재 1.5 / -2.0 = -0.75
- **의미**: free space가 더 강하게 업데이트 → 보수적 맵 (false positive 감소)
- **역할**: occupied가 클수록 소나 반사 신뢰, free가 클수록 빈 공간 신뢰

**문제 가능성**:
- free space가 너무 강하면 → 실제 장애물이 사라짐 (occluded region)
- occupied가 너무 강하면 → 노이즈가 고착됨 (ghost objects)

### C. Adaptive Update Protection
**관계**: `adaptive_update` + `adaptive_threshold` + `adaptive_max_ratio`
- **역할**: 이미 확신 있는 voxel (probability > 0.5)의 급격한 변화 방지
- **max_ratio=0.5**: 보호된 voxel은 기존 업데이트의 50%만 적용

**트레이드오프**:
- **강한 보호 (ratio < 0.5)**: 맵 안정적, 하지만 동적 객체 추적 느림
- **약한 보호 (ratio > 0.7)**: 동적 변화 반응 빠름, 노이즈에 민감

### D. Range Weighting
**관계**: `use_range_weighting` + `lambda_decay`
- **공식**: weight(r) = exp(-λ * r / max_range)
- **현재**: λ=0.1 → weight(20m) ≈ 0.90 (10% 감소)

**의미**:
- 가까운 측정 (0-5m): weight ≈ 0.97-1.0 (높은 신뢰)
- 먼 측정 (15-20m): weight ≈ 0.90-0.93 (약간 낮은 신뢰)

**문제 가능성**:
- λ가 너무 작으면 (0.05) → 먼 거리도 신뢰 → 노이즈 증가
- λ가 너무 크면 (0.3) → 먼 거리 무시 → 맵 범위 축소

### E. Minimum Probability Threshold
**관계**: `min_probability` → 최종 point cloud 필터링
- **현재**: 0.6 → 60% 확률 이상만 "occupied"로 분류
- **효과**: 높을수록 노이즈 감소, 낮을수록 디테일 증가

---

## 테스트 히스토리

### Baseline (초기 설정)
**날짜**: 2025-11-26
**파라미터**:
```yaml
map_3d_voxel_size: 0.5
min_probability: 0.6
log_odds_occupied: 1.5
log_odds_free: -2.0
adaptive_update: true
adaptive_threshold: 0.5
adaptive_max_ratio: 0.5
use_range_weighting: true
lambda_decay: 0.1
```

**실제 실행 파라미터** (launch 파일 override):
```yaml
resolution: 0.1m  # slam.yaml의 0.5가 아님!
frame_interval: 15  # 매 15프레임마다 처리
sonar_tilt_deg: 10.0  # 노드 기본값 (slam.yaml의 30.0 무시됨) ⚠️ 버그!
```

**관측 결과**:
- ✅ **계산 속도**: 준수함
- ⚠️ **Frame step**: 15는 너무 큼, 더 낮춰야 함 (5-10 제안)
- ✅ **처음 관측 영역**: 업데이트 매우 잘됨
- ❌ **재관측 시 문제**: Free space가 너무 쉽게 occupied로 전환
  - Clamp가 작은지, occupied 업데이트가 큰지 불명확
  - **분석**: adaptive_max_ratio=0.5가 너무 약함 (50% 업데이트 허용)
- ❌ **성능 문제**: Voxel 수 증가 → RViz 회전 느림, path following 문제
  - **원인**: Resolution 0.1m가 너무 작음 (8배³ = 512배 더 많은 voxel)
- ❌ **Tilt 버그**: Tilt 30°인데 로봇과 같은 z축에 voxel 생성
  - **원인**: mapping_3d_standalone_node.py의 기본값 10.0° 사용
  - **영향**: 소나가 수평에 가깝게 작동 → 잘못된 맵 생성

---

**성능 데이터**:
```
Frame 15: 1916.2ms (0.5 FPS) | 51200 voxels
Frame 30: 1930.1ms (0.5 FPS) | 51200 voxels
```
- **문제**: 0.5 FPS는 실시간 사용 불가능 (목표: 10+ FPS)
- **원인**: Resolution 0.1m → 125배 더 많은 voxel (0.5m 대비)

**Ctrl+C 종료 에러**: `rcl_shutdown already called` (graceful shutdown 문제, 나중에 수정)

---

## 다음 테스트 계획

### Test 1: 크리티컬 버그 수정 + 성능 개선
**날짜**: 2025-11-26

**변경 사항** (launch 파일):
```yaml
# 버그 수정
sonar.sonar_tilt_deg: 30.0  # 10.0 → 30.0 (시뮬레이터와 일치)

# 성능 개선
resolution: 0.3  # 0.1 → 0.3 (27배 적은 voxel, 예상: 2000 voxels)
frame_interval: 5  # 15 → 5 (3배 빠른 업데이트)

# Adaptive update 강화
mapping_3d.adaptive_max_ratio: 0.3  # 0.5 → 0.3 (free→occupied 어렵게)
mapping_3d.log_odds_max: 15.0  # 10.0 → 15.0 (occupied 더 강하게 고정)
```

**예상 효과**:
1. ✅ Tilt 버그 수정 → 정확한 z축 맵 생성
2. ✅ 성능 27배 개선 → 예상 10+ FPS
3. ✅ Frame interval 3배 감소 → 맵 디테일 증가
4. ✅ Adaptive 강화 → 재관측 시 안정성 증가

**트레이드오프**:
- ⚠️ Resolution 0.3m → 작은 장애물(~0.2m) 표현 어려움
- ⚠️ Voxel 크기 증가 → 경계가 약간 거칠어질 수 있음

**관측 결과** (Test 1):
- ❌ **Tilt 버그 지속**: 여전히 로봇과 동일한 z축에 복셀 생성
  - Launch 파일에서 `sonar.sonar_tilt_deg: 30.0` 설정했으나 적용 안 됨
  - **의심**: C++ backend가 transform 무시? 또는 파라미터 전달 실패?
  - **조치**: 디버깅 로그 추가 필요
- ❌ **Free space 문제 지속**: 여전히 너무 쉽게 occupied로 전환
  - adaptive_max_ratio=0.3으로 강화했으나 효과 없음
  - **근본 원인 발견**: Adaptive protection이 occupied만 보호, free는 보호 안 함
  - **분석**: `adaptive = (avg_update > 0)` → 양수(occupied)만 보호
  - **사용자 통찰**: "소나 반사 전의 확실한 free space가 너무 쉽게 채워짐"
- ⚠️ **Frame step**: 5는 너무 빠름, 10으로 조정 요청

---

### Test 2: Free Space 보호 + Log-Odds 밸런스 재조정
**날짜**: 2025-11-26

**코드 수정**:
1. **mapping_3d.py:879**: `adaptive = True` (항상 보호, free space 포함)
2. **mapping_3d_standalone_node.py:29**: 노드 이름 `slam_node`로 수정 (네임스페이스 통일)
3. **mapping_3d_standalone_node.py:125**: Tilt 디버깅 로그 추가

**파라미터 변경** (slam.yaml 직접 수정):
```yaml
# Frame interval 조정
frame_interval: 10  # 5 → 10 (사용자 요청)

# Python backend 강제 사용 (adaptive protection 테스트)
mapping_3d.use_cpp_backend: false  # C++ backend 비활성화

# Log-odds 밸런스 재조정
mapping_3d.log_odds_occupied: 1.0  # 1.5 → 1.0 (반사 신뢰도 낮춤)
mapping_3d.log_odds_free: -3.0  # -2.0 → -3.0 (확실한 free 강화)

# Clamp 확대 (occupied를 더 강하게 고정)
mapping_3d.log_odds_min: -15.0  # -10.0 → -15.0
mapping_3d.log_odds_max: 20.0  # 15.0 → 20.0
```

**예상 효과**:
1. ✅ **Free space 보호**: `adaptive=True` → free voxel도 adaptive protection 적용
2. ✅ **Log-odds 밸런스**: occupied(1.0) vs free(-3.0) = -3.0 ratio
   - Free space 3번 업데이트 = occupied 1번 상쇄
   - 확실한 free가 더 강하게 유지됨
3. ✅ **Tilt 디버깅**: 로그에서 실제 적용 값 확인 가능
4. ⚠️ **성능**: Python backend는 C++보다 느림 (임시)

**검증 항목**:
- [ ] 로그에서 `sonar_tilt=30.0°` 확인
- [ ] Free space가 occupied로 쉽게 전환되지 않음
- [ ] 재관측 시 맵 안정성 증가
- [ ] Z축 복셀 위치 (로봇보다 아래에 생성되어야 함)

**관측 결과** (Test 2 - 중간):
- ❌ **C++ backend 여전히 활성**: declare_parameter 기본값이 YAML 덮어씀
  - 로그: `[OctreeMapping] OpenMP enabled` → C++ 사용 중
  - 결과: `adaptive=True` 수정이 적용 안 됨 (Python 경로 실행 안 됨)
- ❌ **바닥면 사라짐 문제 심화**: 듬성듬성 빈 공간 많음
  - **사용자 질문**: "Free space는 첫 반사 전만 업데이트하는데, 왜 반사된 바닥이 사라지나?"
  - **답변**: **Occlusion + 시점 변화 문제**
    ```
    관측 1 (로봇 낮은 위치, z=1m):
      Sonar ray → 바닥(z=2m) 반사 → occupied

    관측 2 (로봇 높은 위치, z=0.5m):
      Sonar ray → 먼 벽(z=5m) 반사
      Free space: z=0.5 ~ z=5 (이전 바닥 z=2 포함!)
      → 바닥이 free로 업데이트됨
    ```
  - **근본 원인**: Adaptive protection 없어서 확정된 occupied가 보호 안 됨
- ⚠️ **수정**: `declare_parameter('use_cpp_backend', False)` 기본값 변경

---

### Test 3: Adaptive Protection 양방향 수정
**날짜**: 2025-11-26

**핵심 버그 발견 및 수정**:
1. **octree.py:141**: Adaptive protection이 **한쪽만 보호**하는 버그
   ```python
   # 이전 (버그)
   if adaptive and log_odds_update > 0:  # occupied 업데이트만 보호

   # 수정
   if adaptive:
       if log_odds_update > 0:
           # Occupied 업데이트: free voxel 보호
       else:
           # Free 업데이트: occupied voxel 보호 ← 추가!
   ```

**바닥이 사라지는 이유** (버그):
```
바닥 = occupied (prob=95%)
Free 업데이트 (-3.0) 들어옴
→ log_odds_update < 0 → adaptive 조건 실패
→ -3.0 그대로 적용 × 5회
→ occupied → free 전환
```

**수정 후**:
```
바닥 = occupied (prob=95%)
Free 업데이트 (-3.0) 들어옴
→ current_prob >= 0.5 → adaptive 보호 발동!
→ update_scale = 0.3 → -3.0 × 0.3 = -0.9
→ 천천히 감소, 확정된 occupied 유지
```

**파라미터 변경**:
- `map_3d_voxel_size: 0.2m` (0.3 → 0.2, 사용자 요청)

**Tilt 디버깅**:
- Transform matrix 출력 추가 (실제 rotation 확인)

**검증 항목**:
- [ ] 바닥면이 안정적으로 유지됨
- [x] DEBUG 로그에서 rotation matrix 확인 ✅
- [x] Tilt 30° 정상 작동 확인 ✅
- [❌] **로봇보다 아래에 voxel 생성**: `Voxel z=[5.00, 13.80]m` - **로봇과 같은 높이!**

**Tilt 버그 근본 원인 발견**:
- **Vertical aperture 범위 오류**: `-10° ~ +10°` (양방향)
- **문제**: 음수 각도는 소나 중심선보다 **위**를 향함
- **결과**: 30° 하향 tilt인데도 일부 ray가 수평 또는 위를 향함
- **수정**: `range(0, num_vertical_steps + 1)` - **하향만** (0° ~ +10°)

---

### Test 4: 단방향 Adaptive Protection + Log-Odds 비율 역전
**날짜**: 2025-11-26

**문제 발견**:
- **이전 상황**: 단방향 adaptive protection 적용 후 floor voxel이 여전히 너무 쉽게 사라짐
- **분석**: Adaptive protection은 occupied→free 전환을 완전히 허용 (update_scale=1.0)
- **핵심 원인**: Log-odds 비율 불균형
  ```
  이전: log_odds_occupied=1.0, log_odds_free=-3.0
  비율: occupied:free = 1.0:3.0 (1:3)
  문제: 1번의 free 업데이트 (-3.0)가 3번의 occupied 업데이트 (+1.0)를 상쇄!

  예: floor = occupied (prob=95%, log_odds=2.944)
      1번 free 업데이트 → log_odds = 2.944 - 3.0 = -0.056 (free!)
  ```

**핵심 통찰**:
- Sonar 반사 (occupied): **매우 신뢰할 수 있음** (명확한 반사신호)
- Sonar ray 통과 (free): **신뢰도 낮음** (반사 없는 이유가 ray path 상 occlusion일 수 있음)
- **비율을 역전**: occupied가 free를 압도하도록

**Code (octree.py lines 140-153)** - 단방향 보호 유지:
```python
if adaptive:
    if log_odds_update > 0:
        # Occupied 업데이트: free voxel을 occupied로 만드는 것 제한
        update_scale = self.adaptive_max_ratio
    # else: Occupied → Free 전환은 보호 안 함 (update_scale=1.0)
```

**파라미터 변경** (slam.yaml lines 25-26):
```yaml
# Log-odds 비율 3배 강화 (역전)
log_odds_occupied: 3.0    # 1.0 → 3.0
log_odds_free: -1.0       # -3.0 → -1.0

# 다른 파라미터는 유지
map_3d_voxel_size: 0.2
adaptive_update: true
adaptive_threshold: 0.5
adaptive_max_ratio: 0.3   # Free→Occupied만 보호
frame_interval: 10
```

**Ratio 분석**:
```
이전: occupied(1.0) : free(-3.0) = 1:3 → free가 3배 강함 ❌
수정: occupied(3.0) : free(-1.0) = 3:1 → occupied가 3배 강함 ✅

예: floor = occupied (prob=95%, log_odds≈2.944)
    1번 free 업데이트: log_odds = 2.944 - 1.0 = 1.944 (여전히 occupied) ✅
    3번 free 업데이트: log_odds = 2.944 - 3.0 = -0.056 (free)
    → 3번의 free 업데이트 필요 (합리적 threshold)
```

**예상 효과**:
1. ✅ Floor 안정성 대폭 증가 (3배 강한 occupied)
2. ✅ 단일 free ray로 인한 부정확한 업데이트 방지
3. ✅ Sonar 반사의 신뢰도 우선 (해양 환경의 특성)
4. ✅ Adaptive protection 원래 목적 달성 (occupied만 보호)
5. ⚠️ **트레이드오프**: 실제 비워진 영역 인식이 느려질 수 있음 (3번 통과 필요)

**물리적 근거**:
- **Sonar 반사**: 직접 반사신호 → 매우 신뢰도 높음 (prob↑)
- **Free space**: Ray path에 다른 반사체가 없다는 추론 → 불확실성 높음
- **해상 환경**: 부유물, 기포, 온도 경계 → 거짓 free space 빈번
- **결론**: occupied를 free보다 3배 강하게 신뢰하는 것이 합리적

**검증 항목**:
- [✅] floor voxel이 안정적으로 유지됨
- [ ] 움직임으로 인한 실제 업데이트 (3회 확인 필요)
- [ ] 재관측 시 맵이 수렴함 (진동 없음)

**상태**: 적용됨 - 통합 테스트 진행 중

---

## 파라미터 튜닝 우선순위 (추천 순서)

1. **log_odds_occupied / log_odds_free 비율** (맵 신뢰도 균형)
2. **min_probability** (노이즈 필터링)
3. **lambda_decay** (거리 신뢰도)
4. **map_3d_voxel_size** (해상도 vs 성능)
5. **adaptive_max_ratio** (맵 안정성 vs 반응성)

---

## 참고: 파라미터 권장 범위

| Parameter | Min | Baseline | Max | Notes |
|-----------|-----|----------|-----|-------|
| `voxel_size` | 0.1 | 0.2 | 1.0 | 0.1은 메모리 폭발 위험, 0.2 추천 |
| `min_probability` | 0.5 | 0.6 | 0.8 | 0.8은 보수적 (디테일 손실) |
| `log_odds_occupied` | 1.0 | **3.0** | 5.0 | **3.0 강추** - sonar 신뢰도 우선 |
| `log_odds_free` | -3.0 | **-1.0** | -0.5 | **-1.0 강추** - free space 신뢰도 낮춤 |
| `lambda_decay` | 0.05 | 0.1 | 0.3 | 0.3은 원거리 무시 |
| `adaptive_max_ratio` | 0.2 | 0.3 | 0.5 | 단방향 보호 전용 (free→occupied만) |

**Log-Odds 비율 가이드** (신규):
| Ratio | occupied | free | 특성 | 추천 |
|-------|----------|------|------|------|
| **1:3** (이전) | 1.0 | -3.0 | Free space 우선 | ❌ Floor 소멸 문제 |
| **1:1** | 1.0 | -1.0 | 균형 | ⚠️ 중간값 |
| **3:1** (현재) | 3.0 | -1.0 | Occupied 우선 | ✅ **강추** |
| **5:1** | 5.0 | -1.0 | Occupied 매우 강함 | ⚠️ 동적 변화 느림 |

