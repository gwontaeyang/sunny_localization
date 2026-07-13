# localization

Wheel Odometry + IMU를 EKF로 융합하여 로봇 pose를 추정하는 Lifecycle Node 패키지.  
V-마커 검출 시 EKF-SLAM 모드로 전환하여 랜드마크 기반 위치 보정을 수행한다.

---

## 1. 전체 구조

```
localization/
├── src/
│   ├── localization_node.cpp          ← main() 진입점
│   ├── localization.cpp               ← LifecycleNode, ROS 통신 창구
│   ├── localizer_factory.cpp          ← Factory (알고리즘 타입별 객체 생성)
│   └── localizer/
│       ├── localizer_base.cpp         ← 추상 기반 클래스 구현 (공통 유틸)
│       └── localizer_ekf.cpp          ← EKF (+ SLAM) 알고리즘 핵심 구현
│
└── include/localization/
    ├── localization.hpp               ← Localization LifecycleNode 헤더
    ├── localizer_factory.hpp          ← Factory 선언
    └── localizer/
        ├── localizer_base.hpp         ← 추상 인터페이스 + 공통 멤버 정의
        ├── localizer_ekf.hpp          ← LocalizerEKF 선언
        └── localizer_types.hpp        ← State, Params, 이벤트 상수, 타입 enum
```

### 클래스 관계

```
Localization (LifecycleNode)         ← ROS 통신 창구
  └── LocalizerBase* (인터페이스)    ← 알고리즘 추상화 계층
         └── LocalizerEKF            ← 유일한 현재 구현체
               └── VMarker           ← v_marker_finder 라이브러리 (도킹 시 사용)
```

---

## 2. LocalizerBase 의 Factory 구조

`LocalizerBase`를 인터페이스로 두고 `LocalizerFactory::createLocalizer(type, ...)`로 구체 구현체를 생성한다.  
이는 **추후 로컬라이제이션 알고리즘을 조건에 따라 교체**할 수 있도록 일부러 설계한 구조이다.

```cpp
// localizer_types.hpp 에 정의된 지원 예정 타입들
enum class LocalizerType {
    EKF,       // 현재 유일하게 구현됨
    AMCL,      // 예정 (AMCL — Monte Carlo)
    PARTICLE,  // 예정 (Particle Filter)
    UKF,       // 예정 (Unscented Kalman Filter)
    SLAM       // 예정 (Pure SLAM)
};
```

```cpp
// localizer_factory.cpp — 추가 구현은 아래에 case를 추가하면 됨
if (localizer_type == "EKF") {
    return std::make_unique<LocalizerEKF>(...);
// } else if (localizer_type == "AMCL") {
//     return std::make_unique<AmclLocalizer>(...);
}
```

새 알고리즘 추가 방법:
1. `LocalizerBase`를 상속하는 새 클래스 구현 (`processWheel`, `processImu`, `processPointCloud`, `prediction`, `correctionXxx` 구현)
2. Factory의 `if-else` 에 새 타입 추가
3. YAML 파라미터 `localizer_type` 변경

---

## 3. 현재 사용하는 로컬라이제이션 알고리즘 — EKF

`localizer_type: "EKF"` (설정 파일 기본값)

### 상태 벡터 (8차원)

```
μ = [ x,  y,  yaw,  v,  w,  lm_x,  lm_y,  lm_yaw ]
      ─────────────────────  ─────────────────────
        로봇 상태 (5)           랜드마크 상태 (3)
```

| 인덱스 | 변수 | 설명 |
|---|---|---|
| 0 | `x` | odom 프레임에서의 로봇 x 위치 [m] |
| 1 | `y` | odom 프레임에서의 로봇 y 위치 [m] |
| 2 | `yaw` | odom 프레임에서의 로봇 yaw [rad] |
| 3 | `v` | 선속도 [m/s] |
| 4 | `w` | 각속도 [rad/s] |
| 5 | `lm_x` | V-마커 랜드마크 x 위치 (odom) [m] |
| 6 | `lm_y` | V-마커 랜드마크 y 위치 (odom) [m] |
| 7 | `lm_yaw` | V-마커 랜드마크 yaw (odom) [rad] |

> 랜드마크 상태(5~7)는 일반 주행 중에도 항상 상태 벡터에 포함되어 있다.  
> SLAM 모드가 아닐 때는 랜드마크 공분산이 크게 유지되어 EKF에 영향을 주지 않는다.

---

## 4. 센서 구성 및 Fusion 방식

### 사용 센서

| 센서 | 토픽 | 관측 대상 |
|---|---|---|
| Wheel Odometry | `/motor_driver_node/odom` | 선속도 `v` |
| IMU | `/imu/data` | 각속도 `w` |
| LiDAR PointCloud | `/scan` | V-마커 위치 `[lm_x, lm_y, lm_yaw]` (SLAM 모드만) |

### EKF의 Prediction + Correction 구조

각 센서 콜백마다 **Prediction → Correction** 2단계를 수행한다.

```
센서 토픽 도착
    │
    ▼
[Prediction 단계]  ← 이전 상태의 v, w 를 운동 모델에 적용
    │
    ├── 일반 주행 (Ackermann)
    │     x' = x + R(sin(θ+Δθ) - sin(θ))
    │     y' = y - R(cos(θ+Δθ) - cos(θ))
    │     θ' = θ + w·dt
    │
    └── 제자리 회전 (Spin Turn, |v| ≈ 0)
          x' = x  (이동 없음)
          y' = y  (이동 없음)
          θ' = θ + w·dt
    
    ↓ (Jacobian Gt, 공분산 Σ 예측 포함)
    
[Correction 단계]  ← 센서 종류마다 다른 관측 모델 적용
```

#### Wheel Odometry Correction

```
관측값:  z = [v_measured]          ← 바퀴 인코더에서 선속도만 추출
예측값:  h(μ) = [μ_v]
Jacobian H : (1×8), H(0,3) = 1.0  ← v 상태만 직접 관측

→ Kalman gain을 통해 [x, y, θ, v, w] 모두 공분산 상관관계로 업데이트
```

#### IMU Correction

```
관측값:  z = [w_measured]          ← 자이로에서 각속도만 추출
예측값:  h(μ) = [μ_w]
Jacobian H : (1×8), H(0,4) = 1.0  ← w 상태만 직접 관측

→ K[0], K[1], K[2] (x, y, yaw gain) 는 강제로 0으로 설정
  (이유: yaw는 prediction에서 w·dt 으로 적분되므로 correction에서
         직접 보정하면 cross-covariance 누적으로 yaw 발산 위험)
→ 실질적으로 v, w 속도 상태만 업데이트
```

#### V-마커 Correction (SLAM 모드만)

```
관측값:  z = [lm_x, lm_y, lm_yaw] in base_link frame  ← GICP 정합 결과
예측값:  h(μ) = R(-θ) × (lm_odom - robot_odom)        ← odom→base_link 변환
Jacobian H : (3×8), 로봇 pose와 랜드마크 pose에 모두 의존

→ 초기화 모드: 로봇 상태 + 랜드마크 상태 동시 업데이트
→ 추적 모드:  랜드마크 컬럼을 0으로 설정 → 로봇 상태만 업데이트
```

### 센서 융합 흐름 요약

```
        Wheel Odom 도착
        prediction(v_state, w_state, dt)
        correctionWheel(v_measured)  → v_state 보정
                  ↓
        IMU 도착
        prediction(v_state, w_state, dt)
        correctionImu(w_measured)    → w_state 보정 (pose gain 강제 0)
                  ↓
        [SLAM 모드일 때] PointCloud 도착
        prediction(v_state, w_state, dt)
        correctionVMarker(vmarker_pose) → 로봇 pose + 랜드마크 보정
```

> 모든 Prediction에서 "현재 상태의 v, w" 를 제어 입력으로 사용한다.  
> 즉, 바퀴와 IMU가 각자 상태의 속도 추정치를 최신화하고, 그 값을 다음 Prediction에서 재활용하는 구조이다.

---

## 5. 도킹 모드 (V-마커 EKF-SLAM) 상세

### 활성화 조건

`task_manager_node/using_vmarker_localization` 토픽이 `true`를 수신하면:

```
using_vmarker_localization_ = true
    → localizer_->enableSlamMode()     ← is_slam_mode_ = true
    → pointCloudCallback 에서 processPointCloud 활성화
```

### V-마커 검출 및 위치 추정 (`VMarker` 클래스)

포인트클라우드에서 V자형 마커를 찾는 작업은 `v_marker_finder` 라이브러리의 `VMarker` 클래스에 위임한다.  
주요 파라미터 (YAML 설정 가능):

| 파라미터 | 의미 |
|---|---|
| `roi_max_distance` | 검출 ROI 최대 거리 [m] |
| `roi_angle_range_deg` | 검출 ROI 각도 범위 [°] |
| `cluster_distance_threshold` | 클러스터링 거리 임계값 [m] |
| `gicp_fitness_threshold` | GICP 수렴 판단 threshold |
| `accumulation_frame_limit` | 누적할 프레임 수 (포인트 밀도 향상) |

### 도킹 모드 3단계 흐름

```
[포인트클라우드 콜백]
        │
        ▼
  [Phase 1: 랜드마크 초기화]
  v_marker_->isInitialized() == false
        │
        ├── v_marker_->findVMarker(cloud)  ← GICP 기반 V-마커 검출
        │         성공 시:
        │    base_link 좌표 → odom 좌표 변환
        │    EKF 상태 벡터의 lm_x, lm_y, lm_yaw 직접 초기화
        │    v_marker_->setInitialized(true)
        │         실패 시: 포인트 누적 후 재시도
        ▼
  [Phase 2: 랜드마크 추적 + EKF 보정]
  v_marker_->isInitialized() == true (아직 covariance 미수렴)
        │
        ├── v_marker_->trackVMarker(cloud, landmark_pose_baselink)
        │         성공 시:
        │    correctionVMarker()
        │      ├── H 행렬 계산 (로봇 pose + 랜드마크 pose 의존)
        │      ├── Kalman gain K = Σ·Hᵀ·(H·Σ·Hᵀ + R)⁻¹
        │      └── 로봇 상태 + 랜드마크 상태 동시 업데이트
        │    랜드마크 공분산 수렴 확인 (최근 10회 안정성 체크)
        │      → x_std < 0.1m, y_std < 0.1m, yaw_std < 5° 달성 시
        │         "Tracking Mode" 전환 완료
        ▼
  [Phase 3: Tracking Mode — 로봇 위치만 보정]
  랜드마크 covariance 수렴 완료
        │
        ├── v_marker_->trackVMarker()
        │         성공 시:
        │    correctionVMarker()
        │      ├── H 행렬에서 랜드마크 컬럼 (5~7) 강제 0으로 설정
        │      │   → 랜드마크는 고정, 로봇 pose 만 업데이트
        │      └── 수렴된 랜드마크를 기준으로 로봇 절대위치 보정
        ▼
  [자동 비활성화]
  로봇이 V-마커 corrected pose로부터 1m 이내 진입 시
        → using_vmarker_localization_ = false
        → localizer_->disableSlamMode()
        → 이후 포인트클라우드 처리 중단, 일반 EKF로 복귀
```

### 도킹 모드에서 발행하는 추가 토픽

| 토픽 | 타입 | 설명 |
|---|---|---|
| `~/vmarker_pose` | `PoseWithCovarianceStamped` | 원래 V-마커 검출 위치 |
| `~/corrected_vmarker_pose` | `PoseStamped` | 오프셋 보정된 도킹 목표 위치 (`vmarker_left_offset` = 0.3405m) |

---

## 6. 주요 토픽

| 방향 | 토픽 | 타입 | 설명 |
|---|---|---|---|
| subscribe | `/motor_driver_node/odom` | `nav_msgs/Odometry` | 바퀴 오도메트리 |
| subscribe | `/imu/data` | `sensor_msgs/Imu` | IMU 각속도 |
| subscribe | `/scan` | `sensor_msgs/PointCloud2` | LiDAR 포인트클라우드 (SLAM 모드만) |
| subscribe | `~/set_pose` | `PoseWithCovarianceStamped` | localization 위치 변경 명령 |
| subscribe | `task_manager_node/is_forward` | `std_msgs/Bool` | 전진/후진 방향 |
| subscribe | `task_manager_node/using_vmarker_localization` | `std_msgs/Bool` | V-마커 SLAM 활성화 |
| publish | `/odom` | `nav_msgs/Odometry` | 추정된 로봇 pose |
| publish | `~/vmarker_pose` | `PoseWithCovarianceStamped` | V-마커 검출 위치 |
| publish | `~/corrected_vmarker_pose` | `PoseStamped` | 오프셋 보정 도킹 목표 |
| TF publish | `map → odom → base_link` | TF2 | 10ms 주기 TF 트리 |

---

## 7. 생명주기 동작

| 단계 | 수행 내용 |
|---|---|
| **`on_configure()`** | 파라미터 로드, TF 초기화, Pub/Sub 생성, DiagnosticManager 생성, `LocalizerFactory`로 `LocalizerEKF` 생성·초기화, 원점 초기 pose 설정, Notification 콜백 등록 |
| **`on_activate()`** | Publisher 활성화, **10ms 주기 TF 발행 타이머 생성** |
| **`on_deactivate()`** | 타이머 해제, Publisher 비활성화 |
| **`on_cleanup()`** | Subscriber·Publisher 전체 리셋, Localizer 해제 |
