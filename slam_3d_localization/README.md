# slam_3d_localization

LiDAR + IMU로 로봇의 움직임을 추정하고, 저장된 PCD 맵과 정합해 3D 위치를 구하는 ROS 2 패키지입니다.

## 기본 동작

- **IMU 사용, bias 추정·보정 OFF**: 입력 각속도·가속도를 사용하며 gyro/accel bias는 0으로 고정합니다.
- IMU 예측·중력 처리·deskew와 LiDAR 위치 보정은 유지합니다. wheel 보정은 기본 OFF입니다.
- frontend는 `voxel_surfel`, 고정 맵 정합은 CPU `gicp`입니다. 기본 설정은 연산량 우선입니다.

## 빌드

ROS 2 Humble과 Eigen/PCL, `slam_3d_core` 및 그 의존성이 준비된 workspace에서 실행합니다.

```bash
source /opt/ros/humble/setup.bash
cd ~/gole_ws
colcon build --packages-up-to slam_3d_localization --cmake-args \
  -DSLAM_3D_LOCALIZATION_FAST_GICP=OFF \
  -DSLAM_3D_LOCALIZATION_ENABLE_FAST_VGICP_CUDA=OFF
source install/setup.bash
```

## 실행

센서 드라이버는 별도로 실행해야 합니다.

```bash
ros2 launch slam_3d_localization slam_3d_lio_localization.launch.py \
  map_path:=/absolute/path/to/map.pcd
```

기본 입력은 `/hesai/center/lidar_points`, `/imu/data`입니다.
다른 센서는 [설정 파일](config/slam_3d_lio_gangnam.yaml)을 복사해 토픽·frame·IMU 장착 변환·point timestamp를 맞추고,
`params_file:=/absolute/path/to/params.yaml`로 지정하십시오.

시작 시 최소 0.5 s / IMU 100개의 정지 구간이 필요합니다.
RViz의 **2D Pose Estimate** 또는 `/initialpose`로 맵 기준 초기 위치를 지정하십시오.
높이·기울기가 있으면 z/roll/pitch도 포함한 초기 pose가 필요합니다.

Bag 테스트는 launch에 `use_sim_time:=true`를 추가하고, 별도 터미널에서 센서만 재생합니다.

```bash
ros2 bag play /absolute/path/to/bag --clock \
  --topics /hesai/center/lidar_points /imu/data /tf_static
```

기록된 odom과 dynamic `/tf`는 재생하지 마십시오. 토픽과 QoS는 실제 bag에 맞춰야 합니다.

정지 중에도 다음 scan으로 정합하려면:

```bash
ros2 service call /slam_3d_localizer/request_nomotion_update std_srvs/srv/Empty '{}'
```

## 핵심 구조

```text
LiDAR + IMU → LIO → odom→base_link
                └─ deskewed scan + 동시각 pose → 고정 맵 정합 → map→odom
```

IMU 적분으로 scan 사이의 움직임을 예측하고, 각 점을 같은 시각으로 옮겨 deskew합니다.
scan을 임시 submap에 정합해 LIO pose를 보정한 뒤, 고정 PCD 맵 정합으로 맵 기준 위치를 구합니다.
**IMU bias를 학습하지 않는 것이지, LiDAR 위치 보정을 끄는 것은 아닙니다.**

`map_T_odom = map_T_base(localized) × inverse(odom_T_base)`

같은 scan 시각의 pose로 위 보정값을 계산하고, 이후 odom 입력마다 해당 stamp로 `map → odom` TF를 발행합니다.
`odom → base_link`는 LIO, `map → odom`은 localizer만 발행해야 합니다.

주요 출력은 `/localization_3d/odom`(맵 pose), `/localization_3d/aligned_points`(정합 scan),
`/localization_3d/diagnostics`(상태·정합 시간)입니다.

자동 전역 재탐색과 pose-graph anchor 적용은 지원하지 않습니다. 초기 위치와 실제 주행 정확도는 별도 확인이 필요합니다.

코드: [LIO](src/lio_odometry.cpp) · [IMU/ESKF](src/lio_state_estimator.cpp) · [맵 정합](src/slam_3d_localizer.cpp)
