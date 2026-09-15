# slam_3d_localization

LiDAR + IMU로 움직임을 추정하고, 저장된 PCD 맵과 정합하는 ROS 2 3D localization입니다.

## 빌드·실행

ROS 2 Humble과 Eigen/PCL이 필요합니다. Ceres와 `slam_3d_core`는 필요하지 않습니다.

```bash
source /opt/ros/humble/setup.bash
cd ~/gole_ws
colcon build --packages-up-to slam_3d_localization
source install/setup.bash
```

센서 드라이버는 별도로 실행하고, 사용할 맵을 지정합니다.

```bash
ros2 launch slam_3d_localization slam_3d_lio_localization.launch.py \
  map_path:=/absolute/path/to/map.pcd
```

기본 입력은 `/hesai/center/lidar_points`, `/imu/data`입니다. 추가 옵션은 필요한 경우만 지정합니다.
실행 YAML에는 IMU 장착 위치만 두고, timestamp는 기본 `auto`(epoch 초·상대 초 자동 판별)를 사용합니다.

- 센서 변경: [센서 설정](config/slam_3d_lio_gangnam.yaml)을 복사해 토픽·frame·IMU 장착 변환·점 timestamp를 맞춘 뒤 `params_file:=/absolute/path/to/params.yaml`.
- Bag 재생: 위 launch 명령에 `use_sim_time:=true` 추가.

시작 시 최소 0.5 s·IMU 100개 동안 정지한 뒤, RViz의 **2D Pose Estimate** 또는 `/initialpose`로 초기 위치를 지정합니다.
높이·기울기가 있으면 초기 pose에 z/roll/pitch도 포함해야 합니다. 자동 전역 재탐색은 지원하지 않습니다.

## 핵심 구조

```text
LiDAR + IMU → LIO → odom→base_link
                └─ deskewed scan + 동시각 pose → 고정 맵 정합 → map→odom
```

- IMU 예측·중력 처리·deskew는 유지하고, IMU bias 추정과 wheel 보정은 기본 OFF입니다.
- LIO는 자체 `voxel_surfel` 6×6 Gauss–Newton, 고정 맵 정합은 CPU PCL `gicp`를 사용합니다. 두 노드는 공통 backend의 `align()`을 호출합니다.
- 저연산 기본값: LiDAR 최대 **0.5 Hz·1,024점**, odom/TF 최대 **2 Hz**. IMU 큐의 측정값은 원래 timestamp로 적분합니다.

`map_T_odom = map_T_base(localized) × inverse(odom_T_base)`

같은 scan 시각의 pose로 보정값을 계산하고, odom 입력마다 같은 stamp로 `map → odom`을 발행합니다.
`odom → base_link`는 LIO만, `map → odom`은 localizer만 발행해야 합니다.
중간 scan은 생략하며 IMU 큐 초과 시 손실될 수 있습니다. 정확도·응답성을 희생한 버전으로, 실제 주행은 별도 검증이 필요합니다.

## Bag 테스트

launch에 `use_sim_time:=true`를 추가한 뒤, 다른 터미널에서 센서만 재생합니다.

```bash
ros2 bag play /absolute/path/to/bag --clock \
  --topics /hesai/center/lidar_points /imu/data /tf_static
```

토픽·QoS는 실제 bag에 맞추고, 기록된 odom과 dynamic `/tf`는 재생하지 마십시오.
