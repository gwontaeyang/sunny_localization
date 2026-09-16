# slam_3d_localization

LiDAR + IMU → LIO(`odom → base_link`) → PCD 맵 정합(`map → odom`). ROS 2 Humble·Eigen·PCL을 사용하며 Ceres는 필요 없습니다.

```bash
source /opt/ros/humble/setup.bash
cd ~/gole_ws
colcon build --packages-select slam_3d_localization
source install/setup.bash
ros2 launch slam_3d_localization slam_3d_lio_localization.launch.py \
  map_path:=/absolute/path/to/map.pcd
```

맵 매칭은 실행 명령에 `registration_method:=ndt`, LIO 매칭은 `lio_registration_method:=icp`처럼 추가합니다. 생략하면 config/기본값을 유지합니다(맵 `gicp`, LIO `voxel_surfel`). 실행 중 변경은 지원하지 않습니다.

- 기본 제공: `icp`, `gicp`, `ndt`(PCL), `voxel_surfel`(자체 6×6 Gauss–Newton).
- 선택 의존성: `fast_gicp`, `fast_vgicp`(CPU), `fast_vgicp_cuda`(GPU). 미빌드 방식은 오류로 종료합니다.

Fast/CUDA 의존성은 [설정 명령](docs/registration_dependencies.md)으로 설치·빌드합니다. 고정 버전과 CUDA 호환 패치를 사용하며, 시스템 패키지는 자동 설치하지 않습니다.

기본 YAML은 [LIO](config/slam_3d_lio_gangnam.yaml) **1개**(IMU 장착 위치), [맵 localization](config/slam_3d_localization.yaml) **1개**(맵 경로)입니다. 상세 설정은 `params_file:=/path/config.yaml`로 덮어씁니다. timestamp는 `auto`입니다.

입력: `/hesai/center/lidar_points`, `/imu/data`. 시작 시 최소 0.5 s·IMU 100개 동안 정지하고 `/initialpose`로 초기 위치를 지정합니다(경사에서는 z/roll/pitch 포함). 자동 전역 재탐색은 없습니다.

Bag은 `use_sim_time:=true`로 실행하고 `ros2 bag play /path/bag --clock --topics /hesai/center/lidar_points /imu/data /tf_static`로 재생합니다. 기록된 odom·dynamic `/tf`는 제외합니다.

저연산 기본값: LiDAR 최대 0.5 Hz·1,024점, odom/TF 최대 2 Hz. IMU는 사용하지만 bias 추정·wheel 보정은 OFF입니다. 정확도·응답성을 희생한 설정이므로 실제 주행 검증이 필요합니다.
