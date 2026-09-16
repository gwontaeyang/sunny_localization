# 매칭 의존성 설정

기본 ICP/GICP/NDT/voxel_surfel은 ROS 2 Humble·PCL·Eigen만 사용합니다.
Fast 계열은 아래 스크립트로 별도 설치합니다. `sudo`나 시스템 패키지 설치는 실행하지 않습니다.
Git, CMake, colcon, ROS 2 Humble 개발 환경이 필요하며 GPU 빌드에는 CUDA Toolkit과 호환 드라이버도 필요합니다.

```bash
GICP_PACKAGE=~/gole_ws/src/gole_autonomy/gole_localization/slam_3d_localization
GICP_OUTPUT=~/analysis_outputs/slam_3d_localization/58_registration_dependencies
GICP_ARCH=90-virtual  # 현재 RTX 5070 + CUDA 12.6; Jetson AGX Orin에서는 87
bash "$GICP_PACKAGE/scripts/setup_registration_dependencies.sh" \
  --workspace ~/gole_ws --output-dir "$GICP_OUTPUT" \
  --cuda-root /usr/local/cuda-12.6 --cuda-architectures "$GICP_ARCH"

source /opt/ros/humble/setup.bash
source ~/gole_ws/deps/fast_gicp/install/setup.bash
cd ~/gole_ws
colcon --log-base "$GICP_OUTPUT/localization_log" build \
  --packages-select slam_3d_localization \
  --cmake-args -DSLAM_3D_LOCALIZATION_FAST_GICP=ON \
  -Dfast_gicp_DIR="$HOME/gole_ws/deps/fast_gicp/install/fast_gicp/share/fast_gicp/cmake" \
  -DSLAM_3D_LOCALIZATION_ENABLE_FAST_VGICP_CUDA=ON \
  -DCUDAToolkit_ROOT=/usr/local/cuda-12.6 \
  -DCMAKE_CUDA_ARCHITECTURES="$GICP_ARCH" \
  -DSLAM_3D_LOCALIZATION_FAST_GICP_CUDA_ARCHITECTURES="$GICP_ARCH" \
  -DSLAM_3D_LOCALIZATION_FAST_GICP_REVISION=0e7ec1441c99f7be453db2ea216d5de029387417+patch8c78065c
source install/setup.bash
```

이후 실행은 `source ~/gole_ws/install/setup.bash`만으로 의존 환경까지 읽습니다.
`registration_method:=fast_vgicp_cuda`는 GPU, `fast_gicp` / `fast_vgicp`는 CPU를 선택합니다.

- CPU Fast만 사용: 스크립트에서 CUDA 옵션 대신 `--cpu-only`, localization 빌드에서는 CUDA enable을 `OFF`로 지정합니다.
- Fast/CUDA 없이 사용: localization 빌드에서 `-DSLAM_3D_LOCALIZATION_FAST_GICP=OFF -DSLAM_3D_LOCALIZATION_ENABLE_FAST_VGICP_CUDA=OFF`로 지정합니다.
- 소스·install은 `~/gole_ws/deps/fast_gicp/`, 빌드·로그는 `--output-dir`에 저장합니다. 다음 작업에서는 출력 번호를 늘려주세요.
- 기존 dependency checkout 변경은 덮어쓰지 않습니다. 고정 commit + 동봉 patch와 다르면 중단합니다.
- Jetson은 aarch64에서 `87`로 빌드해야 합니다. x86_64 라이브러리를 복사해서 쓸 수 없습니다.

고정 버전: [fast_gicp `0e7ec144`](https://github.com/koide3/fast_gicp/tree/0e7ec1441c99f7be453db2ea216d5de029387417), [동봉 CUDA 12 patch](../patches/fast_gicp_0e7ec144_cuda12.patch).
CUDA 12.6은 sm120을 생성하지 못하므로 현재 PC에서는 compute90 PTX의 드라이버 JIT를 사용합니다. [NVIDIA 호환 가이드](https://docs.nvidia.com/cuda/blackwell-compatibility-guide/)
