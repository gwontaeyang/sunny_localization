# IMU bias 보정 해제 검증

동작 검증은 통과했지만 절대 정확도는 검증되지 않았다. 맵 잔차는 비슷하며,
IMU-rate 위치 출력의 흔들림은 증가했다. 이번 비교에서 CPU 절감도 확인되지 않았다.

## 조건

- 실행 코드: `56f14f2`, `feature/localization_KT`. 평가 도구·테스트는 `14f7b02`에 포함되어 있으며 이번 추가는 검증 요약이다. 실행 binary hash는 replay 전후 동일했다.
- `gangnam0908_20260908_015440` 첫 120초, 각 1회, 동일 원본 LiDAR 277개·IMU 23,824개.
- stage30 `gangnam_floor_person_filtered_nav_classified_rgb.pcd`; 맵 제작 bag은 별도 `gangnam_mcap_20260827_142600`.
- 같은 저연산 설정에서 `imu_bias_estimation_enabled`만 baseline=`true`, current=`false`로 변경했다. IMU는 둘 다 사용한다.
- wheel·기록된 odom·pose graph·TF는 재생하지 않았다. 기록된 첫 map pose는 공통 초기 위치로만 사용했다.

## 결과

| 지표 | bias 보정 ON | 현재 기본값 OFF |
|---|---:|---:|
| 맵 최근접 거리 p50 / p95 | 0.172 / 0.542 m | 0.171 / 0.538 m |
| 맵 0.5 m 이내 점 비율 | 94.37% | 94.43% |
| LIO / 맵 매칭 승인 | 266/266 · 25/25 | 266/266 · 25/25 |
| 평균 CPU, 두 노드 합계·1 core 기준 | 27.59% | 27.89% |
| IMU-rate 위치 증분 길이 합계¹ | 40.66 m | 62.67 m |

¹ 실제 주행 거리가 아니다. 고주기 예측·보정의 왕복 흔들림도 누적된다.

- baseline 대비 XY 차이 p95 **0.245 m**, 최대 **0.806 m**; Z p95 0.015 m, yaw p95 0.301°.
- 공통 scan posterior 시점의 XY 차이는 p95 **0.086 m**, 최대 0.322 m다. 위 0.806 m는 마지막 LiDAR 이후 1.396초 IMU-only tail 끝에서 발생했다.
- 이 tail을 제외해도 연속 XY 차이 최대 0.416 m다. 맵 TF 이전 LIO odom에서도 위치 증분 길이 합계가 39.54→61.22 m로 증가해, TF만의 문제로 보기는 어렵다.
- 양쪽 모두 scan 출력 267개, IMU coverage 부족으로 10개 제외(최초 출력 전 3개·이후 7개). 미확인 scan 누락은 없었다.
- 같은 scan stamp 중 과거 TF가 있는 264개로 잔차 평가: 고정 평가 맵 voxel 0.35 m, TF age ≤0.1 s, 추가 ICP·경로 정렬·보간 없음.
- TF×odom은 정확히 같은 stamp로 합성했다. 현재 `map→odom` 22,432개 전부 대응 odom이 있다. bias norm은 진단 출력 정밀도에서 모두 0이다.
- 빌드 성공, 테스트 147개 통과·1개 건너뜀(stage21 원본 증거 부재). 설치된 평가 명령도 실행 확인했다.

GNSS covariance가 unknown이므로 GT로 쓰지 않았다. 반복 구조에서 작은 맵 잔차만으로
정확한 위치를 보증할 수 없다. 전체 bag·독립 GT·잘못된 초기 위치 복구·장시간 안정성·Jetson 실기 검증은 남아 있다.
기존 IMU 사용/보정 OFF 기본 정책은 그대로 두었으며, 정확도 개선 또는 배포 승인으로 해석하면 안 된다.

## 결과와 재평가

로컬 원본 증거: `~/analysis_outputs/slam_3d_localization/50_trusted_imu_accuracy/`
주요 파일: `evaluation/comparison.png`, `evaluation/metrics.json`, `REPORT.md`.

```bash
source /opt/ros/humble/setup.bash
source /home/user/gole_ws/install/setup.bash
ros2 run slam_3d_localization evaluate_localization_replay.py \
  --capture <current_capture> --baseline-capture <baseline_capture> \
  --map <map.pcd> --output-dir <new_result_directory>
```

SQLite capture에는 `/tf`, `/localization_3d/lio/{odom,scan_odom,deskewed_points}`가 필요하다.
