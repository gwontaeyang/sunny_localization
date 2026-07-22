# create_tag_map

`/tag_array`(`gole_interfaces/msg/TagInfoArray`) 관측을 태그 ID별로 누적하고 평균 pose를 저장하는 ROS 2 Python 패키지입니다.

## 역할

- relocalization용 JSON을 `~/.ros/tag_maps/generated_tag_map.json`에 저장합니다.
- 기존 `tag_map_db`가 읽을 수 있는 2D YAML을 `~/.ros/db/generated_tag_map.yaml`에 같이 저장할 수 있습니다.
- JSON에는 태그별 평균 position/orientation, 샘플 수, position 표준편차, raw sample을 저장합니다.
- 전체 운용 절차는 상위 문서 `gole_plant/relocalization/README.md`를 기준으로 봅니다.

## 프레임

현재 odom estimation은 `odom -> base_link` TF를 publish하고, `/odom_estimation_node/set_pose`의 x/y/yaw를 EKF state seed로 사용합니다. 그래서 기본 `target_frame`은 `odom`입니다.

Nav2/AMCL 구성처럼 `map -> base_link` TF가 안정적으로 존재하는 경우에는 `target_frame:=map`으로 바꿔서 map 기준 tag map을 만들 수 있습니다.

relocalization용 절대 tag map을 만들 때는 `use_tf_target_frame: true`가 맞습니다. `/tag_array`가 `base_link` 기준인 상태로 그대로 저장하면 로봇 위치가 바뀔 때마다 tag pose도 같이 바뀌어서 relocalization 기준이 되기 어렵습니다.

## 실행

```bash
colcon build --packages-select create_tag_map
source install/setup.bash
ros2 launch create_tag_map create_tag_map.launch.py
```

종료하거나 `capture_duration_sec`에 도달하면 JSON/YAML을 저장합니다.

## 기존 운용 맵 좌표만 대체

기존 `tag_map_db` 운용 YAML의 `edges`는 유지하고, 새로 측정한 맵의 `nodes[].x/y`만 덮어쓴 YAML을 만들 수 있습니다.

```bash
ros2 run create_tag_map merge_tag_map_nodes \
  --base ~/.ros/db/tag_graph_d.yaml \
  --measured ~/.ros/db/generated_tag_map.yaml \
  --output ~/.ros/db/tag_graph_d_measured.yaml
```

먼저 결과만 보고 싶으면 `--dry-run`을 붙입니다. 새로 측정했지만 기존 운용 맵에 없는 tag ID까지 node로 추가하려면 `--add-missing`을 붙일 수 있습니다. 이 경우 edge는 자동 생성하지 않습니다.

생성된 `generated_tag_map.yaml`은 기본적으로 `edges: []`입니다. 기존 운용 graph를 대체할 때는 이 파일을 그대로 쓰지 말고 `merge_tag_map_nodes`로 기존 운용 YAML의 `edges`를 보존한 결과 파일을 사용합니다.

## 주요 파라미터

- `tag_topic`: 구독할 태그 관측 토픽. 기본값 `/tag_array`
- `output_json`: relocalization용 full pose JSON 저장 경로
- `output_yaml`: `tag_map_db` 호환용 2D YAML 저장 경로
- `use_tf_target_frame`: 관측 pose를 `target_frame`으로 변환해 저장할지 여부
- `target_frame`: 저장 기준 프레임. 기본값 `odom`
- `min_samples_per_tag`: 이 개수 이상 관측된 태그만 평균 결과에 포함
- `autosave_period_sec`: 주기 저장 간격
