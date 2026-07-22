# tag_relocalization

`create_tag_map`이 저장한 tag map JSON과 현재 `/tag_array` 관측을 이용해 로봇 pose를 역산하고 localization pose command로 publish합니다.

전체 운용 절차는 상위 문서 `gole_plant/relocalization/README.md`를 기준으로 봅니다.

## 원리

저장된 태그 pose와 현재 관측 pose를 조합합니다.

```text
target_frame -> base_link = target_frame -> tag * inverse(base_link -> tag)
```

동시에 여러 tag가 보이면 각 tag에서 계산한 로봇 pose를 평균해서 publish합니다. `planar_mode: true`에서는 x, y, yaw만 사용합니다.

## 현재 plant 연결

현재 `odom_estimation_node`는 기본적으로 `/odom_estimation_node/set_pose`를 구독합니다. 그래서 이 패키지의 기본 `pose_topic`도 `/odom_estimation_node/set_pose`입니다.

예전 구현의 `/initialpose` 출력이 필요한 경우에는 `pose_topic:=/initialpose`로 바꿔 실행하면 됩니다.

## 실행

```bash
colcon build --packages-select tag_relocalization
source install/setup.bash
ros2 launch tag_relocalization tag_relocalization.launch.py
```

기본 JSON 경로는 `~/.ros/tag_maps/generated_tag_map.json`입니다. 이 파일은 `create_tag_map`으로 먼저 생성되어 있어야 합니다.

확인 토픽:

```bash
ros2 topic echo /odom_estimation_node/set_pose
ros2 topic echo /tag_relocalization_node/estimated_pose
```

## 주요 파라미터

- `tag_map_json`: `create_tag_map`이 만든 JSON 경로
- `tag_topic`: 구독할 `TagInfoArray` 토픽. 기본값 `/tag_array`
- `pose_topic`: 보정 pose를 publish할 토픽. 기본값 `/odom_estimation_node/set_pose`
- `map_frame_id`: 출력 pose의 frame id. 기본값 `odom`
- `base_frame_id`: `/tag_array` 관측 기준 프레임. 기본값 `base_link`
- `relocalize_period_sec`: pose command publish 최소 주기
- `min_map_samples`: tag map 생성 시 최소 관측 샘플 수 필터
- `max_map_position_stddev`: tag map 품질 필터
- `max_detection_distance`: 너무 먼 tag 관측 제외
- `require_base_frame_observation`: `/tag_array.header.frame_id`가 `base_frame_id`와 다르면 관측을 버릴지 여부
