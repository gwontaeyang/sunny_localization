# slam_3d_localization

Prior-map 3D LiDAR localization extracted from the editable `slam_3d_core`
scan-to-map design. This package never inserts keyframes or modifies the loaded
map.

## Data flow

```text
PCD map -> voxel filter -> pose-centered local map
PointCloud2 -> TF to base_link -> range/z/optional robot crop -> voxel filter
  or trusted preprocessed base_link scan -> no second filtering
timestamp-matched wheel/filtered odom delta -> registration initial guess
local scan + local map -> GICP (default) or voxel-surfel -> acceptance gates
  -> /localization_3d/odom
  -> map -> odom TF
  -> aligned scan, local map, diagnostics
```

The node starts in `uninitialized`. Set a 6-DoF pose in `map` using
`/initialpose` before scans can update localization. RViz's standard 2D initial
pose tool supplies x/y/yaw; z, roll, and pitch then remain at their message
values.

## Run

```bash
source /opt/ros/humble/setup.bash
source ~/gole_ws/install/setup.bash

ros2 launch slam_3d_localization slam_3d_localization.launch.py \
  map_path:=/absolute/path/to/prior_map.pcd
```

The generic file above intentionally keeps constant-velocity prediction off.
For the replay-verified indoor XT32 lineage with no declared wheel odometry,
select the dedicated profile instead:

```bash
ros2 launch slam_3d_localization slam_3d_localization.launch.py \
  map_path:=/absolute/path/to/prior_map.pcd \
  params_file:=$(ros2 pkg prefix slam_3d_localization)/share/slam_3d_localization/config/slam_3d_localization_indoor_xt32_no_wheel_replay.yaml
```

`map_path` and `use_sim_time` remain launch-time values. The dedicated replay
profile
contains the exact effective algorithm and interface contract tested in the
Stage23 bag02/bag03 CV-on replay, including the preprocessed frontend cloud,
voxel-surfel registration, and bounded constant-velocity initial guess. Its
reliable queue depth 128 is a replay contract, not yet a live-production QoS
recommendation.

The recorded CV7 mount uses a proper `Rx(180 deg)` sensor-to-base rotation:
vectors map as `(x, y, z) -> (x, -y, -z)`. The upstream SLAM frontend owns
this correction for deskewed clouds; the C++ localizer consumes that corrected
cloud and does not subscribe to IMU directly. The diagnostic open-loop IMU
estimator independently applies the same rotation once to raw bag samples,
including `R * covariance * R^T`. Do not add another axis flip inside the
localizer, because that would double-correct the Stage23 map lineage.

## Shared core, separate profiles

Indoor and outdoor localization use the same localizer, pose state, map lookup,
registration interface, acceptance gates, diagnostics, and AMCL-style no-motion
service. Environment and sensor assumptions live only in parameter profiles:

| Concern | Indoor XT32, no wheel (verified) | Outdoor / wheel-assisted (site-specific) |
|---|---|---|
| Input cloud | trusted deskewed `base_link` frontend output | raw or separately deskewed LiDAR, depending on sensor stack |
| Registration | voxel-surfel, dense planar structure | choose GICP or voxel-surfel from field evidence |
| Local-map/range gates | 15 m local radius, 40 m range | usually wider; validate against scene scale and sparsity |
| Motion initial guess | bounded constant velocity, because wheel odom is absent | synchronized wheel/filtered odom takes precedence |
| Update threshold | every scan (`0 m`, `0 deg`) | tune to motion/noise and compute budget |
| TF publication | disabled in the replay profile | assign one explicit `map -> odom` owner |
| Status | Stage23 bag02/bag03 30 s replay verified | not yet verified; the generic YAML is a safe baseline, not an outdoor claim |

Do not duplicate or fork the C++ core for indoor/outdoor behavior. Add a named
profile and replay evidence when a sensor mount, preprocessing chain, odometry
source, or environment-specific gate changes.

## Resolve the latest bag02/bag03 map pair

Use the resolver before a replay so a partially written newer SLAM stage can
never cause an unnoticed fallback to an older map:

```bash
ros2 run slam_3d_localization resolve_latest_slam_stage.py \
  --check-only

ros2 run slam_3d_localization resolve_latest_slam_stage.py \
  --output ~/analysis_outputs/slam_3d_localization/inputs.json
```

By default it selects the newest numeric stage that advertises the canonical
`pcd_full_maps_only/` contract, records later non-canonical stages in
`selection.skipped_newer_non_map_stages`, then requires exactly one
`bag02_*.pcd` and one `bag03_*.pcd`. It never falls back past a newer stage that
has started advertising the canonical directory but is incomplete or invalid.
`--stage 22`, a stage name, or an absolute stage path pins a particular stage.
`--check-only` and `--output` are mutually exclusive; both successful modes
print the resolved JSON to stdout.

Stage 21 is supported through a strict legacy adapter. It cross-checks each
`full_results/<bag>/replay.log`, metrics JSON, passed validation report, and
`manifest.sha256`; verifies the selected convenience PCD is byte-identical to
the validated full-result PCD; checks PCD headers and trajectories; and
read-only inspects the source bag topics. The known stage-21 lineage is
`/imu/data` in `cv7_link`, with no wheel odometry.

Every stage numbered 22 or later must be published atomically with both
`localization_inputs.json` and `LOCALIZATION_INPUTS_READY`. The JSON contract is:

```json
{
  "schema_version": 1,
  "stage_sequence": 22,
  "stage_name": "22_example",
  "datasets": {
    "bag02": {
      "map": {
        "path": "pcd_full_maps_only/bag02_full_map.pcd",
        "sha256": "<file sha256>",
        "frame": "map"
      },
      "bag": {
        "path": "/absolute/source/bag02",
        "storage_set_sha256": "<sha256sum-lines-v1 digest>"
      },
      "trajectory": {
        "path": "localization_inputs/bag02_trajectory_post_optimization.csv",
        "sha256": "<file sha256>",
        "frame": "map"
      },
      "point_cloud": {"topic": "/points", "frame": "lidar_link"},
      "imu": {"topic": "/imu/data", "frame": "imu_link"},
      "wheel_odometry": {"topic": null}
    },
    "bag03": {
      "map": {
        "path": "pcd_full_maps_only/bag03_full_map.pcd",
        "sha256": "<file sha256>",
        "frame": "map"
      },
      "bag": {
        "path": "/absolute/source/bag03.db3",
        "storage_set_sha256": "<sha256sum-lines-v1 digest>"
      },
      "trajectory": {
        "path": "localization_inputs/bag03_trajectory_post_optimization.csv",
        "sha256": "<file sha256>",
        "frame": "map"
      },
      "point_cloud": {"topic": "/points", "frame": "lidar_link"},
      "imu": {"topic": "/imu/data", "frame": "imu_link"},
      "wheel_odometry": {"topic": null}
    }
  }
}
```

For a bag directory, storage files follow `metadata.yaml`'s
`relative_file_paths` order; a metadata-less directory uses natural filename
order. The storage-set digest is SHA-256 over those ordered
`<storage-file-sha256><two spaces><relative-db3-name>\n` lines. Unlisted `.db3`
files make a metadata-backed bag invalid. The ready marker must contain the
exact contract SHA-256, either alone or in `sha256sum` form. An empty marker is
not ready.

Stage23 predates that producer contract and stored maps under
`full_pcd_maps_only/`. The Sequence20 experiment therefore used a separately
frozen and hash-verified Stage23 contract without changing Stage23. Automatic
resolution still reports Stage23 as skipped and selects Stage21. Future SLAM
stages must publish the canonical directory, contract, and ready marker in the
same stage before localization replay starts; renaming a directory alone is not
sufficient provenance.

The stage producer can publish that contract with the companion tool. Its JSON
spec contains exactly `bag02` and `bag03`; each dataset supplies absolute
`map_source`, `trajectory_source`, `bag.path`, explicit point-cloud and IMU
topic/frame pairs, and a nullable `wheel_odometry.topic`:

Start from the installed `config/localization_publish_spec.example.json` and
replace every placeholder with the current stage's verified inputs.

```bash
ros2 run slam_3d_localization publish_localization_inputs.py \
  --stage ~/analysis_outputs/slam_3d/24_next_stage \
  --spec /absolute/path/to/localization_publish_spec.json \
  --check-only

ros2 run slam_3d_localization publish_localization_inputs.py \
  --stage ~/analysis_outputs/slam_3d/24_next_stage \
  --spec /absolute/path/to/localization_publish_spec.json
```

`--check-only` returns exit code 0 when source validation succeeds even if the
target already contains reserved outputs. Automation must require
`publishable: true`; when it is `false`, inspect `target_conflicts` and do not
run the publishing command.

The tool validates PCD headers, trajectory rows/frames, source bag storage
hashes, declared topic types, and the SHA-256 of each copied map and trajectory.
It refuses to overwrite any existing canonical output. It installs copied
trajectories, the contract, and the ready marker first, then atomically renames
`pcd_full_maps_only/` into place last. That final rename is the resolver's
readiness boundary. If any copy check, rename, or final resolver verification
fails, all reserved paths installed by that attempt are rolled back.

The resolver recomputes map, trajectory, bag-storage, and contract hashes and
emits the declared topics and frames, hashes, PCD dimensions, and selected
wheel-odometry contract. A `nav_msgs/Odometry` topic is never inferred to be
wheel odometry: only a non-null `wheel_odometry.topic` is selected, while other
odometry topics are reported separately as `undeclared_odometry_topics`.
Trajectory rows are checked for finite numeric poses and non-zero quaternions.
Timestamp inversions and duplicates are reported with the required consumer
policy `stable_sort_by_stamp_ns_then_source_row` rather than silently hidden.

Override `cloud_topic`, `odom_topic`, frames, and registration thresholds in a
site-specific parameter file. Set `cloud_subscription_reliable=true` when the
upstream canonical cloud publisher is reliable; keep the default best-effort
mode for direct sensor publishers that offer only sensor-data QoS. The input
cloud can use a LiDAR frame; a timestamped `base_link <- lidar` TF must then be
available.

Set `registration_method=voxel_surfel` to reuse the exported
`slam_3d_core::VoxelSurfelRegistration` backend. Its surfel quality,
observability, correspondence, and runtime values are included in diagnostics.
The default remains `gicp` for backward compatibility. Likewise,
`crop_box_enabled=false`, `input_scan_preprocessed=false`, and
`map_already_voxelized=false` preserve the original pipeline.

For frontend-parity experiments, `slam_3d_core` can publish its exact
deskew/base-frame/range/robot-crop/adaptive-voxel result by enabling
`publish_preprocessed_scan`. Point the localizer at that topic and set
`input_scan_preprocessed=true`; the node then requires an exact `base_link`
frame and does not crop or voxelize the scan a second time. Set
`map_already_voxelized=true` only for a final PCD already written at the desired
map resolution.

Odometry is buffered and looked up at each cloud's header stamp. Bracketed
samples are interpolated only when their span is at most
`odom_max_interpolation_gap_s`; a single nearby sample is accepted only within
`odom_max_nearest_age_s` and only when it is at or before the cloud stamp.
Future-only samples are never used as a pose hold. A timestamp rewind larger
than `odom_timestamp_reset_threshold_s` clears temporal anchors. When no valid
lookup exists, scan-to-map matching continues without an odometry prediction,
`map -> odom` is not published for that cloud, and diagnostics report
`odom_sync_valid=false`.

For wheel-less profiles, `use_constant_velocity_prediction=true` opts into the
same one-accepted-step pose extrapolation used by the indoor mapper frontend.
The generic package default is `false`, while
`slam_3d_localization_indoor_xt32_no_wheel_replay.yaml` enables the
replay-verified setting for the tested 10 Hz XT32 pipeline. The extrapolated
relative motion is bounded by
`constant_velocity_translation_limit_m=0.30` and
`constant_velocity_rotation_limit_deg=10.0`. Only accepted registrations
advance the two-pose history; a rejection keeps the last accepted history for
a bounded retry. Initial-pose resets, odometry timestamp rewinds, and accepted
or prediction gaps beyond `constant_velocity_max_gap_s=0.50` rebase the
history. A cloud timestamp rewind larger than
`cloud_timestamp_reset_threshold_s` resets temporal anchors so a repeated bag
does not remain locked out by the previous epoch. Wheel-less forced no-motion
updates deliberately use pose hold and rebase accepted history, preventing a
stationary scan-match correction from becoming the next velocity estimate.
Diagnostics report prediction source, readiness, status, raw/applied deltas, clamps, and
timing gaps.

### Stage23 bag02/bag03 adoption evidence

Sequence20 ran four fresh, source-window-identical 30 s replays: bag02 and
bag03 with CV off/on. The only parsed A/B parameter change was
`use_constant_velocity_prediction: false -> true`; source LiDAR and IMU header
counts, first/last stamps, and ordered stamp digests matched for both arms.

| Dataset | CV | Tracking | XY RMSE vs same-bag SLAM | 3D RMSE | Orientation RMSE | Verdict |
|---|---:|---:|---:|---:|---:|---|
| bag02 | off | 38.33% | 1.2718 m | 1.3306 m | 53.762° | fail |
| bag02 | on | 100% | 0.0340 m | 0.0523 m | 0.701° | pass |
| bag03 | off | 100% | 0.01645 m | 0.08411 m | 0.379° | pass |
| bag03 | on | 100% | 0.01604 m | 0.08300 m | 0.370° | pass / non-regression |

bag02 CV-off produced 185 `inlier_rmse` rejections after the first threshold
crossing at about 6.7 s; CV-on produced none. CV-on was actually applied on
298/300 diagnostics in each bag and was never clamped. This supports adoption
for this wheel-less indoor profile, but it is not independent ground truth or
cross-site validation: the reference is the same-bag post-optimization SLAM
trajectory and each replay covers only the first 30 s. IMU integration is an
additional first-10-s relative diagnostic only.

Evidence root:
`~/analysis_outputs/slam_3d_localization/20_gangnam_stage23_bag02_bag03_cv_ab_30s/`.

## TF ownership

When odometry is available and `publish_tf=true`, this node publishes
`map -> odom`:

```text
map_T_odom = map_T_base(localized) * inverse(odom_T_base)
```

The odometry source remains responsible for `odom -> base_link`. Do not enable
another `map -> odom` publisher at the same time.

## Failure behavior

Registration is accepted only when the selected backend accepts the result and its fitness,
prediction-to-result translation, and rotation all pass configured gates.
Rejected scans do not alter the accepted pose or odometry anchor. Diagnostics
report `uninitialized`, `idle`, `tracking`, `degraded`, or `lost`, together
with odometry synchronization mode, age, interpolation gap, and buffer size.

## AMCL-style no-motion update

When filtered odometry is available, normal scan matching is skipped until it
exceeds either `update_min_translation_m` or `update_min_rotation_deg`.
Initialization always forces the first valid cloud update. To force another
measurement update while the robot is stationary:

```bash
ros2 service call /slam_3d_localizer/request_nomotion_update std_srvs/srv/Empty
```

Like Nav2 AMCL, the service marks the next valid sensor message for processing;
it does not reuse an old cloud. A forced GICP result still has to pass every
normal fitness and correction gate. Diagnostics distinguish
`initial_pose_update_accepted` from a real service-triggered
`nomotion_update_accepted`.

## Wheel-less IMU diagnostic trajectory

`estimate_imu_open_loop.py` streams `/imu/data` directly from a ROS 2 sqlite
bag and writes a short-horizon relative trajectory for localization comparison:

```bash
ros2 run slam_3d_localization estimate_imu_open_loop.py \
  --bag-db /absolute/path/to/recording.db3 \
  --output-dir ~/analysis_outputs/slam_3d_localization/NN_label/imu_reference
```

The tool is specific to the replay-verified CV7 convention used by the indoor
profiles: it applies `Rx(180 deg)`, mapping sensor vectors `(x, y, z)` to
`(x, -y, -z)`. It rejects a non-`cv7_link` input frame by default. Do not reuse
that correction for Xsens or another mount without a separate calibration.

An initial stationary window estimates gyro bias, gravity direction, and the
observable accelerometer bias component. Duplicate and nonmonotonic header
stamps are counted and dropped; a later integration gap invalidates the
continuous reference instead of being silently bridged. The CSV contains
position, velocity, quaternion, bias, a diagonal covariance proxy,
stationary/ZUPT candidate labels, and an explicit diagnostic horizon. The full
integrated output is retained beyond the horizon for diagnosis. CSV and JSON
files are individually published with atomic replacement; the metrics record
each bag storage file's SHA-256, storage order, metadata hash when present, and
trajectory-anchor SHA-256.

Use `--trajectory-anchor` with a JSON, CSV, or TUM first pose to express the
relative trajectory in the localization/map frame. The anchor only fixes the
initial pose; it does not add position or velocity observations afterward.

This output is not absolute ground truth. Accelerometer position is an
open-loop double integral and can diverge within seconds. Its covariance is a
non-conservative diagnostic proxy: it omits full state cross-covariance and
time-correlated bias uncertainty, so neither covariance, `position_3sigma_m`,
nor the diagnostic-horizon labels may be used as a localization quality or
acceptance gate. `within_valid_horizon` and `validity_reason` remain only as
backward-compatible CSV aliases. ZUPT is deliberately candidate-only: the
script never forces velocity to zero. A future hard ZUPT must require
independent scan/SLAM evidence rather than an IMU-only label.

## Current boundary

This MVP is local tracking, not global localization. It requires an initial pose
inside the GICP convergence basin. Automatic kidnapped-robot recovery should be
implemented as a separate coarse candidate generator (for example Scan Context
or a global descriptor) followed by this node's geometric refinement and gates.
The reported covariance is currently a conservative fitness-derived placeholder,
not a Hessian-derived uncertainty estimate.
