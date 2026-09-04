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
  -> cache map -> odom correction
  -> aligned scan, local map, diagnostics
strictly newer valid odometry -> cached map -> odom TF at its stamp
```

The node starts in `uninitialized`. Set a 6-DoF pose in `map` using
`/initialpose` before scans can update localization. RViz's standard 2D initial
pose tool supplies x/y/yaw; z, roll, and pitch then remain at their message
values.

`map_frame`, `odom_frame`, and `base_frame` must be nonempty and pairwise
distinct. Odometry messages are accepted only when they explicitly declare the
exact configured `odom_frame -> base_frame`; empty frame IDs are rejected.

## Run

```bash
source /opt/ros/humble/setup.bash
source ~/gole_ws/install/setup.bash

ros2 launch slam_3d_localization slam_3d_localization.launch.py \
  map_path:=/absolute/path/to/prior_map.pcd
```

## Registration backends

`registration_method` selects `gicp` (the unchanged PCL CPU default),
`fast_gicp` (OpenMP CPU), `fast_vgicp_cuda` (CUDA), or the existing
`voxel_surfel` path. FastGICP support is optional: a build without that
dependency still provides the two existing backends. Selecting a backend that
was not compiled is a startup error; it never falls back silently.

The validated dependency revision is fast_gicp
`0e7ec1441c99f7be453db2ea216d5de029387417`. Apply
`patches/fast_gicp_0e7ec144_cuda12.patch` to that revision before a CUDA 12.6
build. The patch also makes legacy FindCUDA honor a configurable
`CMAKE_CUDA_ARCHITECTURES`; use `87` when building the dependency natively on
Jetson AGX Orin. Build the CPU backend with
`-DSLAM_3D_LOCALIZATION_FAST_GICP=ON`. Add
`-DSLAM_3D_LOCALIZATION_ENABLE_FAST_VGICP_CUDA=ON` only after the patched CUDA
dependency is in the sourced overlay. Keep both options off for a strict
PCL-only build.

The backend object persists across scans. Set
`local_map_rebuild_distance_m > 0` to reuse the immutable local-map pointer and
its CPU/GPU covariance state until that target is rebuilt; zero retains the
legacy per-scan crop. Diagnostics expose the active backend, build revision,
CUDA architecture metadata, native convergence/iteration result, constructor,
target-update, align and total times, and target-cache counters.

FastVGICP is a voxelized objective, not a GPU implementation mathematically
identical to PCL GICP. Its voxel and neighbor-search parameters must therefore
be reported with comparisons. In the pinned upstream implementation,
`max_correspondence_m` does not gate CUDA optimization and CUDA covariance k is
fixed at 20; the common post-registration overlap diagnostics remain available
for like-for-like residual comparison. `gicp` remains the shipped default.

The current Gangnam policy uses the separate wheel-free LIO launch. It starts
only raw LiDAR+IMU odometry and prior-map localization; it does not start the
legacy wheel/filtered-odometry nodes:

```bash
ros2 launch slam_3d_localization slam_3d_lio_localization.launch.py \
  map_path:=/absolute/path/to/prior_map.pcd \
  use_sim_time:=true
```

The installed default map is the Stage 30 classified Gangnam PCD. The profile
consumes `/hesai/center/lidar_points` and `/imu/data`, follows the Stage 29/30
identity CV7 rotation plus 1.20984 m vertical lever arm, and publishes
`/localization_3d/lio/odom` plus `odom -> base_link` at IMU cadence. A separate
no-TF `/localization_3d/lio/scan_odom` carries the final LIO posterior at each
scan timestamp. The localizer waits for that exact pose before consuming the
deskewed cloud, keeps constant-velocity prediction off, and owns
`map -> odom`. Keep the robot stationary for at least the configured 0.5 s IMU
initialization window.

Wheel velocity remains an opt-in LIO correction through
`wheel_correction_enabled`; it is `false` in this profile, so no wheel
subscription is created. Pose-graph anchors are deliberately not loaded or
applied yet. They should be added later as another disabled-by-default input,
without changing this LIO-only default path. That future input must declare a
versioned file format, map frame, timestamp, anchor pose, and validation
status before a runtime parameter or subscriber is enabled.

For a bounded bag replay, publish only the recorded sensor and static-TF topics
while the launch above runs with simulated time:

```bash
ros2 bag play /absolute/path/to/bag --clock \
  --topics /imu/data /hesai/center/lidar_points /tf_static
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

The legacy Stage23 replay profile used an `Rx(180 deg)` IMU experiment. Its
upstream SLAM frontend owns that correction, so the map localizer must not
apply it again. The current Stage29/30 Gangnam LIO profile instead follows its
verified identity-rotation source contract; keep these map lineages separate.

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

`max_cloud_age_s` rejects queued scans whose age strictly exceeds the configured
limit. The source default is `0.0` (disabled); a bounded value belongs in a
site/replay profile. Once ROS time is greater than zero, freshness is checked
before scan preparation and again immediately after registration, before any
normal cloud timestamp, localization pose, predictor update, or TF correction
is committed. A scan exactly at the limit or stamped in the future is not stale.
When the guard is enabled, scans wait in a degraded state until ROS time has
started instead of bypassing the check. A detected cloud timestamp epoch rewind
is the safety exception: it clears the old temporal state and TF correction
before freshness evaluation, scan preparation, and every early return, even if
that scan is already stale.
Diagnostics distinguish `stale_cloud_before_processing` from
`stale_cloud_after_registration` and report the current age, configured limit,
drop stage, per-stage counts, and total count.

Odometry is buffered and looked up at each cloud's header stamp. Bracketed
samples are interpolated only when their span is at most
`odom_max_interpolation_gap_s`; a single nearby sample is accepted only within
`odom_max_nearest_age_s` and only when it is at or before the cloud stamp.
Future-only samples are never used as a pose hold. A timestamp rewind larger
than `odom_timestamp_reset_threshold_s` clears temporal anchors. Clearing
interpolation samples for an initial pose or cloud reset retains an independent
highest-observed odometry stamp, so a later rewind is still classified as a new
odometry epoch. Small out-of-order samples do not lower that high-water mark.
When no valid lookup exists, scan-to-map matching continues without an
odometry prediction, an accepted scan leaves the last synchronized
`map -> odom` correction in place because it cannot safely replace it, and
diagnostics report `odom_sync_valid=false`.

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

When odometry is available and `publish_tf=true`, an accepted localization with
synchronized odometry caches `map -> odom`:

```text
map_T_odom = map_T_base(localized) * inverse(odom_T_base)
```

The cloud callback only updates this cache; it does not broadcast TF. Starting
with the next valid odometry callback, the node broadcasts one cached transform
for each strictly newer odometry header stamp. Duplicate and slightly
out-of-order stamps are suppressed so TF time never repeats or moves backward.
Reanchoring updates the transform but deliberately preserves the last emitted
stamp. A TF is emitted only when its odometry stamp is newer than both the
localization anchor stamp and the last emitted stamp, preventing queued stale
odometry from reopening the gate. Correction invalidation preserves that
watermark; only a detected odometry timestamp epoch rewind resets it. This
keeps TF cadence aligned with the owner of `odom -> base_link` while scan
matching runs at its own rate. Before the first accepted synchronized
localization, no `map -> odom` TF is published by default. When
`publish_identity_map_to_odom_before_localization=true`, the node instead
broadcasts identity `map -> odom` at each valid odometry stamp while the
startup map/odometry-origin assumption remains valid. This is intended only
when the initial map and odometry origins coincide, for example to make the
loaded map and robot TF tree immediately visible in RViz. An explicit initial
pose, a timestamp epoch reset, or the first accepted synchronized localization
permanently retires the identity bootstrap; accepted localization replaces it
with the measured correction. The node never falls back to identity after
the startup assumption has been retired. A new initial pose, an odometry
timestamp rewind, or a cloud timestamp epoch reset invalidates the correction
until another accepted synchronized localization reanchors it. Only the
odometry rewind also resets the stamp gate.

`publish_tf=false` suppresses every broadcast.

The executable uses a two-thread executor and gives odometry a dedicated
mutually-exclusive callback group. Odometry buffering and TF cache access have
separate short-lived locks, so the normal odometry callback is not serialized
behind the registration state lock. Executor scheduling, DDS delivery, and CPU
load can still affect wall-time cadence. Epoch-reset handling may briefly wait
for localization state after it has already invalidated the old TF correction.

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
