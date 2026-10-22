# Encoder + IMU primary, optional RGB-D VO

All changes are contained in `amr`. `AMR_Private` is an untouched backup.

```mermaid
flowchart LR
  E[Encoder /odom_raw] --> W[Unknown covariance adapter]
  W --> K[EKF 2D 30 Hz]
  U[IMU /imu] --> F[Complementary filter: gyro bias compensation]
  F --> Q[IMU covariance adapter]
  Q --> K
  C[Astra registered RGB + depth] --> R[Shared RGB-D synchronization]
  R --> M[RTAB-Map mapper 1 Hz]
  K -->|/odom and covariance| M
  R -. optional .-> V[RGB-D VO 5 Hz, no TF]
  V --> G[Quality, timestamp and recovery gate]
  G -. body vx/vy only .-> K
  C --> D[Depth obstacle adapter]
  D -->|/depth/scan| N[Navigation]
  K --> N
  M --> N
  N --> A[cmd_vel_arbiter]
  T[Manual control] --> A
  A -->|/cmd_vel| B[Robot]
  A -. observation only .-> S[Motion monitor]
  D --> S
  S --> H[/motion/status: warnings only]
```

## Data contracts

* `/wheel/odom_cov`: encoder measurements, frames and timestamps unchanged.
  Only an all-zero twist covariance gets an explicit default (0.03 m/s standard
  deviation on vx/vy). This is an engineering starting value, not calibration.
* `/imu/data`: complementary filter with bias estimation enabled, magnetometer
  disabled and TF publication disabled. `/imu/data_cov` supplies only missing
  covariances; it does not compensate bias again. `/imu/steady_state` is available
  for checking when the filter considers the IMU stable. Slow constant rotation
  can fool IMU-only steady-state detection; physically verify bias learning.
* `/odom`: EKF uses wheel vx/vy and IMU wz. Its optional `/vo/twist` subscription
  never gates initialization or output. Wheel integrated pose and IMU integrated
  yaw are not fused as independent observations.
* `/camera/rgbd_image`: common synchronized RGB-D input for mapper and optional
  VO, independent of the VO output. Astra hardware depth registration is enabled
  in configuration. Confirm actual RGB/depth alignment on the physical camera;
  an enabled parameter alone is not evidence of good calibration.
* `/depth/scan`: `LaserScan` in `base_footprint`, using transformed 3D depth points
  in a configurable obstacle-height band. Native depth CameraInfo must match the
  image frame/size. Only rectified, unbinned/full-frame depth is accepted. Distorted
  depth requires rectification first. Padded rows, millimetres/metres and byte
  order are handled. Floor returns are excluded as obstacles.
  Camera mounting is assumed rigid: base-to-camera extrinsics use the latest
  available TF because Astra republishes these fixed values more slowly than
  images. The scan retains the original image timestamp for robot pose lookup.
  A moving/pan-tilt camera requires a different, time-synchronized TF strategy.
  Rectified intrinsics are read from a valid canonical CameraInfo projection P,
  with a valid K fallback. This handles Astra registered-depth metadata that
  contains NaNs in K while P matches the registered RGB projection. Invalid
  models are rejected rather than replaced by guessed focal lengths.
* `/depth/clearance`: companion scan with the same source timestamp/flat frame,
  storing conservative observed distances, including floor hits. Unknown bins
  are NaN. Clear obstacle bins use infinity only when sufficient valid depth was
  observed; this is not unlimited free space, and clearance remains bounded.
  Sparse confirmed obstacles survive even when free-space coverage is inadequate.
* `/depth/status`: JSON validity, original timestamp, valid-depth fraction,
  reason and calibration confirmation. Missing/stale depth reports invalid.
* `/vo/twist`: only body-frame vx/vy, with original VO timestamp and a covariance
  floor of 0.04 (m/s)^2. EKF does not fuse VO pose or angular velocity. Gate pairs
  odometry and OdomInfo with exact timestamps, checks tracking/inliers, covariance,
  plausible motion, pose discontinuity, timestamp order/gaps and reset of the
  distance counter. No output on loss; five consecutive good frames are required
  to recover. No zero twists are fabricated during loss. `/vo/status` reports
  INITIALIZING/RECOVERING/VALID/LOST, including silence detected by watchdog.
* `/motion/status`: advisory-only JSON. Checks original timestamps and receive
  ages of raw wheel/IMU as well as EKF output, depth and command streams. A fresh
  predicted EKF output must not hide missing raw inputs. When calibrated it also
  assesses observed stopping distance, rear blind sectors and swept footprint.
  **This node never publishes/intercepts velocity commands.**

TF ownership: EKF alone publishes `odom -> base_footprint`; RTAB-Map publishes
`map -> odom`; static measured transforms connect base, IMU and camera frames.
Neither VO nor complementary filter publishes those TFs.

## Geometry and movement behavior

`ros/src/bringup/config/geometry.yaml` holds camera x/y/z and roll/pitch/yaw,
front/rear body extent and half-width. All measurements remain **0** as requested,
with `calibration_confirmed: false`. There is no calibration movement interlock.
Until dimensions are supplied, navigation retains its previous provisional
0.08 m radius/front/half-width. These are not physical measurements. Once all
body dimensions are positive, launch derives a circumscribed radius and passes
the dimensions to navigation. Rebuild/relaunch after editing the geometry file.

Navigation reads `/depth/scan`; web scan display uses the same stream. Navigation
checks both receipt age and original scan/odom timestamps. Existing obstacle
collision checks, emergency stop and arbiter input timeouts remain active.
Starting forward speeds are 0.12 m/s target, 0.15 m/s maximum; yaw limit 0.30 rad/s.
Manual arbiter behavior and reverse command availability are retained. No new
direction or calibration lock is added, per the user's instruction.

The monitor reports unknown visibility, but does not stop the robot. In
particular, front-only depth does not establish rear/side clearance, blind-zone
clearance or detection of every low/transparent obstacle. Empty unknown sectors
must not be interpreted as verified free space. Physical collision protection
over all directions is not established by this implementation.

## Run and verify

From `amr`, build with `./shell.sh build`; then use the bootstrap/rtabmap/run
commands in README. VO is off by default. Enable explicitly with
`ENABLE_VO=true ./shell.sh rtabmap`; disable with `ENABLE_VO=false`.
Camera/mapper configuration fingerprints cause `shell.sh` to recreate their
containers on a subsequent explicit start when configuration changes, rather
than silently keeping stale startup parameters.

`./tools/verify_architecture.sh` builds bringup in a temporary install prefix and
runs unit/ROS tests, the actual EKF without VO and across VO loss, main launch
smoke tests, and RTAB-Map launches with VO both disabled and enabled. Optional
VO checks inject synthetic odometry+quality messages through the real gate.
Containers use `--network none`, separate domains and no robot device mounts.
The existing compiled EKF is copied into the temporary prefix with the updated
YAML; EKF C++ sources are unchanged. The live install/runtime is not overwritten.
Logs are saved to the printed verification directory. Use `VERIFY_DIR=...` to
choose the location. No physical motion commands/goals are sent by verification.

Physical acceptance still requires measured wheel distances/turns, stationary
and slow-turn bias checks, actual image alignment, camera/footprint measurements,
latency/CPU/RAM measurement under mapping load, and supervised obstacle/stopping
tests. Synthetic tests do not establish those physical properties.
