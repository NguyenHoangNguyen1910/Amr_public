# Architecture verification — 2026-10-22

Implementation scope: `/home/pi/quangnv/amr` only. No source changes or writes
were made to `/home/pi/quangnv/AMR_Private`.

## Results

| Check | Result |
| --- | --- |
| Bringup CMake/ROS interface build in temporary install prefix | Passed |
| Existing and new Python/ROS tests | **66 passed**, no skipped tests |
| Actual compiled EKF starts and publishes using wheel + IMU, with no VO | Passed |
| Actual EKF remains continuous after optional VO twists stop | Passed |
| Main ROS launch starts depth adapter, monitor, encoder adapter, EKF and navigation | Passed |
| Navigation and web subscribe to `/depth/scan` | Passed |
| Arbiter is the sole `/cmd_vel` publisher; monitor does not intercept commands | Passed |
| Zero/unconfirmed geometry does not prevent manual arbiter output in isolated ROS test | Passed |
| Monitor reports stale wheel data even while EKF output is fresh | Passed |
| RTAB-Map launch with VO disabled | Passed |
| RTAB-Map launch with VO enabled, TF broadcast disabled on VO | Passed |
| Real gate transport: admit valid samples, stop output on loss, recover after five good frames | Passed |
| Synthetic RGB-D sync + external odometry produces a real RTAB-Map occupancy grid without VO | Passed |
| Shell syntax, Python compilation, `git diff --check` | Passed |
| Backup content, symlink targets, file mode and modification time comparison | **1,617 entries unchanged** |

Depth tests include padded/big-endian buffers, millimetres/metres, camera
transform/parallax, floor filtering, unknown depth, sparse obstacles and bounded
observed distance. VO tests include stale/future/duplicate timestamps, gaps,
quality, covariance, implausible motion, pose jumps, distance counter reset and
loss/recovery. A TF publisher endpoint can exist on RTAB-Map VO even with
`publish_tf=false`; verification checks the parameter and absence of VO-frame
transform messages, rather than incorrectly treating endpoint existence as a send.

## Reproduce

```bash
cd /home/pi/quangnv/amr
./tools/verify_architecture.sh
```

Verification uses Docker `--network none`, domains 83/84, and no robot device
mounts. All synthetic velocity commands remain within those isolated containers.
The actual EKF binary is copied from the existing install and tested with the new
configuration; its C++ source was not changed. Bringup is built from changed
source into the verification directory.

Recorded artifacts for this run:

* `/tmp/amr-architecture-verify-run.log`: 66 tests and main/RTAB-Map launch checks.
* `/tmp/amr-architecture-verify/unit-tests.log`: detailed 66-test results.
* `/tmp/amr-architecture-verify/build.log`: build result.
* `/tmp/amr-architecture-verify/ekf-test.log`: actual EKF test.
* `/tmp/amr-architecture-verify/bringup-launch.log`: main launch result.
* `/tmp/amr-architecture-verify/rtabmap-vo-off.log`: actual synthetic RGB-D mapping.
* `/tmp/amr-architecture-verify/rtabmap-vo-on.log`: optional VO/gate launch.
* `/tmp/amr-mapper-data-test.log`: synthetic mapping assertion results.
* `/tmp/amr-backup-verification.json`: backup comparison result.

## Runtime and remaining physical checks

At the initial verification, existing live ROS/camera containers were not restarted and their installed
code was not overwritten during verification. `depth_registration=true` is the
new startup configuration; the running camera has not been switched by this work.
Use the normal `shell.sh` build/start workflow to load the changed source.

When ready to replace the running session, from `amr`:

```bash
./shell.sh stop
./shell.sh build
./shell.sh bootstrap
./shell.sh rtabmap
./shell.sh run
```

This starts a new mapping session under the default nonpersistent configuration.
VO remains off; `ENABLE_VO=true ./shell.sh rtabmap` explicitly enables it.

Camera and body measurements remain zero in
`ros/src/bringup/config/geometry.yaml`, as requested. The monitor only warns;
there is no new movement/calibration/direction interlock. Existing emergency stop,
arbiter timeouts and navigation collision checks remain. Physical RGB/depth
alignment, footprint/extrinsics, encoder scale, IMU bias behavior, latency under
real mapping load, and obstacle stopping behavior still require measurements.
The synthetic tests do not establish physical navigation accuracy or rear/side
collision coverage from a front-only camera.

## Follow-up: camera TF timing

A subsequent passive hardware check found `/odom` and `/rtabmap/map` present
but no `/depth/scan`: lookup of camera extrinsics at the image timestamp failed
with future extrapolation because Astra publishes the internal rigid TF more
slowly than images. The adapter now uses the latest robot-fixed camera transform
and retains the original image timestamp on the resulting scan. The ROS
regression checks this TF lookup policy. Verification was rerun with artifacts
in `/tmp/amr-tf-fix-verify` and `/tmp/amr-tf-fix-verify.log`.

This assumes a rigid camera mount. It does not change navigation admission,
calibration interlocks, velocity commands or VO behavior.

The same hardware check then exposed registered-depth CameraInfo K containing
NaNs, while P was valid and matched color CameraInfo (fx=fy=570.3422047415297,
cx=319.5, cy=239.5). The adapter now uses a valid canonical rectified projection P
or a valid K fallback, without guessing intrinsics. Regressions cover both this
metadata and invalid camera models. The updated complete unit/ROS suite passed
**68 tests**, recorded in `/tmp/amr-astra-all-tests.log`.

After confirming all navigation goals were inactive/aborted and `/cmd_vel` was
zero, only the adapter and its Python core were updated in the live install.
The adapter was replaced under a supervisor tied to the original launch lifetime;
EKF, mapping, navigation and arbiter were not restarted. Passive observation
received **32 scans in 7 seconds**, 27 finite obstacle bins on the last scan,
`valid=true`, and maximum sampled acquisition-to-receipt age **0.037 seconds**.
This establishes restored scan delivery, not physical goal completion. No goals
or velocity commands were sent, and no movement/calibration lock was added.
