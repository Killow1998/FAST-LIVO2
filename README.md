# FAST-LIVO2 — shared C++ core for Noetic and Humble

LiDAR–inertial–visual odometry with one ROS-independent C++17 estimator and
thin ROS 1 Noetic / ROS 2 Humble adapters on the same `main` branch. The core
can also run without ROS and supply deskewed clouds and fused odometry to
[N3Mapping](https://github.com/Killow1998/n3mapping).

This repository is a fork of [hku-mars/FAST-LIVO2](https://github.com/hku-mars/FAST-LIVO2).
The original algorithm and research are credited to Chunran Zheng and the
upstream authors. This fork retains the IMU/LiDAR/visual estimation framework
and focuses on integration, correctness, resource use and maintainable boundaries.

[Build guide](docs/build.md) · [Documentation index](docs/README.md) ·
[Changes and validation](docs/n3m_frontend.md) · [Upstream research](docs/upstream.md)

## What changed in this fork

- **Shared estimator:** the core owns synchronization, state, local maps and
  outputs. ROS adapters handle parameters, message conversion, topics and TF.
  Sensor decoding, voxel geometry, map maintenance and estimation have distinct
  implementation files. The public interface is [Mapper](include/fast_livo2/mapper.h).
- **Association and memory fixes:** visual voxel lookup and insertion use the
  same coordinate convention; cached patches and warps have explicit ownership.
  Split voxel nodes release redundant parent buffers, and completed sensor/color
  batches no longer accumulate. ROS Path histories and runtime logs have bounded storage.
- **Correctness fixes:** initialize the `disB` feature threshold, handle invalid
  feature spans, check the actual image-pyramid footprint, and clear skipped
  measurement rows before an update. These address reproduced failures.
- **Less repeated work:** timestamped queue entries replace parallel queues;
  independent correspondence slots use byte flags without a per-point mutex.
  Unused indexing, allocations, code and generated ROS message copies are removed.
- **Consistent outputs:** N3M receives the full deskewed body cloud and the final
  post-visual pose at the same acquisition time. Optional sensor/map exports and
  bounded asynchronous runtime logs remain available.

## Build and run

Choose the build entry for your environment. Keep ROS distributions and
Debug/Release artifacts in separate build directories.

| Environment | Entry | Instructions |
| --- | --- | --- |
| C++ without ROS | `core/` | [Standalone build and API](docs/build.md#standalone) |
| ROS 1 Noetic | `noetic/` | [Catkin workspace and Livox driver](docs/build.md#noetic) |
| ROS 2 Humble | `humble/` | [Colcon workspace and Livox driver2](docs/build.md#humble) |

For a standalone library build, install Eigen, PCL, OpenCV, OpenMP and ccache,
then run:

```bash
git clone https://github.com/Killow1998/FAST-LIVO2.git
cd FAST-LIVO2
cmake -S core -B build/core-release -DCMAKE_BUILD_TYPE=Release \
  -DBUILD_TESTING=OFF \
  -DCMAKE_CXX_COMPILER_LAUNCHER=ccache -DCMAKE_C_COMPILER_LAUNCHER=ccache
cmake --build build/core-release -j4
```

Camera and SO(3)/SE(3) dependencies are pinned under `third_party`; no separate
ROS Vikit package or global Sophus installation is required. Enable
`BUILD_TESTING` with GTest and yaml-cpp installed to build the regression tools.

After building and sourcing the selected ROS adapter, launch it with your
frontend and camera calibration files:

```bash
# Noetic
roslaunch fast_livo n3m_frontend.launch \
  config_file:=/path/frontend.yaml camera_file:=/path/camera.yaml

# Humble
ros2 launch fast_livo n3m_frontend.launch.py \
  config_file:=/path/frontend.yaml camera_file:=/path/camera.yaml
```

Both adapters accept the same frontend/camera YAML dictionaries. Configure
sensor topics, timestamps, intrinsics and extrinsics for your hardware;
[bundled configurations](config/) describe their original sensor setups.

## Connect to N3Mapping

| N3M setting | Value | Meaning |
| --- | --- | --- |
| `cloud_topic` | `/n3m_input/cloud_body` | Complete deskewed LiDAR view in the IMU body frame |
| `odom_topic` | `/n3m_input/odometry` | Final fused pose with the matching acquisition stamp |
| `body_frame` | `body` | IMU origin; apply the physical IMU-to-base transform separately |

The two `n3m_frontend` launch entries disable frontend map/image/trajectory
exports so N3M owns map persistence. `/cloud_registered` is a visualization
output and may contain only camera-visible colored points. Use the body-cloud
topic above for geometric mapping and localization. See the
[integration guide](docs/build.md#n3m-and-recording).

## Validation and measured changes

Recorded four-core tests cover Avia and MID360 raw-input mapping, continuation
and cross-ROS map reload. Full near-wall replay passes AddressSanitizer,
UndefinedBehaviorSanitizer and LeakSanitizer. Test conditions, scoring methods,
source/binary identities and limitations are in the [validation record](docs/n3m_frontend.md).

| Comparison | Recorded result |
| --- | --- |
| Upstream baseline → shared core, same 407-second Avia replay in Noetic | Peak RSS 672 → 232 MiB; stationary RSS becomes approximately stable during the measured window |
| Pre-cleanup shared core → cleaned core, three fixed-input native replays | Total process CPU time decreases 32.7%–37.6%; peak RSS varies by about −1 to +6 MiB |

These comparisons use different baselines. The native CPU comparison is one
A/B sample per fixture. It does not establish a combined upstream-to-current
speedup or a RAM bound during unlimited exploration.

The simulation camera is an ideal 320×240 pinhole. General 10 cm accuracy,
camera-disabled corridor accuracy and RK3588 deployment remain unqualified.
The tested N3M continuation paths require its matched-geometry tracking repair;
already corrupted historical maps require regeneration from raw input.

## Documentation and attribution

The [documentation index](docs/README.md) links build instructions, validation
records, configuration and upstream resources. Original papers, videos,
equipment, datasets and contact details are preserved in
[upstream project references](docs/upstream.md).

## License

[GNU GPL version 2](LICENSE). Vendored source notices and pinned revisions are
documented in [third-party provenance](third_party/README.md). For upstream
commercial licensing inquiries, use the contacts in the upstream references.
