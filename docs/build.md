# Build and use the shared core

One `main` contains the authoritative C++ implementation. `noetic/` and `humble/` only bind parameters, messages, topics and TF. `include/fast_livo2/mapper.h` is the public interface; algorithm headers are private and are not installed. Call one Mapper instance serially. Internal geometric/photometric work can use `common.num_threads` workers (default 4).

## Standalone

```
cmake -S core -B build/core-release -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_CXX_COMPILER_LAUNCHER=ccache -DCMAKE_C_COMPILER_LAUNCHER=ccache
cmake --build build/core-release -j4
ctest --test-dir build/core-release --output-on-failure
```

Use a separate `build/core-debug` with `-DCMAKE_BUILD_TYPE=Debug` for development. CMake's selected configuration controls optimization; no unconditional `-O3`, `-march=native` or `-ffast-math` is added. The core requires Eigen, PCL, OpenCV, OpenMP and Threads. Camera/SO(3)/SE(3) implementations are pinned privately under `third_party`; ROS is not required. Tests additionally use GTest and yaml-cpp. Production builds may set `-DBUILD_TESTING=OFF`.

`fast_livo2_native_replay DATASET OUTPUT [FRONTEND_YAML]` replays the frozen simulator's raw LiDAR/IMU/RGB files through the same Mapper. Truth is excluded from the estimator; score the output separately. It is an E2E fixture runner, not a simulator or alternative algorithm.

On this Jammy/PCL installation, ASan changes Eigen's inferred malloc alignment. An instrumented core linked with the uninstrumented system PCL requires `-DEIGEN_MALLOC_ALREADY_ALIGNED=1` **in that validation configuration only**, matching PCL. Otherwise destruction reports an allocator mismatch. Run LeakSanitizer on the host; the sandbox's ptrace restrictions prevent its exit-time scan.

Installed C++ consumers can use `find_package(fast_livo2_core CONFIG REQUIRED)` and link `fast_livo2::fast_livo2_core`.

## Noetic

In a separate catkin workspace, create `src/fast_livo` and link the selected Noetic entry files, leaving the ROS2 discovery-ignore markers in the source repository:

```
mkdir -p src/fast_livo
for entry in CMakeLists.txt package.xml launch config rviz_cfg; do
  ln -s /path/FAST_LIVO2/noetic/$entry src/fast_livo/$entry
done
```

Also link the ROS1 `livox_ros_driver` package into `src`. Source `/opt/ros/noetic/setup.bash`, then:

```
catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_CXX_COMPILER_LAUNCHER=ccache -DCMAKE_C_COMPILER_LAUNCHER=ccache
catkin build fast_livo -j4 -p1
source devel/setup.bash
roslaunch fast_livo n3m_frontend.launch config_file:=/path/frontend.yaml camera_file:=/path/camera.yaml
```

Use a named Debug profile with distinct build/devel directories for development. On Jammy, the validated Noetic environment is the existing Docker image; do not install Noetic over the native Humble environment.

## Humble

Source `/opt/ros/humble/setup.bash` and the ROS2 `livox_ros_driver2` prefix. Build only the Humble package into a separate prefix:

```
MAKEFLAGS=-j4 colcon build --paths /path/FAST_LIVO2/humble --symlink-install \
  --build-base build/livo2-humble-release --install-base install/livo2-humble-release \
  --cmake-args -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_CXX_COMPILER_LAUNCHER=ccache -DCMAKE_C_COMPILER_LAUNCHER=ccache
source install/livo2-humble-release/setup.bash
ros2 launch fast_livo n3m_frontend.launch.py config_file:=/path/frontend.yaml camera_file:=/path/camera.yaml
```

Both launch entries accept the same frontend and camera YAML. The Humble launch translates these parameter dictionaries into native ROS2 parameters; direct `--params-file` use requires the usual ROS2 node/`ros__parameters` envelope. Humble sensor subscriptions accept best-effort or reliable publishers. Allocate DDS shared memory for the actual point-cloud message size; qualification uses N3M's existing `config/fastdds_local.xml` (64 MiB).

## N3M and recording

Set N3M `cloud_topic=/n3m_input/cloud_body`, `odom_topic=/n3m_input/odometry`, and `body_frame=body`. The deskewed full LiDAR cloud and final post-visual pose share a sensor stamp. The cloud is in the IMU frame; apply the physical IMU-to-base transform outside the frontend. `/cloud_registered` is a visualization/export topic and may contain only camera-visible colored points.

The N3M launch disables frontend PCD/image/COLMAP/trajectory exports because N3M owns map persistence. Optional exports remain available through the existing parameters and `output_directory`. ROS Path is a bounded visualization tail of 10,000 poses; enable trajectory export when a complete frontend trajectory file is required.

`FAST_LIVO2_LOG_DIR` selects an instance's runtime log. The shared logger uses a 1 MiB queue, 250 ms batched writes, 1 Hz frame summaries and a 100,000,000-byte file budget across restarts. Overflow/write failures are reported at shutdown. LIO/VIO timing fields cover processing calls, not ROS transport or point preprocessing. Keep different concurrent instances in different log directories.
