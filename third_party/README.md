# Pinned numerical and camera dependencies

- Sophus: https://github.com/strasdat/Sophus, commit `a621ff2e56c56c839a6c40418d42c3c254424b5c`. The SO(3)/SE(3) code is the version used by the frozen reference; the local Eigen-compatible constructor changes are retained.
- Vikit: https://github.com/uzh-rpg/rpg_vikit, commit `6c886c8e5d83997806e00294826d528cea3581dd`. Camera/math sources match the frozen reference. Local ROS CMake modifications are not imported.

Only private ROS-independent sources are compiled. Source copyright and permission notices remain intact; neither repository at these commits supplied a separate root LICENSE file. ROS loaders, forced optimization flags and native-architecture flags are excluded from the core build.

Unreferenced dependency implementations and headers are omitted. Trailing whitespace is normalized; algorithm code and source notices are retained.
