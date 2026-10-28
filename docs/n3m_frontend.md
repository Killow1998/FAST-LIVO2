# N3M frontend integration — 2026-10-28

One local `main` now contains a ROS-independent C++ Mapper and thin native Noetic/Humble nodes. Both builds and N3M mapping are verified. The MID360 partial-map continuation and cross-ROS reload pass this fixture's 10 cm gate. **The tested Avia continuation/reload cases do not pass; this is not a universal localization qualification.** The installed programs and navigation frozen libraries were not replaced.

## Architecture and input contract

`include/fast_livo2/mapper.h` is the public API. `src/core` owns synchronization, state updates and outputs; the existing IMU, LiDAR and visual modules implement the algorithm once. The ROS nodes contain parameter/message/topic/TF bindings only: 153 lines for Noetic and 171 for Humble. Preprocessing and visual implementation files are divided by responsibility; no alternate ROS-specific solver remains. Unreferenced private dependency sources/headers (3,208 lines) were removed after checking the build and include graph. See [build and usage](build.md).

`/n3m_input/cloud_body` is the complete deskewed LiDAR view in the IMU frame, with intensity. `/n3m_input/odometry` is `camera_init -> body`, has the same acquisition stamp and includes the final visual update. `body` is the IMU origin, not necessarily the robot base. The world/RGB visualization cloud can omit points outside the camera view and must not serve as N3M's geometric input.

The core supports an optional IMU-rate observer. Without an observer it retains no propagation queue. ROS Path is a bounded visualization tail; complete trajectories are optional exports. Runtime logs use bounded asynchronous writing and batch summaries; sensor/map exports are separate opt-in artifacts.

## Corrections and causal checks

- Visual lookup used floor followed by another negative-cell shift, unlike insertion. It missed existing observations and repeatedly stored new map points/images. Insertion and query now share the same voxel convention. Geometric neighbour comparisons also use consistent metre coordinates.
- Retained visual patches and warp-cache entries have explicit ownership. Split geometric voxel nodes release point buffers already distributed to their children. A 10,000-observation probe produced byte-identical plane output while redundant parent storage fell from 27,648 bytes to zero.
- Camera-disabled synchronization no longer retains completed IMU groups. Completed color batches clear even when no point is visible. Sensor type is initialized before IMU processing; an unused uninitialized Boolean was removed after UBSan detected its copy.
- The feature threshold typo `disA=.01; disA=.1` now initializes `disB=.1`. Empty, short, blind-only and exhausted feature spans are checked at their input boundaries. Six boundary tests cover both Avia and ring-cloud preprocessing.
- ASan reproduced an image overread when an EKF iteration projected a pyramid patch outside the image. Alignment now checks the actual pyramid footprint and clears skipped measurement rows; an empty update cannot reuse an old covariance correction. The full near-wall route subsequently passed ASan, UBSan and LeakSanitizer.
- Optional COLMAP files now all use the selected output directory. An export probe checks PCD, images, trajectory, camera metadata and point metadata. Its short initialization-heavy input is an export check, not an accuracy test.

The original near-wall Avia route requires the existing `vio.raycast_en=true` option. In the corrected monolith, enabling it alone reduced the frontend maximum from 2.24 m to 5.62 cm. Reducing pyramid levels without raycasting still produced 27 cm; that change was not adopted alone. Raycasting is a recorded fixture setting, not a newly imposed global default.

## Four-core end-to-end results

All runs restrict execution to four physical cores, replay frozen inputs at 1× and exclude truth from estimation. Mapping scores use one fixed SE(3) alignment from the first output, without whole-trajectory or scale fitting. Each cell below is frontend / N3M **maximum position error in cm**.

| Input | Noetic | Humble |
| --- | ---: | ---: |
| Avia, normal wall distance | 8.01 / 7.70 | 8.09 / 7.98 |
| Avia, near wall, visual raycasting | 5.87 / 5.71 | 5.65 / 5.75 |
| MID360, normal wall distance | 1.80 / 1.38 | 2.11 / 1.33 |
| Avia route plus five-minute hold | 8.01 / 7.70 | 8.09 / 7.98 |

All eight mapping runs completed cleanly, produced paired clouds/poses and saved maps. The long Noetic/Humble runs produced 4,069/4,070 frontend poses and 4,029 N3M poses. Official Retail Street also completed Noetic pairing, mapping and save; it has no independent truth, so endpoint distance is not an accuracy score.

A separate camera-disabled control in the same Noetic environment produces 1,069 frontend poses before and after extraction. The maximum error is 4.113 m before and 4.116 m after; both exceed 10 cm for the same 65.2-second interval. Native Jammy replay reaches 4.282 m. This preserves the operating mode, but does not qualify camera-disabled corridor accuracy.

### Memory and CPU

The same 407.298-second Avia input includes a long stationary segment. CPU below is frontend process CPU seconds divided by input duration, including all its threads. It excludes the bag player, image decoder and N3M. RSS peaks use both sampled resident memory and process high-water marks.

| Implementation | Peak RSS, MiB | CPU/input second, cores | Stationary RSS, first → last MiB |
| --- | ---: | ---: | ---: |
| Original ROS1 baseline | 672.06 | 0.263 | 329.48 → 653.85 |
| Corrected monolith before extraction | 278.79 | 0.259 | 276.47 → 278.65 |
| Shared core, Noetic | 232.26 | 0.236 | 231.70 → 231.95 |
| Shared core, Humble | 310.76 | 0.233 | 305.49 → 307.55 |

The stationary measurement window is wall seconds 130–400. Original growth was approximately 72 MiB/min; the shared core is approximately stable in this window. ROS2 has different middleware overhead, so the strict before/after memory comparison is the Noetic pair. This does not establish bounded memory during unlimited exploration. Retail Street peaks at 690.65 MiB and averages 0.438 CPU/input second in the new Noetic frontend.

## Save, continuation and reload

The first 24 seconds create a partial map, then the complete route extends it. Every stage uses **the same map-to-truth transform obtained from the partial-map frontend's initial pose**; reload is never independently realigned.

| Sensor / stage | Maximum error | Result |
| --- | ---: | --- |
| MID360 partial map | 0.57 cm | Pass |
| MID360 continuation | 1.66 cm | Pass |
| MID360 Humble localization from Noetic's saved map | 2.00 cm | Pass |
| Avia normal-distance partial map | 0.70 cm | Pass |
| Avia normal-distance continuation | 24.90 cm | Accuracy fails |
| Avia normal-distance Humble reload | 55.86 cm | Accuracy fails |
| Avia near-wall partial map | 0.70 cm | Pass |
| Avia near-wall continuation | 23.44 cm | Accuracy fails |
| Avia near-wall Humble reload | 16.24 m | Wrong lock; fails |
| Same Avia map and paired input, Noetic reload | 16.24 m | Wrong lock reproduced |

Normal-distance Avia continuation/reload exceed 10 cm in 46.6% / 50.9% of outputs. Near-wall Avia continuation exceeds 10 cm in 44.5% of outputs. Reload locks near the other corridor end at about 100.7 s and returns to the starting region at about 103.6 s. Both ROS versions reproduce the failure; Humble conversion alone cannot explain it. The visual frontend itself remains below 6 cm, while N3M uses LiDAR geometry. Repeated/weak geometry is a working explanation, not a proven complete root cause. N3M production algorithms and thresholds were not changed for this port.

Next N3M work should isolate the incorrect recovery candidate and tracking constraints using this saved map, paired input and fixed-frame score. Do not hide the failure with a fresh alignment, omit the segment, or describe clean file I/O as reliable localization.

## Evidence and remaining scope

Workspace evidence is under `build/fast_livo2/integration/`: `qualification.json`, `final_source_manifest.json`, `qualified_build_identity.json`, per-route scores and `persistence_*` directories. Frozen raw data are under `build/fast_livo2/sprobo_20261027/`; `monolith_reference/` retains the corrected pre-extraction implementation. [Qualification summary](qualification_20261028.json) records final binaries separately from the full-matrix binaries.

The full ROS matrix preceded the no-observer queue guard and optional COLMAP path correction. These do not execute in that matrix's configuration; targeted native/export checks plus both final builds and preprocessing regressions verify the later changes. Standalone Release builds, installation and an external `find_package(fast_livo2_core)` consumer pass without ROS. The near-wall sanitizer result is a full native replay; the ASan-only Eigen allocation setting matches the system PCL ABI and is not a production flag.

The simulator camera is an ideal 320×240 pinhole, not a calibrated physical D435i optical/noise model. No RK3588 deployment, general 10 cm guarantee, or reliable camera-disabled corridor localization is claimed. Both tested Avia continuation/reload conditions remain unqualified. This is a local development candidate; no remote publication or installed-library replacement is part of this stage.
