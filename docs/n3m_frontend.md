# N3M frontend integration — 2026-10-28

One local `main` contains a ROS-independent C++ Mapper and thin native Noetic/Humble nodes. The tested Avia and MID360 mapping, continuation and regenerated-map reload paths pass the four-core fixtures. **Continuation requires the N3M shared-core repair `eb4421c`; previously corrupted maps must be rebuilt from raw input.** The installed programs and navigation frozen libraries were not replaced.

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

The frozen backend failed before this repair: normal-distance Avia continuation/reload reached 24.90/55.86 cm; near-wall continuation reached 23.44 cm and reload incorrectly locked at the other corridor end, 16.24 m away. These failures remain in `persistence_before_n3m_repair` in the machine record.

### Matched geometry repair in N3M

GICP gives each prepared plane covariance the spectrum `(epsilon, 1, 1)`. Two aligned planes therefore contribute normal stiffness `1/(2*epsilon)` and tangential stiffness `1/2`. Tangential regularization keeps the numerical solve well conditioned; it does not make corridor sliding observable. The former update could turn that numerical stiffness into a persistent map correction.

The shared tracker now uses the **admitted correspondences'** translation Hessian, removes `0.5 * num_inliers * I`, and normalizes by `1/(2*epsilon) - 0.5`. It transports this support into the prediction frame and preserves the frontend translation along directions with less support than the existing normal-neighbour count. Rotation is unchanged. The altered pose is evaluated against the selected target with the same metric, correspondence radius and acceptance gates. Recovery keeps the selected target alive through this check. This is a translation-support rule, not a complete posterior uncertainty calibration.

The change adds 67 core/logging lines, one private mathematical helper and one asynchronous diagnostic field. It adds no thread, cache, dependency, public parameter or ROS-specific algorithm. The initial global search is unchanged. Projection time is included in registration timing; optimizer stage records remain available.

Plane-only registration and a six-DoF projection were rejected by their recorded probes (approximately 71 and 86 cm peaks). Source-only translation support initially passed frozen pairs but failed raw frontend replay with approximately 0.9 m jumps: a newly visible surface may be absent from the reference map. Only matched geometry is retained in the implementation.

### Final native-node matrix

Both ROS adapters replay the original LiDAR, IMU and images through the unchanged final FAST-LIVO2 binaries. Noetic runs in its Focal Docker environment; Humble runs natively. Atlas uses the current production default, enabled. Four cases per input cover both continuation directions and both cross-ROS reload directions. The values are maximum position errors in cm.

| Input | Noetic continuation | Humble reload of Noetic map | Humble continuation | Noetic reload of Humble map |
| --- | ---: | ---: | ---: | ---: |
| Avia, normal distance | 7.79 | 8.02 | 7.99 | 7.51 |
| Avia, near wall | 5.92 | 5.04 | 5.78 | 5.32 |
| MID360 | 1.66 | 2.00 | 1.79 | 1.77 |

All 12 complete cleanly with full output coverage, save/reload success and no output above 10 cm. The near-wall angular peak reaches 2.59 degrees; the 10 cm check measures trajectory position, not every map surface or sub-degree attitude accuracy. The final Humble continuation plus five-minute hold produces 4,065 outputs, peaks at 7.99 cm and saves in 0.40 s. The original 14 fixed positions retain all 4,088 outputs, first lock at frame 4 and a 2.48 cm peak. Two independent motion routes retain 2,232/3,662 outputs and their recovery points, with peaks of 7.94/3.25 cm. The first motion route's peak changes by about +0.04 mm versus the prior baseline; the other is unchanged to numerical precision.

### Same-input ablation and resource cost

The causal comparison disables only the new projection in an experimental library; it does not add a product switch. Both runs consume identical frozen frontend packets, the same original partial map and parameters, with four cores.

| Metric | Projection disabled | Final repair |
| --- | ---: | ---: |
| Position maximum | 24.90 cm | 7.79 cm |
| Position RMSE | 9.64 cm | 4.00 cm |
| Outputs above 10 cm | 46.9% | 0% |
| Output frames | 1,051 | 1,061 |
| Angular maximum | 0.617 degrees | 0.751 degrees |
| Angular RMSE | 0.237 degrees | 0.173 degrees |
| N3M replay CPU | 49.78 s | 52.28 s |

The added pose check costs approximately 5% CPU in this pair. Angular RMSE improves, but its peak increases by 0.134 degrees; this is not a claim that every individual metric improves.

Across the final 12 raw-input runs, N3M uses 0.224–0.278 CPU seconds/input second during localization and 0.372–0.545 during continuation. Its peak RSS including save is 144–258 MiB. Frontend use is 0.178–0.214 cores, with Noetic/Humble peaks up to 230/312 MiB. The long continuation averages 0.365 N3M cores and 0.231 frontend cores. During wall seconds 130–400, N3M RSS changes from 224.0 to 227.4 MiB and frontend RSS from 308.7 to 307.6 MiB. Per-process RSS is not additive physical RAM, and a five-minute hold does not establish bounded memory during unlimited exploration.

### Historical map recovery

The old corrupted near-wall map still produces a 16.24 m wrong lock with the repaired program. Replacing **only the map** with the result regenerated from the original partial map and raw recording reduces the peak to 5.41 cm and eliminates the wrong lock. The independently installed Humble candidate reproduces 5.41 cm. This repair prevents the bad continuation update; it does not rewrite historical map artifacts in-place.

Validated reconstructed maps are in workspace `build/fast_livo2/integration/repair_final_{avia_normal_range,avia_textured_final,mid360_normal_range}_noetic_resume/map/`. Each directory contains `n3map.pbstream` and its map artifacts. The machine record includes their SHA256 values. Retain the original failed maps as evidence and use the regenerated versions for these tests.

## Evidence and remaining scope

Workspace evidence is under `build/fast_livo2/integration/`: `qualification.json` (historical frontend stage), `repair_qualification.json`, `repair_final_build_identity.json`, per-route scores and the original `persistence_*` failure directories. Frozen raw data are under `build/fast_livo2/sprobo_20261027/`; `monolith_reference/` retains the corrected pre-extraction implementation. [Qualification summary](qualification_20261028.json) records final binaries separately from the full-matrix binaries.

The earlier eight-run mapping matrix preceded the no-observer queue guard and optional COLMAP path correction. These do not execute in that matrix's configuration; targeted native/export checks plus both final builds and preprocessing regressions verify the later changes. The final 12-run continuation/reload matrix uses the unchanged final frontend binaries and the repaired backend. Standalone Release builds, installation and an external `find_package(fast_livo2_core)` consumer pass without ROS. The near-wall sanitizer result is a full native replay; the ASan-only Eigen allocation setting matches the system PCL ABI and is not a production flag.

The simulator camera is an ideal 320×240 pinhole, not a calibrated physical D435i optical/noise model. No RK3588 deployment, general 10 cm guarantee, or reliable camera-disabled corridor localization is claimed. The tested regenerated-map paths pass; the historical corrupted map requires rebuilding. This is a local development candidate; no remote publication or installed-library replacement is part of this stage.
