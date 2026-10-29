# N3M frontend integration — 2026-10-28

[Project overview](../README.md) · [Documentation index](README.md)

One `main` contains a ROS-independent C++ Mapper and thin native Noetic/Humble nodes. The tested Avia and MID360 mapping, continuation and regenerated-map reload paths pass the four-core fixtures. **Continuation requires the N3M shared-core repair `eb4421c`; previously corrupted maps must be rebuilt from raw input.** The installed programs and navigation frozen libraries were not replaced.

## Architecture and input contract

`include/fast_livo2/mapper.h` is the public API. `src/core` owns synchronization, state updates and outputs; the existing IMU, LiDAR and visual modules implement the algorithm once. The ROS nodes contain parameter/message/topic/TF bindings only: 153 lines for Noetic and 171 for Humble. Preprocessing and visual implementation files are divided by responsibility; no alternate ROS-specific solver remains. Unreferenced private dependency sources/headers (3,208 lines) were removed after checking the build and include graph. See [build and usage](build.md).

`/n3m_input/cloud_body` is the complete deskewed LiDAR view in the IMU frame, with intensity. `/n3m_input/odometry` is `camera_init -> body`, has the same acquisition stamp and includes the final visual update. `body` is the IMU origin, not necessarily the robot base. The world/RGB visualization cloud can omit points outside the camera view and must not serve as N3M's geometric input.

The core supports an optional IMU-rate observer. Without an observer it retains no propagation queue. ROS Path is a bounded visualization tail; complete trajectories are optional exports. Runtime logs use bounded asynchronous writing and batch summaries; sensor/map exports are separate opt-in artifacts.

### Angular-velocity output repair — 2026-10-29

Scan outputs now use the latest IMU angular measurement at or before the output
time, minus the current corrected state's bias. They previously reused the last
integration interval's average and pre-correction bias. IMU-rate outputs retain
their existing current-sample calculation. The state owner passes the measurement
explicitly into output conversion; the output module does not depend on the IMU
implementation. In no-IMU mode, the constant-motion model stores angular velocity
in `bias_g`; that estimated rate is published directly, rather than treating it
as a sensor bias. Both velocities refer to the IMU origin in body coordinates.

Four-core Humble/Noetic builds and feature tests pass. Frozen-input native core
replays from both ROS toolchains cover Avia, textured Avia, MID360, LIO-only and
LiDAR-only modes, plus a MID360 repeat. Avia and both sensor-disabled modes retain
byte-identical trajectories. MID360's repeated runs differ by at most 3.75 mm at
individual frames; maximum truth error is unchanged and RMSE changes by at most
0.0014 mm. Prior same-version MID360 variation reached 6.13 mm. This is not a claim
of bitwise determinism. Against the latest input gyro during turns, the three
IMU-enabled LIVO fixtures improve angular-rate consistency from about 0.019 rad/s
RMSE to 0.00003–0.00012 rad/s. This measures output consistency, not a new trajectory
accuracy gain. The native replay tool also records `twist.csv` for this check.

See [output-contract evidence](odometry_contract_20261029.json). High-rate pose
covariance propagation is repaired below; twist covariance is still not supplied
and must not be treated as calibrated velocity uncertainty. On these Avia inputs,
disabling camera/IMU also retains the
old 4.1–6.3 m maximum trajectory errors: those modes passed behavior regression,
not 10 cm accuracy acceptance. No original installation or navigation frozen
library was replaced.

### High-rate prediction covariance — 2026-10-29

The optional IMU-rate predictor previously published an advanced pose with the
last correction's covariance. It now advances its own copied P with the same
transition and process-noise model as scan deskew. `imu_covariance.cpp` owns this
shared calculation. No predictor covariance is written back into the corrected
filter state. The no-IMU constant-motion path and all noise parameters are unchanged.
The private `pose_covariance.h` owns projection to parent-frame position/rotation
order; a marked, non-diagonal fixture checks all 36 elements in native Noetic and
Humble Odometry messages, including rotated cross terms.

The analytic test verifies constant-velocity position uncertainty across 200
steps, anisotropic noise, and disabled estimation flags. Frozen-input replays
enable the existing high-rate observer and record its actual pose/covariance.
This fixes state-time consistency; it is not a new sensor-noise calibration or
a complete twist-uncertainty model. See [qualification and cost measurements](prediction_covariance_20261029.json).

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

### Repair qualification matrix

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

The simulator camera is an ideal 320×240 pinhole, not a calibrated physical D435i optical/noise model. No RK3588 deployment, general 10 cm guarantee, or reliable camera-disabled corridor localization is claimed. The tested regenerated-map paths pass; the historical corrupted map requires rebuilding. The initial integration and repair stages used local candidates and did not replace installed libraries.

## Structural cleanup — 2026-10-28

The cleanup starts from FAST-LIVO2 `3bdc4ca` and N3M `dda1c11`. It preserves
the public interfaces, sensor parameters, numerical equations and acceptance
gates. Timestamp and payload now share a queue entry. Voxel geometry, map
maintenance and state estimation have separate implementation files; Livox and
standard point-cloud decoders share the existing dispatcher. Independent byte
flags replace packed flags and their per-correspondence mutex. Unused residual
indexing, an unused matrix allocation, obsolete APIs and disabled code are
removed. No new worker, cache or defensive wrapper is introduced.

Noetic uses the existing Livox driver's generated message headers. Both message
MD5s match the removed copies. N3M shares one 167-field table between both ROS
loaders and canonical configuration records. Its existing configuration E2E
caught a missing empty-map-directory fallback during refactoring; that behavior
was restored in both adapters and verified through their native parameter APIs.

Excluding documentation and third-party sources, FAST-LIVO2 removes 903 net
code lines, including 550 generated message-header lines; N3M removes 455.
The largest FAST-LIVO2 algorithm source is 452 lines. The subsequent removal of
old comments produces identical standalone, Humble and Noetic core binaries.

### Fixed-input ablation

The original and cleaned native cores replay the same raw files on CPUs
`0,2,4,6`, with four workers and writable bounded logs. CPU is total process CPU
seconds, including decoding and all worker threads. Each row is one A/B sample,
not a hardware performance guarantee.

| Input | CPU seconds, before → after | Peak RSS MiB, before → after | Output comparison |
| --- | ---: | ---: | --- |
| Avia normal distance | 17.88 → 11.88 | 215.15 → 218.73 | 1,069 poses, byte-identical |
| Avia near wall | 17.67 → 11.89 | 220.47 → 219.53 | 1,093 poses, byte-identical |
| MID360 | 21.30 → 13.29 | 223.98 → 230.23 | 1,069 poses, maximum position difference 6.26 mm |

The unmodified MID360 baseline itself varies by up to 6.13 mm across repeated
runs. The final MID360 candidate is not byte-identical: truth peak error is
1.6179 cm in both versions, position RMSE changes from 0.48093 to 0.48070 cm,
and angular RMSE does not increase. No general accuracy gain is inferred from
these small differences. Camera-disabled LIO and camera/IMU-disabled LO retain
byte-identical outputs, with 1,069 and 1,073 poses respectively. The earlier
camera-disabled corridor accuracy limitation still applies.

### Integrated regression

The cleaned frontend and backend core repeat all 12 raw-input continuation and
cross-ROS reload cases. Maximum position errors in cm are:

| Input | Noetic continuation | Humble reload | Humble continuation | Noetic reload |
| --- | ---: | ---: | ---: | ---: |
| Avia normal distance | 9.03 | 8.45 | 7.99 | 8.46 |
| Avia near wall | 6.12 | 5.06 | 5.72 | 5.40 |
| MID360 | 1.66 | 2.00 | 1.79 | 1.77 |

All cases save and shut down cleanly. Live scheduling changes the selected
frames and peaks relative to the repair matrix; they are not bitwise trajectory
comparisons. The later adapter-only restoration of the empty-map-directory
fallback is covered by native parameter tests; these 12 runs explicitly select
nonempty output directories. The core binaries are unchanged by that restoration.

N3M's original 14-point A/B produces 4,088 poses, unchanged decisions and locks,
and a 2.48 cm peak. The two motion routes retain 2,232/3,662 poses, locks at
frames `[4]` / `[4,1938]`, and peaks of 7.94/3.25 cm. Their pose differences are
at floating-point rounding scale. Humble passes 69/69 tests; Noetic passes
67/67. All four preprocessing test builds pass. The complete 1,093-pose
near-wall ASan/UBSan/LeakSanitizer replay passes.

The 12 ROS runs use 0.127–0.142 frontend CPU seconds per input second. N3M uses
0.224–0.549; peak RSS including save is at most 314.4 MiB for the frontend and
256.6 MiB for N3M. RSS is not additive physical RAM. These fixture results do
not qualify a universal half-core limit or RK3588 deployment.

[Closeout qualification](closeout_20261028.json) records source fingerprints,
binary identities, all cases and protected-library hashes. Full logs, native
ablation, parameter checks and sanitizer evidence remain under
`build/fast_livo2/closeout_20261028/`; the 12 ROS cases are the integration
directory's `closeout_*` runs. Existing installed/frozen libraries and release
tags remain unchanged. Publication uses the two repositories' `main` branches;
this cleanup does not create a new release tag.
