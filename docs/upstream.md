# Upstream project and research

[Project overview](../README.md) · [Documentation index](README.md)

This page preserves the upstream material from the previous project README.
The current fork's build instructions and changes are maintained in the
[build guide](build.md) and [integration record](n3m_frontend.md).

## Original project

[FAST-LIVO2: Fast, Direct LiDAR-Inertial-Visual Odometry](https://github.com/hku-mars/FAST-LIVO2)
is the upstream LiDAR–inertial–visual estimation system. Its original developer
is [Chunran Zheng 郑纯然](https://github.com/xuankuzcr).

![Upstream FAST-LIVO2 framework](../pics/Framework.png)

The upstream README records these milestones:

- 2025-01-23: code released.
- 2024-10-01: accepted by T-RO '24.
- 2024-07-02: conditionally accepted.

## Papers and videos

- [FAST-LIVO2: Fast, Direct LiDAR-inertial-visual Odometry](https://arxiv.org/pdf/2408.14035)
- [FAST-LIVO2 on Resource-Constrained Platforms](https://arxiv.org/pdf/2501.13876)
- [FAST-LIVO: Fast and Tightly-coupled Sparse-Direct LiDAR-Inertial-Visual Odometry](https://arxiv.org/pdf/2203.00893)
- [FAST-Calib: LiDAR-Camera Extrinsic Calibration in One Second](https://www.arxiv.org/pdf/2507.17210)
- Demonstration videos: [Bilibili](https://www.bilibili.com/video/BV1Ezxge7EEi) and [YouTube](https://youtu.be/6dF2DzgbtlY).

## Equipment, datasets and calibration

- [LIV_handhold](https://github.com/xuankuzcr/LIV_handhold): upstream handheld
  device CAD, synchronization scheme, STM32 source, wiring and sensor drivers.
- [FAST-LIVO2-Dataset](https://connecthkuhk-my.sharepoint.com/:f:/g/personal/zhengcr_connect_hku_hk/ErdFNQtjMxZOorYKDTtK4ugBkogXfq1OfDm90GECouuIQA?e=KngY9Z): the dataset link retained from the upstream README.
- [Global-LVBA dataset entry](https://github.com/xuankuzcr/Global-LVBA#4-dataset-download): the alternative download entry used by the recorded official-data evaluation.
- [FAST-Calib](https://github.com/hku-mars/FAST-Calib): the upstream LiDAR–camera
  calibration toolkit. Use calibration and timing parameters for the selected
  device and recording.

## Contact and license

Upstream project inquiries: [Chunran Zheng](mailto:zhengcr@connect.hku.hk).
The original README directs commercial licensing inquiries to Chunran Zheng
or [Prof. Fu Zhang](mailto:fuzhang@hku.hk).

This repository retains the [GPLv2 license](../LICENSE) and source attribution.
See [third-party provenance](../third_party/README.md) for the vendored numerical
and camera dependencies used by this fork.
