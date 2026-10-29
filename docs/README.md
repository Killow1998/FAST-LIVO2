# FAST-LIVO2 documentation

[Project overview](../README.md)

The project README introduces this fork and its supported entry points. This
index organizes the setup, evidence and upstream material previously linked
from the project homepage.

| Document | Use it for |
| --- | --- |
| [Build and deployment](build.md) | Standalone C++, Noetic, Humble, calibration inputs, N3M topics and logging |
| [Changes and N3M integration](n3m_frontend.md) | Correctness/memory repairs, accuracy and resource comparisons, continuation and reload results |
| [Initial integration qualification](qualification_20261028.json) | Machine-readable identities and measurements before the final cleanup |
| [Cleanup qualification](closeout_20261028.json) | Final source/binary fingerprints, fixed-input comparisons, ROS tests and remaining limits |
| [Sensor and camera configurations](../config/) | Example inputs for the original recorded sensor setups |
| [Public Mapper API](../include/fast_livo2/mapper.h) | Embedding the estimator without ROS |
| [Upstream project and research](upstream.md) | Original authors, papers, videos, devices, datasets and calibration resources |
| [Third-party provenance](../third_party/README.md) | Pinned Sophus/Vikit sources and retained notices |
| [License](../LICENSE) | Project licensing terms |

Use the build guide for current commands. The dated qualification files retain
their original experiment scope; the integration record distinguishes each
baseline and links the final cleanup results.
