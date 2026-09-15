# Voxel-surfel solver provenance

- Source: `gole_slam/slam_3d_core`, commit `17759633c06241e0d3d2bb3a0bcfc1f142bf7a81`.
- Original license: Apache-2.0 (`slam_3d_core/package.xml`); original copyright notices are preserved.
- Imported files: `include/slam_3d_core/voxel_surfel_registration.hpp`, `src/voxel_surfel_registration.cpp`, and `test/test_voxel_surfel_registration.cpp`.
- The initial import changed only include paths, namespace, and header guard. The later registration-interface refactor added immutable `PreparedTarget`, `prepareTarget()`, and `alignPrepared()` so unchanged targets can reuse their surfels. The original `align()` remains a convenience wrapper.
- Residuals, Jacobians, Huber weighting, observable-subspace Gauss–Newton solve, backtracking, and gates retain the imported mathematics. The original 13 tests remain; additional regression tests cover prepared-target equivalence, ownership, and backend cache invalidation. The current files are therefore not byte-identical to the original hashes below.
- The local copy needs Eigen/PCL, not Ceres or the SLAM pose-graph package. SLAM source was not modified.
- This is a pinned copy: future SLAM solver changes do not propagate automatically. Compare and rerun these tests before synchronizing.

Original SHA-256:

| File | SHA-256 |
| --- | --- |
| Header | `a5bba7fa9b94cff4b91db599fea08c8894dd622403ada4f02705fdba7f9631dd` |
| Source | `6ae0c3f93b1198ac3f1dc98f86cb2a4585aafd7bb7c6b3510c28309519cef022` |
| Test | `e6aa0dbdcfd22d40ace771d632b814efb94fd326a8ef9108d94efbf2514805db` |
