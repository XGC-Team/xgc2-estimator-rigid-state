# Standalone core

This package owns the rigid-state estimator core and the ROS node that links it.
Vision-pose publication uses `common/vision_pose_publication.h`; filter equations,
configuration normalization, health thresholds and input-time checks live in the
core.

Build the core without ROS against installed dependencies (Eigen3, xgc2_math and
xgc2_state_machine):

```sh
cmake -S . -B build \
  -DRIGID_STATE_BUILD_ROS=OFF -DRIGID_STATE_BUILD_TESTING=ON \
  -DCMAKE_PREFIX_PATH=/path/to/dependency-prefix \
  -DCMAKE_INSTALL_PREFIX=/path/to/install-prefix
cmake --build build --parallel 2
(cd build && ctest --output-on-failure)
cmake --install build
```

Installed artifacts:

- `lib/libestimator_vrpn_px4_rotor_state_core.so`: the single domain core.
- `find_package(RigidStateEstimator CONFIG REQUIRED)` exports
  `RigidStateEstimator::Core`, including its installed dependency targets.

Package the core library and the xgc2_state_machine shared-library dependency.
`$ORIGIN` resolves colocated installed libraries. ROS is optional at configure,
build and runtime.

Tests include the pure-core and input-timing cases and the shared vision
publication predicate.
