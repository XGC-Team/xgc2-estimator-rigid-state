# Native adapter and standalone core

This package owns the rigid-state estimator core and both transport adapters.
The ROS node and native adapter link the same core. Vision-pose publication uses
`common/vision_pose_publication.h` in both paths; filter equations, configuration
normalization, health thresholds and input-time checks remain in the existing core.

Build against installed dependencies (Eigen3, xgc2_math, xgc2_state_machine and,
for the native adapter, XgcRuntimeSDK):

```sh
cmake -S . -B build \
  -DRIGID_STATE_BUILD_ROS=OFF -DRIGID_STATE_BUILD_NATIVE=ON \
  -DRIGID_STATE_BUILD_TESTING=ON \
  -DCMAKE_PREFIX_PATH=/path/to/dependency-prefix \
  -DCMAKE_INSTALL_PREFIX=/path/to/install-prefix
cmake --build build --parallel 2
(cd build && ctest --output-on-failure)
cmake --install build
```

Installed artifacts:

- `lib/libest_rigid_state.so`: native facade, exports `xgc_rt_plugin_v1`.
- `lib/libestimator_vrpn_px4_rotor_state_core.so`: the single domain core.
- `find_package(RigidStateEstimator CONFIG REQUIRED)` exports
  `RigidStateEstimator::Core`, including its installed dependency targets.

The native adapter consumes `XgcRuntime::SDK` from the installed
`XgcRuntimeSDK` package; it does not compile copies of the core/state-machine or
reach back into the runtime source tree. Package both shared libraries and the
xgc2_state_machine shared-library dependency. `$ORIGIN` resolves colocated
installed libraries. ROS is optional at configure, build and runtime.

Native ports and schemas remain unchanged: `imu`, `pose`, `rigid_state`,
`vision_pose`, and optional `estimate`. The session scheduler must use trigger
`both` and period `1 / state_publish_rate_hz`; state is published at round
boundaries. In session mode, receipt time comes from `xgc_sample_view.t_rx` and
core update time from the step's session clock. Each input port keeps arrival
FIFO order, including backwards source timestamps; merging ports never repairs
that data. `time_source = "input"` is explicit replay mode, with per-sample
updates and state ticks derived from source time. It is not a claim of arbitrary
batching equivalence with a real ROS callback/timer schedule.

Tests include the existing pure-core/input-timing cases, the shared vision
publication predicate, byte-exact replay of state/vision events against the core
(including an observation gap), and backwards timestamps on each port in each
time mode. The native replay fixture and reference belong here. The runtime's
old `plugins/est-rigid-state` implementation is retired; its host integration may
consume this package's built plugin and reference executable as test artifacts.
