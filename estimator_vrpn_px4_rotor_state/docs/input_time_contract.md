# Source time and input-event time

The ROS input producer keeps two separate clocks:

- `sample.stamp_sec` is the source measurement time, used by inertial
  propagation and delayed pose/velocity fusion. A zero ROS header stamp keeps
  the existing fallback to the receipt clock; nonzero stamps are not rewritten.
- `Event::timestamp` is the ROS receipt time of the callback. The runtime uses
  it for its current time and event-driven health evaluation. It must not use
  an arbitrarily old or future source sample as the health evaluation clock.

`input_timing::updateSampleTiming` marks `time_jump` for non-finite source or
receipt times, source timestamps more than 50 ms ahead of receipt, and backward
per-stream timestamps beyond the existing 1 ns duplicate tolerance. The 50 ms
bound matches the existing default in `xgc2_math::sampleTimeJumped`; it is not
latency compensation or a relaxation of Session clock admission policy.

An ordered stream whose entire clock is far in the future remains rejected,
including its first sample. Timing metadata is updated before replacing the
stored source stamp. Corrected source time first produces a backward-jump
sample; the next ordered, timely sample can clear the timing fault. This does
not reinitialize a filter or authorize the downstream controller to resume.

Delayed ordered samples keep their original stamps and are checked for age by
`health_checks`. Their mere arrival does not refresh their measurement age.
Duplicate samples retain the existing rate-estimate behavior.

The existing health policy consumes IMU/pose `time_jump`; the velocity filter
also rejects `VelocityMeasurement::time_jump`. This change does not alter the
Running/Coasting transition table, filter equations, or control-blocking flags.

Direct callers of `VrpnPx4RotorStateEstimatorRuntime::postInputEvent` remain
responsible for providing admitted sample metadata and a health-evaluation
clock in the event. This producer-side fix is not a complete replacement for
Session time authority, ongoing wall-clock-step monitoring, or a receive-age
watchdog during a paused ROS clock.

The added `RigidInputTiming` tests exercise real estimator sample types,
health classification, and the runtime entry boundary. A ROS transport-level
clock/queue fault-injection test and field timing validation are still needed
before deployment; the tests do not establish an end-to-end flight guarantee.
