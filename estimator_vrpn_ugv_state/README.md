# VRPN UGV State Estimator (alias)

This package no longer runs a 2D EKF. It only launches the shared
`Pose3InertialEskf` node from `estimator_vrpn_px4_rotor_state` with a UGV
config (`/imu/data`, HI226 timeouts, no PX4 vision).

UGV controllers subscribe to `RigidStateEstimate` and project to
`x, y, yaw, speed, yaw_rate` at the control boundary. The UGV config does
not subscribe to VRPN twist (pose dirty-diff). Velocity comes from IMU
propagation plus VRPN pose updates. Innovation gates exist only to drop
a wrong tracker lock so IMU can coast; they must not reject healthy VRPN.

```bash
roslaunch estimator_vrpn_ugv_state vrpn_ugv_state_estimator.launch ns:=ugv1
```

Equivalent:

```bash
roslaunch estimator_vrpn_px4_rotor_state vrpn_ugv_rigid_state_estimator.launch ns:=ugv1
```

LIO pose source:

```bash
roslaunch estimator_vrpn_ugv_state vrpn_ugv_state_estimator.launch \
  ns:=ugv1 pose_source:=lio pose_topic:=/Odometry
```
