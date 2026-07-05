# XGC2 Rigid State Estimator

ROS1 VRPN PX4 rotor state estimator for XGC2 UAV control stacks.

The package subscribes to raw IMU and Gazebo/VRPN pose, runs the shared
`xgc2_math::InertialPoseEskf` core estimator, and publishes:

- `alg/state_estimator/state`: `rigid_state_estimator_msgs/RigidStateEstimate`
- `mavros/vision_pose/pose`: corrected VRPN pose for PX4 vision input

The nonlinear estimation algorithm is owned by `libxgc2-math-dev`; this
package owns ROS input events, the estimator state machine, and asynchronous
output publication.

## Install

```bash
sudo apt update
sudo apt install ros-noetic-xgc2-estimator-rigid-state
```

## Launch

```bash
source /opt/ros/noetic/setup.bash
roslaunch estimator_vrpn_px4_rotor_state vrpn_px4_rotor_state_estimator.launch ns:=uav1
```

Parameters are loaded from
`config/vrpn_px4_rotor_state_estimator.yaml` under the node namespace
`/<ns>/vrpn_px4_rotor_state_estimator`.

When launched with the default `request_highres_imu:=true`, the estimator also
requests MAVLink `HIGHRES_IMU(105)` at `250 Hz` through
`/<ns>/mavros/set_message_interval`. This keeps the raw IMU telemetry rate
owned by the state-estimation consumer instead of the simulator wrapper.
