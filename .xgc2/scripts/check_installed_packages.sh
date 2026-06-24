#!/usr/bin/env bash
set -euo pipefail

ROS_DISTRO="${ROS_DISTRO:-noetic}"
set +u
# shellcheck source=/dev/null
source "/opt/ros/${ROS_DISTRO}/setup.bash"
set -u

dpkg -s ros-noetic-xgc2-estimator-rigid-state >/dev/null
dpkg -s libxgc2-math-dev >/dev/null
dpkg -s libxgc2-state-machine-dev >/dev/null
dpkg -s ros-noetic-xgc2-ros1-utils >/dev/null
test -f /usr/include/xgc2_math/estimation/recursive_least_squares.hpp
test -f /usr/include/xgc2_math/estimation/inertial_pose_eskf.hpp
test -f /usr/include/xgc2_math/estimation/planar_inertial_eskf.hpp
test -f /usr/include/state_machine/state_machine.hpp
test -f /usr/include/state_machine/runtime/event_dispatcher.hpp
test "$(rospack find estimator_vrpn_px4_rotor_state)" = "/opt/ros/${ROS_DISTRO}/share/estimator_vrpn_px4_rotor_state"
test "$(rospack find estimator_vrpn_ugv_state)" = "/opt/ros/${ROS_DISTRO}/share/estimator_vrpn_ugv_state"
test -f "/opt/ros/${ROS_DISTRO}/share/estimator_vrpn_px4_rotor_state/config/vrpn_px4_rotor_state_estimator.yaml"
test -f "/opt/ros/${ROS_DISTRO}/share/estimator_vrpn_px4_rotor_state/launch/vrpn_px4_rotor_state_estimator.launch"
test -f "/opt/ros/${ROS_DISTRO}/share/estimator_vrpn_px4_rotor_state/msg/RigidStateEstimate.msg"
test -f "/opt/ros/${ROS_DISTRO}/lib/python3/dist-packages/estimator_vrpn_px4_rotor_state/msg/_RigidStateEstimate.py"
test -f "/opt/ros/${ROS_DISTRO}/lib/libestimator_vrpn_px4_rotor_state_core.so"
test -f "/opt/ros/${ROS_DISTRO}/lib/libestimator_vrpn_px4_rotor_state_ros.so"
test -f "/opt/ros/${ROS_DISTRO}/share/estimator_vrpn_ugv_state/config/vrpn_ugv_state_estimator.yaml"
test -f "/opt/ros/${ROS_DISTRO}/share/estimator_vrpn_ugv_state/launch/vrpn_ugv_state_estimator.launch"
test -f "/opt/ros/${ROS_DISTRO}/share/estimator_vrpn_ugv_state/msg/PlanarStateEstimate.msg"
test -f "/opt/ros/${ROS_DISTRO}/lib/python3/dist-packages/estimator_vrpn_ugv_state/msg/_PlanarStateEstimate.py"
test -f "/opt/ros/${ROS_DISTRO}/lib/libestimator_vrpn_ugv_state_core.so"
test -f "/opt/ros/${ROS_DISTRO}/lib/libestimator_vrpn_ugv_state_ros.so"
rosmsg show estimator_vrpn_px4_rotor_state/RigidStateEstimate | grep -q '^uint8 estimator_state$'
rosmsg show estimator_vrpn_px4_rotor_state/RigidStateEstimate | grep -q '^geometry_msgs/Vector3 angular_velocity$'
rosmsg show estimator_vrpn_ugv_state/PlanarStateEstimate | grep -q '^uint8 estimator_state$'
rosmsg show estimator_vrpn_ugv_state/PlanarStateEstimate | grep -q '^geometry_msgs/Vector3 angular_velocity$'
python3 - <<'PY'
from estimator_vrpn_px4_rotor_state.msg import RigidStateEstimate
from estimator_vrpn_ugv_state.msg import PlanarStateEstimate

msg = RigidStateEstimate()
msg.estimator_state = RigidStateEstimate.STATE_RUNNING
msg.flags = 0
assert msg.estimator_state == RigidStateEstimate.STATE_RUNNING
planar = PlanarStateEstimate()
planar.estimator_state = PlanarStateEstimate.STATE_RUNNING
assert planar.estimator_state == PlanarStateEstimate.STATE_RUNNING
PY
roslaunch --files estimator_vrpn_px4_rotor_state vrpn_px4_rotor_state_estimator.launch >/tmp/xgc2-vrpn-px4-rotor-state-estimator-files.txt
roslaunch --files estimator_vrpn_ugv_state vrpn_ugv_state_estimator.launch >/tmp/xgc2-vrpn-ugv-state-estimator-files.txt

while IFS= read -r file; do
  if ! file -b "${file}" | grep -q '^ELF'; then
    continue
  fi
  if ! ldd "${file}" | awk '/not found/ {missing=1} END {exit missing ? 1 : 0}'; then
    echo "missing shared library dependency in ${file}" >&2
    ldd "${file}" >&2 || true
    exit 1
  fi
done < <(find "/opt/ros/${ROS_DISTRO}/lib/estimator_vrpn_px4_rotor_state" \
  "/opt/ros/${ROS_DISTRO}/lib/libestimator_vrpn_px4_rotor_state_core.so" \
  "/opt/ros/${ROS_DISTRO}/lib/libestimator_vrpn_px4_rotor_state_ros.so" \
  "/opt/ros/${ROS_DISTRO}/lib/estimator_vrpn_ugv_state" \
  "/opt/ros/${ROS_DISTRO}/lib/libestimator_vrpn_ugv_state_core.so" \
  "/opt/ros/${ROS_DISTRO}/lib/libestimator_vrpn_ugv_state_ros.so" -type f 2>/dev/null | sort -u)

echo "Installed package check passed"
