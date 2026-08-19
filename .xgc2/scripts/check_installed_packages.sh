#!/usr/bin/env bash
set -euo pipefail

ROS_DISTRO="${ROS_DISTRO:-noetic}"
set +u
# shellcheck source=/dev/null
source "/opt/ros/${ROS_DISTRO}/setup.bash"
set -u

dpkg -s "ros-${ROS_DISTRO}-xgc2-estimator-rigid-state" >/dev/null
dpkg -s "ros-${ROS_DISTRO}-xgc2-estimator-rigid-state-msgs" >/dev/null
dpkg -s libxgc2-math-dev >/dev/null
dpkg -s libxgc2-state-machine-dev >/dev/null
dpkg -s "ros-${ROS_DISTRO}-xgc2-ros1-utils" >/dev/null
test -f /usr/include/xgc2_math/estimation/recursive_least_squares.hpp
test -f /usr/include/xgc2_math/estimation/pose3_inertial_eskf.hpp
test -f /usr/include/state_machine/state_machine.hpp
test -f /usr/include/state_machine/runtime/event_dispatcher.hpp
test "$(rospack find rigid_state_estimator_msgs)" = "/opt/ros/${ROS_DISTRO}/share/rigid_state_estimator_msgs"
test "$(rospack find estimator_vrpn_px4_rotor_state)" = "/opt/ros/${ROS_DISTRO}/share/estimator_vrpn_px4_rotor_state"
test "$(rospack find estimator_vrpn_ugv_state)" = "/opt/ros/${ROS_DISTRO}/share/estimator_vrpn_ugv_state"
test -f "/opt/ros/${ROS_DISTRO}/share/rigid_state_estimator_msgs/msg/RigidStateEstimate.msg"
test -f "/opt/ros/${ROS_DISTRO}/share/rigid_state_estimator_msgs/msg/PlanarStateEstimate.msg"
test -f "/opt/ros/${ROS_DISTRO}/include/rigid_state_estimator_msgs/RigidStateEstimate.h"
test -f "/opt/ros/${ROS_DISTRO}/include/rigid_state_estimator_msgs/PlanarStateEstimate.h"
python_msg=""
for pyroot in \
  "/opt/ros/${ROS_DISTRO}/lib/python3/dist-packages" \
  "/opt/ros/${ROS_DISTRO}/lib/python2.7/dist-packages"; do
  if [[ -f "${pyroot}/rigid_state_estimator_msgs/msg/_RigidStateEstimate.py" ]]; then
    python_msg="${pyroot}"
    break
  fi
done
test -n "${python_msg}"
test -f "${python_msg}/rigid_state_estimator_msgs/msg/_PlanarStateEstimate.py"
test -f "/opt/ros/${ROS_DISTRO}/share/estimator_vrpn_px4_rotor_state/config/vrpn_px4_rotor_state_estimator.yaml"
test -f "/opt/ros/${ROS_DISTRO}/share/estimator_vrpn_px4_rotor_state/config/vrpn_ugv_rigid_state_estimator.yaml"
test -f "/opt/ros/${ROS_DISTRO}/share/estimator_vrpn_px4_rotor_state/launch/vrpn_px4_rotor_state_estimator.launch"
test -f "/opt/ros/${ROS_DISTRO}/share/estimator_vrpn_px4_rotor_state/launch/vrpn_ugv_rigid_state_estimator.launch"
test -f "/opt/ros/${ROS_DISTRO}/lib/libestimator_vrpn_px4_rotor_state_core.so"
test -f "/opt/ros/${ROS_DISTRO}/lib/libestimator_vrpn_px4_rotor_state_ros.so"
test -f "/opt/ros/${ROS_DISTRO}/share/estimator_vrpn_ugv_state/launch/vrpn_ugv_state_estimator.launch"
test ! -f "/opt/ros/${ROS_DISTRO}/share/estimator_vrpn_ugv_state/config/vrpn_ugv_state_estimator.yaml"
test ! -f "/opt/ros/${ROS_DISTRO}/lib/libestimator_vrpn_ugv_state_core.so"
test ! -f "/opt/ros/${ROS_DISTRO}/lib/libestimator_vrpn_ugv_state_ros.so"
test -x "/opt/ros/${ROS_DISTRO}/lib/estimator_vrpn_px4_rotor_state/vrpn_px4_rotor_state_estimator_node"
test -x "/opt/ros/${ROS_DISTRO}/lib/estimator_vrpn_px4_rotor_state/request_highres_imu_rate.py"
test ! -e "/opt/ros/${ROS_DISTRO}/lib/estimator_vrpn_ugv_state/vrpn_ugv_state_estimator_node"
rosmsg show rigid_state_estimator_msgs/RigidStateEstimate | grep -q '^uint8 estimator_state$'
rosmsg show rigid_state_estimator_msgs/RigidStateEstimate | grep -q '^geometry_msgs/Vector3 angular_velocity$'
rosmsg show rigid_state_estimator_msgs/PlanarStateEstimate | grep -q '^uint8 estimator_state$'
rosmsg show rigid_state_estimator_msgs/PlanarStateEstimate | grep -q '^geometry_msgs/Vector3 angular_velocity$'
msg_python="python3"
if [[ "${ROS_DISTRO}" == "melodic" ]]; then
  msg_python="python2"
fi
"${msg_python}" - <<'PY'
from rigid_state_estimator_msgs.msg import PlanarStateEstimate
from rigid_state_estimator_msgs.msg import RigidStateEstimate

msg = RigidStateEstimate()
msg.estimator_state = RigidStateEstimate.STATE_RUNNING
msg.flags = 0
assert msg.estimator_state == 3
assert RigidStateEstimate.STATE_RUNNING == 3
assert PlanarStateEstimate.STATE_RUNNING == 2
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
  "/opt/ros/${ROS_DISTRO}/lib/libestimator_vrpn_px4_rotor_state_ros.so" -type f 2>/dev/null | sort -u)

echo "Installed package check passed"
