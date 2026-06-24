#!/usr/bin/env bash
set -euo pipefail

ROS_DISTRO="${ROS_DISTRO:-noetic}"
set +u
# shellcheck source=/dev/null
source "/opt/ros/${ROS_DISTRO}/setup.bash"
set -u

dpkg -s ros-noetic-xgc2-estimator-rigid-state >/dev/null
dpkg -s libxgc2-observer-dev >/dev/null
dpkg -s libxgc2-state-machine-dev >/dev/null
dpkg -s ros-noetic-xgc2-ros1-utils >/dev/null
test -f /usr/include/xgc2_observer/recursive_least_squares.hpp
test -f /usr/include/xgc2_observer/inertial_pose_eskf.hpp
test -f /usr/include/state_machine/state_machine.hpp
test -f /usr/include/state_machine/runtime/event_dispatcher.hpp
test "$(rospack find estimator_rigid_state)" = "/opt/ros/${ROS_DISTRO}/share/estimator_rigid_state"
test -f "/opt/ros/${ROS_DISTRO}/share/estimator_rigid_state/config/rigid_state_estimator.yaml"
test -f "/opt/ros/${ROS_DISTRO}/share/estimator_rigid_state/launch/rigid_state_estimator.launch"
test -f "/opt/ros/${ROS_DISTRO}/share/estimator_rigid_state/msg/RigidStateEstimate.msg"
test -f "/opt/ros/${ROS_DISTRO}/lib/python3/dist-packages/estimator_rigid_state/msg/_RigidStateEstimate.py"
test -f "/opt/ros/${ROS_DISTRO}/lib/libestimator_rigid_state_core.so"
test -f "/opt/ros/${ROS_DISTRO}/lib/libestimator_rigid_state_ros.so"
rosmsg show estimator_rigid_state/RigidStateEstimate | grep -q '^uint8 estimator_state$'
rosmsg show estimator_rigid_state/RigidStateEstimate | grep -q '^geometry_msgs/Vector3 angular_velocity$'
python3 - <<'PY'
from estimator_rigid_state.msg import RigidStateEstimate

msg = RigidStateEstimate()
msg.estimator_state = RigidStateEstimate.STATE_RUNNING
msg.flags = 0
assert msg.estimator_state == RigidStateEstimate.STATE_RUNNING
PY
roslaunch --files estimator_rigid_state rigid_state_estimator.launch >/tmp/xgc2-rigid-state-estimator-files.txt

while IFS= read -r file; do
  if ! file -b "${file}" | grep -q '^ELF'; then
    continue
  fi
  if ! ldd "${file}" | awk '/not found/ {missing=1} END {exit missing ? 1 : 0}'; then
    echo "missing shared library dependency in ${file}" >&2
    ldd "${file}" >&2 || true
    exit 1
  fi
done < <(find "/opt/ros/${ROS_DISTRO}/lib/estimator_rigid_state" \
  "/opt/ros/${ROS_DISTRO}/lib/libestimator_rigid_state_core.so" \
  "/opt/ros/${ROS_DISTRO}/lib/libestimator_rigid_state_ros.so" -type f 2>/dev/null | sort -u)

echo "Installed package check passed"
