#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
work_dir="${RUNNER_TEMP:-/tmp}/estimator-rigid-state-compliance"
install_root="${RUNNER_TEMP:-/tmp}/estimator-rigid-state-install-root"

if grep -Eq '^[[:space:]]*continue-on-error:[[:space:]]*true' \
  "${REPO_ROOT}/.github/workflows/ci.yml"; then
  echo "CI quality/test jobs must fail closed" >&2
  exit 1
fi

rm -rf "$work_dir" "$install_root"
mkdir -p "$work_dir/src/estimator-rigid-state"
rsync -a --delete "$REPO_ROOT/" "$work_dir/src/estimator-rigid-state/"

cd "$work_dir"
set +u
source /opt/ros/noetic/setup.bash
set -u
catkin_make run_tests_estimator_vrpn_px4_rotor_state
catkin_test_results
DESTDIR="$install_root" catkin_make install \
  -DCMAKE_INSTALL_PREFIX=/opt/ros/noetic \
  -DCMAKE_BUILD_TYPE=Release
bash "$SCRIPT_DIR/check_installed_core.sh" --prefix "$install_root/opt/ros/noetic"

# Prove that a source overlay cannot satisfy an incomplete selected installation.
consumer_fixture="$(mktemp -d -t xgc2-rigid-state-incomplete-XXXXXX)"
cp -a "$install_root/opt/ros/noetic/." "$consumer_fixture/"
rm -- "$consumer_fixture/include/estimator_vrpn_px4_rotor_state/vrpn_px4_rotor_state_estimator_runtime.h"
if CPATH="$REPO_ROOT/estimator_vrpn_px4_rotor_state/include" \
  C_INCLUDE_PATH="$REPO_ROOT/estimator_vrpn_px4_rotor_state/include" \
  CPLUS_INCLUDE_PATH="$REPO_ROOT/estimator_vrpn_px4_rotor_state/include" \
  bash "$SCRIPT_DIR/check_installed_core.sh" --prefix "$consumer_fixture" \
  >"$consumer_fixture/consumer.log" 2>&1; then
  echo 'core consumer accepted a source-only public header' >&2
  rm -rf -- "$consumer_fixture"
  exit 1
fi
grep -Fq 'vrpn_px4_rotor_state_estimator_runtime.h' "$consumer_fixture/consumer.log"
rm -rf -- "$consumer_fixture"
set +u
source devel/setup.bash
set -u
test "$(rospack find rigid_state_estimator_msgs)" = "/opt/ros/noetic/share/rigid_state_estimator_msgs"
test "$(rospack find estimator_vrpn_px4_rotor_state)" = "$work_dir/src/estimator-rigid-state/estimator_vrpn_px4_rotor_state"
test "$(rospack find estimator_vrpn_ugv_state)" = "$work_dir/src/estimator-rigid-state/estimator_vrpn_ugv_state"
roslaunch --files estimator_vrpn_px4_rotor_state vrpn_px4_rotor_state_estimator.launch >/tmp/xgc2-vrpn-px4-rotor-state-estimator-files.txt
roslaunch --files estimator_vrpn_ugv_state vrpn_ugv_state_estimator.launch >/tmp/xgc2-vrpn-ugv-state-estimator-files.txt
