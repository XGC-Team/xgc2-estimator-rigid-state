#!/usr/bin/env bash
set -euo pipefail

# Dependencies retain their declared CMake prefixes; ambient source includes cannot fill holes.
unset CPATH C_INCLUDE_PATH CPLUS_INCLUDE_PATH

prefix="/opt/ros/${ROS_DISTRO:-noetic}"
if [[ $# == 2 && "$1" == --prefix ]]; then
  prefix="$2"
elif [[ $# != 0 ]]; then
  echo "usage: $0 [--prefix PREFIX]" >&2
  exit 2
fi

work="$(mktemp -d -t xgc2-rigid-state-consumer-XXXXXX)"
trap 'rm -rf -- "$work"' EXIT
cat > "$work/CMakeLists.txt" <<'CMAKE'
cmake_minimum_required(VERSION 3.10)
project(rigid_state_installed_consumer LANGUAGES CXX)
# Require this install tree's export, even if a different version is already on the host.
find_package(RigidStateEstimator REQUIRED CONFIG
  PATHS "${RIGID_STATE_PREFIX}/lib/cmake/RigidStateEstimator" NO_DEFAULT_PATH)
add_executable(consumer main.cpp)
target_link_libraries(consumer PRIVATE RigidStateEstimator::Core)
CMAKE
cat > "$work/main.cpp" <<'CPP'
#include <estimator_vrpn_px4_rotor_state/vrpn_px4_rotor_state_estimator_runtime.h>

int main() {
    estimator_vrpn_px4_rotor_state::VrpnPx4RotorStateEstimatorRuntime runtime;
    (void)runtime.snapshotOutput();
    return 0;
}
CPP
mkdir -p "$work/build"
(cd "$work/build" && cmake "$work" "-DRIGID_STATE_PREFIX=$prefix")
cmake --build "$work/build" -- -j2
"$work/build/consumer"
echo 'Installed RigidStateEstimator::Core consumer passed'
