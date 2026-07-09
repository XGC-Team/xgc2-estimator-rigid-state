#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
ROS_DISTRO="${ROS_DISTRO:-melodic}"

require_command() {
  local command_name="$1"
  if ! command -v "${command_name}" >/dev/null 2>&1; then
    echo "missing required command: ${command_name}" >&2
    exit 1
  fi
}

if [[ ! -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]]; then
  echo "missing ROS setup: /opt/ros/${ROS_DISTRO}/setup.bash" >&2
  exit 1
fi

set +u
# shellcheck source=/dev/null
source "/opt/ros/${ROS_DISTRO}/setup.bash"
set -u

require_command clang-format
require_command clang-tidy
require_command catkin_make
require_command rsync

if git -C "${REPO_ROOT}" rev-parse --is-inside-work-tree >/dev/null 2>&1; then
  mapfile -d '' CXX_FILES < <(
    cd "${REPO_ROOT}"
    find estimator_vrpn_px4_rotor_state estimator_vrpn_ugv_state -type f \
      \( -name '*.cpp' -o -name '*.cc' -o -name '*.cxx' -o \
        -name '*.h' -o -name '*.hpp' -o -name '*.hh' -o -name '*.hxx' \) \
      -print0 | sort -z
  )
else
  mapfile -d '' CXX_FILES < <(
    cd "${REPO_ROOT}"
    find . -type f \( -name '*.cpp' -o -name '*.cc' -o -name '*.cxx' -o -name '*.h' -o -name '*.hpp' -o -name '*.hh' -o -name '*.hxx' \) -print0 |
      sed -z 's#^\./##'
  )
fi

if [[ "${#CXX_FILES[@]}" -eq 0 ]]; then
  echo "no C++ files found" >&2
  exit 1
fi

echo "Running clang-format..."
(
  cd "${REPO_ROOT}"
  tmp_format_dir="$(mktemp -d)"
  trap 'rm -rf "${tmp_format_dir}"' EXIT
  for file in "${CXX_FILES[@]}"; do
    mkdir -p "${tmp_format_dir}/$(dirname "${file}")"
    clang-format "${file}" > "${tmp_format_dir}/${file}"
    diff -u "${file}" "${tmp_format_dir}/${file}"
  done
)

WORK_DIR="${RUNNER_TEMP:-${TMPDIR:-/tmp}}/xgc2-rigid-state-cpp-quality"
rm -rf "${WORK_DIR}"
mkdir -p "${WORK_DIR}/src/estimator-rigid-state"
rsync -a --delete --exclude '.git' "${REPO_ROOT}/" "${WORK_DIR}/src/estimator-rigid-state/"

echo "Generating compile_commands.json..."
(
  cd "${WORK_DIR}"
  catkin_make \
    -DCMAKE_EXPORT_COMPILE_COMMANDS=ON \
    -DCMAKE_BUILD_TYPE=Debug
)

"${REPO_ROOT}/.xgc2/scripts/check_core_libraries.sh" --prefix "${WORK_DIR}/devel"

echo "Running clang-tidy..."
TIDY_SOURCES=(
  "${WORK_DIR}/src/estimator-rigid-state/estimator_vrpn_px4_rotor_state/src/vrpn_px4_rotor_state_estimator_main.cpp"
  "${WORK_DIR}/src/estimator-rigid-state/estimator_vrpn_px4_rotor_state/src/vrpn_px4_rotor_state_estimator_node.cpp"
  "${WORK_DIR}/src/estimator-rigid-state/estimator_vrpn_px4_rotor_state/src/vrpn_px4_rotor_state_estimator_runtime.cpp"
  "${WORK_DIR}/src/estimator-rigid-state/estimator_vrpn_ugv_state/src/vrpn_ugv_state_estimator_main.cpp"
  "${WORK_DIR}/src/estimator-rigid-state/estimator_vrpn_ugv_state/src/vrpn_ugv_state_estimator_node.cpp"
  "${WORK_DIR}/src/estimator-rigid-state/estimator_vrpn_ugv_state/src/vrpn_ugv_state_estimator_runtime.cpp"
)

clang-tidy \
  -p "${WORK_DIR}/build" \
  -header-filter="^${WORK_DIR}/src/estimator-rigid-state/(estimator_vrpn_px4_rotor_state|estimator_vrpn_ugv_state)/(include|src|test)/" \
  -quiet \
  "${TIDY_SOURCES[@]}"

echo "C++ quality check passed"
