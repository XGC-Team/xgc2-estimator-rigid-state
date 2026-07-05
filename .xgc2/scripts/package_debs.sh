#!/usr/bin/env bash
set -euo pipefail

INSTALL_ROOT=""
OUTPUT_DIR=""
ROS_DISTRO="${ROS_DISTRO:-noetic}"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
MSG_PACKAGE="ros-${ROS_DISTRO}-xgc2-estimator-rigid-state-msgs"
IMPL_PACKAGE="ros-${ROS_DISTRO}-xgc2-estimator-rigid-state"
MSG_ROS_PACKAGES=(
  rigid_state_estimator_msgs
)
IMPL_ROS_PACKAGES=(
  estimator_vrpn_px4_rotor_state
  estimator_vrpn_ugv_state
)
ROS_LIBRARIES=(
  libestimator_vrpn_px4_rotor_state_core.so
  libestimator_vrpn_px4_rotor_state_ros.so
  libestimator_vrpn_ugv_state_core.so
  libestimator_vrpn_ugv_state_ros.so
)

product_version() {
  awk -F': *' '/^version:[[:space:]]*/ {print $2; exit}' "${REPO_ROOT}/.xgc2/product.yml"
}

VERSION="${PACKAGE_VERSION:-$(product_version)}"

while [[ $# -gt 0 ]]; do
  case "$1" in
    --install-root)
      INSTALL_ROOT="$2"
      shift 2
      ;;
    --output-dir)
      OUTPUT_DIR="$2"
      shift 2
      ;;
    *)
      echo "unknown argument: $1" >&2
      exit 1
      ;;
  esac
done

if [[ -z "${INSTALL_ROOT}" || -z "${OUTPUT_DIR}" ]]; then
  echo "--install-root and --output-dir are required" >&2
  exit 1
fi

if [[ -z "${VERSION}" ]]; then
  echo "package version is missing" >&2
  exit 1
fi

ARCH="$(dpkg --print-architecture)"
PREFIX="/opt/ros/${ROS_DISTRO}"
PREFIX_ROOT="${INSTALL_ROOT}${PREFIX}"
BUILD_DIR="$(mktemp -d)"

cleanup() {
  rm -rf "${BUILD_DIR}"
}
trap cleanup EXIT

mkdir -p "${OUTPUT_DIR}"
rm -f "${OUTPUT_DIR}"/*.deb

copy_path() {
  local src="$1"
  local dst_root="$2"
  if [[ -e "${src}" ]]; then
    mkdir -p "${dst_root}$(dirname "${src#${INSTALL_ROOT}}")"
    cp -a "${src}" "${dst_root}${src#${INSTALL_ROOT}}"
  fi
}

copy_ros_package() {
  local ros_package="$1"
  local dst_root="$2"
  copy_path "${PREFIX_ROOT}/share/${ros_package}" "${dst_root}"
  copy_path "${PREFIX_ROOT}/lib/${ros_package}" "${dst_root}"
  copy_path "${PREFIX_ROOT}/lib/python3/dist-packages/${ros_package}" "${dst_root}"
  copy_path "${PREFIX_ROOT}/include/${ros_package}" "${dst_root}"
  copy_path "${PREFIX_ROOT}/share/common-lisp/ros/${ros_package}" "${dst_root}"
  copy_path "${PREFIX_ROOT}/share/gennodejs/ros/${ros_package}" "${dst_root}"
  copy_path "${PREFIX_ROOT}/share/roseus/ros/${ros_package}" "${dst_root}"
}

msg_pkg_root="${BUILD_DIR}/${MSG_PACKAGE}"
mkdir -p "${msg_pkg_root}"
for ros_package in "${MSG_ROS_PACKAGES[@]}"; do
  copy_ros_package "${ros_package}" "${msg_pkg_root}"
done

mkdir -p "${msg_pkg_root}/DEBIAN" "${msg_pkg_root}/usr/share/doc/${MSG_PACKAGE}"
cat > "${msg_pkg_root}/DEBIAN/control" <<EOF
Package: ${MSG_PACKAGE}
Version: ${VERSION}
Section: misc
Priority: optional
Architecture: ${ARCH}
Maintainer: XGC2 <apt@example.com>
Depends: ros-${ROS_DISTRO}-message-runtime, ros-${ROS_DISTRO}-std-msgs, ros-${ROS_DISTRO}-geometry-msgs
Description: XGC2 rigid state estimator message interfaces
EOF
printf 'xgc2-estimator-rigid-state message package\n' > "${msg_pkg_root}/usr/share/doc/${MSG_PACKAGE}/README"
chmod 0755 "${msg_pkg_root}/DEBIAN"

impl_pkg_root="${BUILD_DIR}/${IMPL_PACKAGE}"
mkdir -p "${impl_pkg_root}"
for ros_package in "${IMPL_ROS_PACKAGES[@]}"; do
  copy_ros_package "${ros_package}" "${impl_pkg_root}"
done

for ros_library in "${ROS_LIBRARIES[@]}"; do
  copy_path "${PREFIX_ROOT}/lib/${ros_library}" "${impl_pkg_root}"
done

mkdir -p "${impl_pkg_root}/DEBIAN" "${impl_pkg_root}/usr/share/doc/${IMPL_PACKAGE}"
cat > "${impl_pkg_root}/DEBIAN/control" <<EOF
Package: ${IMPL_PACKAGE}
Version: ${VERSION}
Section: misc
Priority: optional
Architecture: ${ARCH}
Maintainer: XGC2 <apt@example.com>
Depends: ${MSG_PACKAGE} (>= ${VERSION}), libxgc2-math-dev (>= 0.5.4-1), libxgc2-state-machine-dev (>= 0.1.2-1~focal), ros-noetic-xgc2-ros1-utils, ros-noetic-message-runtime, ros-noetic-roscpp, ros-noetic-std-msgs, ros-noetic-sensor-msgs, ros-noetic-geometry-msgs, ros-noetic-mavros-msgs, ros-noetic-rospy
Description: XGC2 VRPN/IMU rigid state estimation packages for rotor UAVs and UGVs
EOF
printf 'xgc2-estimator-rigid-state package\n' > "${impl_pkg_root}/usr/share/doc/${IMPL_PACKAGE}/README"
chmod 0755 "${impl_pkg_root}/DEBIAN"

fakeroot dpkg-deb --build "${msg_pkg_root}" "${OUTPUT_DIR}/${MSG_PACKAGE}_${VERSION}_${ARCH}.deb" >/dev/null
fakeroot dpkg-deb --build "${impl_pkg_root}" "${OUTPUT_DIR}/${IMPL_PACKAGE}_${VERSION}_${ARCH}.deb" >/dev/null
find "${OUTPUT_DIR}" -maxdepth 1 -type f -name '*.deb' -print | sort
