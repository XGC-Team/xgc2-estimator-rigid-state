#pragma once

#include <Eigen/Geometry>
#include <cmath>

#include "estimator_vrpn_px4_rotor_state/common/types.h"
#include "xgc2_math/geometry/se3.hpp"

namespace estimator_vrpn_px4_rotor_state::math_utils {

inline bool isFinite(double value) {
    return std::isfinite(value);
}

inline bool isFinite(const Eigen::Vector3d& value) {
    return value.allFinite();
}

inline bool isFinite(const Eigen::Quaterniond& value) {
    return std::isfinite(value.w()) && std::isfinite(value.x()) && std::isfinite(value.y()) &&
           std::isfinite(value.z());
}

inline Eigen::Quaterniond normalized(const Eigen::Quaterniond& q) {
    return xgc2_math::normalizedQuaternion(q);
}

inline Eigen::Quaterniond rpyToQuaternion(const Eigen::Vector3d& rpy) {
    return xgc2_math::rpyToQuaternion(rpy);
}

}  // namespace estimator_vrpn_px4_rotor_state::math_utils
