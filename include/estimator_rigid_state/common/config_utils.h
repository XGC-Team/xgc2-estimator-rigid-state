#pragma once

#include <algorithm>
#include <cmath>

#include "estimator_rigid_state/common/types.h"

namespace estimator_rigid_state::config_utils {

inline double finiteOr(double value, double fallback) {
    return std::isfinite(value) ? value : fallback;
}

inline double positiveOr(double value, double fallback) {
    return std::isfinite(value) && value > 0.0 ? value : fallback;
}

inline double clamp01(double value) {
    if (!std::isfinite(value)) {
        return 0.0;
    }
    return std::clamp(value, 0.0, 1.0);
}

inline void normalizeConfig(RigidStateEstimatorConfig& config) {
    config.loop_rate_hz = std::min(positiveOr(config.loop_rate_hz, 1000.0), 2000.0);
    config.state_publish_rate_hz =
        std::min(positiveOr(config.state_publish_rate_hz, 100.0), config.loop_rate_hz);
    config.vision_publish_rate_hz =
        std::min(positiveOr(config.vision_publish_rate_hz, 30.0), config.loop_rate_hz);

    config.gravity_mps2 = positiveOr(config.gravity_mps2, 9.8066);
    config.imu_timeout_s = positiveOr(config.imu_timeout_s, 0.05);
    config.vrpn_timeout_s = positiveOr(config.vrpn_timeout_s, 0.12);
    config.coasting_timeout_s =
        std::max(positiveOr(config.coasting_timeout_s, 0.5), config.vrpn_timeout_s);
    config.min_imu_rate_hz = positiveOr(config.min_imu_rate_hz, 25.0);
    config.min_vrpn_rate_hz = positiveOr(config.min_vrpn_rate_hz, 20.0);
    config.max_time_jump_s = positiveOr(config.max_time_jump_s, 0.25);

    config.accel_noise_std = positiveOr(config.accel_noise_std, 0.35);
    config.gyro_noise_std = positiveOr(config.gyro_noise_std, 0.03);
    config.vrpn_position_noise_std = positiveOr(config.vrpn_position_noise_std, 0.01);
    config.vrpn_orientation_noise_std = positiveOr(config.vrpn_orientation_noise_std, 0.01);
    config.position_update_gain = clamp01(config.position_update_gain);
    config.velocity_update_gain = clamp01(config.velocity_update_gain);
    config.orientation_update_gain = clamp01(config.orientation_update_gain);
    config.gyro_bias_update_gain = clamp01(config.gyro_bias_update_gain);
    config.accel_bias_update_gain = clamp01(config.accel_bias_update_gain);
    config.innovation_position_gate_m = positiveOr(config.innovation_position_gate_m, 1.5);
    config.innovation_orientation_gate_rad =
        positiveOr(config.innovation_orientation_gate_rad, 0.8);
    config.covariance_high_threshold = positiveOr(config.covariance_high_threshold, 100.0);
}

}  // namespace estimator_rigid_state::config_utils
