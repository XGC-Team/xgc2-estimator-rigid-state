#pragma once

#include <algorithm>
#include <cmath>

#include "estimator_vrpn_px4_rotor_state/common/types.h"

namespace estimator_vrpn_px4_rotor_state::config_utils {

inline double finiteOr(double value, double fallback) {
    return std::isfinite(value) ? value : fallback;
}

inline double positiveOr(double value, double fallback) {
    return std::isfinite(value) && value > 0.0 ? value : fallback;
}

inline double nonNegativeOr(double value, double fallback) {
    return std::isfinite(value) && value >= 0.0 ? value : fallback;
}

inline void normalizeConfig(VrpnPx4RotorStateEstimatorConfig& config) {
    config.loop_rate_hz = std::min(positiveOr(config.loop_rate_hz, 1000.0), 2000.0);
    config.state_publish_rate_hz =
        std::min(positiveOr(config.state_publish_rate_hz, 100.0), config.loop_rate_hz);
    config.vision_publish_rate_hz = nonNegativeOr(config.vision_publish_rate_hz, 30.0);
    if (config.vision_publish_rate_hz > 0.0) {
        config.vision_publish_rate_hz = std::min(config.vision_publish_rate_hz, config.loop_rate_hz);
    }

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
    config.vrpn_velocity_noise_std = positiveOr(config.vrpn_velocity_noise_std, 0.05);
    config.gyro_bias_random_walk_std = nonNegativeOr(config.gyro_bias_random_walk_std, 1.0e-4);
    config.accel_bias_random_walk_std = nonNegativeOr(config.accel_bias_random_walk_std, 1.0e-3);
    config.extrinsic_position_random_walk_std =
        nonNegativeOr(config.extrinsic_position_random_walk_std, 1.0e-5);
    config.extrinsic_orientation_random_walk_std =
        nonNegativeOr(config.extrinsic_orientation_random_walk_std, 1.0e-5);
    config.innovation_position_gate_m = positiveOr(config.innovation_position_gate_m, 1.5);
    config.innovation_orientation_gate_rad =
        positiveOr(config.innovation_orientation_gate_rad, 0.8);
    config.velocity_innovation_gate_mps = positiveOr(config.velocity_innovation_gate_mps, 3.0);
    config.pose_nis_gate = positiveOr(config.pose_nis_gate, 22.5);
    config.covariance_high_threshold = positiveOr(config.covariance_high_threshold, 100.0);
    config.max_propagation_dt_s = positiveOr(config.max_propagation_dt_s, 0.01);
    config.initial_position_variance = positiveOr(config.initial_position_variance, 0.01);
    config.initial_velocity_variance = positiveOr(config.initial_velocity_variance, 0.1);
    config.initial_orientation_variance = positiveOr(config.initial_orientation_variance, 0.01);
    config.initial_gyro_bias_variance = positiveOr(config.initial_gyro_bias_variance, 0.01);
    config.initial_accel_bias_variance = positiveOr(config.initial_accel_bias_variance, 0.1);
    if (config.inertial_buffer_capacity == 0u) {
        config.inertial_buffer_capacity = 1u;
    }
    config.inertial_buffer_capacity = std::min<std::size_t>(config.inertial_buffer_capacity, 256u);
}

}  // namespace estimator_vrpn_px4_rotor_state::config_utils
