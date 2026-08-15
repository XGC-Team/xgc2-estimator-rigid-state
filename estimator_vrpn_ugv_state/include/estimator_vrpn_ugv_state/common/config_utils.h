#pragma once

#include <algorithm>
#include <cmath>

#include "estimator_vrpn_ugv_state/common/types.h"

namespace estimator_vrpn_ugv_state::config_utils {

inline double positiveOr(double value, double fallback) {
    return std::isfinite(value) && value > 0.0 ? value : fallback;
}

inline void normalizeConfig(VrpnUgvStateEstimatorConfig& config) {
    config.loop_rate_hz = std::min(positiveOr(config.loop_rate_hz, 1000.0), 2000.0);
    config.state_publish_rate_hz =
        std::min(positiveOr(config.state_publish_rate_hz, 100.0), config.loop_rate_hz);

    config.field_to_world = xgc2_math::normalized(config.field_to_world);
    config.body_to_vrpn_marker = xgc2_math::normalized(config.body_to_vrpn_marker);
    config.imu_timeout_s = positiveOr(config.imu_timeout_s, 0.2);
    config.vrpn_timeout_s = positiveOr(config.vrpn_timeout_s, 0.12);
    config.coasting_timeout_s =
        std::max(positiveOr(config.coasting_timeout_s, 0.5), config.vrpn_timeout_s);
    config.min_imu_rate_hz = positiveOr(config.min_imu_rate_hz, 5.0);
    config.min_vrpn_rate_hz = positiveOr(config.min_vrpn_rate_hz, 20.0);
    config.max_time_jump_s = positiveOr(config.max_time_jump_s, 0.25);
    if (!std::isfinite(config.max_pose_delay_s) || config.max_pose_delay_s < 0.0) {
        config.max_pose_delay_s = 0.0;
    }

    config.gyro_noise_std = positiveOr(config.gyro_noise_std, 0.03);
    config.accel_noise_std = positiveOr(config.accel_noise_std, 0.35);
    config.gyro_bias_random_walk_std = positiveOr(config.gyro_bias_random_walk_std, 1.0e-4);
    config.accel_bias_random_walk_std = positiveOr(config.accel_bias_random_walk_std, 1.0e-3);
    config.vrpn_position_noise_std = positiveOr(config.vrpn_position_noise_std, 0.01);
    config.vrpn_yaw_noise_std = positiveOr(config.vrpn_yaw_noise_std, 0.01);
    config.innovation_position_gate_m = positiveOr(config.innovation_position_gate_m, 1.5);
    config.innovation_yaw_gate_rad = positiveOr(config.innovation_yaw_gate_rad, 0.8);
    config.covariance_high_threshold = positiveOr(config.covariance_high_threshold, 100.0);
    config.max_propagation_dt_s = positiveOr(config.max_propagation_dt_s, 0.05);
}

inline xgc2_math::Pose2InertialEskfConfig estimatorConfigFromRuntimeConfig(
    const VrpnUgvStateEstimatorConfig& config) {
    xgc2_math::Pose2InertialEskfConfig result;
    result.measurement_frame_to_world = config.field_to_world;
    result.body_to_marker = config.body_to_vrpn_marker;
    result.estimate_extrinsic = false;
    result.gyro_noise_std = config.gyro_noise_std;
    result.accel_noise_std = config.accel_noise_std;
    result.gyro_bias_random_walk_std = config.gyro_bias_random_walk_std;
    result.accel_bias_random_walk_std = config.accel_bias_random_walk_std;
    result.pose_position_noise_std = config.vrpn_position_noise_std;
    result.pose_yaw_noise_std = config.vrpn_yaw_noise_std;
    result.innovation_position_gate_m = config.innovation_position_gate_m;
    result.innovation_yaw_gate_rad = config.innovation_yaw_gate_rad;
    result.covariance_high_threshold = config.covariance_high_threshold;
    result.max_propagation_dt_s = config.max_propagation_dt_s;
    result.inertial_buffer_capacity = config.inertial_buffer_capacity;
    result.vrpn_health = config.vrpn_health;
    return result;
}

}  // namespace estimator_vrpn_ugv_state::config_utils
