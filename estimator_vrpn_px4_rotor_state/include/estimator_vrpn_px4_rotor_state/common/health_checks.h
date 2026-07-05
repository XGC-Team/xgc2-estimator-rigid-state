#pragma once

#include <xgc2_math/utils/sample_timing.hpp>

#include "estimator_vrpn_px4_rotor_state/common/types.h"

namespace estimator_vrpn_px4_rotor_state::health_checks {

inline HealthStatus classify(const VrpnPx4RotorStateEstimatorInput& input,
                             const VrpnPx4RotorStateEstimatorConfig& config, bool initialized,
                             bool self_check_requested, double covariance_trace,
                             uint32_t estimator_flags, double now_sec) {
    HealthStatus health;
    uint32_t flags = estimator_flags;

    if (!config.extrinsic_verified) {
        flags |= kExtrinsicUnverified;
    }

    if (!input.imu.received) {
        flags |= kImuMissing;
    }
    if (!input.vrpn_pose.received) {
        flags |= kVrpnMissing;
    }
    if (input.imu.received && !input.imu.valid) {
        flags |= kInvalidImu;
    }
    if (input.vrpn_pose.received && !input.vrpn_pose.valid) {
        flags |= kInvalidVrpn;
    }
    if (input.imu.time_jump || input.vrpn_pose.time_jump) {
        flags |= kTimeJump;
    }
    if (input.imu.last_dt_sec > config.max_time_jump_s ||
        input.vrpn_pose.last_dt_sec > config.max_time_jump_s) {
        flags |= kTimeJump;
    }

    const bool imu_stale =
        input.imu.received && xgc2_math::sampleStale(now_sec, input.imu.stamp_sec,
                                                     config.imu_timeout_s);
    const bool vrpn_stale =
        input.vrpn_pose.received && xgc2_math::sampleStale(now_sec, input.vrpn_pose.stamp_sec,
                                                           config.vrpn_timeout_s);
    const bool vrpn_loss_coastable =
        input.vrpn_pose.received && !xgc2_math::sampleStale(now_sec, input.vrpn_pose.stamp_sec,
                                                            config.coasting_timeout_s);
    if (imu_stale) {
        flags |= kImuStale;
    }
    if (vrpn_stale) {
        flags |= kVrpnStale;
    }
    if (xgc2_math::sampleRateLow(input.imu.received, input.imu.last_dt_sec,
                                 config.min_imu_rate_hz)) {
        flags |= kImuRateLow;
    }
    if (xgc2_math::sampleRateLow(input.vrpn_pose.received, input.vrpn_pose.last_dt_sec,
                                 config.min_vrpn_rate_hz)) {
        flags |= kVrpnRateLow;
    }
    if (covariance_trace > config.covariance_high_threshold) {
        flags |= kCovarianceHigh;
    }

    const bool imu_ready = input.imu.received && input.imu.valid && !input.imu.time_jump &&
                           input.imu.last_dt_sec <= config.max_time_jump_s &&
                           !imu_stale;
    const bool vrpn_ready =
        input.vrpn_pose.received && input.vrpn_pose.valid && !input.vrpn_pose.time_jump &&
        input.vrpn_pose.last_dt_sec <= config.max_time_jump_s && !vrpn_stale;

    health.imu_ready = imu_ready;
    health.vrpn_ready = vrpn_ready;
    health.flags = flags;

    if (self_check_requested) {
        health.condition = HealthCondition::kInputUnhealthy;
    } else if (!initialized) {
        health.condition = imu_ready && vrpn_ready ? HealthCondition::kInitializationReady
                                                   : HealthCondition::kInputUnhealthy;
    } else if (!imu_ready) {
        health.condition = HealthCondition::kInputUnhealthy;
    } else if (vrpn_ready) {
        health.condition = HealthCondition::kEstimationReady;
    } else if (vrpn_loss_coastable) {
        health.condition = HealthCondition::kVrpnLossCoastable;
        health.flags |= kCoasting;
    } else {
        health.condition = HealthCondition::kInputUnhealthy;
    }
    return health;
}

}  // namespace estimator_vrpn_px4_rotor_state::health_checks
