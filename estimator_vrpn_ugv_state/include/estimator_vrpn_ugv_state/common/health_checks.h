#pragma once

#include <cmath>
#include <limits>

#include "estimator_vrpn_ugv_state/common/types.h"

namespace estimator_vrpn_ugv_state::health_checks {

inline double sampleAge(double now_sec, double stamp_sec) {
    if (!std::isfinite(now_sec) || !std::isfinite(stamp_sec) || stamp_sec <= 0.0) {
        return std::numeric_limits<double>::infinity();
    }
    return now_sec - stamp_sec;
}

inline ::state_machine::EventId transitionEventFor(::state_machine::StateId state) {
    switch (state) {
        case state_type::SelfCheck:
            return event_type::HEALTH_TO_SELF_CHECK;
        case state_type::Running:
            return event_type::HEALTH_TO_RUNNING;
        case state_type::Coasting:
            return event_type::HEALTH_TO_COASTING;
        case state_type::Fault:
            return event_type::HEALTH_TO_FAULT;
        default:
            return event_type::HEALTH_TO_FAULT;
    }
}

inline HealthStatus classify(const VrpnUgvStateEstimatorInput& input,
                             const VrpnUgvStateEstimatorConfig& config, bool initialized,
                             bool fault_requested, double covariance_trace,
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
    if (input.imu.time_jump || input.vrpn_pose.time_jump ||
        input.imu.last_dt_sec > config.max_time_jump_s ||
        input.vrpn_pose.last_dt_sec > config.max_time_jump_s) {
        flags |= kTimeJump;
    }

    const double imu_age = sampleAge(now_sec, input.imu.stamp_sec);
    const double vrpn_age = sampleAge(now_sec, input.vrpn_pose.stamp_sec);
    if (input.imu.received && imu_age > config.imu_timeout_s) {
        flags |= kImuStale;
    }
    if (input.vrpn_pose.received && vrpn_age > config.vrpn_timeout_s) {
        flags |= kVrpnStale;
    }
    if (input.imu.estimated_rate_hz > 0.0 && input.imu.estimated_rate_hz < config.min_imu_rate_hz) {
        flags |= kImuRateLow;
    }
    if (input.vrpn_pose.estimated_rate_hz > 0.0 &&
        input.vrpn_pose.estimated_rate_hz < config.min_vrpn_rate_hz) {
        flags |= kVrpnRateLow;
    }
    if (covariance_trace > config.covariance_high_threshold) {
        flags |= kCovarianceHigh;
    }

    const bool imu_ready = input.imu.received && input.imu.valid && !input.imu.time_jump &&
                           input.imu.last_dt_sec <= config.max_time_jump_s &&
                           imu_age <= config.imu_timeout_s;
    const bool vrpn_ready =
        input.vrpn_pose.received && input.vrpn_pose.valid && !input.vrpn_pose.time_jump &&
        input.vrpn_pose.last_dt_sec <= config.max_time_jump_s && vrpn_age <= config.vrpn_timeout_s;

    health.imu_ready = imu_ready;
    health.vrpn_ready = vrpn_ready;
    health.flags = flags;

    if (fault_requested) {
        health.state = state_type::Fault;
    } else if (vrpn_ready) {
        health.state = state_type::Running;
    } else if (!initialized) {
        health.state = state_type::SelfCheck;
    } else if (vrpn_age <= config.coasting_timeout_s) {
        health.state = state_type::Coasting;
        health.flags |= kCoasting;
    } else {
        health.state = state_type::Fault;
    }

    if (health.state == state_type::Fault) {
        health.flags |= kFault;
    }
    health.transition_event = transitionEventFor(health.state);
    return health;
}

}  // namespace estimator_vrpn_ugv_state::health_checks
