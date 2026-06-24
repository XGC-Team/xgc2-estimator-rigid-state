#pragma once

#include <cstdint>
#include <xgc2_observer/inertial_pose_eskf.hpp>

#include "estimator_rigid_state/common/event_types.h"

namespace estimator_rigid_state {

enum RuntimeFlag : uint32_t {
    kImuMissing = 1u << 0,
    kVrpnMissing = 1u << 1,
    kImuStale = 1u << 2,
    kVrpnStale = 1u << 3,
    kImuRateLow = 1u << 4,
    kVrpnRateLow = 1u << 5,
    kTimeJump = 1u << 6,
    kCoasting = 1u << 7,
    kFault = 1u << 8,
    kInnovationRejected = 1u << 9,
    kExtrinsicUnverified = 1u << 10,
    kCovarianceHigh = 1u << 11,
    kInvalidImu = 1u << 12,
    kInvalidVrpn = 1u << 13,
};

struct RigidStateEstimatorConfig {
    double loop_rate_hz{1000.0};
    double state_publish_rate_hz{100.0};
    double vision_publish_rate_hz{30.0};

    double gravity_mps2{9.8066};
    xgc2_observer::Pose3 field_to_world{};
    xgc2_observer::Pose3 imu_to_vrpn_marker{};
    bool extrinsic_verified{false};
    bool estimate_extrinsic{false};

    double imu_timeout_s{0.05};
    double vrpn_timeout_s{0.12};
    double coasting_timeout_s{0.5};
    double min_imu_rate_hz{25.0};
    double min_vrpn_rate_hz{20.0};
    double max_time_jump_s{0.25};

    double accel_noise_std{0.35};
    double gyro_noise_std{0.03};
    double vrpn_position_noise_std{0.01};
    double vrpn_orientation_noise_std{0.01};
    double position_update_gain{0.85};
    double velocity_update_gain{0.25};
    double orientation_update_gain{0.85};
    double gyro_bias_update_gain{0.002};
    double accel_bias_update_gain{0.0};
    double innovation_position_gate_m{1.5};
    double innovation_orientation_gate_rad{0.8};
    double covariance_high_threshold{100.0};
};

struct RigidStateEstimatorInput {
    xgc2_observer::InertialSample imu{};
    xgc2_observer::PoseMeasurement vrpn_pose{};
};

struct RigidStateEstimatorOutput {
    uint8_t estimator_state{state_type::SelfCheck};
    uint32_t flags{0};
    xgc2_observer::RigidBodyState state{};
    xgc2_observer::Pose3 corrected_vision_pose{};
    bool has_corrected_vision_pose{false};
    double stamp_sec{0.0};
};

struct HealthStatus {
    ::state_machine::StateId state{state_type::SelfCheck};
    ::state_machine::EventId transition_event{0};
    uint32_t flags{kImuMissing | kVrpnMissing | kExtrinsicUnverified};
    bool imu_ready{false};
    bool vrpn_ready{false};
};

}  // namespace estimator_rigid_state
