#pragma once

#include <cstddef>
#include <cstdint>
#include <xgc2_math/estimation.hpp>

#include "estimator_vrpn_ugv_state/common/event_types.h"

namespace estimator_vrpn_ugv_state {

enum RuntimeFlag : uint32_t {
    kImuMissing = 1u << 0,
    kVrpnMissing = 1u << 1,
    kImuStale = 1u << 2,
    kVrpnStale = 1u << 3,
    kImuRateLow = 1u << 4,
    kVrpnRateLow = 1u << 5,
    kTimeJump = 1u << 6,
    kCoasting = 1u << 7,
    kInnovationRejected = 1u << 9,
    kExtrinsicUnverified = 1u << 10,
    kCovarianceHigh = 1u << 11,
    kInvalidImu = 1u << 12,
    kInvalidVrpn = 1u << 13,
    kPoseTimeAlignmentRejected = 1u << 14,
    kVrpnSuspected = 1u << 15,
    kVrpnFault = 1u << 16,
    kVrpnRecovery = 1u << 17,
    kFilterDegraded = 1u << 18,
    kFilterImuOnly = 1u << 19,
};

enum class HealthCondition : uint8_t {
    kInputUnhealthy = 0,
    kEstimationReady = 1,
    kVrpnLossCoastable = 2,
};

struct VrpnUgvStateEstimatorConfig {
    double loop_rate_hz{1000.0};
    double state_publish_rate_hz{100.0};

    xgc2_math::Pose2 field_to_world{};
    xgc2_math::Pose2 body_to_vrpn_marker{};
    bool extrinsic_verified{false};

    double imu_timeout_s{0.2};
    double vrpn_timeout_s{0.12};
    double coasting_timeout_s{0.5};
    double min_imu_rate_hz{5.0};
    double min_vrpn_rate_hz{20.0};
    double max_time_jump_s{0.25};

    double gyro_noise_std{0.03};
    double accel_noise_std{0.35};
    double gyro_bias_random_walk_std{1.0e-4};
    double accel_bias_random_walk_std{1.0e-3};
    double vrpn_position_noise_std{0.01};
    double vrpn_yaw_noise_std{0.01};
    double innovation_position_gate_m{1.5};
    double innovation_yaw_gate_rad{0.8};
    double covariance_high_threshold{100.0};
    double max_propagation_dt_s{0.05};
    std::size_t inertial_buffer_capacity{512};
    xgc2_math::ObservationHealthConfig vrpn_health{};
};

struct VrpnUgvStateEstimatorInput {
    xgc2_math::PlanarInertialSample imu{};
    xgc2_math::PlanarPoseMeasurement vrpn_pose{};
};

struct VrpnUgvStateEstimatorOutput {
    xgc2_math::Pose2 corrected_body_pose{};
    xgc2_math::Pose2 raw_projected_body_pose{};
    xgc2_math::PlanarRigidBodyState state{};
    double last_fused_pose_stamp_sec{0.0};
    double vrpn_innovation_window_chi_square{0.0};
    double stamp_sec{0.0};
    uint32_t flags{0};
    xgc2_math::VrpnObservationState vrpn_observation_state{
        xgc2_math::VrpnObservationState::kTrusted};
    xgc2_math::FilterHealth filter_health{xgc2_math::FilterHealth::kLost};
    xgc2_math::PoseFusionRejectReason last_pose_reject_reason{
        xgc2_math::PoseFusionRejectReason::kNone};
    uint8_t estimator_state{state_type::SelfCheck};
    bool has_corrected_body_pose{false};
    bool has_raw_projected_body_pose{false};
    bool last_pose_accepted{false};
};

struct HealthStatus {
    HealthCondition condition{HealthCondition::kInputUnhealthy};
    uint32_t flags{kImuMissing | kVrpnMissing | kExtrinsicUnverified};
    bool imu_ready{false};
    bool vrpn_ready{false};
};

}  // namespace estimator_vrpn_ugv_state
