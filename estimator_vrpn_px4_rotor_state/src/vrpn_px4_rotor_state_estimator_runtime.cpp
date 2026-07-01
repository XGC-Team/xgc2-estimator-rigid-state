#include "estimator_vrpn_px4_rotor_state/vrpn_px4_rotor_state_estimator_runtime.h"

#include <iostream>
#include <stdexcept>
#include <string>
#include <utility>

#include "estimator_vrpn_px4_rotor_state/common/config_utils.h"
#include "estimator_vrpn_px4_rotor_state/state_machine/coasting_state.h"
#include "estimator_vrpn_px4_rotor_state/state_machine/fault_state.h"
#include "estimator_vrpn_px4_rotor_state/state_machine/health_monitor_state.h"
#include "estimator_vrpn_px4_rotor_state/state_machine/initializing_state.h"
#include "estimator_vrpn_px4_rotor_state/state_machine/running_state.h"
#include "estimator_vrpn_px4_rotor_state/state_machine/self_check_state.h"

namespace estimator_vrpn_px4_rotor_state {
namespace {

namespace sm = ::state_machine;

void requireOk(const sm::Status& status, const char* operation) {
    if (!status.ok()) {
        throw std::runtime_error(std::string(operation) + ": " + status.message);
    }
}

constexpr uint32_t kPoseFusionRuntimeFlags = kInnovationRejected | kPoseTimeAlignmentRejected |
                                             kVrpnSuspected | kVrpnFault | kVrpnRecovery |
                                             kFilterDegraded | kFilterImuOnly;

void clearPoseFusionFlags(uint32_t& flags) {
    flags &= ~kPoseFusionRuntimeFlags;
}

void applyObservationStateFlags(xgc2_math::VrpnObservationState state, uint32_t& flags) {
    switch (state) {
        case xgc2_math::VrpnObservationState::kTrusted:
            return;
        case xgc2_math::VrpnObservationState::kSuspected:
            flags |= kVrpnSuspected;
            return;
        case xgc2_math::VrpnObservationState::kFault:
            flags |= kVrpnFault;
            return;
        case xgc2_math::VrpnObservationState::kRecovery:
            flags |= kVrpnRecovery;
            return;
    }
}

void applyFilterHealthFlags(xgc2_math::FilterHealth health, uint32_t& flags) {
    switch (health) {
        case xgc2_math::FilterHealth::kNominal:
        case xgc2_math::FilterHealth::kLost:
            return;
        case xgc2_math::FilterHealth::kDegraded:
            flags |= kFilterDegraded;
            return;
        case xgc2_math::FilterHealth::kImuOnly:
            flags |= kFilterImuOnly;
            return;
    }
}

xgc2_math::Pose3InertialEskfConfig observerConfigFromRuntimeConfig(
    const VrpnPx4RotorStateEstimatorConfig& config) {
    xgc2_math::Pose3InertialEskfConfig result;
    result.gravity_mps2 = config.gravity_mps2;
    result.measurement_frame_to_world = config.field_to_world;
    result.body_to_marker = config.imu_to_vrpn_marker;
    result.estimate_extrinsic = config.estimate_extrinsic;
    result.accel_noise_std = config.accel_noise_std;
    result.gyro_noise_std = config.gyro_noise_std;
    result.pose_position_noise_std = config.vrpn_position_noise_std;
    result.pose_orientation_noise_std = config.vrpn_orientation_noise_std;
    result.gyro_bias_random_walk_std = config.gyro_bias_random_walk_std;
    result.accel_bias_random_walk_std = config.accel_bias_random_walk_std;
    result.extrinsic_position_random_walk_std = config.extrinsic_position_random_walk_std;
    result.extrinsic_orientation_random_walk_std = config.extrinsic_orientation_random_walk_std;
    result.innovation_position_gate_m = config.innovation_position_gate_m;
    result.innovation_orientation_gate_rad = config.innovation_orientation_gate_rad;
    result.pose_nis_gate = config.pose_nis_gate;
    result.covariance_high_threshold = config.covariance_high_threshold;
    result.max_propagation_dt_s = config.max_propagation_dt_s;
    result.initial_position_variance = config.initial_position_variance;
    result.initial_velocity_variance = config.initial_velocity_variance;
    result.initial_orientation_variance = config.initial_orientation_variance;
    result.initial_gyro_bias_variance = config.initial_gyro_bias_variance;
    result.initial_accel_bias_variance = config.initial_accel_bias_variance;
    result.inertial_buffer_capacity = config.inertial_buffer_capacity;
    result.vrpn_health = config.vrpn_health;
    return result;
}

}  // namespace

VrpnPx4RotorStateEstimatorRuntime::VrpnPx4RotorStateEstimatorRuntime() {
    config_utils::normalizeConfig(config_);
    reset();
}

void VrpnPx4RotorStateEstimatorRuntime::setConfig(const VrpnPx4RotorStateEstimatorConfig& config) {
    config_ = config;
    config_utils::normalizeConfig(config_);
    reset();
}

void VrpnPx4RotorStateEstimatorRuntime::reset() {
    input_ = VrpnPx4RotorStateEstimatorInput{};
    health_ = HealthStatus{};
    estimator_.setConfig(observerConfigFromRuntimeConfig(config_));
    estimator_flags_ = 0;
    last_pose_reject_reason_ = xgc2_math::PoseFusionRejectReason::kNone;
    last_pose_accepted_ = false;
    current_time_sec_ = 0.0;
    fault_requested_ = false;
    state_ = state_type::SelfCheck;
    {
        std::lock_guard<std::mutex> lock(output_mutex_);
        last_output_ = makeOutput(state_, health_.flags);
    }
    setupMachine();
}

sm::Status VrpnPx4RotorStateEstimatorRuntime::postInputEvent(
    sm::Event event, const VrpnPx4RotorStateEstimatorInput& input) {
    input_ = input;
    event.category = sm::EventCategory::kInput;
    return machine_->postEvent(std::move(event));
}

VrpnPx4RotorStateEstimatorOutput VrpnPx4RotorStateEstimatorRuntime::update(double now_sec) {
    current_time_sec_ = now_sec;
    const auto transition_result = machine_->update({64, 64, false});
    const auto tick_result =
        transition_result.status.ok() ? machine_->update({64, 64, true}) : transition_result;
    if (!tick_result.status.ok()) {
        fault_requested_ = true;
        state_ = state_type::Fault;
        estimator_flags_ |= kFault;
        recordStateOutput(state_, health_.flags | estimator_flags_);
    }
    return snapshotOutput();
}

VrpnPx4RotorStateEstimatorOutput VrpnPx4RotorStateEstimatorRuntime::snapshotOutput() const {
    std::lock_guard<std::mutex> lock(output_mutex_);
    return last_output_;
}

VrpnPx4RotorStateEstimatorOutput VrpnPx4RotorStateEstimatorRuntime::refreshOutputSnapshot() {
    recordStateOutput(state_, health_.flags | estimator_flags_);
    return snapshotOutput();
}

void VrpnPx4RotorStateEstimatorRuntime::enterState(::state_machine::StateId state) {
    state_ = state;
}

void VrpnPx4RotorStateEstimatorRuntime::initializeIfReady() {
    if (estimator_.initialized()) {
        return;
    }
    if (!health_.imu_ready || !health_.vrpn_ready) {
        return;
    }
    estimator_.initializeFromPose(input_.vrpn_pose, &input_.imu);
    if (estimator_.initialized()) {
        clearPoseFusionFlags(estimator_flags_);
        applyObservationStateFlags(estimator_.vrpnObservationState(), estimator_flags_);
        applyFilterHealthFlags(estimator_.filterHealth(), estimator_flags_);
        last_pose_reject_reason_ = xgc2_math::PoseFusionRejectReason::kNone;
        last_pose_accepted_ = true;
    }
}

void VrpnPx4RotorStateEstimatorRuntime::processImuInput() {
    estimator_.propagateInertial(input_.imu);
}

void VrpnPx4RotorStateEstimatorRuntime::processVrpnInput() {
    clearPoseFusionFlags(estimator_flags_);
    const auto result = estimator_.updatePose(input_.vrpn_pose);
    last_pose_accepted_ = result.accepted;
    last_pose_reject_reason_ = result.reject_reason;
    if (result.innovation_rejected) {
        estimator_flags_ |= kInnovationRejected;
    }
    if (result.time_alignment_rejected) {
        estimator_flags_ |= kPoseTimeAlignmentRejected;
    }
    applyObservationStateFlags(result.vrpn_observation_state, estimator_flags_);
    applyFilterHealthFlags(result.filter_health, estimator_flags_);
}

void VrpnPx4RotorStateEstimatorRuntime::recordStateOutput(::state_machine::StateId state,
                                                          uint32_t flags) {
    if (state == state_type::Fault) {
        flags |= kFault;
    }
    state_ = state;
    std::lock_guard<std::mutex> lock(output_mutex_);
    last_output_ = makeOutput(state_, flags);
}

void VrpnPx4RotorStateEstimatorRuntime::markInnovationRejected() {
    estimator_flags_ |= kInnovationRejected;
    last_pose_reject_reason_ = xgc2_math::PoseFusionRejectReason::kInnovationGate;
    last_pose_accepted_ = false;
}

void VrpnPx4RotorStateEstimatorRuntime::setupMachine() {
    auto builder = sm::StateMachine::builder("VrpnPx4RotorStateEstimatorMachine");
    builder.region(region_type::HEALTH)
        .name("health")
        .order(0)
        .initial(state_type::HealthMonitor)
        .state(state_type::HealthMonitor)
        .name("HealthMonitor")
        .impl(std::make_unique<HealthMonitorState>(*this))
        .endRegion()
        .region(region_type::ESTIMATION)
        .name("estimation")
        .order(10)
        .initial(state_type::SelfCheck)
        .state(state_type::SelfCheck)
        .name("SelfCheck")
        .impl(std::make_unique<SelfCheckState>(*this))
        .state(state_type::Initializing)
        .name("Initializing")
        .impl(std::make_unique<InitializingState>(*this))
        .state(state_type::Running)
        .name("Running")
        .impl(std::make_unique<RunningState>(*this))
        .state(state_type::Coasting)
        .name("Coasting")
        .impl(std::make_unique<CoastingState>(*this))
        .state(state_type::Fault)
        .name("Fault")
        .impl(std::make_unique<FaultState>(*this))
        .endRegion();

    builder.transition()
        .from(state_type::SelfCheck)
        .to(state_type::Initializing)
        .on(event_type::HEALTH_TO_INITIALIZING)
        .priority(transition_priority::AUTOMATIC)
        .evaluationOrder(0);
    builder.transition()
        .from(state_type::SelfCheck)
        .to(state_type::Fault)
        .on(event_type::HEALTH_TO_FAULT)
        .priority(transition_priority::FAULT)
        .evaluationOrder(0);

    builder.transition()
        .from(state_type::Initializing)
        .to(state_type::SelfCheck)
        .on(event_type::HEALTH_TO_SELF_CHECK)
        .priority(transition_priority::AUTOMATIC)
        .evaluationOrder(0);
    builder.transition()
        .from(state_type::Initializing)
        .to(state_type::Running)
        .on(event_type::HEALTH_TO_RUNNING)
        .priority(transition_priority::AUTOMATIC)
        .evaluationOrder(0);
    builder.transition()
        .from(state_type::Initializing)
        .to(state_type::Fault)
        .on(event_type::HEALTH_TO_FAULT)
        .priority(transition_priority::FAULT)
        .evaluationOrder(0);

    builder.transition()
        .from(state_type::Running)
        .to(state_type::SelfCheck)
        .on(event_type::HEALTH_TO_SELF_CHECK)
        .priority(transition_priority::AUTOMATIC)
        .evaluationOrder(0);
    builder.transition()
        .from(state_type::Running)
        .to(state_type::Coasting)
        .on(event_type::HEALTH_TO_COASTING)
        .priority(transition_priority::AUTOMATIC)
        .evaluationOrder(0);
    builder.transition()
        .from(state_type::Running)
        .to(state_type::Fault)
        .on(event_type::HEALTH_TO_FAULT)
        .priority(transition_priority::FAULT)
        .evaluationOrder(0);

    builder.transition()
        .from(state_type::Coasting)
        .to(state_type::Running)
        .on(event_type::HEALTH_TO_RUNNING)
        .priority(transition_priority::AUTOMATIC)
        .evaluationOrder(0);
    builder.transition()
        .from(state_type::Coasting)
        .to(state_type::SelfCheck)
        .on(event_type::HEALTH_TO_SELF_CHECK)
        .priority(transition_priority::AUTOMATIC)
        .evaluationOrder(0);
    builder.transition()
        .from(state_type::Coasting)
        .to(state_type::Fault)
        .on(event_type::HEALTH_TO_FAULT)
        .priority(transition_priority::FAULT)
        .evaluationOrder(0);

    builder.transition()
        .from(state_type::Fault)
        .to(state_type::SelfCheck)
        .on(event_type::HEALTH_TO_SELF_CHECK)
        .priority(transition_priority::AUTOMATIC)
        .evaluationOrder(0);
    builder.transition()
        .from(state_type::Fault)
        .to(state_type::Initializing)
        .on(event_type::HEALTH_TO_INITIALIZING)
        .priority(transition_priority::AUTOMATIC)
        .evaluationOrder(0);
    builder.transition()
        .from(state_type::Fault)
        .to(state_type::Running)
        .on(event_type::HEALTH_TO_RUNNING)
        .priority(transition_priority::AUTOMATIC)
        .evaluationOrder(0);

    auto machine_result = builder.build();
    requireOk(machine_result.status, "build VRPN PX4 rotor state estimator state machine");
    machine_ = std::move(machine_result.value);
    requireOk(machine_->start(), "start VRPN PX4 rotor state estimator state machine");
}

VrpnPx4RotorStateEstimatorOutput VrpnPx4RotorStateEstimatorRuntime::makeOutput(
    ::state_machine::StateId state, uint32_t flags) const {
    VrpnPx4RotorStateEstimatorOutput output;
    output.estimator_state = static_cast<uint8_t>(state);
    output.flags = flags;
    output.state = estimator_.state();
    output.stamp_sec = current_time_sec_;
    output.vrpn_observation_state = estimator_.vrpnObservationState();
    output.filter_health = estimator_.filterHealth();
    output.last_pose_reject_reason = last_pose_reject_reason_;
    output.last_pose_accepted = last_pose_accepted_;
    output.last_fused_pose_stamp_sec = estimator_.lastFusedPoseStampS();
    output.vrpn_innovation_window_chi_square = estimator_.vrpnInnovationWindowChiSquare();
    if (estimator_.hasCorrectedBodyPose()) {
        output.corrected_vision_pose = estimator_.correctedBodyPose();
        output.has_corrected_vision_pose = true;
    }
    if (estimator_.hasRawProjectedBodyPose()) {
        output.raw_projected_vision_pose = estimator_.rawProjectedBodyPose();
        output.has_raw_projected_vision_pose = true;
    }
    return output;
}

}  // namespace estimator_vrpn_px4_rotor_state
