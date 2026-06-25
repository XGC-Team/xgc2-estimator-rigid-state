#include "estimator_vrpn_ugv_state/vrpn_ugv_state_estimator_runtime.h"

#include <stdexcept>
#include <string>
#include <utility>

#include "estimator_vrpn_ugv_state/common/config_utils.h"
#include "estimator_vrpn_ugv_state/state_machine/coasting_state.h"
#include "estimator_vrpn_ugv_state/state_machine/fault_state.h"
#include "estimator_vrpn_ugv_state/state_machine/health_monitor_state.h"
#include "estimator_vrpn_ugv_state/state_machine/running_state.h"
#include "estimator_vrpn_ugv_state/state_machine/self_check_state.h"

namespace estimator_vrpn_ugv_state {
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

}  // namespace

VrpnUgvStateEstimatorRuntime::VrpnUgvStateEstimatorRuntime() {
    config_utils::normalizeConfig(config_);
    reset();
}

void VrpnUgvStateEstimatorRuntime::setConfig(const VrpnUgvStateEstimatorConfig& config) {
    config_ = config;
    config_utils::normalizeConfig(config_);
    reset();
}

void VrpnUgvStateEstimatorRuntime::reset() {
    input_ = VrpnUgvStateEstimatorInput{};
    health_ = HealthStatus{};
    estimator_.setConfig(config_utils::estimatorConfigFromRuntimeConfig(config_));
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

sm::Status VrpnUgvStateEstimatorRuntime::postInputEvent(sm::Event event,
                                                        const VrpnUgvStateEstimatorInput& input) {
    input_ = input;
    event.category = sm::EventCategory::kInput;
    return machine_->postEvent(std::move(event));
}

VrpnUgvStateEstimatorOutput VrpnUgvStateEstimatorRuntime::update(double now_sec) {
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

VrpnUgvStateEstimatorOutput VrpnUgvStateEstimatorRuntime::snapshotOutput() const {
    std::lock_guard<std::mutex> lock(output_mutex_);
    return last_output_;
}

VrpnUgvStateEstimatorOutput VrpnUgvStateEstimatorRuntime::refreshOutputSnapshot() {
    recordStateOutput(state_, health_.flags | estimator_flags_);
    return snapshotOutput();
}

void VrpnUgvStateEstimatorRuntime::enterState(::state_machine::StateId state) {
    state_ = state;
}

void VrpnUgvStateEstimatorRuntime::initializeIfReady() {
    if (estimator_.initialized() || !health_.vrpn_ready) {
        return;
    }
    estimator_.initializeFromPose(input_.vrpn_pose, input_.imu.received ? &input_.imu : nullptr);
    if (estimator_.initialized()) {
        clearPoseFusionFlags(estimator_flags_);
        applyObservationStateFlags(estimator_.vrpnObservationState(), estimator_flags_);
        applyFilterHealthFlags(estimator_.filterHealth(), estimator_flags_);
        last_pose_reject_reason_ = xgc2_math::PoseFusionRejectReason::kNone;
        last_pose_accepted_ = true;
    }
}

void VrpnUgvStateEstimatorRuntime::processImuInput() {
    estimator_.propagateInertial(input_.imu);
}

void VrpnUgvStateEstimatorRuntime::processVrpnInput() {
    clearPoseFusionFlags(estimator_flags_);
    if (!estimator_.initialized()) {
        initializeIfReady();
        return;
    }
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

void VrpnUgvStateEstimatorRuntime::recordStateOutput(::state_machine::StateId state,
                                                     uint32_t flags) {
    if (state == state_type::Fault) {
        flags |= kFault;
    }
    state_ = state;
    std::lock_guard<std::mutex> lock(output_mutex_);
    last_output_ = makeOutput(state_, flags);
}

void VrpnUgvStateEstimatorRuntime::setupMachine() {
    auto builder = sm::StateMachine::builder("VrpnUgvStateEstimatorMachine");
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
        .to(state_type::Running)
        .on(event_type::HEALTH_TO_RUNNING)
        .priority(transition_priority::AUTOMATIC)
        .evaluationOrder(0);
    builder.transition()
        .from(state_type::SelfCheck)
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
        .to(state_type::Running)
        .on(event_type::HEALTH_TO_RUNNING)
        .priority(transition_priority::AUTOMATIC)
        .evaluationOrder(0);

    auto machine_result = builder.build();
    requireOk(machine_result.status, "build VRPN UGV state estimator state machine");
    machine_ = std::move(machine_result.value);
    requireOk(machine_->start(), "start VRPN UGV state estimator state machine");
}

VrpnUgvStateEstimatorOutput VrpnUgvStateEstimatorRuntime::makeOutput(::state_machine::StateId state,
                                                                     uint32_t flags) const {
    VrpnUgvStateEstimatorOutput output;
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
        output.corrected_body_pose = estimator_.correctedBodyPose();
        output.has_corrected_body_pose = true;
    }
    if (estimator_.hasRawProjectedBodyPose()) {
        output.raw_projected_body_pose = estimator_.rawProjectedBodyPose();
        output.has_raw_projected_body_pose = true;
    }
    return output;
}

}  // namespace estimator_vrpn_ugv_state
