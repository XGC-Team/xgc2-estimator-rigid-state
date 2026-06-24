#include "estimator_rigid_state/rigid_state_estimator_runtime.h"

#include <stdexcept>
#include <string>
#include <utility>

#include "estimator_rigid_state/common/config_utils.h"
#include "estimator_rigid_state/state_machine/coasting_state.h"
#include "estimator_rigid_state/state_machine/fault_state.h"
#include "estimator_rigid_state/state_machine/health_monitor_state.h"
#include "estimator_rigid_state/state_machine/initializing_state.h"
#include "estimator_rigid_state/state_machine/running_state.h"
#include "estimator_rigid_state/state_machine/self_check_state.h"

namespace estimator_rigid_state {
namespace {

namespace sm = ::state_machine;

void requireOk(const sm::Status& status, const char* operation) {
    if (!status.ok()) {
        throw std::runtime_error(std::string(operation) + ": " + status.message);
    }
}

xgc2_observer::InertialPoseEskfConfig observerConfigFromRuntimeConfig(
    const RigidStateEstimatorConfig& config) {
    xgc2_observer::InertialPoseEskfConfig result;
    result.gravity_mps2 = config.gravity_mps2;
    result.measurement_frame_to_world = config.field_to_world;
    result.body_to_marker = config.imu_to_vrpn_marker;
    result.estimate_extrinsic = config.estimate_extrinsic;
    result.accel_noise_std = config.accel_noise_std;
    result.gyro_noise_std = config.gyro_noise_std;
    result.pose_position_noise_std = config.vrpn_position_noise_std;
    result.pose_orientation_noise_std = config.vrpn_orientation_noise_std;
    result.position_update_gain = config.position_update_gain;
    result.velocity_update_gain = config.velocity_update_gain;
    result.orientation_update_gain = config.orientation_update_gain;
    result.gyro_bias_update_gain = config.gyro_bias_update_gain;
    result.accel_bias_update_gain = config.accel_bias_update_gain;
    result.innovation_position_gate_m = config.innovation_position_gate_m;
    result.innovation_orientation_gate_rad = config.innovation_orientation_gate_rad;
    result.covariance_high_threshold = config.covariance_high_threshold;
    return result;
}

}  // namespace

RigidStateEstimatorRuntime::RigidStateEstimatorRuntime() {
    config_utils::normalizeConfig(config_);
    reset();
}

void RigidStateEstimatorRuntime::setConfig(const RigidStateEstimatorConfig& config) {
    config_ = config;
    config_utils::normalizeConfig(config_);
    reset();
}

void RigidStateEstimatorRuntime::reset() {
    input_ = RigidStateEstimatorInput{};
    health_ = HealthStatus{};
    estimator_.setConfig(observerConfigFromRuntimeConfig(config_));
    estimator_flags_ = 0;
    current_time_sec_ = 0.0;
    fault_requested_ = false;
    state_ = state_type::SelfCheck;
    {
        std::lock_guard<std::mutex> lock(output_mutex_);
        last_output_ = makeOutput(state_, health_.flags);
    }
    setupMachine();
}

sm::Status RigidStateEstimatorRuntime::postInputEvent(sm::Event event,
                                                      const RigidStateEstimatorInput& input) {
    input_ = input;
    event.category = sm::EventCategory::kInput;
    return machine_->postEvent(std::move(event));
}

RigidStateEstimatorOutput RigidStateEstimatorRuntime::update(double now_sec) {
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

RigidStateEstimatorOutput RigidStateEstimatorRuntime::snapshotOutput() const {
    std::lock_guard<std::mutex> lock(output_mutex_);
    return last_output_;
}

RigidStateEstimatorOutput RigidStateEstimatorRuntime::refreshOutputSnapshot() {
    recordStateOutput(state_, health_.flags | estimator_flags_);
    return snapshotOutput();
}

void RigidStateEstimatorRuntime::enterState(::state_machine::StateId state) {
    state_ = state;
}

void RigidStateEstimatorRuntime::initializeIfReady() {
    if (estimator_.initialized()) {
        return;
    }
    if (!health_.imu_ready || !health_.vrpn_ready) {
        return;
    }
    estimator_.initializeFromPose(input_.vrpn_pose, &input_.imu);
}

void RigidStateEstimatorRuntime::processImuInput() {
    estimator_.propagateInertial(input_.imu);
}

void RigidStateEstimatorRuntime::processVrpnInput() {
    estimator_flags_ &= ~kInnovationRejected;
    const auto result = estimator_.updatePose(input_.vrpn_pose);
    if (result.innovation_rejected) {
        estimator_flags_ |= kInnovationRejected;
    }
}

void RigidStateEstimatorRuntime::recordStateOutput(::state_machine::StateId state, uint32_t flags) {
    if (state == state_type::Fault) {
        flags |= kFault;
    }
    state_ = state;
    std::lock_guard<std::mutex> lock(output_mutex_);
    last_output_ = makeOutput(state_, flags);
}

void RigidStateEstimatorRuntime::markInnovationRejected() {
    estimator_flags_ |= kInnovationRejected;
}

void RigidStateEstimatorRuntime::setupMachine() {
    auto builder = sm::StateMachine::builder("RigidStateEstimatorMachine");
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
    requireOk(machine_result.status, "build rigid state estimator state machine");
    machine_ = std::move(machine_result.value);
    requireOk(machine_->start(), "start rigid state estimator state machine");
}

RigidStateEstimatorOutput RigidStateEstimatorRuntime::makeOutput(::state_machine::StateId state,
                                                                 uint32_t flags) const {
    RigidStateEstimatorOutput output;
    output.estimator_state = static_cast<uint8_t>(state);
    output.flags = flags;
    output.state = estimator_.state();
    output.stamp_sec = current_time_sec_;
    if (estimator_.hasCorrectedBodyPose()) {
        output.corrected_vision_pose = estimator_.correctedBodyPose();
        output.has_corrected_vision_pose = true;
    }
    return output;
}

}  // namespace estimator_rigid_state
