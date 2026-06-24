#include "estimator_vrpn_px4_rotor_state/state_machine/health_monitor_state.h"

#include <utility>

#include "estimator_vrpn_px4_rotor_state/common/event_types.h"
#include "estimator_vrpn_px4_rotor_state/common/health_checks.h"
#include "estimator_vrpn_px4_rotor_state/vrpn_px4_rotor_state_estimator_runtime.h"

namespace estimator_vrpn_px4_rotor_state {
namespace {

double eventTimeOrCurrent(const ::state_machine::Event& event,
                          const VrpnPx4RotorStateEstimatorRuntime& runtime) {
    return event.timestamp > 0.0 ? event.timestamp : runtime.currentTime();
}

}  // namespace

HealthMonitorState::HealthMonitorState(VrpnPx4RotorStateEstimatorRuntime& runtime)
    : runtime_(runtime) {}

::state_machine::ActionResult HealthMonitorState::onEvent(::state_machine::StateContext& ctx,
                                                          const ::state_machine::Event& event) {
    if (event.category == ::state_machine::EventCategory::kInput) {
        evaluateAndPostTransition(ctx, eventTimeOrCurrent(event, runtime_));
    }
    return {};
}

::state_machine::ActionResult HealthMonitorState::onTick(::state_machine::StateContext& ctx) {
    evaluateAndPostTransition(ctx, runtime_.currentTime());
    return {};
}

void HealthMonitorState::evaluateAndPostTransition(::state_machine::StateContext& ctx,
                                                   double now_sec) const {
    const auto health = health_checks::classify(
        runtime_.input(), runtime_.config(), runtime_.estimator().initialized(),
        runtime_.faultRequested(), runtime_.estimator().state().covariance_trace,
        runtime_.estimatorFlags(), now_sec);
    runtime_.setHealth(health);

    const auto active_state = ctx.currentState(region_type::ESTIMATION);
    if (active_state == health.state || health.transition_event == 0) {
        return;
    }

    ::state_machine::Event event(health.transition_event, ::state_machine::EventTimestamp{now_sec});
    event.source = "rigid_state_health_monitor";
    event.category = ::state_machine::EventCategory::kInternal;
    const auto status = ctx.postInternalEvent(std::move(event));
    if (!status.ok()) {
        runtime_.setHealth(HealthStatus{state_type::Fault, event_type::HEALTH_TO_FAULT,
                                        health.flags | kFault, false, false});
    }
}

}  // namespace estimator_vrpn_px4_rotor_state
