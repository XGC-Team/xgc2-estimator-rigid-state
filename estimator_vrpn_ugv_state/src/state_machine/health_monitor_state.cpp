#include "estimator_vrpn_ugv_state/state_machine/health_monitor_state.h"

#include <state_machine/runtime/event_time.hpp>
#include <stdexcept>
#include <string>
#include <utility>

#include "estimator_vrpn_ugv_state/common/event_types.h"
#include "estimator_vrpn_ugv_state/common/health_checks.h"
#include "estimator_vrpn_ugv_state/vrpn_ugv_state_estimator_runtime.h"

namespace estimator_vrpn_ugv_state {
namespace {

::state_machine::EventId eventForCondition(HealthCondition condition) {
    switch (condition) {
        case HealthCondition::kInputUnhealthy:
            return event_type::HEALTH_INPUT_UNHEALTHY;
        case HealthCondition::kEstimationReady:
            return event_type::HEALTH_ESTIMATION_READY;
        case HealthCondition::kVrpnLossCoastable:
            return event_type::HEALTH_VRPN_LOSS_COASTABLE;
    }
    throw std::runtime_error("unknown VRPN UGV health condition: " +
                             std::to_string(static_cast<int>(condition)));
}

}  // namespace

HealthMonitorState::HealthMonitorState(VrpnUgvStateEstimatorRuntime& runtime) : runtime_(runtime) {}

::state_machine::ActionResult HealthMonitorState::onEvent(::state_machine::StateContext& ctx,
                                                          const ::state_machine::Event& event) {
    if (event.category == ::state_machine::EventCategory::kInput) {
        evaluateAndPostTransition(
            ctx, ::state_machine::runtime::eventTimestampOr(event, runtime_.currentTime()));
    }
    return {};
}

::state_machine::ActionResult HealthMonitorState::onTick(::state_machine::StateContext& ctx) {
    evaluateAndPostTransition(ctx, runtime_.currentTime());
    return {};
}

void HealthMonitorState::evaluateAndPostTransition(::state_machine::StateContext& ctx,
                                                   double now_sec) const {
    const HealthCondition previous_condition = runtime_.health().condition;
    const auto health = health_checks::classify(
        runtime_.input(), runtime_.config(), runtime_.estimator().initialized(),
        runtime_.selfCheckRequested(), runtime_.estimator().state().covariance_trace,
        runtime_.estimatorFlags(), now_sec);
    runtime_.setHealth(health);

    if (health.condition == previous_condition) {
        return;
    }

    ::state_machine::Event event(eventForCondition(health.condition),
                                 ::state_machine::EventTimestamp{now_sec});
    event.source = "vrpn_ugv_state_health_monitor";
    event.category = ::state_machine::EventCategory::kInternal;
    const auto status = ctx.postInternalEvent(std::move(event));
    if (!status.ok()) {
        throw std::runtime_error("post VRPN UGV health event: " + status.message);
    }
}

}  // namespace estimator_vrpn_ugv_state
