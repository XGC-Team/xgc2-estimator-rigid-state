#include "estimator_vrpn_ugv_state/state_machine/fault_state.h"

#include <utility>

#include "estimator_vrpn_ugv_state/common/event_types.h"
#include "estimator_vrpn_ugv_state/vrpn_ugv_state_estimator_runtime.h"

namespace estimator_vrpn_ugv_state {

FaultState::FaultState(VrpnUgvStateEstimatorRuntime& runtime) : runtime_(runtime) {}

::state_machine::ActionResult FaultState::onEnter(::state_machine::StateContext& ctx) {
    (void)ctx;
    runtime_.enterState(state_type::Fault);
    state_publish_gate_.reset();
    return {};
}

::state_machine::ActionResult FaultState::onTick(::state_machine::StateContext& ctx) {
    runtime_.recordStateOutput(state_type::Fault, runtime_.outputFlags() | kFault);
    emitOutputIfDue(ctx);
    return {};
}

void FaultState::emitOutputIfDue(::state_machine::StateContext& ctx) {
    if (state_publish_gate_.due(runtime_.currentTime(),
                                1.0 / runtime_.config().state_publish_rate_hz)) {
        ::state_machine::Event event(output_event_type::PUBLISH_STATE,
                                     ::state_machine::EventTimestamp{runtime_.currentTime()});
        event.category = ::state_machine::EventCategory::kOutput;
        ctx.emitOutput(std::move(event));
    }
}

::state_machine::ActionResult FaultState::onExit(::state_machine::StateContext& ctx) {
    (void)ctx;
    state_publish_gate_.reset();
    return {};
}

}  // namespace estimator_vrpn_ugv_state
