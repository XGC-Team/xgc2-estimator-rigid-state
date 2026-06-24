#include "estimator_rigid_state/state_machine/fault_state.h"

#include <utility>

#include "estimator_rigid_state/common/event_types.h"
#include "estimator_rigid_state/rigid_state_estimator_runtime.h"

namespace estimator_rigid_state {

FaultState::FaultState(RigidStateEstimatorRuntime& runtime) : runtime_(runtime) {}

::state_machine::ActionResult FaultState::onEnter(::state_machine::StateContext& ctx) {
    (void)ctx;
    runtime_.enterState(state_type::Fault);
    state_publish_gate_.reset();
    return {};
}

::state_machine::ActionResult FaultState::onTick(::state_machine::StateContext& ctx) {
    runtime_.recordStateOutput(state_type::Fault,
                               runtime_.health().flags | runtime_.estimatorFlags() | kFault);
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

}  // namespace estimator_rigid_state
