#include "estimator_rigid_state/state_machine/self_check_state.h"

#include <utility>

#include "estimator_rigid_state/common/event_types.h"
#include "estimator_rigid_state/rigid_state_estimator_runtime.h"

namespace estimator_rigid_state {

SelfCheckState::SelfCheckState(RigidStateEstimatorRuntime& runtime) : runtime_(runtime) {}

::state_machine::ActionResult SelfCheckState::onEnter(::state_machine::StateContext& ctx) {
    (void)ctx;
    runtime_.enterState(state_type::SelfCheck);
    state_publish_gate_.reset();
    return {};
}

::state_machine::ActionResult SelfCheckState::onTick(::state_machine::StateContext& ctx) {
    if (runtime_.health().state != state_type::SelfCheck) {
        return {};
    }
    runtime_.recordStateOutput(state_type::SelfCheck,
                               runtime_.health().flags | runtime_.estimatorFlags());
    emitOutputIfDue(ctx);
    return {};
}

void SelfCheckState::emitOutputIfDue(::state_machine::StateContext& ctx) {
    if (state_publish_gate_.due(runtime_.currentTime(),
                                1.0 / runtime_.config().state_publish_rate_hz)) {
        ::state_machine::Event event(output_event_type::PUBLISH_STATE,
                                     ::state_machine::EventTimestamp{runtime_.currentTime()});
        event.category = ::state_machine::EventCategory::kOutput;
        ctx.emitOutput(std::move(event));
    }
}

::state_machine::ActionResult SelfCheckState::onExit(::state_machine::StateContext& ctx) {
    (void)ctx;
    state_publish_gate_.reset();
    return {};
}

}  // namespace estimator_rigid_state
