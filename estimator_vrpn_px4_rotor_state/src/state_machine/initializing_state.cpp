#include "estimator_vrpn_px4_rotor_state/state_machine/initializing_state.h"

#include <utility>

#include "estimator_vrpn_px4_rotor_state/common/event_types.h"
#include "estimator_vrpn_px4_rotor_state/vrpn_px4_rotor_state_estimator_runtime.h"

namespace estimator_vrpn_px4_rotor_state {

InitializingState::InitializingState(VrpnPx4RotorStateEstimatorRuntime& runtime)
    : runtime_(runtime) {}

::state_machine::ActionResult InitializingState::onEnter(::state_machine::StateContext& ctx) {
    (void)ctx;
    runtime_.enterState(state_type::Initializing);
    state_publish_gate_.reset();
    return {};
}

::state_machine::ActionResult InitializingState::onTick(::state_machine::StateContext& ctx) {
    if (runtime_.health().state != state_type::Initializing) {
        return {};
    }
    runtime_.initializeIfReady();
    runtime_.recordStateOutput(state_type::Initializing, runtime_.outputFlags());
    emitOutputIfDue(ctx);
    return {};
}

void InitializingState::emitOutputIfDue(::state_machine::StateContext& ctx) {
    if (state_publish_gate_.due(runtime_.currentTime(),
                                1.0 / runtime_.config().state_publish_rate_hz)) {
        ::state_machine::Event event(output_event_type::PUBLISH_STATE,
                                     ::state_machine::EventTimestamp{runtime_.currentTime()});
        event.category = ::state_machine::EventCategory::kOutput;
        ctx.emitOutput(std::move(event));
    }
}

::state_machine::ActionResult InitializingState::onExit(::state_machine::StateContext& ctx) {
    (void)ctx;
    state_publish_gate_.reset();
    return {};
}

}  // namespace estimator_vrpn_px4_rotor_state
