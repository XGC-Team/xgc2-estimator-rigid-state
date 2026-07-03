#include "estimator_vrpn_px4_rotor_state/state_machine/coasting_state.h"

#include <utility>

#include "estimator_vrpn_px4_rotor_state/common/event_types.h"
#include "estimator_vrpn_px4_rotor_state/vrpn_px4_rotor_state_estimator_runtime.h"

namespace estimator_vrpn_px4_rotor_state {

CoastingState::CoastingState(VrpnPx4RotorStateEstimatorRuntime& runtime) : runtime_(runtime) {}

::state_machine::ActionResult CoastingState::onEnter(::state_machine::StateContext& ctx) {
    (void)ctx;
    runtime_.enterState(state_type::Coasting);
    state_publish_gate_.reset();
    return {};
}

::state_machine::ActionResult CoastingState::onEvent(::state_machine::StateContext& ctx,
                                                     const ::state_machine::Event& event) {
    (void)ctx;
    if (event.id == event_type::INPUT_IMU_UPDATED) {
        runtime_.processImuInput();
    }
    return {};
}

::state_machine::ActionResult CoastingState::onTick(::state_machine::StateContext& ctx) {
    if (runtime_.health().state != state_type::Coasting) {
        return {};
    }
    runtime_.recordStateOutput(state_type::Coasting, runtime_.outputFlags() | kCoasting);
    emitOutputIfDue(ctx);
    return {};
}

void CoastingState::emitOutputIfDue(::state_machine::StateContext& ctx) {
    if (state_publish_gate_.due(runtime_.currentTime(),
                                1.0 / runtime_.config().state_publish_rate_hz)) {
        ::state_machine::Event event(output_event_type::PUBLISH_STATE,
                                     ::state_machine::EventTimestamp{runtime_.currentTime()});
        event.category = ::state_machine::EventCategory::kOutput;
        ctx.emitOutput(std::move(event));
    }
}

::state_machine::ActionResult CoastingState::onExit(::state_machine::StateContext& ctx) {
    (void)ctx;
    state_publish_gate_.reset();
    return {};
}

}  // namespace estimator_vrpn_px4_rotor_state
