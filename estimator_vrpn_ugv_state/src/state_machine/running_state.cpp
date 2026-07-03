#include "estimator_vrpn_ugv_state/state_machine/running_state.h"

#include <utility>

#include "estimator_vrpn_ugv_state/common/event_types.h"
#include "estimator_vrpn_ugv_state/vrpn_ugv_state_estimator_runtime.h"

namespace estimator_vrpn_ugv_state {

RunningState::RunningState(VrpnUgvStateEstimatorRuntime& runtime) : runtime_(runtime) {}

::state_machine::ActionResult RunningState::onEnter(::state_machine::StateContext& ctx) {
    (void)ctx;
    runtime_.enterState(state_type::Running);
    state_publish_gate_.reset();
    runtime_.initializeIfReady();
    return {};
}

::state_machine::ActionResult RunningState::onEvent(::state_machine::StateContext& ctx,
                                                    const ::state_machine::Event& event) {
    (void)ctx;
    if (event.id == event_type::INPUT_IMU_UPDATED) {
        runtime_.processImuInput();
    } else if (event.id == event_type::INPUT_VRPN_POSE_UPDATED) {
        runtime_.processVrpnInput();
    }
    return {};
}

::state_machine::ActionResult RunningState::onTick(::state_machine::StateContext& ctx) {
    if (runtime_.health().state != state_type::Running) {
        return {};
    }
    runtime_.recordStateOutput(state_type::Running, runtime_.outputFlags());
    emitOutputIfDue(ctx);
    return {};
}

void RunningState::emitOutputIfDue(::state_machine::StateContext& ctx) {
    if (state_publish_gate_.due(runtime_.currentTime(),
                                1.0 / runtime_.config().state_publish_rate_hz)) {
        ::state_machine::Event event(output_event_type::PUBLISH_STATE,
                                     ::state_machine::EventTimestamp{runtime_.currentTime()});
        event.category = ::state_machine::EventCategory::kOutput;
        ctx.emitOutput(std::move(event));
    }
}

::state_machine::ActionResult RunningState::onExit(::state_machine::StateContext& ctx) {
    (void)ctx;
    state_publish_gate_.reset();
    return {};
}

}  // namespace estimator_vrpn_ugv_state
