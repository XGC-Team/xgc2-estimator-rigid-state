#pragma once

#include <state_machine/state_machine.hpp>

namespace estimator_vrpn_px4_rotor_state {

class VrpnPx4RotorStateEstimatorRuntime;

class HealthMonitorState final : public ::state_machine::State {
   public:
    explicit HealthMonitorState(VrpnPx4RotorStateEstimatorRuntime& runtime);
    std::string name() const override {
        return "HealthMonitor";
    }
    ::state_machine::ActionResult onEvent(::state_machine::StateContext& ctx,
                                          const ::state_machine::Event& event) override;
    ::state_machine::ActionResult onTick(::state_machine::StateContext& ctx) override;

   private:
    void evaluateAndPostTransition(::state_machine::StateContext& ctx, double now_sec) const;
    VrpnPx4RotorStateEstimatorRuntime& runtime_;
};

}  // namespace estimator_vrpn_px4_rotor_state
