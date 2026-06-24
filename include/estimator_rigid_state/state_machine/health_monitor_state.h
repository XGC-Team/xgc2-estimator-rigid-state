#pragma once

#include <state_machine/state_machine.hpp>

namespace estimator_rigid_state {

class RigidStateEstimatorRuntime;

class HealthMonitorState final : public ::state_machine::State {
   public:
    explicit HealthMonitorState(RigidStateEstimatorRuntime& runtime);
    std::string name() const override {
        return "HealthMonitor";
    }
    ::state_machine::ActionResult onEvent(::state_machine::StateContext& ctx,
                                          const ::state_machine::Event& event) override;
    ::state_machine::ActionResult onTick(::state_machine::StateContext& ctx) override;

   private:
    void evaluateAndPostTransition(::state_machine::StateContext& ctx, double now_sec) const;
    RigidStateEstimatorRuntime& runtime_;
};

}  // namespace estimator_rigid_state
