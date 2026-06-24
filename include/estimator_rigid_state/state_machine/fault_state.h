#pragma once

#include <state_machine/state_machine.hpp>

#include "estimator_rigid_state/common/periodic_gate.h"

namespace estimator_rigid_state {

class RigidStateEstimatorRuntime;

class FaultState final : public ::state_machine::State {
   public:
    explicit FaultState(RigidStateEstimatorRuntime& runtime);
    std::string name() const override {
        return "Fault";
    }
    ::state_machine::ActionResult onEnter(::state_machine::StateContext& ctx) override;
    ::state_machine::ActionResult onTick(::state_machine::StateContext& ctx) override;
    ::state_machine::ActionResult onExit(::state_machine::StateContext& ctx) override;

   private:
    void emitOutputIfDue(::state_machine::StateContext& ctx);
    RigidStateEstimatorRuntime& runtime_;
    PeriodicGate state_publish_gate_{};
};

}  // namespace estimator_rigid_state
