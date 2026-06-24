#pragma once

#include <state_machine/state_machine.hpp>

#include "estimator_vrpn_ugv_state/common/periodic_gate.h"

namespace estimator_vrpn_ugv_state {

class VrpnUgvStateEstimatorRuntime;

class SelfCheckState final : public ::state_machine::State {
   public:
    explicit SelfCheckState(VrpnUgvStateEstimatorRuntime& runtime);
    std::string name() const override {
        return "SelfCheck";
    }
    ::state_machine::ActionResult onEnter(::state_machine::StateContext& ctx) override;
    ::state_machine::ActionResult onTick(::state_machine::StateContext& ctx) override;
    ::state_machine::ActionResult onExit(::state_machine::StateContext& ctx) override;

   private:
    void emitOutputIfDue(::state_machine::StateContext& ctx);
    VrpnUgvStateEstimatorRuntime& runtime_;
    PeriodicGate state_publish_gate_{};
};

}  // namespace estimator_vrpn_ugv_state
