#pragma once

#include <memory>
#include <mutex>
#include <state_machine/state_machine.hpp>

#include "estimator_vrpn_ugv_state/common/event_types.h"
#include "estimator_vrpn_ugv_state/common/types.h"

namespace estimator_vrpn_ugv_state {

class VrpnUgvStateEstimatorRuntime {
   public:
    VrpnUgvStateEstimatorRuntime();
    VrpnUgvStateEstimatorRuntime(const VrpnUgvStateEstimatorRuntime&) = delete;
    VrpnUgvStateEstimatorRuntime& operator=(const VrpnUgvStateEstimatorRuntime&) = delete;

    void setConfig(const VrpnUgvStateEstimatorConfig& config);
    void reset();
    ::state_machine::Status postInputEvent(::state_machine::Event event,
                                           const VrpnUgvStateEstimatorInput& input);
    VrpnUgvStateEstimatorOutput update(double now_sec);
    VrpnUgvStateEstimatorOutput snapshotOutput() const;
    VrpnUgvStateEstimatorOutput refreshOutputSnapshot();

    ::state_machine::StateMachine& getStateMachine() {
        return *machine_;
    }
    const ::state_machine::StateMachine& getStateMachine() const {
        return *machine_;
    }
    const VrpnUgvStateEstimatorConfig& config() const {
        return config_;
    }
    const VrpnUgvStateEstimatorInput& input() const {
        return input_;
    }
    const HealthStatus& health() const {
        return health_;
    }
    void setHealth(const HealthStatus& health) {
        health_ = health;
    }
    ::state_machine::StateId currentState() const {
        return state_;
    }
    double currentTime() const {
        return current_time_sec_;
    }
    bool faultRequested() const {
        return fault_requested_;
    }
    uint32_t estimatorFlags() const {
        return estimator_flags_;
    }
    xgc2_math::PlanarInertialEskf& estimator() {
        return estimator_;
    }
    const xgc2_math::PlanarInertialEskf& estimator() const {
        return estimator_;
    }

    void enterState(::state_machine::StateId state);
    void initializeIfReady();
    void processImuInput();
    void processVrpnInput();
    void recordStateOutput(::state_machine::StateId state, uint32_t flags);

   private:
    void setupMachine();
    VrpnUgvStateEstimatorOutput makeOutput(::state_machine::StateId state, uint32_t flags) const;

    VrpnUgvStateEstimatorConfig config_{};
    VrpnUgvStateEstimatorInput input_{};
    HealthStatus health_{};
    xgc2_math::PlanarInertialEskf estimator_{};
    std::unique_ptr<::state_machine::StateMachine> machine_;
    ::state_machine::StateId state_{state_type::SelfCheck};
    uint32_t estimator_flags_{0};
    double current_time_sec_{0.0};
    bool fault_requested_{false};
    mutable std::mutex output_mutex_;
    VrpnUgvStateEstimatorOutput last_output_{};
};

}  // namespace estimator_vrpn_ugv_state
