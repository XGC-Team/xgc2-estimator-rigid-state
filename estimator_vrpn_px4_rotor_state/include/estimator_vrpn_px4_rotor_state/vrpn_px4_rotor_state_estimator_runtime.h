#pragma once

#include <memory>
#include <mutex>
#include <state_machine/state_machine.hpp>

#include "estimator_vrpn_px4_rotor_state/common/event_types.h"
#include "estimator_vrpn_px4_rotor_state/common/types.h"

namespace estimator_vrpn_px4_rotor_state {

class VrpnPx4RotorStateEstimatorRuntime {
   public:
    VrpnPx4RotorStateEstimatorRuntime();
    VrpnPx4RotorStateEstimatorRuntime(const VrpnPx4RotorStateEstimatorRuntime&) = delete;
    VrpnPx4RotorStateEstimatorRuntime& operator=(const VrpnPx4RotorStateEstimatorRuntime&) = delete;

    void setConfig(const VrpnPx4RotorStateEstimatorConfig& config);
    void reset();
    ::state_machine::Status postInputEvent(::state_machine::Event event,
                                           const VrpnPx4RotorStateEstimatorInput& input);
    VrpnPx4RotorStateEstimatorOutput update(double now_sec);
    VrpnPx4RotorStateEstimatorOutput snapshotOutput() const;
    VrpnPx4RotorStateEstimatorOutput refreshOutputSnapshot();

    ::state_machine::StateMachine& getStateMachine() {
        return *machine_;
    }
    const ::state_machine::StateMachine& getStateMachine() const {
        return *machine_;
    }

    const VrpnPx4RotorStateEstimatorConfig& config() const {
        return config_;
    }
    const VrpnPx4RotorStateEstimatorInput& input() const {
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
    uint32_t outputFlags() const;
    xgc2_math::Pose3InertialEskf& estimator() {
        return estimator_;
    }
    const xgc2_math::Pose3InertialEskf& estimator() const {
        return estimator_;
    }

    void enterState(::state_machine::StateId state);
    void initializeIfReady();
    void processImuInput();
    void processVrpnInput();
    void processVrpnVelocityInput();
    void recordStateOutput(::state_machine::StateId state, uint32_t flags);
    void markInnovationRejected();

   private:
    void setupMachine();
    VrpnPx4RotorStateEstimatorOutput makeOutput(::state_machine::StateId state,
                                                uint32_t flags) const;

    VrpnPx4RotorStateEstimatorConfig config_{};
    VrpnPx4RotorStateEstimatorInput input_{};
    HealthStatus health_{};
    xgc2_math::Pose3InertialEskf estimator_{};
    std::unique_ptr<::state_machine::StateMachine> machine_;
    ::state_machine::StateId state_{state_type::SelfCheck};
    uint32_t estimator_flags_{0};
    xgc2_math::PoseFusionRejectReason last_pose_reject_reason_{
        xgc2_math::PoseFusionRejectReason::kNone};
    xgc2_math::Pose3InertialEskf::PoseUpdateResult last_pose_update_result_{};
    bool last_pose_accepted_{false};
    double current_time_sec_{0.0};
    bool fault_requested_{false};
    mutable std::mutex output_mutex_;
    VrpnPx4RotorStateEstimatorOutput last_output_{};
};

}  // namespace estimator_vrpn_px4_rotor_state
