#pragma once

#include <ros/ros.h>

#include <state_machine/runtime/async_task_executor.hpp>
#include <state_machine/runtime/event_dispatcher.hpp>
#include <string>

namespace estimator_vrpn_px4_rotor_state {

class VrpnPx4RotorStateEstimatorRuntime;

class RigidStateOutputConsumer final : public ::state_machine::runtime::EventConsumer {
   public:
    RigidStateOutputConsumer(ros::NodeHandle& nh,
                             ::state_machine::runtime::AsyncTaskExecutor<ros::NodeHandle>& executor,
                             VrpnPx4RotorStateEstimatorRuntime& runtime, std::string state_topic,
                             std::string vision_pose_topic, uint32_t queue_size);

    std::string name() const override {
        return "RigidStateOutputConsumer";
    }
    bool handle(const ::state_machine::Event& event) override;

   private:
    ::state_machine::runtime::AsyncTaskExecutor<ros::NodeHandle>& executor_;
    VrpnPx4RotorStateEstimatorRuntime& runtime_;
    ros::Publisher state_pub_;
    ros::Publisher vision_pose_pub_;
};

}  // namespace estimator_vrpn_px4_rotor_state
