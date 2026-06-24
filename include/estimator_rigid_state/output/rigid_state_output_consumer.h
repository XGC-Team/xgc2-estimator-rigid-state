#pragma once

#include <estimator_rigid_state/RigidStateEstimate.h>
#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>

#include <state_machine/runtime/async_task_executor.hpp>
#include <state_machine/runtime/event_dispatcher.hpp>
#include <string>

#include "estimator_rigid_state/common/types.h"

namespace estimator_rigid_state {

class RigidStateEstimatorRuntime;

class RigidStateOutputConsumer final : public ::state_machine::runtime::EventConsumer {
   public:
    RigidStateOutputConsumer(ros::NodeHandle& nh,
                             ::state_machine::runtime::AsyncTaskExecutor<ros::NodeHandle>& executor,
                             RigidStateEstimatorRuntime& runtime, std::string state_topic,
                             std::string vision_pose_topic, uint32_t queue_size);

    std::string name() const override {
        return "RigidStateOutputConsumer";
    }
    bool handle(const ::state_machine::Event& event) override;

   private:
    static estimator_rigid_state::RigidStateEstimate makeStateMessage(
        const RigidStateEstimatorOutput& output, const ros::Time& stamp);
    static geometry_msgs::PoseStamped makeVisionPoseMessage(const RigidStateEstimatorOutput& output,
                                                            const ros::Time& stamp);
    static bool canPublishVisionPose(const RigidStateEstimatorOutput& output);

    ::state_machine::runtime::AsyncTaskExecutor<ros::NodeHandle>& executor_;
    RigidStateEstimatorRuntime& runtime_;
    ros::Publisher state_pub_;
    ros::Publisher vision_pose_pub_;
};

}  // namespace estimator_rigid_state
