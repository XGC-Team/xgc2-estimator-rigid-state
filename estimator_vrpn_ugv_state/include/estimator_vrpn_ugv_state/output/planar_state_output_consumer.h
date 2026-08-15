#pragma once

#include <rigid_state_estimator_msgs/PlanarStateEstimate.h>
#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>

#include <state_machine/runtime/async_task_executor.hpp>
#include <state_machine/runtime/event_dispatcher.hpp>
#include <string>

#include "estimator_vrpn_ugv_state/common/types.h"

namespace estimator_vrpn_ugv_state {

class VrpnUgvStateEstimatorRuntime;

class PlanarStateOutputConsumer final : public ::state_machine::runtime::EventConsumer {
   public:
    PlanarStateOutputConsumer(
        ros::NodeHandle& nh, ::state_machine::runtime::AsyncTaskExecutor<ros::NodeHandle>& executor,
        VrpnUgvStateEstimatorRuntime& runtime, std::string state_topic, std::string world_frame,
        std::string estimator_frame, bool publish_tf, uint32_t queue_size);

    std::string name() const override {
        return "PlanarStateOutputConsumer";
    }
    bool handle(const ::state_machine::Event& event) override;

    static rigid_state_estimator_msgs::PlanarStateEstimate makeStateMessage(
        const VrpnUgvStateEstimatorOutput& output, const ros::Time& stamp,
        const std::string& frame_id);

   private:
    ros::Publisher state_pub_;
    tf2_ros::TransformBroadcaster tf_br_;
    ::state_machine::runtime::AsyncTaskExecutor<ros::NodeHandle>& executor_;
    VrpnUgvStateEstimatorRuntime& runtime_;
    std::string world_frame_;
    std::string estimator_frame_;
    bool publish_tf_{true};
};

}  // namespace estimator_vrpn_ugv_state
