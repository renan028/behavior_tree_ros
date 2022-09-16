#pragma once

// bt
#include "bt_ros/service_node.h"

// ros
#include <std_srvs/SetBool.h>
#include <std_msgs/Bool.h>
#include <ros/ros.h>

namespace bt_ros::test
{

class DummyService : public BT::ServiceNode<std_srvs::SetBool>
{
private:
  ros::Publisher on_failed_pub_;
  ros::Publisher on_response_pub_;

  inline void sendRequest(RequestType& request) final
  {
    request.data = true;
  }

  inline BT::NodeStatus onFailedRequest(FailureCause failure) final
  {
    std_msgs::Bool msg;
    if (FailureCause::MISSING_SERVER == failure)
    {
      msg.data = false;
    }
    else
    {
      msg.data = true;
    }
    on_failed_pub_.publish(msg);

    return BT::NodeStatus::FAILURE;
  }

  inline BT::NodeStatus onResponse(const ResponseType& rep)
  {
    std_msgs::Bool msg;
    msg.data = true;
    on_response_pub_.publish(msg);
    return BT::NodeStatus::SUCCESS;
  }

public:
  DummyService(const std::string& name, const BT::NodeConfiguration& conf)
    : BT::ServiceNode<std_srvs::SetBool>(name, conf)
  {
    on_failed_pub_ = node_.advertise<std_msgs::Bool>("on_failed", 1, true);
    on_response_pub_ = node_.advertise<std_msgs::Bool>("on_response", 1, true);
  }
};

}  // namespace bt_ros::test
