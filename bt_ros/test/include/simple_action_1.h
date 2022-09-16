#pragma once

#include "bt_ros/action_node.h"
#include <actionlib/TestAction.h>
#include <std_msgs/Bool.h>

namespace bt_ros::test
{
class SimpleAction1 : public BT::SimpleActionClientNode<actionlib::TestAction>
{
private:
  static constexpr char LOGNAME[] = "SimpleActionTest1";

  ros::Publisher on_server_unavailable_pub_;
  ros::Publisher on_success_pub_;
  ros::Publisher on_aborted_pub_;
  ros::Publisher on_cancelled_pub_;
  ros::Publisher on_feedback_pub_;
  ros::Publisher on_new_goal_received_pub_;
  int tick_count_ = 0;

public:
  SimpleAction1(const std::string& xml_tag_name, const BT::NodeConfiguration& conf)
    : BT::SimpleActionClientNode<actionlib::TestAction>(xml_tag_name, conf)
  {
    ros::NodeHandle nh;
    on_server_unavailable_pub_ = nh.advertise<std_msgs::Bool>("on_server_unavailable", 1, true);
    on_success_pub_ = nh.advertise<std_msgs::Bool>("on_success", 1, true);
    on_aborted_pub_ = nh.advertise<std_msgs::Bool>("on_aborted", 1, true);
    on_cancelled_pub_ = nh.advertise<std_msgs::Bool>("on_cancelled", 1, true);
    on_feedback_pub_ = nh.advertise<std_msgs::Bool>("on_feedback", 1, true);
    on_new_goal_received_pub_ = nh.advertise<std_msgs::Bool>("on_new_goal_received", 1, true);
  }

  static BT::PortsList providedPorts()
  {
    return { BT::OutputPort<std::string>("out", "some output") };
  }

private:
  inline void onServerUnavailable() final
  {
    ROS_INFO_STREAM("Action client SimpleActionTest1 could not reach action server");
    std_msgs::Bool msg;
    msg.data = true;
    on_server_unavailable_pub_.publish(msg);
  }

  inline BT::NodeStatus onSuccess(const ResultConstPtr& res) final
  {
    ROS_INFO_STREAM("Action client SimpleActionTest1 succeeded");
    std_msgs::Bool msg;
    msg.data = true;
    on_success_pub_.publish(msg);
    return BT::NodeStatus::SUCCESS;
  }

  inline BT::NodeStatus onAborted(const ResultConstPtr& res) final
  {
    ROS_INFO_STREAM("Action client SimpleActionTest1 Aborted");
    std_msgs::Bool msg;
    msg.data = true;
    on_aborted_pub_.publish(msg);
    return BT::NodeStatus::FAILURE;
  }

  inline BT::NodeStatus onCancelled() final
  {
    ROS_INFO_STREAM("Action client SimpleActionTest1 Cancelled");
    std_msgs::Bool msg;
    msg.data = true;
    on_cancelled_pub_.publish(msg);
    return BT::NodeStatus::SUCCESS;
  }

  inline void onFeedback(const FeedbackConstPtr& feedback) final
  {
    ROS_INFO_STREAM("Action client SimpleActionTest1 waiting for result. Feedback: " << feedback->feedback);
    if (goal_.goal == 3 && feedback->feedback > 3)
    {
      std_msgs::Bool msg;
      msg.data = true;
      on_feedback_pub_.publish(msg);
      action_client_->cancelGoal();
    }
    if (goal_.goal == 6 && feedback->feedback == 999)
    {
      std_msgs::Bool msg;
      msg.data = true;
      on_feedback_pub_.publish(msg);
      action_client_->cancelGoal();
    }
  }

  inline bool onNewGoalReceived(const GoalType& goal) final
  {
    ROS_INFO_STREAM("Action client SimpleActionTest1 onNewGoalReceived");
    std_msgs::Bool msg;
    msg.data = true;
    on_new_goal_received_pub_.publish(msg);
    return true;
  }

  inline void onTick() final
  {
    ROS_INFO_STREAM("Action client SimpleActionTest1 ticked");
    ++tick_count_;
    if (goal_.goal == 4 && tick_count_ > 3)
    {
      ROS_INFO_STREAM("Action client SimpleActionTest1 cancelling");
      action_client_->cancelGoal();
    }
    if (goal_.goal == 5)
    {
      setOutput("out", "1");
    }
  }
};
}  // namespace bt_ros::test

namespace BT
{
template <>
inline actionlib::TestGoal convertFromString(StringView str)
{
  auto parts = splitString(str, ';');
  if (parts.size() != 1)
  {
    throw RuntimeError("invalid input TestGoal, expecting a single integer");
  }

  actionlib::TestGoal goal;
  goal.goal = convertFromString<int>(parts[0]);
  return goal;
}
}  // namespace BT
