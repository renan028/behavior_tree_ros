#include <actionlib/TestAction.h>
#include "bt_ros/action_server_node.h"
#include <std_msgs/Bool.h>

class SimpleServer1 : public BT::SimpleActionServerNode<actionlib::TestAction>
{
private:
  ros::Publisher on_configuration_;
  ros::Publisher on_load_fail_;
  ros::Publisher on_new_goal_;

  bool onConfiguration(const GoalConstPtr& goal) final
  {
    ROS_INFO_STREAM("Action server SimpleServer1 configured");
    result_.result = 11;
    std_msgs::Bool msg;
    msg.data = !fail_configuration;
    on_configuration_.publish(msg);
    return !fail_configuration;
  }

  void onLoadFail() final
  {
    ROS_INFO_STREAM("Action server SimpleServer1 failed to load");
    result_.result = 12;
    std_msgs::Bool msg;
    msg.data = true;
    on_load_fail_.publish(msg);
  }

  bool onNewGoal(const GoalConstPtr& goal) final
  {
    ROS_INFO_STREAM("Action server SimpleServer1 received new goal");
    std_msgs::Bool msg;
    msg.data = true;
    on_new_goal_.publish(msg);
    if (abort_on_new_goal)
    {
      result_.result = 7;
      return false;
    }
    return true;
  }

  void onFeedback() final
  {
    ROS_INFO_STREAM("Action server SimpleServer1 sending feedback");
    FeedbackType feedback;
    feedback.feedback = feedback_count;
    std_msgs::Bool msg;
    msg.data = true;
    action_server_.publishFeedback(feedback);
    feedback_count = feedback_count == INT_MAX ? 0 : feedback_count + 1;
  }

  void onFinish(const BT::NodeStatus status) final
  {
    if (status == BT::NodeStatus::SUCCESS)
    {
      ROS_INFO_STREAM("Action server SimpleServer1 succeeded");
      result_.result = 5;
    }
    else if (status == BT::NodeStatus::FAILURE)
    {
      ROS_INFO_STREAM("Action server SimpleServer1 failed");
      result_.result = 1;
    }
    else
    {
      ROS_INFO_STREAM("Action server SimpleServer1 unknown status");
      result_.result = 2;
    }
    return;
  }

  void onCancel() final
  {
    ROS_INFO_STREAM("Action server SimpleServer1 cancelled");
    result_.result = 3;
    return;
  }

public:
  SimpleServer1(ros::NodeHandle& pnh, const std::string& name)
    : SimpleActionServerNode<actionlib::TestAction>(pnh, name)
  {
    ros::NodeHandle nh;
    on_configuration_ = nh.advertise<std_msgs::Bool>("on_configuration", 1, true);
    on_load_fail_ = nh.advertise<std_msgs::Bool>("on_load_fail", 1, true);
    on_new_goal_ = nh.advertise<std_msgs::Bool>("on_new_goal", 1, true);
  }

public:
  bool fail_configuration = false;
  int feedback_count = 1;
  bool abort_on_new_goal = false;

  void setRestartOnNewGoal(bool restart)
  {
    restart_on_new_goal_ = restart;
  }
};
