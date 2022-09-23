#include "include/test_helper.h"
#include "bt_ros/executor_bt.h"
#include <behaviortree_cpp_v3/utils/shared_library.h>
#include <std_msgs/Bool.h>
#include "include/simple_server_1.h"
#include <actionlib/client/simple_action_client.h>
#include <actionlib/TestAction.h>

class SimpleServer1Test : public TestHelper
{
protected:
  std_msgs::BoolConstPtr on_configuration_msg_ = nullptr;
  std_msgs::BoolConstPtr on_load_fail_msg_ = nullptr;
  std_msgs::BoolConstPtr on_new_goal_msg_ = nullptr;
  ros::Subscriber on_configuration_sub_;
  ros::Subscriber on_load_fail_sub_;
  ros::Subscriber on_new_goal_sub_;
  actionlib::SimpleActionClient<actionlib::TestAction> client;

  SimpleServer1Test() : client("action", true)
  {
  }

  virtual void SetUp()
  {
    ros::NodeHandle nh;
    on_configuration_sub_ = nh.subscribe("on_configuration", 1, &SimpleServer1Test::onConfigurationCb, this);
    on_load_fail_sub_ = nh.subscribe("on_load_fail", 1, &SimpleServer1Test::onLoadFailCb, this);
    on_new_goal_sub_ = nh.subscribe("on_new_goal", 1, &SimpleServer1Test::onNewGoalCb, this);

    TestHelper::SetUp();
  }

  void onConfigurationCb(const std_msgs::BoolConstPtr& msg)
  {
    on_configuration_msg_ = msg;
  }

  void onLoadFailCb(const std_msgs::BoolConstPtr& msg)
  {
    on_load_fail_msg_ = msg;
  }

  void onNewGoalCb(const std_msgs::BoolConstPtr& msg)
  {
    on_new_goal_msg_ = msg;
  }

  template <typename T>
  void waitForMsg(T msg, double timeout)
  {
    ros::Time start = ros::Time::now();
    while (ros::ok() && !msg && (ros::Time::now() - start).toSec() < timeout)
    {
      ros::spinOnce();
      ros::Duration(0.1).sleep();
    }
  }
};

TEST_F(SimpleServer1Test, ConfigFail)
{
  ros::NodeHandle pnh("~");
  SimpleServer1 server(pnh, "action");
  server.fail_configuration = true;

  actionlib::TestGoal goal;
  client.sendGoal(goal);

  waitForMsg(on_configuration_msg_, 2.0);
  ASSERT_TRUE(on_configuration_msg_);
  EXPECT_FALSE(on_configuration_msg_->data);

  ASSERT_TRUE(client.waitForResult(ros::Duration(1.0)));
  EXPECT_EQ(client.getState(), actionlib::SimpleClientGoalState::ABORTED);
  EXPECT_EQ(client.getResult()->result, 11);
}

TEST_F(SimpleServer1Test, LoadFail)
{
  ros::NodeHandle pnh("~");
  SimpleServer1 server(pnh, "action");
  actionlib::TestGoal goal;
  client.sendGoal(goal);

  waitForMsg(on_configuration_msg_, 2.0);
  waitForMsg(on_load_fail_msg_, 2.0);
  ASSERT_TRUE(on_configuration_msg_);
  ASSERT_TRUE(on_load_fail_msg_);
  EXPECT_TRUE(on_configuration_msg_->data);
  EXPECT_TRUE(on_load_fail_msg_->data);

  ASSERT_TRUE(client.waitForResult(ros::Duration(1.0)));
  EXPECT_EQ(client.getState(), actionlib::SimpleClientGoalState::ABORTED);
  EXPECT_EQ(client.getResult()->result, 12);
}

TEST_F(SimpleServer1Test, Success)
{
  ros::NodeHandle pnh("~");
  pnh.setParam("behavior_name", "test/trees/wait");

  SimpleServer1 server(pnh, "action");
  actionlib::TestGoal goal;
  client.sendGoal(goal);

  waitForMsg(on_configuration_msg_, 1.0);
  ASSERT_TRUE(on_configuration_msg_);

  ASSERT_TRUE(client.waitForResult(ros::Duration(2.0)));
  EXPECT_EQ(client.getState(), actionlib::SimpleClientGoalState::SUCCEEDED);
  EXPECT_EQ(client.getResult()->result, 5);

  pnh.deleteParam("behavior_name");
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "tree_executor");

  ros::AsyncSpinner spinner(0);
  spinner.start();
  testing::InitGoogleTest(&argc, argv);
  auto result = RUN_ALL_TESTS();
  spinner.stop();
  return result;
}
