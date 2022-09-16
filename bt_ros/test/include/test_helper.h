// behavior_tree_ros
#include <bt_ros_msgs/BtState.h>
#include <bt_ros_msgs/LoadTree.h>
#include <bt_ros/executor_bt.h>

// Gtest
#include <gtest/gtest.h>

// ros
#include <ros/ros.h>

class TestHelper : public ::testing::Test
{
protected:
  ros::ServiceClient load_tree_client;
  ros::Subscriber bt_state_sub_;
  ros::Subscriber error_sub_;
  bt_ros_msgs::BtState::Ptr bt_state_;

  virtual void SetUp()
  {
    ros::NodeHandle pnh("~");
    load_tree_client = pnh.serviceClient<bt_ros_msgs::LoadTree>("load_behavior");
    bt_state_sub_ = pnh.subscribe("bt_state", 10, &TestHelper::btStateCallback, this);
  }
  virtual void TearDown()
  {
    bt_state_ = nullptr;
  }

  void loadTree(const std::string& tree_xml_name)
  {
    ASSERT_TRUE(load_tree_client.waitForExistence(ros::Duration(0.1))) << "Timeout Tree not loaded";
    bt_ros_msgs::LoadTree msg;
    msg.request.tree_xml_name = tree_xml_name;
    load_tree_client.call(msg);
    ASSERT_TRUE(msg.response.success) << "Tree not loaded successfully";
  }

  void btStateCallback(const bt_ros_msgs::BtState::ConstPtr& state)
  {
    bt_state_ = boost::make_shared<bt_ros_msgs::BtState>(*state);
  }

  void waitBtStateCb(double timeout = 5.0)
  {
    ros::Rate rate(10);
    auto start = ros::Time::now();
    auto d = ros::Duration(timeout);
    while (bt_state_ == nullptr && ros::ok() && (timeout < 0 || ros::Time::now() - start < d))
    {
      ros::spinOnce();
      rate.sleep();
    }
  }

  bool waitTreeState(bt_ros_msgs::BtState state, double timeout = 5.0)
  {
    ros::Rate rate(10);
    auto start = ros::Time::now();
    auto d = ros::Duration(timeout);
    while (ros::ok() && (timeout < 0 || ros::Time::now() - start < d))
    {
      if (bt_state_->state == state.state)
      {
        ROS_INFO_STREAM("Tree state: " << bt_state_->state);
        return true;
      }
      ros::spinOnce();
      rate.sleep();
    }
    return false;
  }

  void testSuccess(const std::string& tree)
  {
    test(tree, bt_ros_msgs::BtState::SUCCESS);
  }

  void testSuccess(behavior_tree_ros::ExecutorBT& executor, const std::string& tree)
  {
    test(executor, tree, bt_ros_msgs::BtState::SUCCESS);
  }

  void testFailure(const std::string& tree)
  {
    test(tree, bt_ros_msgs::BtState::FAILURE);
  }

  void testFailure(behavior_tree_ros::ExecutorBT& executor, const std::string& tree)
  {
    test(executor, tree, bt_ros_msgs::BtState::FAILURE);
  }

  void test(const std::string& tree, int result)
  {
    bt_ros_msgs::BtState state;
    state.state = result;

    bt_state_ = nullptr;
    loadTree(tree);
    waitBtStateCb(5.0);
    ASSERT_EQ(bt_state_->behavior_xml, tree) << "Tree name not correct";
    ASSERT_TRUE(waitTreeState(state)) << "Tree not in state " << result;
  }

  void test(behavior_tree_ros::ExecutorBT& executor, const std::string& xml_tree, int result)
  {
    bt_ros_msgs::BtState state;
    state.state = result;

    bt_state_ = nullptr;
    executor.load(xml_tree);
    waitBtStateCb(5.0);
    ASSERT_TRUE(waitTreeState(state)) << "Tree not in state " << result;
  }
};
