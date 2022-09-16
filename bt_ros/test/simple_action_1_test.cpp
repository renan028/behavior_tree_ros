#include "include/test_helper.h"
#include "bt_ros/executor_bt.h"
#include <behaviortree_cpp_v3/utils/shared_library.h>
#include <std_msgs/Bool.h>

class SimpleAction1Test : public TestHelper
{
protected:
  behavior_tree_ros::ExecutorBT bt_exec;
  std_msgs::BoolConstPtr on_server_unavailable_msg_ = nullptr;
  std_msgs::BoolConstPtr on_success_msg_ = nullptr;
  std_msgs::BoolConstPtr on_cancelled_msg_ = nullptr;
  std_msgs::BoolConstPtr on_aborted_msg_ = nullptr;
  std_msgs::BoolConstPtr on_feedback_msg_ = nullptr;
  std_msgs::BoolConstPtr on_new_goal_msg_ = nullptr;
  ros::Subscriber on_server_unavailable_sub_;
  ros::Subscriber on_success_sub_;
  ros::Subscriber on_cancelled_sub_;
  ros::Subscriber on_aborted_sub_;
  ros::Subscriber on_feedback_sub_;
  ros::Subscriber on_new_goal_sub_;

  virtual void SetUp()
  {
    ros::NodeHandle nh;
    on_server_unavailable_sub_ =
        nh.subscribe("on_server_unavailable", 1, &SimpleAction1Test::onServerUnavailableCb, this);
    on_success_sub_ = nh.subscribe("on_success", 1, &SimpleAction1Test::onSuccessCb, this);
    on_cancelled_sub_ = nh.subscribe("on_cancelled", 1, &SimpleAction1Test::onCancelledCb, this);
    on_aborted_sub_ = nh.subscribe("on_aborted", 1, &SimpleAction1Test::onAbortedCb, this);
    on_feedback_sub_ = nh.subscribe("on_feedback", 1, &SimpleAction1Test::onFeedbackCb, this);
    on_new_goal_sub_ = nh.subscribe("on_new_goal_received", 1, &SimpleAction1Test::onNewGoalCb, this);

    TestHelper::SetUp();
    auto& factory = bt_exec.getFactory();
    factory.registerFromPlugin(BT::SharedLibrary::getOSName("dummy_nodes_dyn"));
  }

  void onServerUnavailableCb(const std_msgs::BoolConstPtr& msg)
  {
    on_server_unavailable_msg_ = msg;
  }
  void onSuccessCb(const std_msgs::BoolConstPtr& msg)
  {
    on_success_msg_ = msg;
  }
  void onCancelledCb(const std_msgs::BoolConstPtr& msg)
  {
    on_cancelled_msg_ = msg;
  }
  void onAbortedCb(const std_msgs::BoolConstPtr& msg)
  {
    on_aborted_msg_ = msg;
  }
  void onFeedbackCb(const std_msgs::BoolConstPtr& msg)
  {
    on_feedback_msg_ = msg;
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

TEST_F(SimpleAction1Test, TestOk)
{
  static const char* xml_text = R"(
  <root main_tree_to_execute="BehaviorTree">
    <BehaviorTree ID="BehaviorTree">
      <Action ID="SimpleAction1"
              action_name="reference_action"
              server_timeout="5.0"
              goal="1" />
    </BehaviorTree>
  </root>)";

  testSuccess(bt_exec, xml_text);
  waitForMsg(on_success_msg_, 2.0);
  ASSERT_TRUE(on_success_msg_);
  EXPECT_TRUE(on_success_msg_->data);
  EXPECT_FALSE(on_aborted_msg_);
  EXPECT_FALSE(on_cancelled_msg_);
  EXPECT_FALSE(on_feedback_msg_);
}

TEST_F(SimpleAction1Test, TestFail)
{
  static const char* xml_text = R"(
  <root main_tree_to_execute="BehaviorTree">
    <BehaviorTree ID="BehaviorTree">
      <Action ID="SimpleAction1"
              action_name="reference_action"
              server_timeout="5.0"
              goal="2" />
    </BehaviorTree>
  </root>)";

  testFailure(bt_exec, xml_text);
  waitForMsg(on_aborted_msg_, 2.0);
  ASSERT_TRUE(on_aborted_msg_);
  EXPECT_TRUE(on_aborted_msg_->data);
  EXPECT_FALSE(on_success_msg_);
  EXPECT_FALSE(on_cancelled_msg_);
  EXPECT_FALSE(on_feedback_msg_);
}

TEST_F(SimpleAction1Test, ServerUnavailable)
{
  static const char* xml_text = R"(
  <root main_tree_to_execute="BehaviorTree">
    <BehaviorTree ID="BehaviorTree">
      <Action ID="SimpleAction1"
              action_name="reference_action_2"
              server_timeout="2.0"
              goal="2" />
    </BehaviorTree>
  </root>)";

  testFailure(bt_exec, xml_text);
  waitForMsg(on_server_unavailable_msg_, 2.0);
  ASSERT_TRUE(on_server_unavailable_msg_);
  EXPECT_TRUE(on_server_unavailable_msg_->data);
  EXPECT_FALSE(on_success_msg_);
  EXPECT_FALSE(on_cancelled_msg_);
  EXPECT_FALSE(on_aborted_msg_);
  EXPECT_FALSE(on_feedback_msg_);
}

TEST_F(SimpleAction1Test, Preempted)
{
  static const char* xml_text = R"(
  <root main_tree_to_execute="BehaviorTree">
    <BehaviorTree ID="BehaviorTree">
      <Action ID="SimpleAction1" 
              action_name="reference_action"
              server_timeout="2.0" 
              goal="4" />
    </BehaviorTree>
  </root>)";

  testSuccess(bt_exec, xml_text);
  waitForMsg(on_cancelled_msg_, 3.0);
  EXPECT_TRUE(on_cancelled_msg_);
  EXPECT_TRUE(on_cancelled_msg_ && on_cancelled_msg_->data);
  EXPECT_FALSE(on_success_msg_);
  EXPECT_FALSE(on_aborted_msg_);
  EXPECT_FALSE(on_feedback_msg_);
}

TEST_F(SimpleAction1Test, FeedbackPreempted)
{
  static const char* xml_text = R"(
  <root main_tree_to_execute="BehaviorTree">
    <BehaviorTree ID="BehaviorTree">
      <Action ID="SimpleAction1"
              action_name="reference_action"
              server_timeout="2.0"
              goal="3" />
    </BehaviorTree>
  </root>)";

  testSuccess(bt_exec, xml_text);
  waitForMsg(on_feedback_msg_, 2.0);
  ASSERT_TRUE(on_feedback_msg_);
  EXPECT_TRUE(on_feedback_msg_->data);
  EXPECT_TRUE(on_cancelled_msg_->data);
  EXPECT_FALSE(on_aborted_msg_);
}

TEST_F(SimpleAction1Test, NewGoal)
{
  static const char* xml_text = R"(
  <root main_tree_to_execute="BehaviorTree">
    <BehaviorTree ID="BehaviorTree">
      <Sequence>
        <SetBlackboard output_key="out" value="0"/>
        <ReactiveSequence>
          <Switch2 case_1="0" case_2="1" variable="{out}">
            <SetBlackboard output_key="goal" value="5"/>
            <SetBlackboard output_key="goal" value="6"/>
            <SetBlackboard output_key="goal" value="5"/>
          </Switch2>
          <Action ID="SimpleAction1" action_name="reference_action" goal="{goal}" out="{out}" server_timeout="2"/>
        </ReactiveSequence>
      </Sequence>
    </BehaviorTree>
  </root>)";

  testSuccess(bt_exec, xml_text);
  waitForMsg(on_new_goal_msg_, 2.0);
  waitForMsg(on_feedback_msg_, 2.0);

  ASSERT_TRUE(on_feedback_msg_);
  ASSERT_TRUE(on_new_goal_msg_);

  EXPECT_TRUE(on_feedback_msg_->data);
  EXPECT_TRUE(on_new_goal_msg_->data);
  EXPECT_TRUE(on_success_msg_->data);
  EXPECT_FALSE(on_aborted_msg_);
  EXPECT_FALSE(on_cancelled_msg_);
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
