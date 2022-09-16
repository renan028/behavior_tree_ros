// bt
#include "include/test_helper.h"
#include "bt_ros/executor_bt.h"

// ros
#include <behaviortree_cpp_v3/utils/shared_library.h>
#include <std_msgs/Bool.h>
#include <std_srvs/SetBool.h>

class ServiceNodeTest : public TestHelper
{
protected:
  behavior_tree_ros::ExecutorBT bt_exec;
  std_msgs::BoolConstPtr on_failed_msg_ = nullptr;
  std_msgs::BoolConstPtr on_response_msg_ = nullptr;
  ros::Subscriber on_failed_sub_;
  ros::Subscriber on_response_sub_;
  ros::ServiceServer service_server_;

  bool return_false_ = false;

  virtual void SetUp()
  {
    ros::NodeHandle nh;
    on_failed_sub_ = nh.subscribe("on_failed", 1, &ServiceNodeTest::onFailedCb, this);
    on_response_sub_ = nh.subscribe("on_response", 1, &ServiceNodeTest::onResponseCb, this);
    service_server_ = nh.advertiseService("dummy_service", &ServiceNodeTest::dummyServiceCb, this);

    TestHelper::SetUp();
    auto& factory = bt_exec.getFactory();
    factory.registerFromPlugin(BT::SharedLibrary::getOSName("dummy_nodes_dyn"));
  }

  void onFailedCb(const std_msgs::BoolConstPtr& msg)
  {
    on_failed_msg_ = msg;
  }

  void onResponseCb(const std_msgs::BoolConstPtr& msg)
  {
    on_response_msg_ = msg;
  }

  bool dummyServiceCb(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res)
  {
    if (return_false_)
    {
      res.success = false;
      return false;
    }
    else
    {
      res.success = true;
      return true;
    }
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

TEST_F(ServiceNodeTest, TestOk)
{
  static const char* xml_text = R"(
  <root main_tree_to_execute="BehaviorTree">
    <BehaviorTree ID="BehaviorTree">
      <Action ID="DummyService"
              service_name="dummy_service"
              timeout="5.0"/>
    </BehaviorTree>
  </root>)";

  testSuccess(bt_exec, xml_text);
  waitForMsg(on_response_msg_, 2.0);
  ASSERT_TRUE(on_response_msg_);
  EXPECT_TRUE(on_response_msg_->data);
  ASSERT_FALSE(on_failed_msg_);
}

TEST_F(ServiceNodeTest, MissingServer)
{
  static const char* xml_text = R"(
  <root main_tree_to_execute="BehaviorTree">
    <BehaviorTree ID="BehaviorTree">
      <Action ID="DummyService"
              service_name="dummy_service_2"
              timeout="1.0"/>
    </BehaviorTree>
  </root>)";

  testFailure(bt_exec, xml_text);
  waitForMsg(on_failed_msg_, 2.0);
  ASSERT_TRUE(on_failed_msg_);
  EXPECT_FALSE(on_failed_msg_->data);
  ASSERT_FALSE(on_response_msg_);
}

TEST_F(ServiceNodeTest, FailedCall)
{
  static const char* xml_text = R"(
  <root main_tree_to_execute="BehaviorTree">
    <BehaviorTree ID="BehaviorTree">
      <Action ID="DummyService"
              service_name="dummy_service"
              timeout="1.0"/>
    </BehaviorTree>
  </root>)";

  return_false_ = true;

  testFailure(bt_exec, xml_text);
  waitForMsg(on_failed_msg_, 2.0);
  ASSERT_TRUE(on_failed_msg_);
  EXPECT_TRUE(on_failed_msg_->data);
  ASSERT_FALSE(on_response_msg_);
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
