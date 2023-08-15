#include "bt_ros/controls/executor_node.h"
#include <behaviortree_cpp_v3/bt_factory.h>

#include <gtest/gtest.h>
#include <ros/ros.h>

namespace bt_ros::test
{

class StatefulActionTest : public BT::StatefulActionNode
{
public:
  StatefulActionTest(const std::string& name, const BT::NodeConfiguration& config) : StatefulActionNode(name, config)
  {
    expected_result = BT::NodeStatus::RUNNING;
  }

  BT::NodeStatus onStart() override
  {
    on_start_count++;
    return expected_result;
  }
  BT::NodeStatus onRunning() override
  {
    on_running_count++;
    return expected_result;
  }
  inline void onHalted() override
  {
    on_halted_count++;
  }

  int on_start_count = 0;
  int on_running_count = 0;
  int on_halted_count = 0;
  BT::NodeStatus expected_result;
};

class ExecutorTest : public ::testing::Test
{
};

TEST_F(ExecutorTest, success)
{
  StatefulActionTest stateful("stateful", {});
  std::shared_ptr<BT::ExecutorNode> executor = std::make_shared<BT::ExecutorNode>("executor", 1);
  executor->addChild(&stateful);

  auto status = executor->executeTick();
  EXPECT_EQ(status, BT::NodeStatus::RUNNING);

  stateful.expected_result = BT::NodeStatus::SUCCESS;

  status = executor->executeTick();
  EXPECT_EQ(status, BT::NodeStatus::SUCCESS);
  EXPECT_EQ(stateful.on_start_count, 1);
  EXPECT_EQ(stateful.on_running_count, 1);
  EXPECT_EQ(stateful.on_halted_count, 0);
}

TEST_F(ExecutorTest, failure_after_success)
{
  StatefulActionTest stateful_1("stateful_1", {});
  StatefulActionTest stateful_2("stateful_2", {});
  std::shared_ptr<BT::ExecutorNode> executor = std::make_shared<BT::ExecutorNode>("executor", 1);
  executor->addChild(&stateful_1);
  executor->addChild(&stateful_2);

  auto status = executor->executeTick();
  EXPECT_EQ(status, BT::NodeStatus::RUNNING);

  stateful_1.expected_result = BT::NodeStatus::SUCCESS;
  status = executor->executeTick();
  EXPECT_EQ(status, BT::NodeStatus::RUNNING);
  EXPECT_EQ(stateful_1.on_start_count, 1);
  EXPECT_EQ(stateful_1.on_running_count, 1);
  EXPECT_EQ(stateful_1.on_halted_count, 0);
  EXPECT_EQ(stateful_2.on_start_count, 1);

  executor->halt();
  stateful_1.expected_result = BT::NodeStatus::RUNNING;
  status = executor->executeTick();
  EXPECT_EQ(status, BT::NodeStatus::RUNNING);
  EXPECT_EQ(stateful_1.on_start_count, 2);
  EXPECT_EQ(stateful_1.on_running_count, 1);
  EXPECT_EQ(stateful_1.on_halted_count, 0);
  EXPECT_EQ(stateful_2.on_start_count, 1);
  EXPECT_EQ(stateful_2.on_halted_count, 1);

  stateful_1.expected_result = BT::NodeStatus::FAILURE;
  stateful_2.expected_result = BT::NodeStatus::FAILURE;
  status = executor->executeTick();
  EXPECT_EQ(status, BT::NodeStatus::FAILURE);
  EXPECT_EQ(stateful_2.on_start_count, 2);
  EXPECT_EQ(stateful_2.on_running_count, 0);
}

TEST_F(ExecutorTest, one_success_ok)
{
  StatefulActionTest stateful_1("stateful_1", {});
  StatefulActionTest stateful_2("stateful_2", {});
  std::shared_ptr<BT::ExecutorNode> executor = std::make_shared<BT::ExecutorNode>("executor", 1);
  executor->addChild(&stateful_1);
  executor->addChild(&stateful_2);

  auto status = executor->executeTick();
  EXPECT_EQ(status, BT::NodeStatus::RUNNING);
  EXPECT_EQ(stateful_1.on_start_count, 1);
  EXPECT_EQ(stateful_1.on_running_count, 0);
  EXPECT_EQ(stateful_1.on_halted_count, 0);
  EXPECT_EQ(stateful_2.on_start_count, 0);

  stateful_1.expected_result = BT::NodeStatus::SUCCESS;
  status = executor->executeTick();
  EXPECT_EQ(status, BT::NodeStatus::RUNNING);
  EXPECT_EQ(stateful_1.on_start_count, 1);
  EXPECT_EQ(stateful_1.on_running_count, 1);
  EXPECT_EQ(stateful_1.on_halted_count, 0);
  EXPECT_EQ(stateful_2.on_start_count, 1);

  stateful_2.expected_result = BT::NodeStatus::FAILURE;
  status = executor->executeTick();
  EXPECT_EQ(status, BT::NodeStatus::SUCCESS);
  EXPECT_EQ(stateful_1.on_start_count, 1);
  EXPECT_EQ(stateful_1.on_running_count, 1);
  EXPECT_EQ(stateful_1.on_halted_count, 0);
  EXPECT_EQ(stateful_2.on_start_count, 1);
  EXPECT_EQ(stateful_2.on_halted_count, 0);
}

TEST_F(ExecutorTest, one_success_one_failure)
{
  StatefulActionTest stateful_1("stateful_1", {});
  StatefulActionTest stateful_2("stateful_2", {});
  std::shared_ptr<BT::ExecutorNode> executor = std::make_shared<BT::ExecutorNode>("executor", 2);
  executor->addChild(&stateful_1);
  executor->addChild(&stateful_2);

  auto status = executor->executeTick();
  EXPECT_EQ(status, BT::NodeStatus::RUNNING);
  EXPECT_EQ(stateful_1.on_start_count, 1);
  EXPECT_EQ(stateful_1.on_running_count, 0);
  EXPECT_EQ(stateful_1.on_halted_count, 0);
  EXPECT_EQ(stateful_2.on_start_count, 0);

  stateful_1.expected_result = BT::NodeStatus::SUCCESS;
  status = executor->executeTick();
  EXPECT_EQ(status, BT::NodeStatus::RUNNING);
  EXPECT_EQ(stateful_1.on_start_count, 1);
  EXPECT_EQ(stateful_1.on_running_count, 1);
  EXPECT_EQ(stateful_1.on_halted_count, 0);
  EXPECT_EQ(stateful_2.on_start_count, 1);

  stateful_2.expected_result = BT::NodeStatus::FAILURE;
  status = executor->executeTick();
  EXPECT_EQ(status, BT::NodeStatus::FAILURE);
  EXPECT_EQ(stateful_1.on_start_count, 1);
  EXPECT_EQ(stateful_1.on_running_count, 1);
  EXPECT_EQ(stateful_1.on_halted_count, 0);
  EXPECT_EQ(stateful_2.on_start_count, 1);
  EXPECT_EQ(stateful_2.on_halted_count, 0);
}

TEST_F(ExecutorTest, one_failure_one_success)
{
  StatefulActionTest stateful_1("stateful_1", {});
  StatefulActionTest stateful_2("stateful_2", {});
  std::shared_ptr<BT::ExecutorNode> executor = std::make_shared<BT::ExecutorNode>("executor", 1);
  executor->addChild(&stateful_1);
  executor->addChild(&stateful_2);

  auto status = executor->executeTick();
  EXPECT_EQ(status, BT::NodeStatus::RUNNING);
  EXPECT_EQ(stateful_1.on_start_count, 1);
  EXPECT_EQ(stateful_1.on_running_count, 0);
  EXPECT_EQ(stateful_1.on_halted_count, 0);
  EXPECT_EQ(stateful_2.on_start_count, 0);

  stateful_1.expected_result = BT::NodeStatus::FAILURE;
  status = executor->executeTick();
  EXPECT_EQ(status, BT::NodeStatus::RUNNING);
  EXPECT_EQ(stateful_1.on_start_count, 1);
  EXPECT_EQ(stateful_1.on_running_count, 1);
  EXPECT_EQ(stateful_1.on_halted_count, 0);
  EXPECT_EQ(stateful_2.on_start_count, 1);

  stateful_2.expected_result = BT::NodeStatus::SUCCESS;
  status = executor->executeTick();
  EXPECT_EQ(status, BT::NodeStatus::SUCCESS);
  EXPECT_EQ(stateful_1.on_start_count, 1);
  EXPECT_EQ(stateful_1.on_running_count, 1);
  EXPECT_EQ(stateful_1.on_halted_count, 0);
  EXPECT_EQ(stateful_2.on_start_count, 1);
  EXPECT_EQ(stateful_2.on_halted_count, 0);
}

}  // namespace bt_ros::test

int main(int argc, char** argv)
{
  ros::init(argc, argv, "executor_test");

  // enable debug logging for tests
  if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug))
    ros::console::notifyLoggerLevelsChanged();

  ros::AsyncSpinner spinner(0);
  spinner.start();
  testing::InitGoogleTest(&argc, argv);
  auto result = RUN_ALL_TESTS();
  spinner.stop();
  return result;
}
