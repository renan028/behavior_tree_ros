#include <gtest/gtest.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>

#include "bt_ros/subscriber_node.h"

namespace BT::test
{

class BoolSubscriber : public BT::SubscriberNode<std_msgs::Bool>
{
public:
  int finish_count = 0;
  int failure_count = 0;
  int true_count = 0;
  int false_count = 0;
  int halt_count = 0;

protected:
  inline BT::NodeStatus onFinish() override
  {
    finish_count++;
    if (msg_.data)
    {
      true_count++;
    }
    else
    {
      false_count++;
    }
    return BT::NodeStatus::SUCCESS;
  }

  inline BT::NodeStatus onFailure() override
  {
    failure_count++;
    return BT::NodeStatus::FAILURE;
  }

  inline void onHalt() override
  {
    halt_count++;
  }

public:
  BoolSubscriber(const std::string& name, const BT::NodeConfiguration& conf)
    : BT::SubscriberNode<std_msgs::Bool>(name, conf)
  {
  }
};

class SubscriberNodeTest : public ::testing::Test
{
protected:
  virtual void SetUp()
  {
    config.blackboard = BT::Blackboard::create();
    config.blackboard->set("topic_name", "topic");
    config.blackboard->set("timeout", 1.0);
    BT::assignDefaultRemapping<BoolSubscriber>(config);

    ros::NodeHandle nh;
    latch_pub_ = nh.advertise<std_msgs::Bool>("topic", 10, true);
    pub_ = nh.advertise<std_msgs::Bool>("topic", 10, false);
  }

  virtual void TearDown()
  {
    pub_.shutdown();
    latch_pub_.shutdown();
  }

  inline BT::NodeStatus tickWhileRunning(std::shared_ptr<BT::ControlNode> cn, double timeout = 5)
  {
    ros::Time start_time = ros::Time::now();
    BT::NodeStatus status;

    while (ros::ok() && ros::Time::now() - start_time < ros::Duration(timeout))
    {
      status = cn->executeTick();
      if (status != BT::NodeStatus::RUNNING)
      {
        return status;
      }
    }
    return status;
  }

  inline void keepPublishing(bool data) const
  {
    ros::Rate rate(10);
    while (ros::ok() && !terminate)
    {
      ROS_DEBUG_STREAM("Publishing " << data << " at time " << ros::Time::now());
      std_msgs::Bool msg;
      msg.data = data;
      pub_.publish(msg);
      rate.sleep();
    }
  }

  inline void pubLatch(bool data) const
  {
    std_msgs::Bool msg;
    msg.data = data;
    latch_pub_.publish(msg);
  }

  bool terminate = false;
  BT::NodeConfiguration config;

private:
  ros::Publisher latch_pub_;
  ros::Publisher pub_;
};

TEST_F(SubscriberNodeTest, success)
{
  BoolSubscriber subscriber("subscriber", config);
  std::shared_ptr<BT::SequenceNode> sequence = std::make_shared<BT::SequenceNode>("sequence");
  sequence->addChild(&subscriber);

  // run node until it finishes
  std::thread pub_thread([&]() { keepPublishing(true); });
  const auto status = tickWhileRunning(sequence);
  terminate = true;
  pub_thread.join();
  EXPECT_EQ(status, BT::NodeStatus::SUCCESS);
  EXPECT_EQ(subscriber.finish_count, 1);
  EXPECT_EQ(subscriber.failure_count, 0);
  EXPECT_EQ(subscriber.true_count, 1);
  EXPECT_EQ(subscriber.false_count, 0);
  EXPECT_EQ(subscriber.halt_count, 0);
}

TEST_F(SubscriberNodeTest, success_latch)
{
  BoolSubscriber subscriber("subscriber", config);
  std::shared_ptr<BT::SequenceNode> sequence = std::make_shared<BT::SequenceNode>("sequence");
  sequence->addChild(&subscriber);

  // run node until it finishes
  pubLatch(true);
  const auto status = tickWhileRunning(sequence);
  EXPECT_EQ(status, BT::NodeStatus::SUCCESS);
  EXPECT_EQ(subscriber.finish_count, 1);
  EXPECT_EQ(subscriber.failure_count, 0);
  EXPECT_EQ(subscriber.true_count, 1);
  EXPECT_EQ(subscriber.false_count, 0);
  EXPECT_EQ(subscriber.halt_count, 0);
}

TEST_F(SubscriberNodeTest, failure)
{
  BoolSubscriber subscriber("subscriber", config);
  std::shared_ptr<BT::SequenceNode> sequence = std::make_shared<BT::SequenceNode>("sequence");
  sequence->addChild(&subscriber);

  const auto status = tickWhileRunning(sequence);
  EXPECT_EQ(status, BT::NodeStatus::FAILURE);
  EXPECT_EQ(subscriber.finish_count, 0);
  EXPECT_EQ(subscriber.failure_count, 1);
  EXPECT_EQ(subscriber.true_count, 0);
  EXPECT_EQ(subscriber.false_count, 0);
  EXPECT_EQ(subscriber.halt_count, 0);
}

TEST_F(SubscriberNodeTest, rerun_change)
{
  BoolSubscriber subscriber("subscriber", config);
  std::shared_ptr<BT::SequenceNode> sequence = std::make_shared<BT::SequenceNode>("sequence");
  sequence->addChild(&subscriber);

  // run node until it finishes
  std::thread pub_thread([&]() { keepPublishing(true); });
  const auto status = tickWhileRunning(sequence);
  terminate = true;
  pub_thread.join();
  EXPECT_EQ(status, BT::NodeStatus::SUCCESS);
  EXPECT_EQ(subscriber.finish_count, 1);
  EXPECT_EQ(subscriber.failure_count, 0);
  EXPECT_EQ(subscriber.true_count, 1);
  EXPECT_EQ(subscriber.false_count, 0);
  EXPECT_EQ(subscriber.halt_count, 0);

  // change pub to false and run again
  sequence->halt();
  terminate = false;
  pub_thread = std::thread([&]() { keepPublishing(false); });
  const auto status2 = tickWhileRunning(sequence);
  terminate = true;
  pub_thread.join();
  EXPECT_EQ(status2, BT::NodeStatus::SUCCESS);
  EXPECT_EQ(subscriber.finish_count, 2);
  EXPECT_EQ(subscriber.failure_count, 0);
  EXPECT_EQ(subscriber.true_count, 1);
  EXPECT_EQ(subscriber.false_count, 1);
  EXPECT_EQ(subscriber.halt_count, 0);
}

TEST_F(SubscriberNodeTest, rerun_latch_no_change)
{
  BoolSubscriber subscriber("subscriber", config);
  std::shared_ptr<BT::SequenceNode> sequence = std::make_shared<BT::SequenceNode>("sequence");
  sequence->addChild(&subscriber);

  // run node until it finishes
  pubLatch(true);
  const auto status = tickWhileRunning(sequence);
  EXPECT_EQ(status, BT::NodeStatus::SUCCESS);
  EXPECT_EQ(subscriber.finish_count, 1);
  EXPECT_EQ(subscriber.failure_count, 0);
  EXPECT_EQ(subscriber.true_count, 1);
  EXPECT_EQ(subscriber.false_count, 0);
  EXPECT_EQ(subscriber.halt_count, 0);

  // change pub to false and run again
  sequence->halt();
  const auto status2 = tickWhileRunning(sequence);
  EXPECT_EQ(status2, BT::NodeStatus::SUCCESS);
  EXPECT_EQ(subscriber.finish_count, 2);
  EXPECT_EQ(subscriber.failure_count, 0);
  EXPECT_EQ(subscriber.true_count, 2);
  EXPECT_EQ(subscriber.false_count, 0);
  EXPECT_EQ(subscriber.halt_count, 0);
}

TEST_F(SubscriberNodeTest, rerun_latch_change)
{
  BoolSubscriber subscriber("subscriber", config);
  std::shared_ptr<BT::SequenceNode> sequence = std::make_shared<BT::SequenceNode>("sequence");
  sequence->addChild(&subscriber);

  // run node until it finishes
  pubLatch(true);
  const auto status = tickWhileRunning(sequence);
  EXPECT_EQ(status, BT::NodeStatus::SUCCESS);
  EXPECT_EQ(subscriber.finish_count, 1);
  EXPECT_EQ(subscriber.failure_count, 0);
  EXPECT_EQ(subscriber.true_count, 1);
  EXPECT_EQ(subscriber.false_count, 0);
  EXPECT_EQ(subscriber.halt_count, 0);

  // change pub to false and run again
  sequence->halt();
  pubLatch(false);
  ros::Duration(0.1).sleep();  // wait for message to be changed
  const auto status2 = tickWhileRunning(sequence);
  EXPECT_EQ(status2, BT::NodeStatus::SUCCESS);
  EXPECT_EQ(subscriber.finish_count, 2);
  EXPECT_EQ(subscriber.failure_count, 0);
  EXPECT_EQ(subscriber.true_count, 1);
  EXPECT_EQ(subscriber.false_count, 1);
  EXPECT_EQ(subscriber.halt_count, 0);
}

TEST_F(SubscriberNodeTest, halt_rerun)
{
  BoolSubscriber subscriber("subscriber", config);
  std::shared_ptr<BT::SequenceNode> sequence = std::make_shared<BT::SequenceNode>("sequence");
  sequence->addChild(&subscriber);

  // no pub
  auto status = sequence->executeTick();
  EXPECT_EQ(status, BT::NodeStatus::RUNNING);
  EXPECT_EQ(subscriber.finish_count, 0);
  EXPECT_EQ(subscriber.failure_count, 0);
  EXPECT_EQ(subscriber.halt_count, 0);

  // halt
  sequence->haltChildren();
  EXPECT_EQ(subscriber.finish_count, 0);
  EXPECT_EQ(subscriber.failure_count, 0);
  EXPECT_EQ(subscriber.halt_count, 1);

  // rerun
  status = sequence->executeTick();
  EXPECT_EQ(status, BT::NodeStatus::RUNNING);
  EXPECT_EQ(subscriber.finish_count, 0);
  EXPECT_EQ(subscriber.failure_count, 0);
  EXPECT_EQ(subscriber.halt_count, 1);

  // run node until it finishes
  pubLatch(true);
  status = tickWhileRunning(sequence);
  EXPECT_EQ(status, BT::NodeStatus::SUCCESS);
  EXPECT_EQ(subscriber.finish_count, 1);
  EXPECT_EQ(subscriber.failure_count, 0);
  EXPECT_EQ(subscriber.true_count, 1);
  EXPECT_EQ(subscriber.false_count, 0);
  EXPECT_EQ(subscriber.halt_count, 1);
}
}  // namespace BT::test

int main(int argc, char** argv)
{
  ros::init(argc, argv, "subscriber_test");

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
