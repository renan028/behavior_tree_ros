#pragma once

// behaviortree_cpp
#include <behaviortree_cpp_v3/behavior_tree.h>
#include <behaviortree_cpp_v3/bt_factory.h>

// std
#include <optional>

// ros
#include <ros/ros.h>

namespace BT
{

/**
 * @brief A ROS subscriber action for BehaviorTree.CPP that subscribes to a
 * topic until a message is received or a timeout (sec). The cleanup erases the
 * message, making sure that the next time the action is ticked, it will wait
 * for a new message.
 */
template <class SubscriberT>
class SubscriberNode : public BT::StatefulActionNode
{
protected:
  SubscriberNode(const std::string& name, const BT::NodeConfiguration& conf) : BT::StatefulActionNode(name, conf)
  {
  }

public:
  using SubscriberType = SubscriberT;

  SubscriberNode() = delete;

  static PortsList providedPorts()
  {
    return { InputPort<std::string>("topic_name", "name of the ROS topic"),
             InputPort<double>("timeout", 1.0, "timeout to subscribe to topic (sec)") };
  }

private:
  void cleanup()
  {
    nmsg_.reset();
    sub_.shutdown();
  }

protected:
  virtual BT::NodeStatus onFinish()
  {
     return BT::NodeStatus::SUCCESS;
  }

  virtual BT::NodeStatus onFailure()
  {
     return BT::NodeStatus::FAILURE;
  };

  virtual void onHalt()
  {
  }

public:
  inline NodeStatus onStart() override final
  {
     start_time_ = ros::Time::now();

     getInput("topic_name", topic_);
     if (!topic_.empty())
     {
       sub_ = ros::NodeHandle().subscribe(topic_, 1, &SubscriberNode::callback, this);
     }
     else
     {
       ROS_ERROR_NAMED("SubscriberNode", "topic_name is empty");
       return NodeStatus::FAILURE;
     }

     double sec;
     getInput("timeout", sec);
     timeout_ = ros::Duration(sec);
     return NodeStatus::RUNNING;
  }

  inline NodeStatus onRunning() override final
  {
    if (nmsg_)
    {
      msg_ = nmsg_.value();
      const auto status = onFinish();
      cleanup();
      return status;
    }

    if (ros::Time::now() - start_time_ > timeout_)
    {
      ROS_ERROR_STREAM_NAMED("SubscriberNode",
                             "No message received in topic " << topic_ << " after " << timeout_.toSec());
      const auto status = onFailure();
      cleanup();
      return status;
    }

    return NodeStatus::RUNNING;
  }

  inline void onHalted() override final
  {
    onHalt();
    cleanup();
  }

protected:
  SubscriberT msg_;
  std::optional<SubscriberT> nmsg_;
  ros::Time start_time_;
  ros::Subscriber sub_;
  std::string topic_;
  ros::Duration timeout_;

  bool isEmpty() const
  {
    return !nmsg_;
  }

  void callback(const SubscriberT& msg)
  {
    nmsg_ = msg;
  }
};

template <class DerivedT>
static void RegisterSubscriber(BT::BehaviorTreeFactory& factory, const std::string& registration_ID)
{
  BT::NodeBuilder builder = [](const std::string& name, const BT::NodeConfiguration& config)
  { return std::make_unique<DerivedT>(name, config); };

  BT::TreeNodeManifest manifest;
  manifest.type = getType<DerivedT>();
  manifest.ports = DerivedT::providedPorts();
  manifest.registration_ID = registration_ID;
  const auto& basic_ports = SubscriberNode<typename DerivedT::SubscriberType>::providedPorts();
  manifest.ports.insert(basic_ports.begin(), basic_ports.end());
  factory.registerBuilder(manifest, builder);
}

}  // namespace BT
