// Copyright (c) 2022 Renan Freitas

#pragma once

// behaviortree_cpp
#include <behaviortree_cpp_v3/action_node.h>
#include <behaviortree_cpp_v3/bt_factory.h>

// ros
#include <ros/ros.h>

namespace BT
{

template <class SubscriberT>
class SubscriberNode : virtual public BT::SyncActionNode
{
protected:
  SubscriberNode(const std::string& name, const BT::NodeConfiguration& conf) : BT::SyncActionNode(name, conf)
  {
  }

public:
  using SubscriberType = SubscriberT;
  SubscriberNode() = delete;

  virtual ~SubscriberNode() = default;

  static PortsList providedPorts()
  {
    return { InputPort<std::string>("topic_name", "name of the ROS topic"),
             InputPort<unsigned>("timeout", 100, "timeout to subscribe to topic (milliseconds)") };
  }

protected:
  ros::Subscriber sub_;
  ros::NodeHandle node_;
  SubscriberT msg_;

  virtual BT::NodeStatus onFinish() = 0;
  virtual void onFailure(){};

  BT::NodeStatus tick() override
  {
    std::string topic;
    getInput("topic_name", topic);

    unsigned msec;
    getInput("timeout", msec);
    auto timeout = ros::Duration(static_cast<double>(msec) * 1e-3);

    auto result = ros::topic::waitForMessage<SubscriberT>(topic, node_, timeout);
    if (!result)
    {
      ROS_ERROR_STREAM_NAMED("SubscriberNode", "No message received in topic: " << topic);
      onFailure();
      return BT::NodeStatus::FAILURE;
    }
    msg_ = *result;
    return onFinish();
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
