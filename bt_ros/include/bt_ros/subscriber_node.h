#pragma once

// behaviortree_cpp
#include <behaviortree_cpp_v3/behavior_tree.h>
#include <behaviortree_cpp_v3/bt_factory.h>

// std
#include <optional>

// ros
#include <ros/ros.h>
#include <rosfmt/full.h>

namespace BT
{

template <class SubscriberT>
class SubscriberNode : public BT::CoroActionNode
{
private:
  virtual BT::NodeStatus onFinish()
  {
    return BT::NodeStatus::SUCCESS;
  }

  virtual BT::NodeStatus onFailure()
  {
    return BT::NodeStatus::FAILURE;
  };

protected:
  static constexpr unsigned MIN_WAIT_TIME_MS = 200;
  std::mutex mtx_;

  SubscriberNode(const std::string& name, const BT::NodeConfiguration& conf) : BT::CoroActionNode(name, conf)
  {
  }

public:
  using SubscriberType = SubscriberT;
  SubscriberNode() = delete;

  virtual ~SubscriberNode() = default;

  static PortsList providedPorts()
  {
    return { InputPort<std::string>("topic_name", "name of the ROS topic"),
             InputPort<unsigned>("timeout", MIN_WAIT_TIME_MS, "timeout to subscribe to topic (milliseconds)") };
  }

protected:
  ros::NodeHandle node_;
  SubscriberT msg_;
  std::optional<SubscriberT> nmsg_;
  bool first_tick_{ true };
  bool is_latched_{ false };
  ros::Time start_time_;
  ros::Subscriber sub_;
  std::string topic_;

  BT::NodeStatus tick() override
  {
    if (first_tick_)
    {
      first_tick_ = false;
      start_time_ = ros::Time::now();
      std::lock_guard<std::mutex> lock(mtx_);
      if (!is_latched_)
      {
        nmsg_ = std::nullopt;
      }
    }

    std::string topic;
    getInput("topic_name", topic);
    if (topic != topic_ && !topic.empty())
    {
      sub_ = node_.subscribe(topic, 1, &SubscriberNode::callback, this);
      topic_ = topic;
    }

    unsigned msec;
    getInput("timeout", msec);
    msec = std::max(msec, MIN_WAIT_TIME_MS);
    double sec = static_cast<double>(msec) * 1e-3;

    while (true)
    {
      {
        std::lock_guard<std::mutex> lock(mtx_);
        if (nmsg_)
        {
          msg_ = nmsg_.value();
          break;
        }
      }

      if (ros::Time::now() - start_time_ > ros::Duration(sec))
      {
        first_tick_ = true;
        ROSFMT_ERROR_NAMED("SubscriberNode", "No message received in topic {} after {} sec ", topic, sec);
        return onFailure();
      }

      setStatusRunningAndYield();
    }

    first_tick_ = true;
    return onFinish();
  }

  void halt() override
  {
    first_tick_ = true;
    CoroActionNode::halt();
  }

  void callback(const ros::MessageEvent<SubscriberT>& msg_event)
  {
    nmsg_ = *msg_event.getConstMessage();
    boost::shared_ptr<const ros::M_string> const& connection_header = msg_event.getConnectionHeaderPtr();
    auto it = connection_header->find("latching");
    if ((it != connection_header->end()) && (it->second == "1"))
    {
      ROS_DEBUG_ONCE("input topic is latched");
      is_latched_ = true;
    }
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
