#pragma once

// behaviortree_cpp
#include <behaviortree_cpp_v3/action_node.h>
#include <behaviortree_cpp_v3/bt_factory.h>

#include "bt_ros/subscriber_node.h"

namespace BT
{

/**
 * Generic get topic data and output the data.
 */
template <class T>
class GetTopic : public SubscriberNode<T>
{
private:
  inline BT::NodeStatus onFinish() override
  {
    this->setOutput("msg", this->msg_);
    return BT::NodeStatus::SUCCESS;
  }

public:
  GetTopic() = delete;
  virtual ~GetTopic() = default;

  static PortsList providedPorts()
  {
    return { OutputPort<T>("msg") };
  }

protected:
  GetTopic(const std::string& name, const BT::NodeConfiguration& conf) : SubscriberNode<T>(name, conf)
  {
  }
};

template <class DerivedT>
static void RegisterGetTopic(BT::BehaviorTreeFactory& factory, const std::string& registration_ID)
{
  BT::NodeBuilder builder = [](const std::string& name, const BT::NodeConfiguration& config)
  { return std::make_unique<DerivedT>(name, config); };

  BT::TreeNodeManifest manifest;
  manifest.type = getType<DerivedT>();
  manifest.ports = DerivedT::providedPorts();
  manifest.registration_ID = registration_ID;
  const auto& basic_ports = SubscriberNode<typename DerivedT::SubscriberType>::providedPorts();
  const auto& get_topic_ports = GetTopic<typename DerivedT::SubscriberType>::providedPorts();
  manifest.ports.insert(basic_ports.begin(), basic_ports.end());
  manifest.ports.insert(get_topic_ports.begin(), get_topic_ports.end());
  factory.registerBuilder(manifest, builder);
}

}  // namespace BT
