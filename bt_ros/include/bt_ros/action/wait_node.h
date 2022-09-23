#pragma once

// behaviortree_cpp_v3
#include <behaviortree_cpp_v3/behavior_tree.h>

// std
#include <ctime>
#include <string>

// ros
#include <ros/ros.h>

namespace behavior_tree_ros
{

class WaitNode : public BT::CoroActionNode
{
public:
  WaitNode(const std::string& name, const BT::NodeConfiguration& conf)
    : BT::CoroActionNode(name, conf), first_tick_(true)
  {
  }
  ~WaitNode() = default;

  inline static BT::PortsList providedPorts()
  {
    return { BT::InputPort<float>("wait", 1, "wait seconds before success") };
  }

  inline BT::NodeStatus tick() final
  {
    if (first_tick_)
    {
      first_tick_ = false;
      start_time_ = std::time(nullptr);
    }

    float sec;
    while (true)
    {
      getInput("wait", sec);
      float elapsed_time = static_cast<float>(std::difftime(std::time(nullptr), start_time_));
      if (elapsed_time >= sec)
      {
        first_tick_ = true;
        return BT::NodeStatus::SUCCESS;
      }
      setStatusRunningAndYield();
    }
  }

  inline void halt() final
  {
    first_tick_ = true;
    CoroActionNode::halt();
  }

private:
  std::time_t start_time_;
  bool first_tick_;
};

}  // namespace behavior_tree_ros
