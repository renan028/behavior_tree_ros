#pragma once

#include "behaviortree_cpp_v3/control_node.h"

namespace BT
{
/**
 * @brief The ExecutorNode is used to tick children in an ordered sequence.
 * It will execute all children in order, regardless of their status.
 *
 * It returns success if number of children that returns success is above THRESHOLD_SUCCESS
 * Otherwise, it returns failure.
 */
class ExecutorNode : public ControlNode
{
public:
  ExecutorNode(const std::string& name, unsigned success_threshold);
  ExecutorNode(const std::string& name, const NodeConfiguration& config);

  virtual ~ExecutorNode() override = default;

  static PortsList providedPorts()
  {
    return { InputPort<unsigned>(THRESHOLD_SUCCESS, "number of children which need to succeed to trigger a SUCCESS") };
  }

  virtual void halt() override;

private:
  size_t current_child_idx_;
  unsigned int success_threshold_;
  bool read_parameter_from_ports_;
  size_t success_childred_num_;
  size_t failure_childred_num_;

  virtual BT::NodeStatus tick() override;

  static constexpr auto THRESHOLD_SUCCESS = "success_threshold";
};

}  // namespace BT
