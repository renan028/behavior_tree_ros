#include "bt_ros/controls/executor_node.h"
#include "behaviortree_cpp_v3/action_node.h"

namespace BT
{

ExecutorNode::ExecutorNode(const std::string& name, unsigned success_threshold)
  : ControlNode::ControlNode(name, {})
  , current_child_idx_(0)
  , success_threshold_(success_threshold)
  , read_parameter_from_ports_(false)
  , success_childred_num_(0)
  , failure_childred_num_(0)
{
  setRegistrationID("Executor");
}

ExecutorNode::ExecutorNode(const std::string& name, const NodeConfiguration& config)
  : ControlNode::ControlNode(name, config)
  , current_child_idx_(0)
  , success_threshold_(1)
  , read_parameter_from_ports_(true)
  , success_childred_num_(0)
  , failure_childred_num_(0)
{
}

void ExecutorNode::halt()
{
  current_child_idx_ = 0;
  success_childred_num_ = 0;
  failure_childred_num_ = 0;
  ControlNode::halt();
}

NodeStatus ExecutorNode::tick()
{
  if (read_parameter_from_ports_)
  {
    if (!getInput(THRESHOLD_SUCCESS, success_threshold_))
    {
      throw RuntimeError("Missing parameter [", THRESHOLD_SUCCESS, "] in ExecutorNode");
    }
  }

  const size_t children_count = children_nodes_.size();

  setStatus(NodeStatus::RUNNING);

  while (current_child_idx_ < children_count)
  {
    TreeNode* current_child_node = children_nodes_[current_child_idx_];
    const NodeStatus child_status = current_child_node->executeTick();

    switch (child_status)
    {
      case NodeStatus::RUNNING:
      {
        return NodeStatus::RUNNING;
      }
      case NodeStatus::FAILURE:
      {
        current_child_idx_++;
        failure_childred_num_++;
        break;
      }
      case NodeStatus::SUCCESS:
      {
        current_child_idx_++;
        success_childred_num_++;
        break;
      }
      case NodeStatus::IDLE:
      {
        throw LogicError("A child node must never return IDLE");
      }
    }
  }

  haltChildren();
  auto result = (success_childred_num_ >= success_threshold_) ? NodeStatus::SUCCESS : NodeStatus::FAILURE;
  current_child_idx_ = 0;
  success_childred_num_ = 0;
  failure_childred_num_ = 0;
  return result;
}

}  // namespace BT
