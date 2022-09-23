#include "bt_ros/action/get_ros_msgs.h"
#include "bt_ros/action/wait_node.h"
#include "behaviortree_cpp_v3/bt_factory.h"

namespace BT
{
template <typename T>
static void RegisterAction(BehaviorTreeFactory& factory, const std::string& registration_ID)
{
  NodeBuilder builder = [](const std::string& name, const BT::NodeConfiguration& config)
  { return std::make_unique<T>(name, config); };
  factory.registerBuilder<T>(registration_ID, builder);
}
}  // namespace BT

namespace behavior_tree_ros
{
static void registerNodes(BT::BehaviorTreeFactory& factory)
{
  BT::RegisterGetTopic<GetBool>(factory, "GetBool");
  BT::RegisterGetTopic<GetFloat32>(factory, "GetFloat32");
  BT::RegisterGetTopic<GetInt32>(factory, "GetInt32");
  BT::RegisterGetTopic<GetInt64>(factory, "GetInt64");
  BT::RegisterGetTopic<GetString>(factory, "GetString");
  BT::RegisterGetTopic<GetPoseStamped>(factory, "GetPoseStamped");
  BT::RegisterGetTopic<GetTwistStamped>(factory, "GetTwistStamped");
  BT::RegisterGetTopic<GetPoseWithCovariance>(factory, "GetPoseWithCovariance");
  BT::RegisterGetTopic<GetPoseWithCovarianceStamped>(factory, "GetPoseWithCovarianceStamped");
  BT::RegisterGetTopic<GetTransformStamped>(factory, "GetTransformStamped");
  BT::RegisterGetTopic<GetPolygonStamped>(factory, "GetPolygonStamped");
  BT::RegisterGetTopic<GetJointState>(factory, "GetJointState");
  BT::RegisterGetTopic<GetPointCloud2>(factory, "GetPointCloud2");
  BT::RegisterGetTopic<GetImu>(factory, "GetImu");
  BT::RegisterGetTopic<GetRange>(factory, "GetRange");
  BT::RegisterGetTopic<GetLaserScan>(factory, "GetLaserScan");
  BT::RegisterGetTopic<GetBatteryState>(factory, "GetBatteryState");
  BT::RegisterGetTopic<GetImage>(factory, "GetImage");
  BT::RegisterGetTopic<GetOdometry>(factory, "GetOdometry");
  BT::RegisterGetTopic<GetPath>(factory, "GetPath");
  BT::RegisterGetTopic<GetOccupancyGrid>(factory, "GetOccupancyGrid");

  BT::RegisterAction<WaitNode>(factory, "WaitNode");
}
}  // namespace behavior_tree_ros
