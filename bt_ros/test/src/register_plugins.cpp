#include "../include/simple_action_1.h"
#include "../include/dummy_service.h"
#include "behaviortree_cpp_v3/bt_factory.h"

namespace bt_ros::test
{
BT_REGISTER_NODES(factory)
{
  BT::RegisterSimpleActionClient<SimpleAction1>(factory, "SimpleAction1");
  BT::RegisterService<DummyService>(factory, "DummyService");
}
}  // namespace bt_ros::test
