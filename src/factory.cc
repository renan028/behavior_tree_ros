// behavior_tree_ros
#include "behavior_tree_ros/factory.h"

namespace behavior_tree_ros {

BT::BehaviorTreeFactory Factory::factory;

BT::BehaviorTreeFactory& Factory::getFactory() { return factory; }

void Factory::make() {}

}  // namespace behavior_tree_ros
