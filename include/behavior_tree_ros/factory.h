#pragma once

// std
#include <functional>
#include <memory>

// ros
#include <ros/ros.h>

// behaviortree_cpp_v3
#include <behaviortree_cpp_v3/behavior_tree.h>
#include <behaviortree_cpp_v3/bt_factory.h>

namespace BT {

template <typename T>
static void RegisterAction(BT::BehaviorTreeFactory& factory,
                           const std::string& registration_ID) {
  BT::NodeBuilder builder = [](const std::string& name,
                               const BT::NodeConfiguration& config) {
    return std::make_unique<T>(name, config);
  };
  factory.registerBuilder<T>(registration_ID, builder);
}

}  // namespace BT

namespace behavior_tree_ros {

class Factory {
  static BT::BehaviorTreeFactory factory;

 public:
  void make();
  static BT::BehaviorTreeFactory& getFactory();

  inline static BT::Tree createTreeFromFile(
      const std::string& file_path,
      BT::Blackboard::Ptr blackboard = BT::Blackboard::create()) {
    return factory.createTreeFromFile(file_path, blackboard);
  }
};

}  // namespace behavior_tree_ros
