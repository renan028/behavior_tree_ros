#pragma once

// bt_ros
#include <bt_ros_msgs/BtState.h>
#include <bt_ros_msgs/LoadTree.h>
#include "bt_ros/service_node.h"
#include "bt_ros/subscriber_node.h"
#include "bt_ros/get_topic.h"
#include "bt_ros/action_node.h"

// std
#include <atomic>
#include <chrono>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

// behaviortree_cpp_v3
#include <behaviortree_cpp_v3/behavior_tree.h>
#include <behaviortree_cpp_v3/bt_factory.h>
#include <behaviortree_cpp_v3/loggers/bt_cout_logger.h>
#include <behaviortree_cpp_v3/loggers/bt_file_logger.h>
#include <behaviortree_cpp_v3/loggers/bt_zmq_publisher.h>
#include <behaviortree_cpp_v3/xml_parsing.h>

// ros
#include <ros/package.h>
#include <ros/ros.h>
#include <std_srvs/Trigger.h>

namespace behavior_tree_ros
{

class ExecutorBT
{
private:
  BT::BehaviorTreeFactory factory_;
  ros::NodeHandle pnh_;
  ros::ServiceServer srv_load_path_, srv_unload_;
  ros::Publisher status_pub_;

  std::string behavior_name_;
  std::string behavior_package_;

  std::thread thread_;
  std::atomic<bool> exit_;
  std::mutex mutex_;

  BT::NodeStatus status_;
  uint32_t tree_id_;
  int zmq_port_;
  std::unique_ptr<BT::PublisherZMQ> zmq_publisher_;
  int tree_sleep_;
  BT::Tree tree_;

public:
  ExecutorBT();
  ~ExecutorBT();
  BT::NodeStatus getStatus();
  bool load(const std::string& tree_xml);
  bool unload();
  BT::BehaviorTreeFactory& getFactory();

private:
  void execute();
  bool load(bt_ros_msgs::LoadTreeRequest& req, bt_ros_msgs::LoadTreeResponse& res);
  bool unload(std_srvs::TriggerRequest& req, std_srvs::TriggerResponse& res);
  void logger(const BT::Tree& node);
  void stop();
};

}  // namespace behavior_tree_ros
