#pragma once

// behavior_tree_ros
#include "behavior_tree_ros/BtState.h"
#include "behavior_tree_ros/LoadTree.h"
#include "behavior_tree_ros/factory.h"

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
  ros::NodeHandle pnh_;
  ros::NodeHandle nh_;
  behavior_tree_ros::Factory factory_;

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

public:
  ExecutorBT();
  ~ExecutorBT();
  BT::NodeStatus getStatus();
  bool load(const std::string& tree_xml);
  bool unload();

private:
  void execute(const std::string& tree_xml);
  bool load(LoadTreeRequest& req, LoadTreeResponse& res);
  bool unload(std_srvs::TriggerRequest& req, std_srvs::TriggerResponse& res);
};

}  // namespace behavior_tree_ros
