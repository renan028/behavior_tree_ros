// behavior_tree_cpp_v3
#include <behaviortree_cpp_v3/loggers/bt_minitrace_logger.h>
#include <behaviortree_cpp_v3/bt_factory.h>

// behavior_tree_ros
#include "bt_ros/executor_bt.h"

// std
#include <stdlib.h>
#include <filesystem>

// bt
#include "bt_ros/plugins.h"

namespace behavior_tree_ros
{
namespace fs = std::filesystem;

ExecutorBT::ExecutorBT()
  : pnh_("~")
  , srv_load_path_(pnh_.advertiseService("load_behavior", &ExecutorBT::load, this))
  , srv_unload_(pnh_.advertiseService("unload_behavior", &ExecutorBT::unload, this))
  , status_pub_(pnh_.advertise<bt_ros_msgs::BtState>("bt_state", 100, true))
  , exit_(true)
  , status_(BT::NodeStatus::IDLE)
  , tree_id_(0)
  , behavior_name_("")
{
  pnh_.param<std::string>("behavior_tree_package", behavior_package_, "bt_ros");
  pnh_.param<int>("zmq_port", zmq_port_, 0);
  pnh_.param<int>("tree_sleep", tree_sleep_, 100);

  registerNodes(factory_);
}

ExecutorBT::~ExecutorBT()
{
  stop();
}

BT::BehaviorTreeFactory& ExecutorBT::getFactory()
{
  return factory_;
}

BT::NodeStatus ExecutorBT::getStatus()
{
  const std::lock_guard<std::mutex> lock(mutex_);
  return status_;
}

bool ExecutorBT::load(const std::string& tree_xml)
{
  stop();
  behavior_name_ = "behavior_" + std::to_string(tree_id_);
  try
  {
    tree_ = factory_.createTreeFromText(tree_xml);
  }
  catch (const BT::RuntimeError& e)
  {
    ROS_WARN_STREAM_NAMED("ExecutorBT", e.what());
    ROS_ERROR_STREAM_NAMED("ExecutorBT", "Failed to load tree " << tree_xml << ": " << e.what());
    return false;
  }
  thread_ = std::thread(&ExecutorBT::execute, this);
  return true;
}

bool ExecutorBT::unload()
{
  std_srvs::Trigger trigger_msg;
  return unload(trigger_msg.request, trigger_msg.response);
}

void ExecutorBT::execute()
{
  tree_id_++;
  logger(tree_);

  bt_ros_msgs::BtState bt_status;
  bt_status.behavior_xml = behavior_name_;
  while (ros::ok() && !exit_)
  {
    {
      const std::lock_guard<std::mutex> lock(mutex_);
      status_ = tree_.tickRoot();
      bt_status.state = static_cast<unsigned>(status_);
      bt_status.header.stamp = ros::Time::now();
      status_pub_.publish(bt_status);
      if (status_ != BT::NodeStatus::RUNNING)
      {
        exit_ = true;
      }
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(tree_sleep_));
  }
  tree_.haltTree();
}

void ExecutorBT::stop()
{
  {
    const std::lock_guard<std::mutex> lock(mutex_);
    exit_ = true;
    if (thread_.joinable())
    {
      thread_.join();
    }
    exit_ = false;
  }
}

bool ExecutorBT::load(bt_ros_msgs::LoadTreeRequest& req, bt_ros_msgs::LoadTreeResponse& res)
{
  stop();
  behavior_name_ = req.tree_xml_name;
  std::string path_to_file = ros::package::getPath(behavior_package_) + req.tree_xml_name;

  try
  {
    tree_ = factory_.createTreeFromFile(path_to_file, BT::Blackboard::create());
  }
  catch (const BT::RuntimeError& e)
  {
    ROS_WARN_STREAM_NAMED("ExecutorBT", e.what());
    ROS_ERROR_STREAM_NAMED("ExecutorBT", "Failed to load tree " << path_to_file << ": " << e.what());
    return false;
  }

  thread_ = std::thread(&ExecutorBT::execute, this);
  res.success = true;
  return true;
}

bool ExecutorBT::unload(std_srvs::TriggerRequest& req, std_srvs::TriggerResponse& res)
{
  exit_ = true;
  if (thread_.joinable())
  {
    thread_.join();
    res.success = true;
    res.message = "behavior unloaded";
    return true;
  }
  res.success = false;
  res.message = "behavior unload fails";
  return true;
}

void ExecutorBT::logger(const BT::Tree& tree)
{
  time_t t = time(nullptr);
  tm currentTime;
  memset(&currentTime, 0, sizeof(currentTime));
  localtime_r(&t, &currentTime);

  char buf[256];
  strftime(buf, sizeof(buf), "_%Y_%m_%d_%H_%M_%S", &currentTime);

  const auto env = std::getenv("BT_TRACE_PATH");
  std::string bt_record = env ? std::string(env) : "";
  if (bt_record.empty())
  {
    bt_record = "bt_trace";
  }
  auto path = fs::path(bt_record);
  if (!path.parent_path().empty())
  {
    fs::create_directories(path.parent_path());
  }

  std::string log_name = bt_record + std::string(buf) + ".fbl";
  std::string log_name_json = bt_record + std::string(buf) + ".json";
  BT::FileLogger logger_file(tree, log_name.c_str());
  BT::MinitraceLogger logger_minitrace(tree, log_name_json.c_str());

  bool cout_logger;
  std::unique_ptr<BT::StdCoutLogger> logger_cout;
  if (pnh_.getParam("cout_logger", cout_logger) && cout_logger)
  {
    logger_cout = std::make_unique<BT::StdCoutLogger>(tree);
  }

  if (zmq_port_ != 0)
  {
    int server_port = zmq_port_ + 1;
    zmq_publisher_ = std::make_unique<BT::PublisherZMQ>(tree, 25, zmq_port_, server_port);
  }
}

}  // namespace behavior_tree_ros
