#include "behavior_tree_ros/executor_bt.h"

namespace behavior_tree_ros {

ExecutorBT::ExecutorBT()
    : nh_("~"),
      srv_load_path_(
          nh_.advertiseService("load_behavior", &ExecutorBT::load, this)),
      srv_unload_(
          nh_.advertiseService("unload_behavior", &ExecutorBT::unload, this)),
      status_pub_(nh_.advertise<BtState>("bt_state", 100, true)),
      exit_(true),
      status_(BT::NodeStatus::IDLE),
      tree_id_(0),
      behavior_name_("") {
  nh_.param<std::string>("behavior_tree_package", behavior_package_,
                         "behavior_tree_ros");

  std::string tree_xml;
  if (nh_.getParam("tree_xml", tree_xml)) {
    load(tree_xml);
  }

  factory_.make();

  nh_.param("zmq_port", zmq_port_, 0);
  nh_.param("tree_sleep", tree_sleep_, 100);
}

ExecutorBT::~ExecutorBT() {
  exit_ = true;
  if (thread_.joinable()) {
    thread_.join();
  }
  status_pub_.shutdown();
  srv_load_path_.shutdown();
  srv_unload_.shutdown();
}

BT::NodeStatus ExecutorBT::getStatus() {
  const std::lock_guard<std::mutex> lock(mutex_);
  return status_;
}

bool ExecutorBT::load(const std::string& tree_xml) {
  LoadTree load_msg;
  load_msg.request.tree_xml_name = tree_xml;
  return load(load_msg.request, load_msg.response);
}

bool ExecutorBT::unload() {
  std_srvs::Trigger trigger_msg;
  return unload(trigger_msg.request, trigger_msg.response);
}

void ExecutorBT::execute(const std::string& tree_xml) {
  tree_id_++;

  BT::Tree tree;
  try {
    tree = Factory::createTreeFromFile(tree_xml);
  } catch (const BT::RuntimeError& e) {
    std::cerr << e.what() << '\n';
    return;
  }

  BT::StdCoutLogger logger_cout(tree);
  std::string log_name = "ulm_trace_" + std::to_string(tree_id_) + ".fbl";
  BT::FileLogger logger_file(tree, log_name.c_str());

  if (zmq_port_ != 0) {
    int server_port = zmq_port_ + 1;
    zmq_publisher_ =
        std::make_unique<BT::PublisherZMQ>(tree, 25, zmq_port_, server_port);
  }

  auto bt_state_to_msg_state = [](const BT::NodeStatus& state) {
    switch (state) {
      case BT::NodeStatus::IDLE:
        return BtState::IDLE;
      case BT::NodeStatus::RUNNING:
        return BtState::RUNNING;
      case BT::NodeStatus::SUCCESS:
        return BtState::SUCCESS;
      case BT::NodeStatus::FAILURE:
        return BtState::FAILURE;
      default:
        return BtState::UNKNOWN;
    }
  };

  BtState bt_status;
  bt_status.behavior_xml = behavior_name_;
  while (!exit_) {
    {
      const std::lock_guard<std::mutex> lock(mutex_);
      status_ = tree.tickRoot();
      bt_status.state = bt_state_to_msg_state(status_);
      bt_status.header.stamp = ros::Time::now();
      status_pub_.publish(bt_status);
      if (status_ != BT::NodeStatus::RUNNING) exit_ = true;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(tree_sleep_));
  }
  tree.haltTree();
}

bool ExecutorBT::load(LoadTreeRequest& req, LoadTreeResponse& res) {
  {
    const std::lock_guard<std::mutex> lock(mutex_);
    exit_ = true;
    if (thread_.joinable()) thread_.join();
    exit_ = false;
  }

  behavior_name_ = req.tree_xml_name;
  std::string path_to_file =
      ros::package::getPath(behavior_package_) + req.tree_xml_name;

  thread_ = std::thread(&ExecutorBT::execute, this, path_to_file);
  res.success = true;
  return true;
}

bool ExecutorBT::unload(std_srvs::TriggerRequest& req,
                        std_srvs::TriggerResponse& res) {
  exit_ = true;
  if (thread_.joinable()) {
    thread_.join();
    res.success = true;
    res.message = "behavior unloaded";
    return true;
  }
  res.success = false;
  res.message = "behavior unload fails";
  return true;
}

}  // namespace behavior_tree_ros
