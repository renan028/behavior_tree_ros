#pragma once

// behaviortree_cpp
#include <behaviortree_cpp_v3/behavior_tree.h>
#include <behaviortree_cpp_v3/bt_factory.h>

// std
#include <optional>
#include <mutex>

// ros
#include <ros/ros.h>

namespace BT
{

/**
 * @brief A ROS subscriber action for BehaviorTree.CPP that subscribes to a topic until a message is received or a
 * timeout (sec)
 */
template <class SubscriberT>
class SubscriberNode : public BT::StatefulActionNode
{
protected:
  SubscriberNode(const std::string& name, const BT::NodeConfiguration& conf) : BT::StatefulActionNode(name, conf)
  {
  }

public:
  using SubscriberType = SubscriberT;
  SubscriberNode() = delete;

  virtual ~SubscriberNode() = default;

  static PortsList providedPorts()
  {
    return { InputPort<std::string>("topic_name", "name of the ROS topic"),
             InputPort<double>("timeout", 1.0, "timeout to subscribe to topic (sec)") };
  }

private:
  static constexpr auto LOGNAME = "SubscriberNode";

  std::optional<SubscriberT> nmsg_;
  SubscriberT msg_;
  std::mutex async_spin_mtx_;
  std::mutex msg_mtx_;
  ros::Time start_time_;
  ros::Subscriber sub_;

  void callback(const SubscriberT& msg)
  {
    // lock for async spinners
    std::lock_guard lock(async_spin_mtx_);
    nmsg_ = msg;
  }

  virtual BT::NodeStatus onFinish()
  {
    return BT::NodeStatus::SUCCESS;
  }

  virtual BT::NodeStatus onFailure()
  {
    return BT::NodeStatus::FAILURE;
  };

  virtual void onHalt()
  {
  }

public:
  inline NodeStatus onStart() override final
  {
    start_time_ = ros::Time::now();

    getInput("topic_name", topic_);
    if (!topic_.empty())
    {
      sub_ = ros::NodeHandle().subscribe(topic_, 1, &SubscriberNode::callback, this);
    }
    else
    {
      ROS_ERROR_NAMED(LOGNAME, "topic_name is empty");
      return NodeStatus::FAILURE;
    }

    double sec;
    getInput("timeout", sec);
    sec = sec < 0 ? 0 : sec;
    timeout_ = ros::Duration(sec);
    return NodeStatus::RUNNING;
  }

  inline NodeStatus onRunning() override final
  {
    BT::NodeStatus status;
    {
      // lock for async spinners
      std::lock_guard lock(async_spin_mtx_);
      if (nmsg_)
      {
        {
          std::lock_guard lock(msg_mtx_);
          msg_ = nmsg_.value();
        }
        status = onFinish();
        nmsg_.reset();
      }
      else
      {
        if (timeout_ != ros::Duration(0) && ros::Time::now() - start_time_ > timeout_)
        {
          ROS_ERROR_STREAM_NAMED(LOGNAME, "No message received in topic " << topic_ << " after " << timeout_.toSec());
          status = onFailure();
        }
        else
        {
          ROS_DEBUG_STREAM_THROTTLE_NAMED(1.0, LOGNAME,
                                          "Waiting for message from topic "
                                              << topic_ << "; elapsed time: " << ros::Time::now() - start_time_);
          return NodeStatus::RUNNING;
        }
      }
    }

    sub_.shutdown();
    return status;
  }

  inline void onHalted() override final
  {
    onHalt();
    {
      std::lock_guard lock(async_spin_mtx_);
      nmsg_.reset();
    }
    sub_.shutdown();
  }

protected:
  std::string topic_;
  ros::Duration timeout_;

  /**
   * @brief get the reference to the message and the lock to the mutex. This is thread-safe if used correctly.
   * This does not return a copy of the message, but a reference to it. The lock is returned to avoid copying it.
   * Use it carefully, i.e., do not modify the message outside the lock scope.
   * Example of not to do:
   *
    SubscriberT& msgRef;
    {
      auto [lock, msg] = getMsg();
      msgRef = msg;
      // lock is destroyed here, msg is no longer protected by mutex
    }
   */
  std::pair<std::unique_lock<std::mutex>, SubscriberT&> getMsg()
  {
    std::unique_lock lock(msg_mtx_);
    return { std::move(lock), msg_ };
  }

  /**
   * @brief get a copy of the message (thread safe)
   * @return a copy of the message
   */
  SubscriberT getMsgCopy()
  {
    std::lock_guard lock(msg_mtx_);
    return msg_;
  }
};

template <class DerivedT>
static void RegisterSubscriber(BT::BehaviorTreeFactory& factory, const std::string& registration_ID)
{
  BT::NodeBuilder builder = [](const std::string& name, const BT::NodeConfiguration& config)
  { return std::make_unique<DerivedT>(name, config); };

  BT::TreeNodeManifest manifest;
  manifest.type = getType<DerivedT>();
  manifest.ports = DerivedT::providedPorts();
  manifest.registration_ID = registration_ID;
  const auto& basic_ports = SubscriberNode<typename DerivedT::SubscriberType>::providedPorts();
  manifest.ports.insert(basic_ports.begin(), basic_ports.end());
  factory.registerBuilder(manifest, builder);
}

}  // namespace BT
