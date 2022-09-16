#pragma once

// behaviortree_cpp
#include <behaviortree_cpp_v3/action_node.h>
#include <behaviortree_cpp_v3/bt_factory.h>
#include <behaviortree_cpp_v3/tree_node.h>

// ros
#include <ros/ros.h>
#include <ros/service_client.h>

namespace BT
{

template <class ServiceT>
class ServiceNode : public BT::SyncActionNode
{
public:
  using ServiceType = ServiceT;

protected:
  using BaseClass = ServiceNode<ServiceT>;
  using RequestType = typename ServiceT::Request;
  using ResponseType = typename ServiceT::Response;

  enum class FailureCause
  {
    MISSING_SERVER = 0,
    FAILED_CALL = 1
  };

  ServiceNode(const std::string& name, const BT::NodeConfiguration& conf) : BT::SyncActionNode(name, conf)
  {
  }

private:
  // User must implement.
  virtual void sendRequest(RequestType& request) = 0;

  // User can decide which NodeStatus it will return (SUCCESS or FAILURE).
  virtual NodeStatus onResponse(const ResponseType& rep)
  {
    return NodeStatus::SUCCESS;
  }

  // Called when a service call failed. Can be overriden by the user.
  virtual NodeStatus onFailedRequest(FailureCause failure)
  {
    return NodeStatus::FAILURE;
  }

public:
  ServiceNode() = delete;

  virtual ~ServiceNode() = default;

  static PortsList providedPorts()
  {
    return { InputPort<std::string>("service_name", "name of the ROS service"),
             InputPort<double>("timeout", 2.0, "timeout to connect to server (seconds)") };
  }

protected:
  ros::ServiceClient service_client_;
  typename ServiceT::Response reply_;
  ros::NodeHandle node_;

  BT::NodeStatus tick() override
  {
    if (!service_client_.isValid())
    {
      std::string server = getInput<std::string>("service_name").value();
      service_client_ = node_.serviceClient<ServiceT>(server);
    }

    unsigned sec;
    getInput("timeout", sec);
    ros::Duration timeout(sec);

    bool connected = service_client_.waitForExistence(timeout);
    if (!connected)
    {
      return onFailedRequest(FailureCause::MISSING_SERVER);
    }

    typename ServiceT::Request request;
    sendRequest(request);
    bool received = service_client_.call(request, reply_);
    if (!received)
    {
      return onFailedRequest(FailureCause::FAILED_CALL);
    }
    return onResponse(reply_);
  }
};

template <class DerivedT>
static void RegisterService(BT::BehaviorTreeFactory& factory, const std::string& registration_ID)
{
  BT::NodeBuilder builder = [](const std::string& name, const BT::NodeConfiguration& config)
  { return std::make_unique<DerivedT>(name, config); };

  BT::TreeNodeManifest manifest;
  manifest.type = getType<DerivedT>();
  manifest.ports = DerivedT::providedPorts();
  manifest.registration_ID = registration_ID;
  const auto& basic_ports = ServiceNode<typename DerivedT::ServiceType>::providedPorts();
  manifest.ports.insert(basic_ports.begin(), basic_ports.end());
  factory.registerBuilder(manifest, builder);
}

}  // namespace BT
