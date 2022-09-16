#pragma once

// bt
#include "bt_ros/get_topic.h"

// ros
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Int64.h>
#include <std_msgs/String.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PolygonStamped.h>

#include <sensor_msgs/JointState.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Range.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/BatteryState.h>
#include <sensor_msgs/Image.h>

#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/OccupancyGrid.h>

namespace behavior_tree_ros
{

class GetBool : public BT::GetTopic<std_msgs::Bool>
{
public:
  GetBool(const std::string& name, const BT::NodeConfiguration& conf) : BT::GetTopic<std_msgs::Bool>(name, conf)
  {
  }
};

class GetFloat32 : public BT::GetTopic<std_msgs::Float32>
{
public:
  GetFloat32(const std::string& name, const BT::NodeConfiguration& conf) : BT::GetTopic<std_msgs::Float32>(name, conf)
  {
  }
};

class GetInt32 : public BT::GetTopic<std_msgs::Int32>
{
public:
  GetInt32(const std::string& name, const BT::NodeConfiguration& conf) : BT::GetTopic<std_msgs::Int32>(name, conf)
  {
  }
};

class GetInt64 : public BT::GetTopic<std_msgs::Int64>
{
public:
  GetInt64(const std::string& name, const BT::NodeConfiguration& conf) : BT::GetTopic<std_msgs::Int64>(name, conf)
  {
  }
};

class GetString : public BT::GetTopic<std_msgs::String>
{
public:
  GetString(const std::string& name, const BT::NodeConfiguration& conf) : BT::GetTopic<std_msgs::String>(name, conf)
  {
  }
};

class GetPoseStamped : public BT::GetTopic<geometry_msgs::PoseStamped>
{
public:
  GetPoseStamped(const std::string& name, const BT::NodeConfiguration& conf)
    : BT::GetTopic<geometry_msgs::PoseStamped>(name, conf)
  {
  }
};

class GetTwistStamped : public BT::GetTopic<geometry_msgs::TwistStamped>
{
public:
  GetTwistStamped(const std::string& name, const BT::NodeConfiguration& conf)
    : BT::GetTopic<geometry_msgs::TwistStamped>(name, conf)
  {
  }
};

class GetPoseWithCovariance : public BT::GetTopic<geometry_msgs::PoseWithCovariance>
{
public:
  GetPoseWithCovariance(const std::string& name, const BT::NodeConfiguration& conf)
    : BT::GetTopic<geometry_msgs::PoseWithCovariance>(name, conf)
  {
  }
};

class GetPoseWithCovarianceStamped : public BT::GetTopic<geometry_msgs::PoseWithCovarianceStamped>
{
public:
  GetPoseWithCovarianceStamped(const std::string& name, const BT::NodeConfiguration& conf)
    : BT::GetTopic<geometry_msgs::PoseWithCovarianceStamped>(name, conf)
  {
  }
};

class GetTransformStamped : public BT::GetTopic<geometry_msgs::TransformStamped>
{
public:
  GetTransformStamped(const std::string& name, const BT::NodeConfiguration& conf)
    : BT::GetTopic<geometry_msgs::TransformStamped>(name, conf)
  {
  }
};

class GetPolygonStamped : public BT::GetTopic<geometry_msgs::PolygonStamped>
{
public:
  GetPolygonStamped(const std::string& name, const BT::NodeConfiguration& conf)
    : BT::GetTopic<geometry_msgs::PolygonStamped>(name, conf)
  {
  }
};

class GetJointState : public BT::GetTopic<sensor_msgs::JointState>
{
public:
  GetJointState(const std::string& name, const BT::NodeConfiguration& conf)
    : BT::GetTopic<sensor_msgs::JointState>(name, conf)
  {
  }
};

class GetPointCloud2 : public BT::GetTopic<sensor_msgs::PointCloud2>
{
public:
  GetPointCloud2(const std::string& name, const BT::NodeConfiguration& conf)
    : BT::GetTopic<sensor_msgs::PointCloud2>(name, conf)
  {
  }
};

class GetImu : public BT::GetTopic<sensor_msgs::Imu>
{
public:
  GetImu(const std::string& name, const BT::NodeConfiguration& conf) : BT::GetTopic<sensor_msgs::Imu>(name, conf)
  {
  }
};

class GetRange : public BT::GetTopic<sensor_msgs::Range>
{
public:
  GetRange(const std::string& name, const BT::NodeConfiguration& conf) : BT::GetTopic<sensor_msgs::Range>(name, conf)
  {
  }
};

class GetLaserScan : public BT::GetTopic<sensor_msgs::LaserScan>
{
public:
  GetLaserScan(const std::string& name, const BT::NodeConfiguration& conf)
    : BT::GetTopic<sensor_msgs::LaserScan>(name, conf)
  {
  }
};

class GetBatteryState : public BT::GetTopic<sensor_msgs::BatteryState>
{
public:
  GetBatteryState(const std::string& name, const BT::NodeConfiguration& conf)
    : BT::GetTopic<sensor_msgs::BatteryState>(name, conf)
  {
  }
};

class GetImage : public BT::GetTopic<sensor_msgs::Image>
{
public:
  GetImage(const std::string& name, const BT::NodeConfiguration& conf) : BT::GetTopic<sensor_msgs::Image>(name, conf)
  {
  }
};

class GetOdometry : public BT::GetTopic<nav_msgs::Odometry>
{
public:
  GetOdometry(const std::string& name, const BT::NodeConfiguration& conf) : BT::GetTopic<nav_msgs::Odometry>(name, conf)
  {
  }
};

class GetPath : public BT::GetTopic<nav_msgs::Path>
{
public:
  GetPath(const std::string& name, const BT::NodeConfiguration& conf) : BT::GetTopic<nav_msgs::Path>(name, conf)
  {
  }
};

class GetOccupancyGrid : public BT::GetTopic<nav_msgs::OccupancyGrid>
{
public:
  GetOccupancyGrid(const std::string& name, const BT::NodeConfiguration& conf)
    : BT::GetTopic<nav_msgs::OccupancyGrid>(name, conf)
  {
  }
};

}  // namespace behavior_tree_ros
