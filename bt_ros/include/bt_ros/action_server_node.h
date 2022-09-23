#pragma once

// bt
#include "bt_ros/executor_bt.h"

// ros
#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/action_definition.h>

// boost
#include <boost/function.hpp>

namespace BT
{

template <class ActionT>
class SimpleActionServerNode
{
protected:
  using GoalType = typename ActionT::_action_goal_type::_goal_type;
  using ResultType = typename ActionT::_action_result_type::_result_type;
  using FeedbackType = typename ActionT::_action_feedback_type::_feedback_type;
  typedef boost::shared_ptr<const GoalType> GoalConstPtr;

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  bool restart_on_new_goal_ = true;
  std::string behavior_name_;
  behavior_tree_ros::ExecutorBT executor_;

  std::string action_name_;
  typename actionlib::SimpleActionServer<ActionT> action_server_;
  GoalType goal_;
  ResultType result_;

private:
  /*
   * @brief Called when a goal is received from the action client. The user should change the result_ if configuration
   * fails.
   * @return true if the goal should be accepted, false otherwise.
   */
  virtual bool onConfiguration(const GoalConstPtr& goal)
  {
    return true;
  }

  /**
   * @brief Called when the action server fails to load the BT. The user should change the result_ variable accordingly.
   */
  virtual void onLoadFail()
  {
    return;
  }

  /**
   * @brief Called when a new goal is received
   * @param goal The goal
   * @return True if the new goal is accepted, false otherwise
   */
  virtual bool onNewGoal(const GoalConstPtr& goal)
  {
    return true;
  }

  /*
   * @brief Called when the executor finishes. The user should change the member variable result_ to the desired result.
   * @param status The final status of the BT
   */
  virtual void onFinish(const BT::NodeStatus status)
  {
    return;
  }

  /*
   * @brief Called each loop (10Hz) while the BT is running. The user should change the member variable feedback_ to the
   * desired feedback.
   */
  virtual void onFeedback()
  {
    return;
  }

  /**
   * @brief Called when the action server is preempted. The user should change the member variable result_ to the
   * desired
   */
  virtual void onCancel()
  {
    return;
  }

protected:
  SimpleActionServerNode(ros::NodeHandle& pnh, const std::string& name)
    : nh_()
    , pnh_(pnh)
    , behavior_name_(pnh.param<std::string>("behavior_name", "main"))
    , action_name_(name)
    , action_server_(nh_, name, boost::bind(&SimpleActionServerNode::executeCB, this, _1), false)
  {
    action_server_.start();
  }

public:
  virtual ~SimpleActionServerNode() = default;

private:
  inline void executeCB(const GoalConstPtr& goal)
  {
    if (!onConfiguration(goal))
    {
      action_server_.setAborted(result_);
      return;
    }

    if (!executor_.loadFile(behavior_name_))
    {
      onLoadFail();
      action_server_.setAborted(result_);
      return;
    }

    ros::Rate loop_rate(10);
    while (true)
    {
      if (!ros::ok())
      {
        executor_.stop();
        return;
      }

      if (action_server_.isPreemptRequested())
      {
        if (action_server_.isNewGoalAvailable())
        {
          const GoalConstPtr new_goal = action_server_.acceptNewGoal();
          if (!onNewGoal(new_goal))
          {
            executor_.stop();
            action_server_.setAborted(result_);
            return;
          }

          if (restart_on_new_goal_)
          {
            executor_.stop();
            executor_.loadFile(behavior_name_);
            continue;
          }
        }
        else
        {
          executor_.stop();
          onCancel();
          action_server_.setPreempted(result_);
          return;
        }
      }

      if (!executor_.isRunning())
      {
        const BT::NodeStatus status = executor_.getStatus();

        onFinish(status);
        if (status == BT::NodeStatus::SUCCESS)
        {
          action_server_.setSucceeded(result_);
        }
        else if (status == BT::NodeStatus::FAILURE)
        {
          action_server_.setAborted(result_);
        }
        else
        {
          action_server_.setAborted(result_);
        }
        return;
      }
      onFeedback();
      loop_rate.sleep();
    }
  }
};

}  // namespace BT
