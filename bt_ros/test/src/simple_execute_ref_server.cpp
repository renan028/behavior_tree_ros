/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

//! \author Vijay Pradeep

#include <actionlib/server/simple_action_server.h>
#include <actionlib/TestAction.h>
#include <ros/ros.h>

namespace actionlib
{

class SimpleExecuteRefServer
{
public:
  typedef ServerGoalHandle<TestAction> GoalHandle;

  SimpleExecuteRefServer();

private:
  ros::NodeHandle nh_;
  SimpleActionServer<TestAction> as_;

  void executeCallback(const TestGoalConstPtr& goal);
};

}  // namespace actionlib

using namespace actionlib;

SimpleExecuteRefServer::SimpleExecuteRefServer()
  : as_(nh_, "reference_action", boost::bind(&SimpleExecuteRefServer::executeCallback, this, _1), false)
{
  as_.start();
}

void SimpleExecuteRefServer::executeCallback(const TestGoalConstPtr& goal)
{
  ROS_INFO_NAMED("actionlib", "Got a goal of type [%u]", goal->goal);
  switch (goal->goal)
  {
    case 1:
      ROS_INFO_NAMED("actionlib", "Got goal #1");
      as_.setSucceeded(TestResult(), "The ref server has succeeded");
      break;
    case 2:
      ROS_INFO_NAMED("actionlib", "Got goal #2");
      as_.setAborted(TestResult(), "The ref server has aborted");
      break;
    case 3:
    {
      ROS_INFO_NAMED("actionlib", "Got goal #3");
      ros::Duration sleep_dur(.1);
      int i = 0;
      while (true)
      {
        i = i == INT_MAX ? 0 : i + 1;
        sleep_dur.sleep();
        ROS_INFO_STREAM_NAMED("actionlib", "Running: " << i);
        if (as_.isPreemptRequested())
        {
          ROS_INFO_NAMED("actionlib", "Preempted!");
          as_.setPreempted();
          return;
        }
        TestFeedback feedback;
        feedback.feedback = i;
        as_.publishFeedback(feedback);
      }
      break;
    }
    case 4:
    {
      ROS_INFO_NAMED("actionlib", "Got goal #4");
      ros::Duration sleep_dur(.1);
      for (unsigned int i = 0; i < 100; i++)
      {
        sleep_dur.sleep();
        ROS_INFO_STREAM_NAMED("actionlib", "Running: " << i);
        if (as_.isPreemptRequested())
        {
          ROS_INFO_NAMED("actionlib", "Preempted!");
          as_.setPreempted();
          return;
        }
      }
      ROS_INFO_NAMED("actionlib", "Aborted!");
      as_.setAborted();
      break;
    }
    case 5:
    {
      ROS_INFO_NAMED("actionlib", "Got goal #5");
      ros::Duration sleep_dur(.1);
      TestFeedback feedback;

      for (unsigned int i = 0; i < 10; i++)
      {
        sleep_dur.sleep();
        ROS_INFO_STREAM_NAMED("actionlib", "Running: " << i);
        if (as_.isPreemptRequested())
        {
          if (as_.isNewGoalAvailable())
          {
            ROS_INFO_NAMED("actionlib", "New goal available!");
            auto new_goal = as_.acceptNewGoal();
            if (new_goal->goal == 6)
            {
              feedback.feedback = 999;
              as_.publishFeedback(feedback);
            }
            else
            {
              as_.setAborted();
            }
          }
        }
      }
      ROS_INFO_NAMED("actionlib", "Goal 5 Success!");
      as_.setSucceeded();
      break;
    }
    case 6:
    {
      ROS_INFO_NAMED("actionlib", "Got goal #6");
      as_.setSucceeded();
      break;
    }
    default:
      break;
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ref_server");

  SimpleExecuteRefServer server;

  ros::spin();

  return 0;
}
