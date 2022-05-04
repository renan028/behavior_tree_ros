// behavior_tree_ros
#include "behavior_tree_ros/executor_bt.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "tree_executor");
  behavior_tree_ros::ExecutorBT bt_exec;

  ros::MultiThreadedSpinner spinner(0);
  spinner.spin();
}
