#include <ros/ros.h>

#include "global_planner.h"

using namespace Global_Planning;

int main(int argc, char **argv)
{
  ROS_INFO("===============global_planner===========");
  ros::init(argc, argv, "global_planner");

  ros::NodeHandle nh("~");

  Global_Planner global_planner;
  global_planner.init(nh);

  // 多线程旋转
  ros::AsyncSpinner spinner(4); // 使用4个线程
  spinner.start();
  ros::waitForShutdown();
  return 0;
}
