// ROS and node class header file
#include <ros/ros.h>
#include "lidar_filter.hpp"

int main(int argc, char** argv)
{
  // Initialize ROS and declare node handles
  ros::init(argc, argv, "lidar_filter");
  ros::NodeHandle n;
  ros::NodeHandle pn("~");
  
  // Instantiate node class
  golfcart_push::LidarFilter node(n, pn);
  
  // Spin and process callbacks
  ros::spin();
}
