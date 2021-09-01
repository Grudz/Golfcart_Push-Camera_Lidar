// Include guard to prevent multiple declarations
#ifndef R3D3JOINTPUB_H
#define R3D3JOINTPUB_H

// ROS header
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

// Namespace matches ROS package name
namespace golfcart_push{

  class r3d3JointPub
  {

    public:
      r3d3JointPub(ros::NodeHandle n, ros::NodeHandle pn);
      
    private: 

      void timerCallback(const ros::TimerEvent& event);

      ros::Publisher pub_joint_state;
      ros::Timer timer;

      sensor_msgs::JointState joint_state_msg;
      double joint_angle;

    };

  }

#endif // NODECLASS_H

