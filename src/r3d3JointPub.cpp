// Header file for the class
#include "r3d3JointPub.hpp"

// Namespace matches ROS package name
namespace golfcart_push {

// Constructor with global and private node handle arguments
r3d3JointPub::r3d3JointPub(ros::NodeHandle n, ros::NodeHandle pn)
{
  joint_angle = 0;
  pub_joint_state = n.advertise<sensor_msgs::JointState>("joint_states",1);
  timer = n.createTimer(ros::Duration(0.02), &r3d3JointPub::timerCallback, this); 
  joint_state_msg.position.resize(2);
  joint_state_msg.name.resize(2);
  joint_state_msg.name[1] = "left_wheel_joint"; // Only do once so put in constructor
  joint_state_msg.name[0] = "right_wheel_joint"; // Compare this names to the urdf file
}

void r3d3JointPub::timerCallback(const ros::TimerEvent& event){
  double constant_speed = 1.0;
  

  joint_state_msg.header.stamp = event.current_real;
  joint_angle += 0.02*constant_speed;
  joint_state_msg.position[0] = joint_angle;
  joint_state_msg.position[1] = -joint_angle; // Left/right wheel going in opposite directions

  
  pub_joint_state.publish(joint_state_msg);
}


}
