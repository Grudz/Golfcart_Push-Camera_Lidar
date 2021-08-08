#include "lidar_filter.hpp"

namespace golfcart_push {

LidarFilter::LidarFilter(ros::NodeHandle n, ros::NodeHandle pn)
{

  // This timer just refreshes the output displays to always reflect the current threshold settings
  timer_ = n.createTimer(ros::Duration(0.05), &LidarFilter::timerCallback, this);

  // Set up dynamic reconfigure server to adjust threshold parameters
  srv_.setCallback(boost::bind(&LidarFilter::reconfig, this, _1, _2));

}

  void LidarFilter::timerCallback(const ros::TimerEvent& event)
  {

    ROS_WARN("HERE\n");
    
  }
  void LidarFilter::reconfig(GolfcartPushConfig& config, uint32_t level)
  {

    return;

  }

}
