#pragma once

// ROS headers
#include <ros/ros.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <pcl_ros/transforms.h>

// Dynamic reconfigure stuff
#include <dynamic_reconfigure/server.h>  
#include <golfcart_push/HsvExampleConfig.h>

// Message headers
#include <sensor_msgs/PointCloud2.h>  // Message definition
#include <pcl_conversions/pcl_conversions.h>  // Allows converation to ROS message
#include <geometry_msgs/PoseArray.h>
#include <avs_lecture_msgs/TrackedObjectArray.h>  // Create purple wire frames

// PCL processing headers
#include <pcl/point_types.h>
#include <pcl/common/common.h>  // copy/paste point cloud
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/passthrough.h>

namespace golfcart_push {

  class LidarFilter
  {
    public:
      LidarFilter(ros::NodeHandle n, ros::NodeHandle pn);
      
    private:
    
      void timerCallback(const ros::TimerEvent& event);
      void reconfig(HsvExampleConfig& config, uint32_t level);

      dynamic_reconfigure::Server<HsvExampleConfig> srv_;
      
      
      ros::Timer timer_;
      
  };
}
