/*

~ Linux Penguin ~

        _nnnn_
        dGGGGMMb
       @p~qp~~qMb
       M|@||@) M|
       @,----.JM|
      JS^\__/  qKL
     dZP        qKRb
    dZP          qKKb
   fZP            SMMb
   HZM            MMMM
   FqM            MMMM
 __| ".        |\dS"qML
 |    `.       | `' \Zq
_)      \.___.,|     .'
\____   )MMMMMP|   .'
     `-'       `--' 

*/

#pragma once

// ROS headers
#include <ros/ros.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <pcl_ros/transforms.h>

// Dynamic reconfigure stuff
#include <dynamic_reconfigure/server.h>  
#include <golfcart_push/GolfcartPushConfig.h>

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
#include <pcl/filters/statistical_outlier_removal.h>

// Image processing and camera geometry headers
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_geometry/pinhole_camera_model.h>

namespace golfcart_push {

  class CameraVision
  {
    public:
      CameraVision(ros::NodeHandle n, ros::NodeHandle pn);
      
    private:
    
      void timerCallback(const ros::TimerEvent& event);
      void reconfig(GolfcartPushConfig& config, uint32_t level);
      void recvImage(const sensor_msgs::ImageConstPtr& msg); 
      void segmentImage(const cv::Mat& raw_img, cv::Mat& bin_img); 
      void detectWhite(const cv::Mat& sat_img, const cv::Mat& val_img, cv::Mat& white_bin_img);

      dynamic_reconfigure::Server<GolfcartPushConfig> srv_;
      GolfcartPushConfig cfg_;
      
      ros::Timer timer_;

      ros::Subscriber sub_camera_;

      std::string camera_name_;

      
  };
}