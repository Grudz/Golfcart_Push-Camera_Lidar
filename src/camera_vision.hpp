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
#include <visualization_msgs/MarkerArray.h>

namespace golfcart_push {

  // Custom variable type
  typedef struct {
    // (float is 32bit while double is 64bit)
    std::vector<double> poly_coeff; // Coefficients of the polynomial

    // Because curve is neg infinity to pos infinity
    double min_x; // Minimum x value where polynomial is defined
    double max_x; // Maximum x value where polynomial is defined
  } CurveFit;

  class CameraVision
  {
    public:
      CameraVision(ros::NodeHandle n, ros::NodeHandle pn);
      
    private:
      
      void timerCallback(const ros::TimerEvent& event); // Use later maybe
      void reconfig(GolfcartPushConfig& config, uint32_t level);
      void recvImage(const sensor_msgs::ImageConstPtr& msg); 
      void segmentImage(const cv::Mat& raw_img, cv::Mat& bin_img); // Get HSV out of image
      void detectTape(const cv::Mat& hue_img, const cv::Mat& sat_img, const cv::Mat& val_img, cv::Mat& white_bin_img); // Filter image using HSV to get tape
      void recvCameraInfo(const sensor_msgs::CameraInfoConstPtr& msg);
      pcl::PointXYZ projectPoint(const image_geometry::PinholeCameraModel& model, const cv::Point2d& p); // Create PCL point
      
      // Create line to fit to camera PCL points
      void publishMarkers(const std::vector<CurveFit>& curves); 
      void visualizePoints(const CurveFit& curve, std::vector<geometry_msgs::Point>& points);
      bool checkCurve(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, const CurveFit& curve);
      bool fitPoints(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, int order, CurveFit& curve);

      // Reconfigure setup
      dynamic_reconfigure::Server<GolfcartPushConfig> srv_;
      GolfcartPushConfig cfg_;
      
      ros::Timer timer_;

      ros::Publisher pub_cam_cloud_;
      ros::Publisher pub_markers_;
      ros::Publisher pub_cam_bbox_;
      ros::Subscriber sub_camera_;
      ros::Subscriber sub_cam_info_;
      sensor_msgs::CameraInfo camera_info_;

      // Camera transform variables
      tf2::Transform camera_transform_; // Coordinate transformation from base_footprint to camera
      bool looked_up_camera_transform_;
      tf2_ros::TransformListener listener_;
      tf2_ros::Buffer buffer_;  


      // Publishing bounding box message
      avs_lecture_msgs::TrackedObjectArray bbox_cam_array_;

      // KD search tree object for use by PCL functions
      pcl::search::Search<pcl::PointXYZ>::Ptr kd_tree_;

  };
}
