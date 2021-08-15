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
#include <avs_lecture_msgs/TrackedObjectArray.h>

namespace golfcart_push {

  class LidarFilter
  {
    public:
      LidarFilter(ros::NodeHandle n, ros::NodeHandle pn);
      
    private:
    
      void timerCallback(const ros::TimerEvent& event);
      void reconfig(GolfcartPushConfig& config, uint32_t level);
      void recvCloud(const sensor_msgs::PointCloud2ConstPtr& msg);  

      dynamic_reconfigure::Server<GolfcartPushConfig> srv_;
      GolfcartPushConfig cfg_;
      
      ros::Timer timer_;

      ros::Subscriber sub_cloud_;
      ros::Publisher pub_cloud_;

      // KD search tree object for use by PCL functions
      pcl::search::Search<pcl::PointXYZI>::Ptr kd_tree_;
      ros::Publisher pub_cluster_cloud_;
      ros::Publisher pub_bbox_;

      // Publishing bounding box message
      avs_lecture_msgs::TrackedObjectArray bbox_array_;
      int bbox_id_;
      
  };
}
