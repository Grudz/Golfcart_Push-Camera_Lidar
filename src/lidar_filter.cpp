#include "lidar_filter.hpp"

namespace golfcart_push {

LidarFilter::LidarFilter(ros::NodeHandle n, ros::NodeHandle pn)
{
  // Pointcloud subscriber
  sub_cloud_ = n.subscribe("input_points", 10, &LidarFilter::recvCloud, this);

  pub_cloud_= n.advertise<sensor_msgs::PointCloud2>("filtered_cloud", 1);

  // This timer just refreshes the output displays to always reflect the current threshold settings
  timer_ = n.createTimer(ros::Duration(0.05), &LidarFilter::timerCallback, this);

  // Set up dynamic reconfigure server to adjust threshold parameters
  srv_.setCallback(boost::bind(&LidarFilter::reconfig, this, _1, _2));

}
  // Passthrough filter function
  void LidarFilter::recvCloud(const sensor_msgs::PointCloud2ConstPtr& msg)
  {

    // Create pointer to PCL type variable
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZI>);
    //pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_out(new pcl::PointCloud<pcl::PointXYZI>);
    //pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out(new pcl::PointCloud<pcl::PointXYZ>);

    pcl::fromROSMsg(*msg, *cloud_in); 

    // Start filtering
    //pcl::IndicesPtr roi_indices(new std::vector <int>); // Will store subset of original array. ROI = region of interest
    //pcl::PassThrough<pcl::PointXYZI> passthrough_filter; // Instantiate filter. Will find x, y, z points that will satisfy ROI

    // Put pointer to input cloud in passthrough filter
   //passthrough_filter.setInputCloud(cloud_in);
    /*
    // Index is relative to the Lidar frame
    // Extract X points
    passthrough_filter.setFilterFieldName("x"); // 3 fields, hence PointXYZI
    passthrough_filter.setFilterLimits(cfg_.x_min, cfg_.x_max);
    passthrough_filter.filter(*roi_indices);    // Referes to input cloud

    // Extract Y points
    passthrough_filter.setIndices(roi_indices);
    passthrough_filter.setFilterFieldName("y"); 
    passthrough_filter.setFilterLimits(cfg_.y_min, cfg_.y_max);
    passthrough_filter.filter(*roi_indices);    
 
    // Extract Z points. 
    ROS_INFO("Total points in cloud in = %d\n", (int)cloud_in->size());
    //passthrough_filter.setIndices(roi_indices);
    passthrough_filter.setFilterFieldName("z"); 
    passthrough_filter.setFilterLimits(cfg_.z_min, cfg_.z_max);
    passthrough_filter.filter(*cloud_in);     // Final stage so pass filter cloud so instead of int array it's a corresponding point cloud  
    ROS_INFO("Total points in cloud out = %d\n", (int)cloud_out->size());

    // Downsample the cloud with Voxel Grid filter
    */
    pcl::VoxelGrid<pcl::PointXYZI> downsample;
    downsample.setInputCloud(cloud_in);  
    downsample.setLeafSize(cfg_.voxel_size, cfg_.voxel_size, cfg_.voxel_size); // All same so cube
    downsample.filter(*cloud_in); 
    

    



    // Copy filtered data into a ROS message
    sensor_msgs::PointCloud2 filtered_cloud;
    pcl::toROSMsg(*cloud_in, filtered_cloud);

    filtered_cloud.header = msg->header;

    // Publish Passthrough filter PointCloud
    pub_cloud_.publish(filtered_cloud);

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
