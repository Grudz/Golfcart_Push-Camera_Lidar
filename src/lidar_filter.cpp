#include "lidar_filter.hpp"

namespace golfcart_push {

LidarFilter::LidarFilter(ros::NodeHandle n, ros::NodeHandle pn) : kd_tree_(new pcl::search::KdTree<pcl::PointXYZI>)
{
  // Pubs and subs
  sub_cloud_ = n.subscribe("input_points", 10, &LidarFilter::recvCloud, this);
  pub_cloud_= n.advertise<sensor_msgs::PointCloud2>("filtered_cloud", 1);
  pub_bbox_= n.advertise<avs_lecture_msgs::TrackedObjectArray>("bounding_boxes", 1);

  // This timer just refreshes the output displays to always reflect the current threshold settings
  timer_ = n.createTimer(ros::Duration(0.02), &LidarFilter::timerCallback, this);

  // Set up dynamic reconfigure server to adjust threshold parameters
  srv_.setCallback(boost::bind(&LidarFilter::reconfig, this, _1, _2));

}
  // Passthrough filter function
  void LidarFilter::recvCloud(const sensor_msgs::PointCloud2ConstPtr& msg)
  {

    // Create pointer to PCL type variable
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_out(new pcl::PointCloud<pcl::PointXYZI>);
    
    pcl::fromROSMsg(*msg, *cloud_in); 

    // Start filtering
    pcl::IndicesPtr roi_indices(new std::vector <int>); // Will store subset of original array. ROI = region of interest
    pcl::PassThrough<pcl::PointXYZI> passthrough_filter; // Instantiate filter. Will find x, y, z points that will satisfy ROI

    // Put pointer to input cloud in passthrough filter
    passthrough_filter.setInputCloud(cloud_in);

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

    // Extract Y points
    passthrough_filter.setIndices(roi_indices);
    passthrough_filter.setFilterFieldName("z"); 
    passthrough_filter.setFilterLimits(cfg_.z_min, cfg_.z_max);
    passthrough_filter.filter(*roi_indices);  
 
    // Extract intensity points. 
    passthrough_filter.setIndices(roi_indices);
    passthrough_filter.setFilterFieldName("intensity"); 
    passthrough_filter.setFilterLimits(cfg_.i_min, cfg_.i_max);
    passthrough_filter.filter(*cloud_out);     // Final stage so pass filter cloud so instead of int array it's a corresponding point cloud 

    // Statistical filter for removing outliers - This affects random clusters on ground and point density of large objects
    pcl::StatisticalOutlierRemoval<pcl::PointXYZI> sor;
    sor.setInputCloud(cloud_out);
    sor.setMeanK(cfg_.sor_mean);
    sor.setStddevMulThresh(cfg_.sor_stddev);
    sor.filter(*cloud_out); 

    // Copy filtered data into a ROS message
    sensor_msgs::PointCloud2 filtered_cloud;
    pcl::toROSMsg(*cloud_out, filtered_cloud);

    filtered_cloud.header = msg->header;

    // Publish Passthrough filter PointCloud
    pub_cloud_.publish(filtered_cloud);

    // Euclidean clustering 
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec;
    ec.setClusterTolerance(cfg_.cluster_tol);
    ec.setMinClusterSize(cfg_.min_cluster_size); 
    ec.setMaxClusterSize(cfg_.max_cluster_size);
    kd_tree_->setInputCloud(cloud_out);
    ec.setSearchMethod(kd_tree_);
    ec.setInputCloud(cloud_out);
    ec.extract(cluster_indices);

    // Use indices arrays to separate point cloud into individual clouds for each cluster
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cluster_clouds;
    for (auto indices : cluster_indices) 
    {
      pcl::PointCloud<pcl::PointXYZ>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZ>);
      pcl::copyPointCloud(*cloud_out, indices, *cluster);
      cluster->width = cluster->points.size();
      cluster->height = 1;
      cluster->is_dense = true;
      cluster_clouds.push_back(cluster);
    }    

    // Use max and min of each cluster to create bbox
    pcl::PointXYZ min, max;  // Relative to the Lidar

    // Copy header from passthrough cloud and clear array
    bbox_array_.header = pcl_conversions::fromPCL(cloud_out->header); // Nice way to get entire header easily
    bbox_array_.objects.clear();
    bbox_id_ = 0;

    // Loop through clusters and box up
    for (auto& cluster : cluster_clouds) 
    {  
      
      // Applying the min/max function
      pcl::getMinMax3D(*cluster, min, max);  // Get min/max 3D

      // Create bbox message, fill in fields, push it into bbox array
      avs_lecture_msgs::TrackedObject bbox;  // rosmsg show TrackedObjectArray
      
      bbox.header = bbox_array_.header;
      bbox.spawn_time.ros::Time::now(); // Spawn it right now
      bbox.id = bbox_id_++;
      bbox.bounding_box_scale.x = max.x - min.x;
      bbox.bounding_box_scale.y = max.y - min.y;
      bbox.bounding_box_scale.z = max.z - min.z;
      bbox.pose.position.x = (max.x + min.x) / 2; 
      bbox.pose.position.y = (max.y + min.y) / 2; 
      bbox.pose.position.z = (max.z + min.z) / 2; 
      bbox.pose.orientation.w = 1.0;

      if (abs(bbox.bounding_box_scale.z) < cfg_.z_large)
      {
        continue;
      } 
      else
      {
        bbox_array_.objects.push_back(bbox);
      }      
    } 
    
    pub_bbox_.publish(bbox_array_);

  }

  void LidarFilter::timerCallback(const ros::TimerEvent& event)
  {   

    static tf2_ros::TransformBroadcaster br;
    geometry_msgs::TransformStamped transformStamped;
      
    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = "map";
    transformStamped.child_frame_id = "base_footprint";
    transformStamped.transform.translation.x = -push_;
    transformStamped.transform.translation.y = 0.0;
    transformStamped.transform.translation.z = 0.0;
    tf2::Quaternion quat;
    quat.setRPY(0, 0, -1.5);
    transformStamped.transform.rotation.x = quat.x();
    transformStamped.transform.rotation.y = quat.y();
    transformStamped.transform.rotation.z = quat.z();
    transformStamped.transform.rotation.w = quat.w();

    br.sendTransform(transformStamped);  
    push_ = push_ + cfg_.tf_increment;
    if (push_ >= 11.011) // Calculated based on timer freq and bag time
    {
      push_ = -0.007;
    }
  }

  void LidarFilter::reconfig(GolfcartPushConfig& config, uint32_t level)
  {

    cfg_ = config;

  }

}
