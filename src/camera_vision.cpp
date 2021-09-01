#include "camera_vision.hpp"

namespace golfcart_push {

CameraVision::CameraVision(ros::NodeHandle n, ros::NodeHandle pn):
  listener_(buffer_), 
  kd_tree_(new pcl::search::KdTree<pcl::PointXYZ>)
{
  // Pubs and subs
  sub_camera_ = n.subscribe("input_images", 1, &CameraVision::recvImage, this);
  sub_cam_info_ = n.subscribe("/front_camera/camera_info", 1, &CameraVision::recvCameraInfo, this); // Message with intrinsic properties
  pub_cam_cloud_ = n.advertise<sensor_msgs::PointCloud2>("cam_cloud", 1);
  pub_markers_ = n.advertise<visualization_msgs::MarkerArray>("projected_lines", 1);
  pub_cam_bbox_= n.advertise<avs_lecture_msgs::TrackedObjectArray>("cam_bounding_boxes", 1);

  // This timer just refreshes the output displays to always reflect the current threshold settings
  timer_ = n.createTimer(ros::Duration(0.05), &CameraVision::timerCallback, this);

  // Set up dynamic reconfigure server to adjust threshold parameters
  srv_.setCallback(boost::bind(&CameraVision::reconfig, this, _1, _2));

  looked_up_camera_transform_ = false;

  cv::namedWindow("Binary", cv::WINDOW_AUTOSIZE);

}

  // Passthrough filter function
  void CameraVision::recvImage(const sensor_msgs::ImageConstPtr& msg)
  {
    
    // Do nothing until the coordinate transform from footprint to camera is valid,
    // because otherwise there is no point in detecting a lane!
    if (!looked_up_camera_transform_) {
      try {                                           // Forward Transform      FROM              TO
        geometry_msgs::TransformStamped transform = buffer_.lookupTransform("base_footprint", "camera", msg->header.stamp);
        tf2::convert(transform.transform, camera_transform_);
        looked_up_camera_transform_ = true; // Once the lookup is successful, there is no need to keep doing the lookup
                                            // because the transform is constant
      } catch (tf2::TransformException& ex) {
        ROS_WARN_THROTTLE(1.0, "%s", ex.what());
      }
      return;
    }

    // Convert ROS image message into an OpenCV Mat
    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);  // Kinda like PCL library
    cv::Mat raw_img = cv_ptr->image;
    cv::Mat bin_img;
    segmentImage(raw_img, bin_img);
    cv::imshow("Binary", bin_img);
    cv::waitKey(1);

    // Create pinhole camera model instance and load
    // its parameters from the camera info
    // generated using the checkerboard calibration program
    image_geometry::PinholeCameraModel model;
    model.fromCameraInfo(camera_info_);

    // Project points from 2D pixel coordinates into 3D where it intersects
    // the ground plane, and put them in a PCL point cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr bin_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr bin_cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);

    // Project every fourth row of the image to save some computational resources
    for (int i = 0; i < bin_img.rows; i += 4) {
      for (int j = 0; j < bin_img.cols; j++) {  // Side to side more important, so hit every col
        if (bin_img.at<uint8_t>(i, j) == 255) {
          // We found a white pixel corresponding to a lane marking. Project to ground
          // and add to point cloud
          pcl::PointXYZ proj_p = projectPoint(model, cv::Point(j, i)); // cv::Point x and y so col, row (compare with line 69)
          bin_cloud->points.push_back(proj_p);
        }
      }
    }

    pcl::PassThrough<pcl::PointXYZ> passthrough_filter; // Instantiate filter. Will find x, y, z points that will satisfy ROI
    pcl::IndicesPtr roi_indices(new std::vector <int>);

    // Put pointer to input cloud in passthrough filter
    passthrough_filter.setInputCloud(bin_cloud);

    // Index is relative to the Lidar frame
    // Extract X points
    passthrough_filter.setFilterFieldName("x"); // 3 fields, hence PointXYZI
    passthrough_filter.setFilterLimits(cfg_.cam_x_min, cfg_.cam_x_max);
    passthrough_filter.filter(*roi_indices);    // Referes to input cloud

    // Extract Y points
    passthrough_filter.setIndices(roi_indices);
    passthrough_filter.setFilterFieldName("y"); 
    passthrough_filter.setFilterLimits(cfg_.cam_y_min, cfg_.cam_y_max);
    passthrough_filter.filter(*roi_indices);  

    // Extract Y points
    passthrough_filter.setIndices(roi_indices);
    passthrough_filter.setFilterFieldName("z"); 
    passthrough_filter.setFilterLimits(cfg_.cam_z_min, cfg_.cam_z_max);
    passthrough_filter.filter(*bin_cloud_filtered);  
 
    //std::cout << bin_cloud_filtered->size() << std::endl;

    // Publish point cloud to visualize in Rviz
    sensor_msgs::PointCloud2 cam_cloud_msg;
    pcl::toROSMsg(*bin_cloud_filtered, cam_cloud_msg);
    cam_cloud_msg.header.frame_id = "camera";
    cam_cloud_msg.header.stamp = msg->header.stamp;
    cam_cloud_msg.header.seq = msg->header.seq;
    pub_cam_cloud_.publish(cam_cloud_msg);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cam_cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(cam_cloud_msg, *cam_cloud_filtered);

    // Use Euclidean clustering to group dashed lines together
    // and separate each distinct line into separate point clouds
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance(cfg_.cam_cluster_tol);
    ec.setMinClusterSize(cfg_.cam_min_cluster_size);
    ec.setMaxClusterSize(cfg_.cam_max_cluster_size);
    kd_tree_->setInputCloud(cam_cloud_filtered);
    ec.setSearchMethod(kd_tree_);
    ec.setInputCloud(cam_cloud_filtered);
    ec.extract(cluster_indices);

    //ROS_INFO("Camera Header = %s\n", cam_cloud_filtered->header.frame_id.c_str());

    // Use indices arrays to separate point cloud into individual clouds for each cluster
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cluster_clouds;
    for (auto indices : cluster_indices) {
      pcl::PointCloud<pcl::PointXYZ>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZ>);
      pcl::copyPointCloud(*cam_cloud_filtered, indices, *cluster);
      cluster->width = cluster->points.size();
      cluster->height = 1;
      cluster->is_dense = true;
      //ROS_INFO("Camera Header = %s\n", cluster->header.frame_id.c_str());
      cluster_clouds.push_back(cluster);
    }
    

    // Construct polynomial curve fits to each cluster cloud
    std::vector<CurveFit> curves;
    for (size_t i = 0; i < cluster_clouds.size(); i++) {
      CurveFit new_curve;
      bool successful_fit = fitPoints(cluster_clouds[i], cfg_.fit_order, new_curve);
      if (!successful_fit) {
        continue;
      }

      bool good_fit = checkCurve(cluster_clouds[i], new_curve);
      if (good_fit) {
        curves.push_back(new_curve);
      }
    }

    // Construct Rviz marker output to visualize curve fit
    publishMarkers(curves);
    
    // Use max and min of each cluster to create bbox
    pcl::PointXYZ min, max;  
    //ROS_INFO("\n----- Loop Start -----\n");
    //ROS_INFO("Clusters in scan = %d\n", (int)cluster_clouds.size());  // Is this so bad? Draw a new box for each new sequence?

    // Copy header from passthrough cloud and clear array
    bbox_cam_array_.header = pcl_conversions::fromPCL(cam_cloud_filtered->header); // Nice way to get entire header easily
    bbox_cam_array_.objects.clear();

    // Loop through clusters and box up
    for (auto& cluster : cluster_clouds) 
    {  
      
      // Applying the min/max function
      pcl::getMinMax3D(*cluster, min, max);  // Get min/max 3D
      //ROS_INFO("Camera - Header seq per cluster = %d\n", (int)cluster->header.seq);

      //ROS_INFO("Cluster Header = %s\n", cluster->header.frame_id.c_str());

      // Create bbox message, fill in fields, push it into bbox array
      avs_lecture_msgs::TrackedObject bbox;  // rosmsg show TrackedObjectArray
      
      bbox.header = bbox_cam_array_.header;
      bbox.spawn_time.ros::Time::now(); // Spawn it right now
      bbox.id = 2;
      bbox.bounding_box_scale.x = max.x - min.x;
      bbox.bounding_box_scale.y = max.y - min.y;
      bbox.bounding_box_scale.z = 0.1;
      bbox.pose.position.x = (max.x + min.x) / 2; 
      bbox.pose.position.y = (max.y + min.y) / 2; 
      bbox.pose.position.z = (max.z + min.z) / 2; 
      bbox.pose.orientation.w = 1.0;   
      bbox_cam_array_.objects.push_back(bbox);
    } 
    
    pub_cam_bbox_.publish(bbox_cam_array_);

  }

  // This function inputs a PCL point cloud and applies a least squares curve fit
  // to the points which fits an optimal polynomial curve of the desired order
  bool CameraVision::fitPoints(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, int order, CurveFit& curve)
  {
    // Check if it is mathematically possible to fit a curve to the data
    if (cloud->points.size() <= order) {
      return false;
    }

    Eigen::MatrixXd regression_matrix(cloud->points.size(), order + 1);
    Eigen::VectorXd y_samples(cloud->points.size());

    // These initializers are important
    curve.min_x = INFINITY;
    curve.max_x = 0.0;
    for (int i = 0; i < cloud->points.size(); i++) {
      y_samples(i) = cloud->points[i].y;

      // Fill in row of regression matrix
      // [1, x, x^2, ..., x^N]
      double tx = 1.0;
      for (int j = 0; j <= order; j++) {
        regression_matrix(i, j) = tx;
        tx *= cloud->points[i].x;
      }

      // Compute the minimum value of x to constrain
      // the polynomial curve
      if (cloud->points[i].x < curve.min_x) {
        curve.min_x = cloud->points[i].x;
      }

      // Compute the maximum value of x to constrain
      // the polynomial curve
      if (cloud->points[i].x > curve.max_x) {
        curve.max_x = cloud->points[i].x;
      }
    }

    // Invert regression matrix with left pseudoinverse operation - Slide 9
    Eigen::MatrixXd pseudoinverse_matrix = (regression_matrix.transpose() * regression_matrix).inverse() * regression_matrix.transpose();

    // Perform least squares estimation and obtain polynomial coefficients
    Eigen::VectorXd curve_fit_coefficients = pseudoinverse_matrix * y_samples;

    // Populate 'poly_coeff' field of the 'curve' argument output
    for (int i = 0; i < curve_fit_coefficients.rows(); i++) {
      curve.poly_coeff.push_back(curve_fit_coefficients(i)); // () not [] for eigen arrays
    }

    return true; // Successful curve fit!
  }

  bool CameraVision::checkCurve(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, const CurveFit& curve)
  {
    // Compute error between each sample point y and the expected value of y
    std::vector<double> error_samples;
    for (size_t i = 0; i < cloud->points.size(); i++) {
      double new_error;

      // Compute expected y value based on the order of the curve
      // y = a0 + a1 * x + a2 * x^2 + ... + aM * x^M
      double y_hat = 0.0;
      double t = 1.0;
      for (size_t j = 0; j < curve.poly_coeff.size(); j++) {
        // Add a term to y_hat for each coefficient in the polynomial
        y_hat += t * curve.poly_coeff[j];
        t *= cloud->points[i].x;
      }
      new_error = cloud->points[i].y - y_hat;
      error_samples.push_back(new_error);
    }

    // Compute mean squared error
    double mean_square = 0;
    for (size_t i = 0; i < cloud->points.size(); i++) {
      mean_square += error_samples[i] * error_samples[i];
    }
    mean_square /= (double)cloud->points.size();

    // RMS value is the square root of the mean squared error
    double rms = sqrt(mean_square);

    // Return boolean indicating success or failure
    return rms < cfg_.rms_tolerance;  // 1 if satisfied, 0 if not
  }

  void CameraVision::publishMarkers(const std::vector<CurveFit>& curves)
  {
    visualization_msgs::MarkerArray marker_msg;
    visualization_msgs::Marker viz_marker;
    viz_marker.header.frame_id = "base_footprint";
    viz_marker.header.stamp = ros::Time::now();
    viz_marker.action = visualization_msgs::Marker::ADD;
    viz_marker.pose.orientation.w = 1;
    viz_marker.id = 0;

    // Use the LINE_STRIP type to display a line connecting each point
    viz_marker.type = visualization_msgs::Marker::LINE_STRIP;

    // Red
    viz_marker.color.a = 1.0;
    viz_marker.color.r = 0.0;
    viz_marker.color.g = 1.0;
    viz_marker.color.b = 0.0;

    // 0.1 meters thick line
    viz_marker.scale.x = 0.06;

    // Sample polynomial and add line strip marker to array
    // for each separate cluster
    for (auto& curve : curves) {
      visualizePoints(curve, viz_marker.points);
      marker_msg.markers.push_back(viz_marker);
      viz_marker.id++;
    }

    // Delete markers to avoid ghost markers from lingering if
    // the number of markers being published changes
    visualization_msgs::MarkerArray delete_markers;
    delete_markers.markers.resize(1);
    delete_markers.markers[0].action = visualization_msgs::Marker::DELETEALL;
    pub_markers_.publish(delete_markers);

    // Publish for visualization
    pub_markers_.publish(marker_msg);
  }

  // This method samples a curve fit between its minimum and maximum valid values
  // and outputs an array of geometry_msgs::Point to put into a Rviz marker message
  void CameraVision::visualizePoints(const CurveFit& curve, std::vector<geometry_msgs::Point>& points)
  {
    points.clear();

    // Sample points at 0.05 meter resolution
    for (double x = curve.min_x; x <= curve.max_x; x += 0.05) {
      geometry_msgs::Point p;
      // Copy x value
      p.x = x;

      double t = 1.0;
      for (size_t i = 0; i < curve.poly_coeff.size(); i++) {
        p.y += curve.poly_coeff[i] * t;
        t *= p.x;
      }

      points.push_back(p);
    }
  }

  // Project 2D pixel point 'p' into vehicle's frame and return as 3D point
  pcl::PointXYZ CameraVision::projectPoint(const image_geometry::PinholeCameraModel& model, const cv::Point2d& p)
  {
    // Convert the input pixel coordinates into a 3d ray, where x and y are projected to the point where z is equal to 1.0
    cv::Point3d cam_frame_ray = model.projectPixelTo3dRay(p);
    //std::cout << "x " << cam_frame_ray.x << std::endl;
    //std::cout << "y " << cam_frame_ray.y << std::endl;
    //std::cout << "z " << cam_frame_ray.z << std::endl; // Z = 1

    // Represent camera frame ray in footprint frame -> Slide 18 equation
    // getRotation returns quat but getBasis returns tf type
    tf2::Vector3 footprint_frame_ray = camera_transform_.getBasis() * tf2::Vector3(cam_frame_ray.x, cam_frame_ray.y, cam_frame_ray.z);
    //tf2::Vector3 footprint_frame_ray = camera_transform_.getBasis() * tf2::Vector3(cfg_.cam_frame_y, cfg_.cam_frame_x, cam_frame_ray.z);

    // Using the concept of similar triangles, scale the unit vector such that the end is on the ground plane.
    //double s = -camera_transform_.getOrigin().z() / footprint_frame_ray.z(); // Slide 19 equation
    //tf2::Vector3 ground_plane_ray = s * footprint_frame_ray;
    tf2::Vector3 ground_plane_ray = cfg_.scale_factor * footprint_frame_ray;

    // Then add camera position offset to obtain the final coordinates in footprint frame
    tf2::Vector3 vehicle_frame_point = ground_plane_ray + camera_transform_.getOrigin();

    // Fill output point with the result of the projection
    pcl::PointXYZ point;
    point.x = vehicle_frame_point.x() + cfg_.cam_frame_x;
    point.y = vehicle_frame_point.y() + cfg_.cam_frame_y;
    //point.x = vehicle_frame_point.y() + cfg_.cam_frame_y;
    //point.y = vehicle_frame_point.y() + cfg_.cam_frame_y;
    point.z = vehicle_frame_point.z();
    return point;
  }

  void CameraVision::segmentImage(const cv::Mat& raw_img, cv::Mat& bin_img)
  {
    // Convert to HSV colorspace
    cv::Mat raw_hsv;
    cv::cvtColor(raw_img, raw_hsv, CV_BGR2HSV);

    // Split HSV image into separate single-channel images for H, S, and V
    // and store each in dedicated variables
    std::vector<cv::Mat> split_img;
    cv::split(raw_hsv, split_img);
    cv::Mat hue_img = split_img[0]; 
    cv::Mat sat_img = split_img[1];
    cv::Mat val_img = split_img[2];

    // Detect white lane marking pixels in the image
    cv::Mat white_bin_img = cv::Mat::zeros(raw_hsv.size(), CV_8U); // Have to initialize to 0's
    detectTape(hue_img, sat_img, val_img, white_bin_img);

    bin_img = white_bin_img;
    
  }

  void CameraVision::detectTape(const cv::Mat& hue_img, const cv::Mat& sat_img, const cv::Mat& val_img, cv::Mat& white_bin_img)
  {
    // Apply threshold to generate a binary value image. White lines have
    // higher value than the road
    cv::Mat val_thres;
    cv::threshold(val_img, val_thres, cfg_.val_thres, 255, cv::THRESH_BINARY);

    // Apply inverse threshold to generate a binary saturation image. We want
    // to throw out high saturation pixels because white has very low saturation
    cv::Mat sat_thres;
    cv::threshold(sat_img, sat_thres, cfg_.sat_thres, 255, cv::THRESH_BINARY_INV); // Low saturation

    // Threshold hue
    cv::Mat t1;
    cv::Mat t2;
    cv::Mat hue_thres;
    int h_pos_edge = cfg_.h_center + cfg_.h_width; // Upper edge of hue window
    int h_neg_edge = cfg_.h_center - cfg_.h_width; // Lower edge of hue window
    if (h_pos_edge > 180) {
      // Apply thresholds when upper edge overflows 180
      cv::threshold(hue_img, t1, h_pos_edge - 180, 255, cv::THRESH_BINARY_INV);  
      cv::threshold(hue_img, t2, cfg_.h_center - cfg_.h_width, 255, cv::THRESH_BINARY);  
      cv::bitwise_or(t1, t2, hue_thres);
    } else if (h_neg_edge < 0) {
      // Apply thresholds when lower edge underflows 0
      cv::threshold(hue_img, t1, h_neg_edge + 180, 255, cv::THRESH_BINARY);  
      cv::threshold(hue_img, t2, cfg_.h_center + cfg_.h_width, 255, cv::THRESH_BINARY_INV);  
      cv::bitwise_or(t1, t2, hue_thres);
    } else {
      // Apply thresholds when hue window is continuous
      cv::threshold(hue_img, t1, cfg_.h_center - cfg_.h_width, 255, cv::THRESH_BINARY);
      cv::threshold(hue_img, t2, cfg_.h_center + cfg_.h_width, 255, cv::THRESH_BINARY_INV);
      cv::bitwise_and(t1, t2, hue_thres);
    }

    // Apply bitwise AND to make sure only pixels that satisfy both value and saturation
    // thresholds make it out. Store result in ouput (white_bin_img)
    cv::Mat temp_img;
    cv::bitwise_and(val_thres, sat_thres, temp_img);
    cv::bitwise_and(hue_thres, temp_img, white_bin_img);

  }

  void CameraVision::recvCameraInfo(const sensor_msgs::CameraInfoConstPtr& msg)
  {
    camera_info_ = *msg;
  }

  void CameraVision::timerCallback(const ros::TimerEvent& event)
  {

    return;
    
  }
  void CameraVision::reconfig(GolfcartPushConfig& config, uint32_t level)
  {

    cfg_ = config;

  }

}
