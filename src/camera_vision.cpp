#include "camera_vision.hpp"

namespace golfcart_push {

CameraVision::CameraVision(ros::NodeHandle n, ros::NodeHandle pn)
{
  // Pointcloud subscriber
  sub_camera_ = n.subscribe("input_images", 1, &CameraVision::recvImage, this);

  // This timer just refreshes the output displays to always reflect the current threshold settings
  timer_ = n.createTimer(ros::Duration(0.05), &CameraVision::timerCallback, this);

  // Set up dynamic reconfigure server to adjust threshold parameters
  srv_.setCallback(boost::bind(&CameraVision::reconfig, this, _1, _2));

  cv::namedWindow("Binary", cv::WINDOW_AUTOSIZE);

}
  // Passthrough filter function
  void CameraVision::recvImage(const sensor_msgs::ImageConstPtr& msg)
  {

    // Convert ROS image message into an OpenCV Mat
    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);  // Kinda like PCL library
    cv::Mat raw_img = cv_ptr->image;
    cv::Mat bin_img;
    segmentImage(raw_img, bin_img);
    cv::imshow("Binary", bin_img);
    cv::waitKey(1);

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

  void CameraVision::timerCallback(const ros::TimerEvent& event)
  {

    return;
    
  }
  void CameraVision::reconfig(GolfcartPushConfig& config, uint32_t level)
  {

    cfg_ = config;

  }

}
