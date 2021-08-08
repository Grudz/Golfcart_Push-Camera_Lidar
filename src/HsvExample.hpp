#pragma once

// ROS header
#include <ros/ros.h>
#include <opencv2/opencv.hpp> // Basic opencv lib

#include <dynamic_reconfigure/server.h>
#include <golfcart_push/HsvExampleConfig.h>
#include <ros/package.h>

namespace golfcart_push {

  class HsvExample
  {
    public:
      HsvExample(ros::NodeHandle n, ros::NodeHandle pn);
      
    private:
      void reconfig(HsvExampleConfig& config, uint32_t level);
      void timerCallback(const ros::TimerEvent& event);
      
      ros::Timer refresh_timer_;
      
      dynamic_reconfigure::Server<HsvExampleConfig> srv_;
      
      // Input image channels
      cv::Mat hue_channel; // cv::Mat is for 1 image and a matrix, we will split into each channel
      cv::Mat sat_channel;
      cv::Mat val_channel;

      // Threshold output images
      cv::Mat hue_thres;
      cv::Mat sat_thres;
      cv::Mat val_thres;
  };
}
