#ifndef __ROS_MONO_HPP__
#define __ROS_MONO_HPP__


#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>

#include <rclcpp/rclcpp.hpp>
#include <cv_bridge/cv_bridge.h>

#include<opencv2/core/core.hpp>

#include"System.h"

using namespace std;

ORB_SLAM2::System* mpSLAM;

class Mono_ImageGrabber : public rclcpp::Node
{
public:

    Mono_ImageGrabber(const std::string &node_name,const rclcpp::NodeOptions &node_options);
    ~Mono_ImageGrabber();
public:
    std::string ros_mono_voc_dir;
    std::string ros_mono_config_dir;

    cv_bridge::CvImageConstPtr cv_ptr;

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr mono_image_sub;

    void init();
    void GrabImage(const sensor_msgs::msg::Image::SharedPtr msg);

};



#endif


