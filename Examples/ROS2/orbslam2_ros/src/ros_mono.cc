/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/


#include "orbslam2_ros/ros_mono.hpp"

Mono_ImageGrabber::Mono_ImageGrabber(const std::string &node_name,const rclcpp::NodeOptions &node_options)
    :Node(node_name,node_options)
{ 
    init();
}

Mono_ImageGrabber::~Mono_ImageGrabber()
{
    mpSLAM->Shutdown();

    mpSLAM->SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
}

void Mono_ImageGrabber::init()
{
    this->declare_parameter("ros_mono_voc_dir", "");
    ros_mono_voc_dir = this->get_parameter("ros_mono_voc_dir").get_parameter_value().get<std::string>();

    this->declare_parameter("ros_mono_config_dir", "");
    ros_mono_config_dir = this->get_parameter("ros_mono_config_dir").get_parameter_value().get<std::string>();
    
    if (ros_mono_voc_dir.empty()) {
        std::cerr << "Vocabulary file is not set correctly!" << std::endl;
    }
    if (ros_mono_config_dir.empty()) {
        std::cerr << "Setting file is not set correctly!" << std::endl;
    }

    std::cout<<"vocabulary_file: "<<ros_mono_voc_dir<<std::endl;
    std::cout<<"setting_file: "<<ros_mono_config_dir<<std::endl;

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    mpSLAM = new ORB_SLAM2::System(ros_mono_voc_dir,ros_mono_config_dir,ORB_SLAM2::System::MONOCULAR,true); 

    std::cout <<"mpSlAM address: " << mpSLAM << std::endl;
    mono_image_sub = this->create_subscription<sensor_msgs::msg::Image>("/camera/camera/color/image_raw", 10,
                        std::bind(&Mono_ImageGrabber::GrabImage, this, std::placeholders::_1));
}

void Mono_ImageGrabber::GrabImage(const sensor_msgs::msg::Image::SharedPtr msg)
{
    // Copy the ros image message to cv::Mat.
    try
    {
        cv_ptr = cv_bridge::toCvShare(msg);
    }
    catch (cv_bridge::Exception& e)
    {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }

    //mpSLAM->TrackMonocular(cv_ptr->image,msg->header.stamp.sec);
    mpSLAM->TrackMonocular(cv_ptr->image, cv_ptr->header.stamp.sec + cv_ptr->header.stamp.nanosec * 1e-9);
    std::cout << msg->header.stamp.sec << std::endl;
}

void saveCvImage(const cv_bridge::CvImageConstPtr& cvImagePtr, const std::string& filePath) {
    try {
        // Convert cv_bridge::CvImageConstPtr to cv::Mat
        cv::Mat image = cvImagePtr->image;

        // Save the image to a file
        // if (!cv::imwrite(filePath, image)) {
        //     throw std::runtime_error("Failed to save the image to file: " + filePath);
        // }
        std::cout << "Image saved to " << filePath << std::endl;
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
    }
}

int main(int argc, char **argv)
{

    rclcpp::init(argc, argv);

    rclcpp::NodeOptions mono_options;

    auto node = std::make_shared<Mono_ImageGrabber>("orbslam2_ros",mono_options);

    rclcpp::executors::MultiThreadedExecutor mono_exector;

    mono_exector.add_node(node);
    mono_exector.spin();

    rclcpp::shutdown();

    return 0;
}



