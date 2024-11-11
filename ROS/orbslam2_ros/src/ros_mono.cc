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
    // std::string filename = "/home/wh/wh_repository/slam/orbslam2_study/Examples/Monocular/TUM1.yaml";

    // cv::FileStorage fs(filename, cv::FileStorage::READ);
    // if (!fs.isOpened()) {
    //     std::cerr << "Failed to open settings file at: " << filename << std::endl;
    // }
    
    // std::cout << "Successfully opened settings file!" << std::endl;
    // fs.release(); // 记得关闭文件

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

    /*fault*/
    cv::FileStorage fs(ros_mono_config_dir, cv::FileStorage::READ);
    if (!fs.isOpened()) {
        std::cerr << "Failed to open settings file at: " << ros_mono_config_dir << std::endl;
    }
    
    std::cout << "Successfully opened settings file!" << std::endl;
    fs.release(); // 记得关闭文件

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(ros_mono_voc_dir,ros_mono_config_dir,ORB_SLAM2::System::MONOCULAR,true); 

    mpSLAM =  &SLAM;

    mono_image_sub = this->create_subscription<sensor_msgs::msg::Image>("/camera/color/image_raw", 10,
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

    mpSLAM->TrackMonocular(cv_ptr->image,msg->header.stamp.sec);
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



