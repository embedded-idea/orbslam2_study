echo "Building ROS2 nodes"
#check opencv version python3 -c "import cv2; print(cv2.__version__)"
export LD_LIBRARY_PATH=/home/mzhang/home_work/orb-slam2/orbslam2_study/lib:$LD_LIBRARY_PATH

rm -rf Examples/ROS2/build Examples/ROS2/install Examples/ROS2/log

source /opt/ros/humble/setup.bash

export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:/home/mzhang/home_work/orb-slam2/orbslam2_study/Examples/ROS2
export ORB_SLAM2_ROOT_DIR=/home/mzhang/home_work/orb-slam2/orbslam2_study

cd Examples/ROS2/
colcon build

#if you want to run 
#before you run, you need to run camera first i have realsense D455
#ros2 run realsense2_camera realsense2_camera_node 
source install/setup.sh
ros2 launch orbslam2_ros mono.launch.py
