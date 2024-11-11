1. nano ~/.bashrc
	export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:/path/to/ros_package
	# Set ORB_SLAM2 directory
	export ORB_SLAM2_ROOT_DIR=/path/to/orbslam2_lib
2. source ~/.bashrc

3. 修改ROS/orbslam2_ros/src/ros_mono.cc文件  
	修改订阅topic为相机输出的原始图像topic  79行

4. 替代Examples下的ROS

5. 替代根目录下的build_ros.sh

6. 运行build_ros.sh(注意权限)

7. 进入/Examples/ROS目录，会看到已经构建好的功能包 build install log orbslam2_ros

8. source install/setup.sh

9. 启动节点 ros2 launch orbslam2_ros mono.launch.py
