from ament_index_python.packages import get_package_share_directory
import launch
import launch.actions
import launch.substitutions
import launch_ros.actions
import os.path

def generate_launch_description():
    para_dir = os.path.join(get_package_share_directory('orbslam2_ros'), 'config', 'ros_tum.yaml')
    return launch.LaunchDescription([

        launch_ros.actions.Node(
            package='orbslam2_ros',
            executable='mono',
            name='mono',
            parameters=[para_dir],
            output='screen'
        )
    ])
