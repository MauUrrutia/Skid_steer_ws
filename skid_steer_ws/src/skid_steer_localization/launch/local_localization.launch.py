from launch_ros.actions import Node
from launch import LaunchDescription
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    robot_localization = Node (
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="screen",
        parameters=[os.path.join(get_package_share_directory("skid_steer_localization"), 
                                 "config", "ekf.yaml")]
    )   
    return LaunchDescription([
        robot_localization
    ])