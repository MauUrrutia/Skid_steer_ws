from launch_ros.actions import Node
from launch import LaunchDescription
import os
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    use_sim_time_arg = DeclareLaunchArgument(
    "use_sim_time",
    default_value="false",
    description="Use simulation time"
    )
    robot_localization = Node (
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="screen",
        parameters=[os.path.join(get_package_share_directory("skid_steer_localization"), 
                                 "config", "ekf.yaml"), {"use_sim_time" : use_sim_time}]
    )   
    return LaunchDescription([
        use_sim_time_arg,
        robot_localization
    ])