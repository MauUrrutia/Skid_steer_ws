from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir
from launch_ros.actions import Node
import os

def generate_launch_description():
    # Ruta al archivo de lanzamiento de nav2_bringup
    nav2_launch_file = os.path.join(
        '/opt/ros/humble/share/nav2_bringup/launch',
        'navigation_launch.py'
    )

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(nav2_launch_file),
            launch_arguments={
                'use_sim_time': 'true'
            }.items()
        ),
        Node(
            package='nav2_bringup',
            executable='lifecycle_manager',
            output='screen',
            remappings=[
                ('/cmd_vel', '/skid_steer_V1/cmd_vel')
            ]
        )
    ])