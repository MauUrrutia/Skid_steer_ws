import os
from launch_ros.actions import Node
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    gazebo = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("skid_steer_description"),
            "launch",
            "gazebo_V1.launch.py"
        ),
        launch_arguments={
                          "use_sim_time": use_sim_time
                         }.items()
    )

    controller = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("skid_steer_controller"),
            "launch",
            "controller_V1.launch.py"
        ),
        launch_arguments={"use_simple_controller": "False",
                          "use_sim_time": use_sim_time,
                          "use_python": "False"}.items()
    )

    ekf_localization_filter = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("skid_steer_localization"),
            "launch",
            "local_localization.launch.py"
        ),
    launch_arguments={"use_sim_time": use_sim_time}.items()
)
    # SLAM Toolbox 
    slam_toolbox = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            os.path.join(
                get_package_share_directory('skid_steer_slam'),
                'config',
                'mapper_params_online_async.yaml'
            ),
            {"use_sim_time": use_sim_time}
        ],
        remappings=[
            ("/scan", "/ldlidar_node/scan")
        ]
    )
    nav2_launch = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory('nav2_bringup'),
            'launch',
            'navigation_launch.py'
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            'params_file': os.path.join(
                get_package_share_directory('skid_steer_navigation'),
                'config',
                'nav2_params.yaml'
            )
        }.items()
    )
    #  Twist Mux
    twist_mux_config = os.path.join(
        get_package_share_directory('skid_steer_controller'),
        'config',
        'twist_mux.yaml'
    )
    
    twist_mux = Node(
        package='twist_mux',
        executable='twist_mux',
        name='twist_mux',
        parameters=[twist_mux_config, {"use_sim_time": use_sim_time}],
        remappings=[
            ('/cmd_vel_out', '/skid_steer_controller_V1/cmd_vel_unstamped')
        ],
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value='true'),
        gazebo,
        controller,
        ekf_localization_filter,
        twist_mux,
        nav2_launch,
        slam_toolbox
    ])