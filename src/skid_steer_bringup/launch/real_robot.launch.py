import os
from ament_index_python import get_package_share_directory, get_package_share_path
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    urdf_path = os.path.join(
        get_package_share_path('skid_steer_description'),
        'urdf',
        'skid_steer_V2.urdf.xacro'
    )
    # robot_description = ParameterValue(
    #     Command(['xacro ', urdf_path]),
    #     value_type=str
    # )

    # robot_state_publisher_node = Node(
    #     package="robot_state_publisher",
    #     executable="robot_state_publisher",
    #     parameters=[{'robot_description': robot_description}, {"use_sim_time": use_sim_time}],
    # )

    #  Hardware interface
    hardware_interface = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("skid_steer_firmware"),
            "launch",
            "hardware_interface.launch.py"
        )
    )
    
    #  Lidar
    ldlidar_launch = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("ldlidar_node"),
            "launch",
            "ldlidar_with_mgr.launch.py"
        ),
    )

    #  Controller
    controller = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("skid_steer_controller"),
            "launch",
            "controller_V1.launch.py"
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "use_simple_controller": "False",
            "use_python": "False"
        }.items()
    )

    #  EKF Localization
    ekf_localization_filter = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("skid_steer_localization"),
            "launch",
            "local_localization.launch.py"
        ),
        launch_arguments={
                        "use_sim_time": use_sim_time,
                        }.items()
    )
    
    #  IMU Driver
    imu_driver = Node(
        package='imu_driver',
        executable='mpu6050_driver',
        name='mpu6050_driver_node',
        output='screen',
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

    # RealSense D415
    realsense_node = Node(
        package='realsense2_camera',
        executable='realsense2_camera_node',
        name='realsense2_camera_node',
        namespace='camera',
        parameters=[{
            'camera_name': 'd415',
            'enable_color': True,
            'enable_depth': True,
            'enable_infra': False,
            'enable_infra1': False,
            'enable_infra2': False,
            'initial_reset': False,
            'rgb_camera.color_profile': '640x480x30',
            'rgb_camera.color_format': 'RGB8',
            'depth_module.depth_profile': '640x480x30',
            'depth_module.depth_format': 'Z16',

            'align_depth.enable': False,
            'pointcloud.enable': True,
            'depth_module.enable_auto_exposure': True,
            'pointcloud.stream_filter': 2,  # RS2_STREAM_COLOR 
            'pointcloud.stream_index_filter': 0,
            'pointcloud.ordered_pc': False,

            'decimation_filter.enable': True,  
            'spatial_filter.enable': True,     
            'temporal_filter.enable': True,   
            'colorizer.enable': False,
            'accelerate_gpu_with_glsl': True,

            'publish_tf': True,
        }],
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
            ('/scan', '/ldlidar_node/scan')
        ]
    )
    # Nav2 (navigation_launch.py)
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

    lora_goal_receiver = Node(
        package='lora_node',
        executable='lora_goal_receiver',
        name='lora_goal_receiver',
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value='False'),
        realsense_node,
        lora_goal_receiver,   
        hardware_interface,
        controller,
        twist_mux,  
        ldlidar_launch,
        imu_driver,
        ekf_localization_filter,
        slam_toolbox,
        nav2_launch
    ])
