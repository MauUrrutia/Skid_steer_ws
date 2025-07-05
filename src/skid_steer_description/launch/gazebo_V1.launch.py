from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.parameter_descriptions import ParameterValue
import os
from pathlib import Path
from launch.substitutions import Command, LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    use_sim_time_arg = DeclareLaunchArgument(
    "use_sim_time",
    default_value="true",
    description="Use simulation time"
    )
    skid_steer_share_dir = get_package_share_directory('skid_steer_description')
    ros_distro = os.environ["ROS_DISTRO"]

    is_gazebo_ign = "true" if ros_distro == "humble" else "false"

    model_arg = DeclareLaunchArgument(name="model", default_value=os.path.join(skid_steer_share_dir, 'urdf', 'skid_steer_V2.urdf.xacro'),
                                       description="Path to robot urdf file")
    urdf_file = ParameterValue(Command([
                                    "xacro ", 
                                    LaunchConfiguration("model"), 
                                    " is_gazebo_ign:=",
                                    is_gazebo_ign,
                                    " is_sim:=",
                                    LaunchConfiguration("use_sim_time")]),
                                value_type=str)

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[
            {'robot_description': urdf_file}, {'use_sim_time' : use_sim_time}
        ]
    )

    gazebo_resource_path = SetEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH",
        value=[
            str(Path(skid_steer_share_dir).parent.resolve())
            ]
    )
    gazebo_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory("ros_gz_sim"), "launch"), "/gz_sim.launch.py"]),
        #launch_arguments=[
        #    ("gz_args", [" -v 4", " -r", " empty.sdf"])
        #]
        launch_arguments=[
            ("gz_args", [" -v 4", " -r", " /home/mau/skid_steer_ws/src/skid_steer_description/worlds/world01.sdf"])
        ]
    )

    gazebo_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        output ="screen", 
        parameters=[
            {'use_sim_time' : use_sim_time}
        ],
        arguments=["-topic", "robot_description",
                   "-name", "skid_steer",
                   "-x", "0.0",                         # Posición en el eje X
                   "-y", "0.0",                         # Posición en el eje Y
                   "-z", "0"                        # Altura inicial en Z
                #    "-x", "0.0",                         # Posición en el eje X
                #    "-y", "-5.0",                         # Posición en el eje Y
                #     "-z", "2.5"                        # Altura inicial en Z
        ]
    )

    gz_ros2_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        parameters=[
            {'use_sim_time' : use_sim_time}
        ],
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            "/imu@sensor_msgs/msg/Imu@gz.msgs.IMU",
            "/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan"
        ],
        remappings=[
            ("/imu", "imu/out"),
            ("/scan", "/ldlidar_node/scan")
        ]
    )

    rviz2_node=Node(
        package="rviz2",
        executable="rviz2",

    )

    return LaunchDescription([
        use_sim_time_arg,
        model_arg,
        gazebo_resource_path,
        robot_state_publisher_node,
        gazebo_server,
        gazebo_spawn_entity,
        gz_ros2_bridge,
        rviz2_node
    ])