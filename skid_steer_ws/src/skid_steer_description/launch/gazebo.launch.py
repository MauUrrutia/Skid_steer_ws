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
    skid_steer_share_dir = get_package_share_directory('skid_steer_description')
    ros_distro = os.environ["ROS_DISTRO"]

    is_gazebo_ign = "True" if ros_distro == "humble" else "False"

    model_arg = DeclareLaunchArgument(name="model", default_value=os.path.join(skid_steer_share_dir, 'urdf', 'skid_steer.urdf.xacro'),
                                       description="Path to robot urdf file")
    urdf_file = ParameterValue(Command([
                                    "xacro ", 
                                    LaunchConfiguration("model"),
                                    " is_gazebo_ign:=",
                                    is_gazebo_ign]),
                                value_type=str)

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[
            {'robot_description': urdf_file}, {'use_sim_time' : True}
        ]
    )

    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{'use_sim_time' : True}]
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
        launch_arguments=[
            ("gz_args", [" -v 4", " -r", " empty.sdf"])
        ]
    )

    gazebo_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        output ="screen", 
        arguments=["-topic", "robot_description",
                   "-name", "skid_steer"],
        parameters=[{'use_sim_time' : True}]
    )

    urdf_spawn_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'skid_steer',
            '-topic', 'robot_description'
        ],
        output='screen',
        parameters=[{'use_sim_time' : True}]
    )

    rviz2_node=Node(
        package="rviz2",
        executable="rviz2",
        parameters=[{'use_sim_time' : True}]
    )

    return LaunchDescription([
        model_arg,
        gazebo_resource_path,
        robot_state_publisher_node,
        joint_state_publisher_node,
        gazebo_server,
        gazebo_spawn_entity,
        urdf_spawn_node,
        rviz2_node
    ])
