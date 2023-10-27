import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import FindExecutable, PathJoinSubstitution, Command
from launch_ros.actions import Node


def generate_launch_description():
    package_dir = get_package_share_directory('rigel_description')

    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]), " ",
        PathJoinSubstitution(
            [package_dir, "urdf", "rigel.urdf.xacro"]
        ),
    ])
    print(robot_description_content)
    robot_state_pub_node = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'use_sim_time': False,
                'robot_description': robot_description_content
            }]),

    return LaunchDescription(
        robot_state_pub_node
    )
