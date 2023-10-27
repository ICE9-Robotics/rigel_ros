from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

import os

def generate_launch_description():
    rviz_dir = os.path.join(get_package_share_directory('rigel_ros'), 'rviz', 'rigel.rviz')
    rviz_node = Node(
            package='rviz2',
            namespace='c16',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_dir],
            output='screen')

    return LaunchDescription([rviz_node])