import os
import subprocess
import stat

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def setup_can(package_dir):
    setup_can_script = os.path.join(package_dir, 'scripts', 'setup_can.sh')
    st = os.stat(setup_can_script)
    os.chmod(setup_can_script, st.st_mode | stat.S_IEXEC)
    try:
        subprocess.run(setup_can_script, shell=True).check_returncode()
    except subprocess.CalledProcessError:
        print("Error: Fail to setup CAN interface.")
        exit()

def generate_launch_description():
    package_dir = get_package_share_directory('rigel_ros')

    setup_can(package_dir)

    scout_dir = get_package_share_directory('scout_base')
    scout_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                    scout_dir, 'launch','scout_mini_base.launch.py')))

    velodyne_launch_dir = get_package_share_directory('velodyne')
    velodyne_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                    velodyne_launch_dir, 'launch','velodyne-all-nodes-VLP16-launch.py')))
    
    lslidar_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                    package_dir, 'launch', 'include', 'lslidar_c16_launch.py')))
    
    realsense_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                    package_dir, 'launch', 'include', 'realsense_d435i_launch.py')))
    
    rigel_description_dir = get_package_share_directory('rigel_description')
    rigel_description_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(rigel_description_dir,'launch','rigel_description_launch.py')))

    rviz_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                    package_dir, 'launch', 'include', 'rviz_launch.py')))

    

    return LaunchDescription([
        scout_launch,
        velodyne_launch,
        lslidar_launch,
        realsense_launch,
        rigel_description_launch,
        rviz_launch
    ])
