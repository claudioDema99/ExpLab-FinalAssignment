# combined_launch.py
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    return LaunchDescription([
        # Launch the first package's launch file
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('rosbot_description'), 
            'launch', 
            'rosbot_sim.launch.py'))
        ),

        # Launch the second package's launch file
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('plansys2_assignment_pkg'), 
            'launch', 
            'plansys2_assignment_launch.py'))
        ),
    ])
