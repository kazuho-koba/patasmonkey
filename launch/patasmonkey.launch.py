from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Get package paths
    teleop_pkg = get_package_share_directory('patasmonkey_teleop')
    vehicle_interface_pkg = get_package_share_directory('patasmonkey_vehicle_interface')

    # Paths to existing launch files
    teleop_launch = os.path.join(teleop_pkg, 'launch', 'joy_teleop.launch.py')
    vehicle_interface_launch = os.path.join(vehicle_interface_pkg, 'launch', 'vehicle_interface.launch.py')

    return LaunchDescription([
        # Include joy_teleop.launch.py
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(teleop_launch)
        ),

        # Include vehicle_interface.launch.py
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(vehicle_interface_launch)
        )
    ])
