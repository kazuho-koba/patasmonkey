#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import ament_index_python.packages


def generate_launch_description():
    # get the path of patasmonkey_teleop package's launch file
    teleop_launch_file = os.path.join(
        ament_index_python.packages.get_package_share_directory("patasmonkey_teleop"),
        "launch",
        "joy_teleop.launch.py",
    )
    # get the path of patasmonkey_vehicle_interface package's launch file
    vehicle_launch_file = os.path.join(
        ament_index_python.packages.get_package_share_directory(
            "patasmonkey_vehicle_interface"
        ),
        "launch",
        "vehicle_interface.launch.py",
    )

    teleop_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(teleop_launch_file)
    )
    vehicle_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(vehicle_launch_file)
    )

    return LaunchDescription([
        teleop_launch,
        vehicle_launch,
    ])
