from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource

from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():

    sim_launch = os.path.join(
        get_package_share_directory("arm"),
        "launch",
        "sim.launch.py"
    )

    move_group_launch = os.path.join(
        get_package_share_directory("arm_moveit_config"),
        "launch",
        "move_group.launch.py"
    )

    rviz_launch = os.path.join(
        get_package_share_directory("arm_moveit_config"),
        "launch",
        "moveit_rviz.launch.py"
    )

    return LaunchDescription([

        # Gazebo + robot spawn + ros2_control + bridge
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                sim_launch
            )
        ),

        # Wait for Gazebo and controllers
        TimerAction(
            period=5.0,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        move_group_launch
                    )
                )
            ]
        ),

        # Wait for MoveGroup to come up
        TimerAction(
            period=7.0,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        rviz_launch
                    )
                )
            ]
        )

    ])