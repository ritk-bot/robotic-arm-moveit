from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():

    pkg_arm = get_package_share_directory("arm")

    urdf_file = os.path.join(
        pkg_arm,
        "urdf",
        "arm.urdf"
    )

    return LaunchDescription([

        ExecuteProcess(
    cmd=["gz", "sim", "-r", "empty.sdf"],
    output="screen"
        ),   

        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            arguments=[urdf_file],
            output="screen"
        ),

        Node(
            package="ros_gz_sim",
            executable="create",
            arguments=[
                "-file",
                urdf_file,
                "-name",
                "custom_arm"
            ],
            output="screen"
        ),

        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["joint_state_broadcaster"],
            output="screen"
        ),

        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["arm_controller"],
            output="screen"
        )
    ])