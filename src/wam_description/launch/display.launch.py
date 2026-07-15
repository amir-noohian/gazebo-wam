from launch import LaunchDescription
from launch_ros.actions import Node

import os
import xacro

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    pkg = get_package_share_directory(
        "wam_description"
    )

    xacro_file = os.path.join(
        pkg,
        "robots",
        "wam7.urdf.xacro"
    )


    robot_description = {
        "robot_description":
        xacro.process_file(xacro_file).toxml()
    }


    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",

        parameters=[
            robot_description
        ]
    )


    joint_state_publisher_gui = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui"
    )


    rviz = Node(
        package="rviz2",
        executable="rviz2"
    )


    return LaunchDescription([
        robot_state_publisher,
        joint_state_publisher_gui,
        rviz
    ])
