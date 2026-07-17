import os

from launch import LaunchDescription
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory

import xacro
import yaml


def load_yaml(package_name, relative_path):
    package_path = get_package_share_directory(package_name)
    absolute_path = os.path.join(package_path, relative_path)

    with open(absolute_path, "r") as file:
        return yaml.safe_load(file)


def generate_launch_description():

    # ---------------------------------------------------------
    # Robot description: URDF/Xacro
    # ---------------------------------------------------------
    xacro_file = os.path.join(
        get_package_share_directory("wam_description"),
        "robots",
        "wam7.urdf.xacro"
    )

    robot_description_content = xacro.process_file(xacro_file).toxml()

    robot_description = {
        "robot_description": robot_description_content
    }

    # ---------------------------------------------------------
    # Semantic robot description: SRDF
    # ---------------------------------------------------------
    srdf_file = os.path.join(
        get_package_share_directory("wam_moveit_config"),
        "config",
        "wam.srdf"
    )

    with open(srdf_file, "r") as file:
        robot_description_semantic_content = file.read()

    robot_description_semantic = {
        "robot_description_semantic":
            robot_description_semantic_content
    }

    # ---------------------------------------------------------
    # Kinematics configuration
    # ---------------------------------------------------------
    kinematics_yaml = load_yaml(
        "wam_moveit_config",
        "config/kinematics.yaml"
    )

    robot_description_kinematics = {
        "robot_description_kinematics": kinematics_yaml
    }

    # ---------------------------------------------------------
    # Start move_home
    # ---------------------------------------------------------
    move_home_node = Node(
        package="wam_pick_place",
        executable="move_home",
        name="move_home",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
        ],
    )

    return LaunchDescription([
        move_home_node
    ])