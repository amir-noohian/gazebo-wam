from launch import LaunchDescription
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory

import os
import xacro
import yaml


def load_yaml(package_name, relative_path):
    package_path = get_package_share_directory(package_name)
    yaml_path = os.path.join(package_path, relative_path)

    with open(yaml_path, "r") as file:
        return yaml.safe_load(file)


def generate_launch_description():

    # ---------------------------------------------------------
    # URDF
    # ---------------------------------------------------------
    xacro_file = os.path.join(
        get_package_share_directory("wam_description"),
        "robots",
        "wam7.urdf.xacro"
    )

    robot_description = {
        "robot_description":
        xacro.process_file(xacro_file).toxml()
    }

    # ---------------------------------------------------------
    # SRDF
    # ---------------------------------------------------------
    srdf_file = os.path.join(
        get_package_share_directory("wam_moveit_config"),
        "config",
        "wam.srdf"
    )

    with open(srdf_file, "r") as file:
        robot_description_semantic = {
            "robot_description_semantic": file.read()
        }

    # ---------------------------------------------------------
    # MoveIt kinematics configuration
    # ---------------------------------------------------------
    kinematics_yaml = load_yaml(
        "wam_moveit_config",
        "config/kinematics.yaml"
    )

    robot_description_kinematics = {
        "robot_description_kinematics": kinematics_yaml
    }

    # ---------------------------------------------------------
    # Joint motion node
    # ---------------------------------------------------------
    joint_motion_node = Node(
        package="wam_motion_control",
        executable="joint_motion",
        name="wam_joint_motion",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            {
                "use_sim_time": True
            }
        ]
    )

    return LaunchDescription([
        joint_motion_node
    ])