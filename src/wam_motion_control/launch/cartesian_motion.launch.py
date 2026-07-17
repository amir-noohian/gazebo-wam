from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

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

    config_file = LaunchConfiguration("config_file")
    plan_only = LaunchConfiguration("plan_only")
    use_sim_time = LaunchConfiguration("use_sim_time")

    config_file_arg = DeclareLaunchArgument(
        "config_file",
        default_value=os.path.join(
            get_package_share_directory("wam_motion_control"),
            "config",
            "cartesian_motion.yaml"
        ),
        description="YAML file containing the Cartesian target and settings"
    )

    plan_only_arg = DeclareLaunchArgument(
        "plan_only",
        default_value="false",
        description="Plan the motion without sending it to the controller"
    )

    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Use the Gazebo simulation clock"
    )

    xacro_file = os.path.join(
        get_package_share_directory("wam_description"),
        "robots",
        "wam7.urdf.xacro"
    )

    robot_description = {
        "robot_description":
        xacro.process_file(xacro_file).toxml()
    }

    srdf_file = os.path.join(
        get_package_share_directory("wam_moveit_config"),
        "config",
        "wam.srdf"
    )

    with open(srdf_file, "r") as file:
        robot_description_semantic = {
            "robot_description_semantic": file.read()
        }

    robot_description_kinematics = {
        "robot_description_kinematics": load_yaml(
            "wam_moveit_config",
            "config/kinematics.yaml"
        )
    }

    cartesian_motion_node = Node(
        package="wam_motion_control",
        executable="cartesian_motion",
        output="screen",
        parameters=[
            config_file,
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            {
                "plan_only": ParameterValue(plan_only, value_type=bool),
                "use_sim_time": ParameterValue(use_sim_time, value_type=bool)
            }
        ]
    )

    return LaunchDescription([
        config_file_arg,
        plan_only_arg,
        use_sim_time_arg,
        cartesian_motion_node
    ])
