from launch import LaunchDescription
from launch_ros.actions import Node

import os
import xacro
import yaml

from ament_index_python.packages import get_package_share_directory


def load_yaml(package_name, relative_path):
    path = os.path.join(
        get_package_share_directory(package_name),
        relative_path
    )

    with open(path, "r") as file:
        return yaml.safe_load(file)


def generate_launch_description():

    # ---------------------------------------------------------
    # Robot description: URDF
    # ---------------------------------------------------------
    description_path = os.path.join(
        get_package_share_directory("wam_description"),
        "robots",
        "wam7.urdf.xacro"
    )

    robot_description_content = xacro.process_file(
        description_path
    ).toxml()

    robot_description = {
        "robot_description": robot_description_content
    }

    # ---------------------------------------------------------
    # Semantic description: SRDF
    # ---------------------------------------------------------
    srdf_path = os.path.join(
        get_package_share_directory("wam_moveit_config"),
        "config",
        "wam.srdf"
    )

    with open(srdf_path, "r") as file:
        robot_description_semantic = {
            "robot_description_semantic": file.read()
        }

    # ---------------------------------------------------------
    # MoveIt configuration
    # ---------------------------------------------------------
    robot_description_kinematics = {
        "robot_description_kinematics": load_yaml(
            "wam_moveit_config",
            "config/kinematics.yaml"
        )
    }

    robot_description_planning = {
        "robot_description_planning": load_yaml(
            "wam_moveit_config",
            "config/joint_limits.yaml"
        )
    }

    ompl_planning_pipeline_config = {
        "planning_pipelines": ["ompl"],
        "default_planning_pipeline": "ompl",

        "ompl": {
            "planning_plugin": "ompl_interface/OMPLPlanner",

            "request_adapters":
                "default_planner_request_adapters/AddTimeParameterization "
                "default_planner_request_adapters/FixWorkspaceBounds "
                "default_planner_request_adapters/FixStartStateBounds "
                "default_planner_request_adapters/FixStartStateCollision "
                "default_planner_request_adapters/FixStartStatePathConstraints",

            "start_state_max_bounds_error": 0.1,

            **load_yaml(
                "wam_moveit_config",
                "config/ompl_planning.yaml"
            )
        }
    }

    moveit_controllers = load_yaml(
        "wam_moveit_config",
        "config/moveit_controllers.yaml"
    )

    trajectory_execution = load_yaml(
        "wam_moveit_config",
        "config/trajectory_execution.yaml"
    )

    move_group_config = load_yaml(
        "wam_moveit_config",
        "config/move_group.yaml"
    )

    # ---------------------------------------------------------
    # Robot state publisher
    # ---------------------------------------------------------

    robot_state_publisher = Node(
	    package="robot_state_publisher",
	    executable="robot_state_publisher",
	    output="screen",
	    parameters=[
		robot_description
	    ]
	)

    # Temporary joint-state publisher for RViz testing
    joint_state_publisher = Node(
	    package="joint_state_publisher",
	    executable="joint_state_publisher",
	    output="screen",
	    parameters=[
		robot_description
	    ]
	)

    # ---------------------------------------------------------
    # MoveIt move_group
    # ---------------------------------------------------------
    move_group = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            robot_description_planning,
            ompl_planning_pipeline_config,
            moveit_controllers,
            trajectory_execution,
            move_group_config
        ]
    )

    # ---------------------------------------------------------
    # RViz
    # ---------------------------------------------------------


    rviz_config = os.path.join(
    get_package_share_directory("wam_moveit_config"),
    "rviz",
    "wam.rviz"
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        output="screen",
        arguments=["-d", rviz_config],
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            robot_description_planning
        ]
    )

    return LaunchDescription([
        # robot_state_publisher,
        # joint_state_publisher,
        move_group,
        rviz
    ])
