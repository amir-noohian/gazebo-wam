from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory

import os
import xacro


def generate_launch_description():

    # ---------------------------------------------------------
    # Launch arguments
    # ---------------------------------------------------------
    gui_arg = DeclareLaunchArgument(
        "gui",
        default_value="true",
        description="Start Gazebo with the graphical interface"
    )

    gui = LaunchConfiguration("gui")

    # ---------------------------------------------------------
    # Generate WAM URDF from Xacro
    # ---------------------------------------------------------
    xacro_file = os.path.join(
        get_package_share_directory("wam_description"),
        "robots",
        "wam7.urdf.xacro"
    )

    robot_description_content = xacro.process_file(
        xacro_file
    ).toxml()

    robot_description = {
        "robot_description": robot_description_content
    }

    # ---------------------------------------------------------
    # Start Gazebo
    # ---------------------------------------------------------
    gazebo_launch_file = os.path.join(
        get_package_share_directory("gazebo_ros"),
        "launch",
        "gazebo.launch.py"
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            gazebo_launch_file
        ),
        launch_arguments={
            "gui": gui,
            "pause": "true",
        }.items()
    )

    # ---------------------------------------------------------
    # Robot state publisher
    # ---------------------------------------------------------
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            robot_description,
            {
                "use_sim_time": True
            }
        ]
    )

    controller_switcher = Node(
        package="wam_gazebo",
        executable="controller_switcher",
        name="wam_controller_switcher",
        output="screen",
    )

    # ---------------------------------------------------------
    # Spawn the WAM in Gazebo
    # ---------------------------------------------------------
    spawn_wam = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        name="spawn_wam",
        output="screen",
        arguments=[
            "-entity",
            "wam",
            "-topic",
            "robot_description",
            "-x",
            "0.0",
            "-y",
            "0.0",
            "-z",
            "0.0"
        ]
    )

    # ---------------------------------------------------------
    # Load both arm controllers, then start joint-space control while Cartesian
    # control remains inactive for a later explicit switch.
    # ---------------------------------------------------------
    start_controllers = ExecuteProcess(
        cmd=[
            "bash",
            "-c",
            (
                "set -e; "

                "echo 'Waiting for controller_manager...'; "

                "until ros2 service call "
                "/controller_manager/list_controllers "
                "controller_manager_msgs/srv/ListControllers "
                "'{}' >/dev/null 2>&1; "
                "do "
                "  sleep 0.2; "
                "done; "

                "echo 'Loading joint_state_broadcaster'; "

                "ros2 run controller_manager spawner.py "
                "joint_state_broadcaster "
                "--controller-manager /controller_manager "
                "--stopped; "

                "echo 'Loading joint-space model controller'; "

                "ros2 run controller_manager spawner.py "
                "wam_model_controller "
                "--controller-manager /controller_manager "
                "--stopped; "

                "echo 'Loading Cartesian controller'; "

                "ros2 run controller_manager spawner.py "
                "wam_cartesian_controller "
                "--controller-manager /controller_manager "
                "--stopped; "

                "echo 'Activating controllers'; "

                "ros2 service call "
                "/controller_manager/switch_controller "
                "controller_manager_msgs/srv/SwitchController "
                "\"{"
                "start_controllers: "
                "[joint_state_broadcaster, wam_model_controller], "
                "stop_controllers: [], "
                "strictness: 2, "
                "start_asap: true, "
                "timeout: {sec: 0, nanosec: 0}"
                "}\" & "
                "switch_pid=$!; "

                "echo 'Unpausing Gazebo'; "

                "ros2 service call "
                "/unpause_physics "
                "std_srvs/srv/Empty "
                "'{}' >/dev/null; "

                "wait ${switch_pid}; "

                "echo 'Controller states:'; "

                "ros2 control list_controllers "
                "--controller-manager /controller_manager"
            )
        ],
        output="screen"
    )

    # Start controller setup after Gazebo finishes spawning the robot
    start_controllers_after_spawn = RegisterEventHandler(
        OnProcessExit(
            target_action=spawn_wam,
            on_exit=[
                start_controllers
            ]
        )
    )

    return LaunchDescription([
        gui_arg,
        gazebo,
        robot_state_publisher,
        controller_switcher,
        spawn_wam,
        start_controllers_after_spawn,
    ])
