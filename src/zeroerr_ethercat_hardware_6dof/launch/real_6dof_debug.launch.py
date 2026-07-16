from pathlib import Path

from ament_index_python.packages import (
    get_package_share_directory,
)
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    description_share = Path(
        get_package_share_directory(
            "erobot_urdf_description"
        )
    )

    hardware_share = Path(
        get_package_share_directory(
            "zeroerr_ethercat_hardware_6dof"
        )
    )

    urdf_path = (
        description_share
        / "urdf"
        / "robot_real.urdf"
    )

    controller_path = (
        hardware_share
        / "config"
        / "controllers_6dof.yaml"
    )

    robot_description = {
        "robot_description":
            urdf_path.read_text(
                encoding="utf-8"
            )
    }

    return LaunchDescription([
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="screen",
            parameters=[
                robot_description,
            ],
        ),

        Node(
            package="controller_manager",
            executable="ros2_control_node",
            output="screen",
            emulate_tty=True,
            parameters=[
                robot_description,
                str(controller_path),
            ],
            arguments=[
                "--ros-args",
                "--log-level",
                "info",
            ],
        ),
    ])
