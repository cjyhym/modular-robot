from launch import LaunchDescription
from launch.substitutions import Command
from launch.substitutions import FindExecutable
from launch.substitutions import PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    package_share = FindPackageShare(
        "zeroerr_ethercat_hardware"
    )

    xacro_file = PathJoinSubstitution([
        package_share,
        "urdf",
        "single_joint_test.urdf.xacro",
    ])

    controllers_file = PathJoinSubstitution([
        package_share,
        "config",
        "controllers.yaml",
    ])

    robot_description = {
        "robot_description": Command([
            FindExecutable(name="xacro"),
            " ",
            xacro_file,
        ])
    }

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
    )

    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        output="screen",
        parameters=[
            controllers_file,
        ],
        remappings=[
            (
                "~/robot_description",
                "/robot_description",
            ),
        ],
    )

    return LaunchDescription([
        robot_state_publisher,
        controller_manager,
    ])
