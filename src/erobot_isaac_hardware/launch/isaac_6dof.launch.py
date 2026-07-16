#!/usr/bin/env python3

from pathlib import Path

from ament_index_python.packages import (
    get_package_share_directory,
)

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    TimerAction,
)
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # ============================================================
    # 1. 统一时间源
    # ============================================================
    use_sim_time = LaunchConfiguration("use_sim_time")

    use_sim_time_value = ParameterValue(
        use_sim_time,
        value_type=bool,
    )

    # ============================================================
    # 2. 软件包路径
    # ============================================================
    description_share = Path(
        get_package_share_directory(
            "erobot_urdf_description"
        )
    )

    isaac_hardware_share = Path(
        get_package_share_directory(
            "erobot_isaac_hardware"
        )
    )

    # ============================================================
    # 3. URDF 与控制器参数文件
    # ============================================================
    urdf_path = (
        description_share
        / "urdf"
        / "robot_isaac.urdf"
    )

    controller_path = (
        isaac_hardware_share
        / "config"
        / "controllers_isaac.yaml"
    )

    robot_description = urdf_path.read_text(
        encoding="utf-8"
    )

    controller_path_string = str(controller_path)

    # ============================================================
    # 4. robot_state_publisher
    # ============================================================
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            {
                "robot_description": robot_description,
                "use_sim_time": use_sim_time_value,
            },
        ],
    )

    # ============================================================
    # 5. controller_manager
    #
    # 关键修复：
    # 提前声明 controller_name.params_file。
    #
    # 这样 Controller Manager 在创建控制器节点时，
    # 能直接把 controllers_isaac.yaml 传给控制器。
    # ============================================================
    controller_manager_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        output="screen",
        parameters=[
            controller_path_string,
            {
                "robot_description": robot_description,
                "use_sim_time": use_sim_time_value,

                # 关键：在 controller_manager 启动时预先声明
                "joint_state_broadcaster.params_file":
                    controller_path_string,

                "arm_controller.params_file":
                    controller_path_string,
            },
        ],
    )

    # ============================================================
    # 6. joint_state_broadcaster
    #
    # 不再使用 --param-file。
    # controller_manager 已提前保存对应 params_file。
    # ============================================================
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        name="spawner_joint_state_broadcaster",
        output="screen",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
            "--controller-manager-timeout",
            "60",
        ],
    )

    # ============================================================
    # 7. arm_controller
    #
    # 同样不再使用 --param-file。
    # ============================================================
    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        name="spawner_arm_controller",
        output="screen",
        arguments=[
            "arm_controller",
            "--controller-manager",
            "/controller_manager",
            "--controller-manager-timeout",
            "60",
        ],
    )

    # 给 controller_manager 留出初始化硬件的时间
    delayed_controller_spawners = TimerAction(
        period=2.0,
        actions=[
            joint_state_broadcaster_spawner,
            arm_controller_spawner,
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
            description="Use Isaac Sim /clock",
        ),

        robot_state_publisher_node,
        controller_manager_node,
        delayed_controller_spawners,
    ])
