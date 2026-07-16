#!/usr/bin/env python3

from pathlib import Path

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # ============================================================
    # 1. 仿真/实机统一时间源参数
    #
    # false：真实 EtherCAT 机械臂，使用 Linux/ROS 系统时间
    # true ：仿真模式，使用模拟器发布的 /clock
    #
    # 注意：
    # 本文件加载的是 robot_real.urdf，因此仍属于实机启动文件。
    # Isaac Sim 后续应使用单独的 isaac_6dof.launch.py。
    # ============================================================
    use_sim_time = LaunchConfiguration("use_sim_time")
    start_rviz = LaunchConfiguration("start_rviz")

    # 将命令行中的 "true"/"false" 明确转换成布尔参数
    use_sim_time_value = ParameterValue(
        use_sim_time,
        value_type=bool,
    )

    # ============================================================
    # 2. 获取已有软件包路径
    # ============================================================
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

    # ============================================================
    # 3. 原有 URDF 和控制器配置
    # ============================================================
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

    robot_description = urdf_path.read_text(
        encoding="utf-8"
    )

    # ============================================================
    # 4. robot_state_publisher
    #
    # 根据 /joint_states 计算并发布机器人 TF。
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
    # 5. ros2_control controller_manager
    #
    # controllers_6dof.yaml 保持原样。
    # 统一时间参数放在参数列表最后，避免被 YAML 中的同名参数覆盖。
    # ============================================================
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        name="controller_manager",
        output="screen",
        parameters=[
            str(controller_path),
            {
                "robot_description": robot_description,
                "use_sim_time": use_sim_time_value,
            },
        ],
    )

    # ============================================================
    # 6. 普通 RViz
    #
    # 当 MoveIt 启动文件已经启动 MoveIt RViz 时，
    # 可以使用 start_rviz:=false 关闭这里的普通 RViz。
    # ============================================================
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        condition=IfCondition(start_rviz),
        parameters=[
            {
                "use_sim_time": use_sim_time_value,
            },
        ],
    )

    # ============================================================
    # 7. Launch 参数与节点
    # ============================================================
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description=(
                    "false: EtherCAT real robot system time; "
                    "true: simulator /clock"
                ),
            ),

            DeclareLaunchArgument(
                "start_rviz",
                default_value="true",
                description=(
                    "Start the RViz instance contained "
                    "in real_6dof.launch.py"
                ),
            ),

            robot_state_publisher_node,
            control_node,
            rviz_node,
        ]
    )
