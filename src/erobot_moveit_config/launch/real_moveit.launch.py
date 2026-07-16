#!/usr/bin/env python3

from pathlib import Path

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    # ============================================================
    # 1. 仿真/实机统一时间源
    #
    # true  : 使用 Isaac Sim 发布的 /clock
    # false : 使用 Linux 系统时间
    # ============================================================
    use_sim_time = LaunchConfiguration("use_sim_time")
    start_rviz = LaunchConfiguration("start_rviz")
    start_static_tf = LaunchConfiguration("start_static_tf")

    # 将命令行的 true/false 字符串明确转换为布尔参数
    use_sim_time_value = ParameterValue(
        use_sim_time,
        value_type=bool,
    )

    # ============================================================
    # 2. MoveIt 配置包路径
    # ============================================================
    package_share = Path(
        get_package_share_directory(
            "erobot_moveit_config"
        )
    )

    # ============================================================
    # 3. MoveIt 配置
    # ============================================================
    moveit_config = (
        MoveItConfigsBuilder(
            "erobot_urdf.SLDASM",
            package_name="erobot_moveit_config",
        )
        .robot_description(
            file_path="config/robot_real.urdf"
        )
        .robot_description_semantic(
            file_path=(
                "config/"
                "erobot_urdf.SLDASM.srdf"
            )
        )
        .robot_description_kinematics(
            file_path="config/kinematics.yaml"
        )
        .joint_limits(
            file_path="config/joint_limits.yaml"
        )
        .trajectory_execution(
            file_path=(
                "config/"
                "moveit_controllers.yaml"
            ),
            # 控制器由 ros2_control 启动和管理
            moveit_manage_controllers=False,
        )
        .planning_pipelines(
            default_planning_pipeline="ompl",
            pipelines=["ompl"],
            load_all=False,
        )
        .planning_scene_monitor(
            publish_planning_scene=True,
            publish_geometry_updates=True,
            publish_state_updates=True,
            publish_transforms_updates=True,
            publish_robot_description=True,
            publish_robot_description_semantic=True,
        )
        .to_moveit_configs()
    )

    # ============================================================
    # 4. MoveIt move_group
    # ============================================================
    move_group = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        name="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {
                "use_sim_time": use_sim_time_value,

                # 允许轨迹发送给 joint_trajectory_controller
                "allow_trajectory_execution": True,
            },
        ],
    )

    # ============================================================
    # 5. world -> base_link 静态 TF
    #
    # 如果其他启动文件已经发布该 TF，
    # 启动时使用 start_static_tf:=false。
    # ============================================================
    static_world_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="world_to_base_link",
        output="screen",
        condition=IfCondition(start_static_tf),
        arguments=[
            "--x", "0",
            "--y", "0",
            "--z", "0",
            "--roll", "0",
            "--pitch", "0",
            "--yaw", "0",
            "--frame-id", "world",
            "--child-frame-id", "base_link",
        ],
        parameters=[
            {
                "use_sim_time": use_sim_time_value,
            },
        ],
    )

    # ============================================================
    # 6. RViz 配置
    # ============================================================
    rviz_config = (
        package_share
        / "config"
        / "moveit.rviz"
    )

    rviz_arguments = []

    if rviz_config.exists():
        rviz_arguments = [
            "-d",
            str(rviz_config),
        ]

    moveit_rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="moveit_rviz",
        output="screen",
        condition=IfCondition(start_rviz),
        arguments=rviz_arguments,
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
            {
                "use_sim_time": use_sim_time_value,
            },
        ],
    )

    # ============================================================
    # 7. Launch 参数
    # ============================================================
    return LaunchDescription([
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description=(
                "true: use Isaac Sim /clock; "
                "false: use system clock"
            ),
        ),

        DeclareLaunchArgument(
            "start_rviz",
            default_value="true",
            description="Start MoveIt RViz",
        ),

        DeclareLaunchArgument(
            "start_static_tf",
            default_value="true",
            description=(
                "Publish static transform "
                "from world to base_link"
            ),
        ),

        static_world_tf,
        move_group,
        moveit_rviz,
    ])
