#!/usr/bin/env python3
import math
import rclpy

from rclpy.node import Node
from rclpy.action import ActionClient

from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory


class PTPJointClient(Node):
    def __init__(self):
        super().__init__("ptp_joint_client")

        # ============================================================
        # 控制器 action 名称
        # 如果你的 action list 显示不是这个名字，需要改这里
        # ros2 action list | grep follow_joint_trajectory
        # ============================================================
        self.controller_action = "/arm_controller/follow_joint_trajectory"

        # ============================================================
        # 根据你的 URDF 设置真实关节名
        # 顺序必须对应 J1 J2 J3 J4 J5 J6
        # ============================================================
        self.joint_names = [
            "shoulder_joint",   # J1
            "arm1_joint",       # J2
            "arm2_joint",       # J3
            "wrist1_joint",     # J4
            "wrist2_joint",     # J5
            "end_joint",        # J6
        ]

        # URDF 中关节限位为 -3.14 到 3.14 rad
        self.lower_limits = [-3.14] * 6
        self.upper_limits = [3.14] * 6

        # URDF 中 velocity="0.1"，单位 rad/s
        self.max_velocity = 0.3

        self.current_positions = None

        self.joint_state_sub = self.create_subscription(
            JointState,
            "/joint_states",
            self.joint_state_callback,
            10
        )

        self.client = ActionClient(
            self,
            FollowJointTrajectory,
            self.controller_action
        )

    def joint_state_callback(self, msg: JointState):
        name_to_pos = dict(zip(msg.name, msg.position))

        positions = []
        for joint_name in self.joint_names:
            if joint_name not in name_to_pos:
                return

            positions.append(name_to_pos[joint_name])

        self.current_positions = positions

    def wait_for_current_state(self):
        self.get_logger().info("等待 /joint_states 当前关节角...")

        while rclpy.ok() and self.current_positions is None:
            rclpy.spin_once(self, timeout_sec=0.1)

        self.get_logger().info(
            "当前关节角 rad: " +
            str([round(v, 6) for v in self.current_positions])
        )

        self.get_logger().info(
            "当前关节角 deg: " +
            str([round(math.degrees(v), 3) for v in self.current_positions])
        )

    def send_absolute_joint_goal(self, target_deg, duration_sec=None):
        """
        发送绝对关节角目标。

        target_deg:
            绝对关节角，单位 deg。
            不是相对当前角度的增量。
            例如 [0, -10, 20, 0, 10, 0]
            表示每个关节运动到相对零位的这个角度。

        duration_sec:
            运动时间，单位秒。
            如果设为 None，则根据当前角度和最大速度自动估算。
        """

        if len(target_deg) != len(self.joint_names):
            raise ValueError("target_deg 数量必须等于 6")

        self.wait_for_current_state()

        # ============================================================
        # 核心修改：
        # 这里不再用 current + delta
        # 而是直接把目标绝对角度转换为 rad
        # ============================================================
        target_positions = [math.radians(v) for v in target_deg]

        # 关节限位检查
        for i, q in enumerate(target_positions):
            if q < self.lower_limits[i] or q > self.upper_limits[i]:
                self.get_logger().error(
                    f"{self.joint_names[i]} 目标角度超限: "
                    f"{q:.4f} rad = {math.degrees(q):.2f} deg"
                )
                rclpy.shutdown()
                return

        # 根据当前位置和目标位置计算最大运动距离
        max_delta = max(
            abs(target - current)
            for target, current in zip(target_positions, self.current_positions)
        )

        if duration_sec is None:
            duration_sec = max(max_delta / self.max_velocity * 1.5, 3.0)

        goal_msg = FollowJointTrajectory.Goal()

        traj = JointTrajectory()
        traj.joint_names = self.joint_names

        point = JointTrajectoryPoint()
        point.positions = target_positions

        # 到达目标点时速度为 0
        point.velocities = [0.0] * len(self.joint_names)

        point.time_from_start.sec = int(duration_sec)
        point.time_from_start.nanosec = int(
            (duration_sec - int(duration_sec)) * 1e9
        )

        traj.points.append(point)
        goal_msg.trajectory = traj

        self.get_logger().info(f"等待控制器 action: {self.controller_action}")
        self.client.wait_for_server()

        self.get_logger().info(
            "发送绝对目标角度 deg: " +
            str([round(v, 3) for v in target_deg])
        )

        self.get_logger().info(
            "发送绝对目标角度 rad: " +
            str([round(v, 6) for v in target_positions])
        )

        self.get_logger().info(f"运动时间: {duration_sec:.2f} s")

        future = self.client.send_goal_async(goal_msg)
        future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().error("轨迹目标被控制器拒绝")
            rclpy.shutdown()
            return

        self.get_logger().info("轨迹目标已接受，开始执行...")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)

    def result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f"轨迹执行完成，error_code = {result.error_code}")
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)

    node = PTPJointClient()

    # ============================================================
    # 这里填写绝对关节角，单位 deg
    #
    # 顺序：
    # [J1, J2, J3, J4, J5, J6]
    #
    # 对应：
    # [
    #   shoulder_joint,
    #   arm1_joint,
    #   arm2_joint,
    #   wrist1_joint,
    #   wrist2_joint,
    #   end_joint
    # ]
    # ============================================================

    target_deg = [
        -10,     # J1 shoulder_joint
        -30,   # J2 arm1_joint
        -15,    # J3 arm2_joint
        0.0,     # J4 wrist1_joint
        0,    # J5 wrist2_joint
        0.0,     # J6 end_joint
    ]

    # 第一次真机测试建议手动给大一点时间，慢慢走
    duration_sec =15

    node.send_absolute_joint_goal(
        target_deg=target_deg,
        duration_sec=duration_sec
    )

    rclpy.spin(node)


if __name__ == "__main__":
    main()
