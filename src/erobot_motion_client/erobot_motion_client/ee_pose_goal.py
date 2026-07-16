#!/usr/bin/env python3
import argparse
import math
import sys

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from builtin_interfaces.msg import Duration
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory
from moveit_msgs.srv import GetPositionIK
from moveit_msgs.msg import MoveItErrorCodes

from tf_transformations import quaternion_from_euler


JOINT_NAMES = [
    "shoulder_joint",   # J1
    "arm1_joint",       # J2
    "arm2_joint",       # J3
    "wrist1_joint",     # J4
    "wrist2_joint",     # J5
    "end_joint",        # J6
]


class EEPoseIKPTPClient(Node):
    def __init__(self, args):
        super().__init__("ee_pose_ik_ptp_client")

        self.args = args

        self.group_name = args.group
        self.base_frame = args.base
        self.tip_link = args.tip

        self.ik_service_name = args.ik_service
        self.controller_action = args.controller_action

        self.current_joint_state = None

        self.create_subscription(
            JointState,
            "/joint_states",
            self.joint_state_callback,
            10
        )

        self.ik_client = self.create_client(
            GetPositionIK,
            self.ik_service_name
        )

        self.traj_client = ActionClient(
            self,
            FollowJointTrajectory,
            self.controller_action
        )

    def joint_state_callback(self, msg):
        name_to_pos = dict(zip(msg.name, msg.position))

        for joint_name in JOINT_NAMES:
            if joint_name not in name_to_pos:
                return

        self.current_joint_state = msg

    def wait_for_joint_state(self):
        self.get_logger().info("等待 /joint_states...")

        while rclpy.ok() and self.current_joint_state is None:
            rclpy.spin_once(self, timeout_sec=0.1)

        name_to_pos = dict(
            zip(self.current_joint_state.name, self.current_joint_state.position)
        )

        current_deg = [
            math.degrees(name_to_pos[name])
            for name in JOINT_NAMES
        ]

        self.get_logger().info(
            "当前关节角 deg: " +
            str([round(v, 3) for v in current_deg])
        )

    def wait_for_ik_service(self):
        self.get_logger().info(f"等待 IK 服务: {self.ik_service_name}")

        if not self.ik_client.wait_for_service(timeout_sec=10.0):
            self.get_logger().error(
                f"没有找到 {self.ik_service_name}。请确认 move_group / MoveIt 已经启动。"
            )
            rclpy.shutdown()
            sys.exit(1)

        self.get_logger().info("IK 服务已连接。")

    def build_pose(self):
        q = quaternion_from_euler(
            self.args.roll,
            self.args.pitch,
            self.args.yaw
        )

        pose = PoseStamped()
        pose.header.frame_id = self.base_frame
        pose.header.stamp = self.get_clock().now().to_msg()

        pose.pose.position.x = float(self.args.x)
        pose.pose.position.y = float(self.args.y)
        pose.pose.position.z = float(self.args.z)

        pose.pose.orientation.x = float(q[0])
        pose.pose.orientation.y = float(q[1])
        pose.pose.orientation.z = float(q[2])
        pose.pose.orientation.w = float(q[3])

        return pose

    def compute_ik(self):
        self.wait_for_joint_state()
        self.wait_for_ik_service()

        pose = self.build_pose()

        self.get_logger().info("========== 输入的末端目标位姿 ==========")
        self.get_logger().info(f"frame = {self.base_frame}")
        self.get_logger().info(f"tip   = {self.tip_link}")
        self.get_logger().info(f"x = {self.args.x:.6f} m")
        self.get_logger().info(f"y = {self.args.y:.6f} m")
        self.get_logger().info(f"z = {self.args.z:.6f} m")
        self.get_logger().info(f"roll  = {self.args.roll:.6f} rad")
        self.get_logger().info(f"pitch = {self.args.pitch:.6f} rad")
        self.get_logger().info(f"yaw   = {self.args.yaw:.6f} rad")
        self.get_logger().info("=======================================")

        req = GetPositionIK.Request()

        req.ik_request.group_name = self.group_name
        req.ik_request.ik_link_name = self.tip_link
        req.ik_request.pose_stamped = pose

        # 使用当前关节角作为 IK 初值，非常重要
        req.ik_request.robot_state.joint_state = self.current_joint_state

        # 是否避障
        req.ik_request.avoid_collisions = bool(self.args.avoid_collisions)

        # IK 求解超时时间
        req.ik_request.timeout = Duration(sec=1, nanosec=0)

        self.get_logger().info("开始调用 MoveIt /compute_ik 反解...")

        future = self.ik_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is None:
            self.get_logger().error("IK 服务调用失败，没有返回结果。")
            rclpy.shutdown()
            sys.exit(1)

        res = future.result()

        if res.error_code.val != MoveItErrorCodes.SUCCESS:
            self.get_logger().error(
                f"IK 失败，MoveIt error_code = {res.error_code.val}"
            )
            self.get_logger().error(
                "常见原因：目标点不可达、姿态不可达、tip/base/group 名字不对、kinematics.yaml 没配置、发生碰撞。"
            )
            rclpy.shutdown()
            sys.exit(1)

        solution = res.solution.joint_state

        name_to_pos = dict(zip(solution.name, solution.position))

        target_positions = []

        for joint_name in JOINT_NAMES:
            if joint_name not in name_to_pos:
                self.get_logger().error(
                    f"IK 解里没有关节 {joint_name}，请检查 MoveIt group 是否包含这个关节。"
                )
                rclpy.shutdown()
                sys.exit(1)

            target_positions.append(name_to_pos[joint_name])

        self.get_logger().info("========== IK 反解得到的 6 个关节角 ==========")

        for name, q in zip(JOINT_NAMES, target_positions):
            self.get_logger().info(
                f"{name:20s}: {q:+.6f} rad, {math.degrees(q):+.3f} deg"
            )

        self.get_logger().info("=============================================")

        return target_positions

    def send_trajectory(self, target_positions):
        self.get_logger().info(f"等待控制器 action: {self.controller_action}")

        if not self.traj_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error(
                f"没有找到 {self.controller_action}。请确认 arm_controller 已 active。"
            )
            rclpy.shutdown()
            sys.exit(1)

        traj = JointTrajectory()
        traj.joint_names = JOINT_NAMES

        point = JointTrajectoryPoint()
        point.positions = target_positions
        point.velocities = [0.0] * len(JOINT_NAMES)

        duration_sec = float(self.args.duration)
        point.time_from_start.sec = int(duration_sec)
        point.time_from_start.nanosec = int(
            (duration_sec - int(duration_sec)) * 1e9
        )

        traj.points.append(point)

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = traj

        self.get_logger().warn("准备下发 IK 结果到真实机器人，请确认周围安全。")
        self.get_logger().info(f"运动时间 duration = {duration_sec:.3f} s")

        send_future = self.traj_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future)

        goal_handle = send_future.result()

        if not goal_handle.accepted:
            self.get_logger().error("轨迹目标被控制器拒绝。")
            rclpy.shutdown()
            sys.exit(1)

        self.get_logger().info("轨迹目标已接受，等待执行完成...")

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)

        result = result_future.result().result
        self.get_logger().info(f"轨迹执行完成，error_code = {result.error_code}")


def parse_args():
    parser = argparse.ArgumentParser(
        description="End-effector pose IK + PTP controller for erobot"
    )

    parser.add_argument("--group", type=str, default="arm")
    parser.add_argument("--base", type=str, default="base_link")
    parser.add_argument("--tip", type=str, default="end_Link")

    parser.add_argument("--ik-service", type=str, default="/compute_ik")
    parser.add_argument(
        "--controller-action",
        type=str,
        default="/arm_controller/follow_joint_trajectory"
    )

    parser.add_argument("--x", type=float, required=True)
    parser.add_argument("--y", type=float, required=True)
    parser.add_argument("--z", type=float, required=True)

    parser.add_argument("--roll", type=float, default=0.0)
    parser.add_argument("--pitch", type=float, default=0.0)
    parser.add_argument("--yaw", type=float, default=0.0)

    parser.add_argument(
        "--avoid-collisions",
        type=int,
        default=0,
        help="0: 不检查碰撞，1: 检查碰撞"
    )

    parser.add_argument(
        "--execute",
        type=int,
        default=0,
        help="0: 只反解打印关节角；1: 反解后下发给 arm_controller"
    )

    parser.add_argument(
        "--duration",
        type=float,
        default=30,
        help="PTP 运动时间，单位秒"
    )

    args, _ = parser.parse_known_args()
    return args


def main():
    args = parse_args()

    rclpy.init()

    node = EEPoseIKPTPClient(args)

    target_positions = node.compute_ik()

    if args.execute == 1:
        node.send_trajectory(target_positions)
    else:
        node.get_logger().info("当前 execute=0，只完成 IK 反解，没有下发给真实机器人。")
        node.get_logger().info("确认关节角合理后，把 --execute 0 改成 --execute 1。")

    rclpy.shutdown()


if __name__ == "__main__":
    main()
