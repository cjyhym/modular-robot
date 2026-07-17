#!/usr/bin/env python3
import argparse
import copy
import csv
from datetime import datetime
import math
import os
import sys

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from builtin_interfaces.msg import Duration
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory
from moveit_msgs.srv import GetPositionIK, GetPositionFK
from moveit_msgs.msg import MoveItErrorCodes

from tf_transformations import quaternion_from_euler, euler_from_quaternion


JOINT_NAMES = [
    "shoulder_joint",   # J1
    "arm1_joint",       # J2
    "arm2_joint",       # J3
    "wrist1_joint",     # J4
    "wrist2_joint",     # J5
    "end_joint",        # J6
]

DEFAULT_STEP_SIZE = 0.005          # m
DEFAULT_MAX_CARTESIAN_VEL = 0.02   # m/s
DEFAULT_JOINT_VELOCITY = 0.1       # rad/s (10% scaling)
DEFAULT_MIN_DURATION = 3.0         # s


class LinearInterpolationClient(Node):
    def __init__(self, args):
        super().__init__("linear_interpolation_client")

        self.args = args

        self.group_name = args.group
        self.base_frame = args.base
        self.tip_link = args.tip

        self.ik_service_name = args.ik_service
        self.fk_service_name = args.fk_service
        self.controller_action = args.controller_action

        self.current_joint_state = None

        # 订阅 /joint_states 获取当前关节角
        self.create_subscription(
            JointState,
            "/joint_states",
            self.joint_state_callback,
            10
        )

        # FK 服务客户端
        self.fk_client = self.create_client(
            GetPositionFK,
            self.fk_service_name
        )

        # IK 服务客户端
        self.ik_client = self.create_client(
            GetPositionIK,
            self.ik_service_name
        )

        # 轨迹 Action 客户端
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
        self.get_logger().info("等待 /joint_states 当前关节角...")

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

    def wait_for_fk_service(self):
        self.get_logger().info(f"等待 FK 服务: {self.fk_service_name}")

        if not self.fk_client.wait_for_service(timeout_sec=10.0):
            self.get_logger().error(
                f"没有找到 {self.fk_service_name}。请确认 move_group / MoveIt 已经启动。"
            )
            rclpy.shutdown()
            sys.exit(1)

        self.get_logger().info("FK 服务已连接。")

    def wait_for_ik_service(self):
        self.get_logger().info(f"等待 IK 服务: {self.ik_service_name}")

        if not self.ik_client.wait_for_service(timeout_sec=10.0):
            self.get_logger().error(
                f"没有找到 {self.ik_service_name}。请确认 move_group / MoveIt 已经启动。"
            )
            rclpy.shutdown()
            sys.exit(1)

        self.get_logger().info("IK 服务已连接。")

    def compute_fk(self, joint_state):
        """
        调用 /compute_fk 获取当前末端位姿。
        返回 PoseStamped。
        """
        req = GetPositionFK.Request()
        req.header.frame_id = self.base_frame
        req.header.stamp = self.get_clock().now().to_msg()
        req.fk_link_names = [self.tip_link]
        req.robot_state.joint_state = joint_state

        future = self.fk_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is None:
            self.get_logger().error("FK 服务调用失败，没有返回结果。")
            rclpy.shutdown()
            sys.exit(1)

        res = future.result()

        if res.error_code.val != MoveItErrorCodes.SUCCESS:
            self.get_logger().error(
                f"FK 失败，MoveIt error_code = {res.error_code.val}"
            )
            rclpy.shutdown()
            sys.exit(1)

        return res.pose_stamped[0]

    def get_start_pose(self):
        """
        获取起始末端位姿。
        优先使用用户提供的 --start-x/y/z，否则通过 /compute_fk 获取。
        返回 (x, y, z, roll, pitch, yaw) 元组。
        """
        # 检查用户是否提供了完整的起始位置
        if (self.args.start_x is not None and
            self.args.start_y is not None and
            self.args.start_z is not None):

            self.get_logger().info("========== 起始末端位姿 (用户指定) ==========")
            x = float(self.args.start_x)
            y = float(self.args.start_y)
            z = float(self.args.start_z)
            roll = float(self.args.start_roll)
            pitch = float(self.args.start_pitch)
            yaw = float(self.args.start_yaw)

            self.get_logger().info(f"x = {x:.6f} m")
            self.get_logger().info(f"y = {y:.6f} m")
            self.get_logger().info(f"z = {z:.6f} m")
            self.get_logger().info(f"roll  = {roll:.6f} rad")
            self.get_logger().info(f"pitch = {pitch:.6f} rad")
            self.get_logger().info(f"yaw   = {yaw:.6f} rad")
            self.get_logger().info("=============================================")

            return (x, y, z, roll, pitch, yaw)

        # 默认：通过 FK 获取当前末端位姿
        self.wait_for_fk_service()
        pose_stamped = self.compute_fk(self.current_joint_state)

        x = pose_stamped.pose.position.x
        y = pose_stamped.pose.position.y
        z = pose_stamped.pose.position.z

        q = pose_stamped.pose.orientation
        roll, pitch, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])

        self.get_logger().info("========== 起始末端位姿 (FK) ==========")
        self.get_logger().info(f"x = {x:.6f} m")
        self.get_logger().info(f"y = {y:.6f} m")
        self.get_logger().info(f"z = {z:.6f} m")
        self.get_logger().info(f"roll  = {roll:.6f} rad")
        self.get_logger().info(f"pitch = {pitch:.6f} rad")
        self.get_logger().info(f"yaw   = {yaw:.6f} rad")
        self.get_logger().info("=======================================")

        return (x, y, z, roll, pitch, yaw)

    def compute_num_waypoints(self, start_xyz, target_xyz):
        """
        根据起止位置计算路径点数量 N。
        --step-size 优先于 --num-waypoints。
        如果都没指定，默认 step_size = 0.005 m。
        """
        dx = target_xyz[0] - start_xyz[0]
        dy = target_xyz[1] - start_xyz[1]
        dz = target_xyz[2] - start_xyz[2]
        distance = math.sqrt(dx*dx + dy*dy + dz*dz)

        if self.args.step_size is not None:
            step = self.args.step_size
            if step <= 0:
                self.get_logger().error("--step-size 必须大于 0")
                rclpy.shutdown()
                sys.exit(1)
            N = max(2, int(math.ceil(distance / step)) + 1)
        elif self.args.num_waypoints is not None:
            N = self.args.num_waypoints
            if N < 2:
                self.get_logger().warn(
                    f"--num-waypoints = {N}，至少需要 2 个路径点，已强制设为 2"
                )
                N = 2
        else:
            step = DEFAULT_STEP_SIZE
            N = max(2, int(math.ceil(distance / step)) + 1)

        if N > 100:
            self.get_logger().warn(
                f"路径点数量 {N} 较多，可能影响性能，建议增大 --step-size"
            )

        self.get_logger().info(
            f"直线距离 = {distance:.4f} m, 生成 {N} 个路径点"
        )

        return N

    def generate_waypoints(self, start_pose, target_pose, N):
        """
        在笛卡尔空间线性插值生成 N 个路径点。
        每个路径点是 (x, y, z, roll, pitch, yaw) 元组。
        """
        waypoints = []

        for i in range(N):
            alpha = i / (N - 1)  # 0.0 ~ 1.0

            x = start_pose[0] + alpha * (target_pose[0] - start_pose[0])
            y = start_pose[1] + alpha * (target_pose[1] - start_pose[1])
            z = start_pose[2] + alpha * (target_pose[2] - start_pose[2])
            roll = start_pose[3] + alpha * (target_pose[3] - start_pose[3])
            pitch = start_pose[4] + alpha * (target_pose[4] - start_pose[4])
            yaw = start_pose[5] + alpha * (target_pose[5] - start_pose[5])

            waypoints.append((x, y, z, roll, pitch, yaw))

        return waypoints

    def _call_ik_once(self, waypoint, seed_joint_state, timeout_sec=2.0):
        """
        单次 IK 调用，返回 (success, joint_positions_list_or_None)。
        """
        x, y, z, roll, pitch, yaw = waypoint

        q = quaternion_from_euler(roll, pitch, yaw)

        pose = PoseStamped()
        pose.header.frame_id = self.base_frame
        pose.header.stamp = self.get_clock().now().to_msg()

        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.position.z = float(z)

        pose.pose.orientation.x = float(q[0])
        pose.pose.orientation.y = float(q[1])
        pose.pose.orientation.z = float(q[2])
        pose.pose.orientation.w = float(q[3])

        req = GetPositionIK.Request()

        req.ik_request.group_name = self.group_name
        req.ik_request.ik_link_name = self.tip_link
        req.ik_request.pose_stamped = pose

        req.ik_request.robot_state.joint_state = seed_joint_state

        req.ik_request.avoid_collisions = bool(self.args.avoid_collisions)
        req.ik_request.timeout = Duration(sec=int(timeout_sec), nanosec=0)

        future = self.ik_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is None:
            return False, None

        res = future.result()

        if res.error_code.val != MoveItErrorCodes.SUCCESS:
            return False, None

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

        return True, target_positions

    def compute_ik_for_waypoint(self, waypoint, seed_joint_state):
        """
        对单个路径点调用 /compute_ik 反解关节角。
        waypoint: (x, y, z, roll, pitch, yaw)
        seed_joint_state: 用作 IK 初值的 JointState
        成功返回 6 个关节角 (rad) 的列表，失败返回 None。

        如果 IK 失败，会用微扰动种子重试（最多 3 次），
        以应对 KDL 求解器在奇异点附近的不稳定性。
        """
        import random

        MAX_RETRIES = 3
        IK_TIMEOUT = 2.0
        PERTURBATION_RAD = 0.1  # 每次重试时种子关节角扰动幅度 (rad)

        # 第一次尝试：用原始种子
        success, positions = self._call_ik_once(waypoint, seed_joint_state, IK_TIMEOUT)
        if success:
            return positions

        self.get_logger().warn(
            "IK 首次尝试失败，将用微扰动种子重试..."
        )

        # 重试：对种子关节角加小扰动
        for attempt in range(1, MAX_RETRIES + 1):
            perturbed_seed = copy.deepcopy(seed_joint_state)

            for j in range(len(perturbed_seed.position)):
                if perturbed_seed.name[j] in JOINT_NAMES:
                    perturbed_seed.position[j] += random.uniform(
                        -PERTURBATION_RAD * attempt,
                        PERTURBATION_RAD * attempt
                    )

            self.get_logger().info(
                f"IK 重试 {attempt}/{MAX_RETRIES} (扰动幅度 {PERTURBATION_RAD * attempt:.2f} rad)..."
            )

            success, positions = self._call_ik_once(waypoint, perturbed_seed, IK_TIMEOUT)
            if success:
                self.get_logger().info(f"IK 重试 {attempt} 成功！")
                return positions

        # 所有重试都失败
        self.get_logger().error(
            f"IK 失败：{MAX_RETRIES} 次重试后仍未找到解。"
            f"KDL 求解器可能在奇异点附近无法收敛。"
        )
        return None

    def _try_ik_quiet(self, waypoint, seed_joint_state):
        """
        静默版 IK 检查：尝试单次 IK + 最多 2 次扰动重试，不打印日志。
        成功返回 6 个关节角 (rad) 列表，失败返回 None。
        """
        import random
        PERTURBATION_RAD = 0.1

        success, positions = self._call_ik_once(waypoint, seed_joint_state, 2.0)
        if success:
            return positions

        for attempt in range(1, 3):
            perturbed = copy.deepcopy(seed_joint_state)
            for j in range(len(perturbed.position)):
                if perturbed.name[j] in JOINT_NAMES:
                    perturbed.position[j] += random.uniform(
                        -PERTURBATION_RAD * attempt,
                        PERTURBATION_RAD * attempt
                    )
            success, positions = self._call_ik_once(waypoint, perturbed, 2.0)
            if success:
                return positions

        return None

    def suggest_alternative_targets(self, current_pose, target_pose):
        """
        当规划无解时，生成一组候选目标位姿并逐个测试 IK 可达性。
        current_pose: (x, y, z, roll, pitch, yaw) 当前末端位姿
        target_pose:  (x, y, z, roll, pitch, yaw) 原始目标位姿

        候选策略：
          1. 沿当前→目标方向，在不同距离比例处采样（20% ~ 80%）
          2. 保持目标姿态不变
          3. 每个候选目标调用 IK 测试可达性
        """
        self.get_logger().info("")
        self.get_logger().info("=" * 60)
        self.get_logger().info("  规划无解 — 正在生成替代目标位姿推荐...")
        self.get_logger().info("=" * 60)

        # 计算方向向量
        dx = target_pose[0] - current_pose[0]
        dy = target_pose[1] - current_pose[1]
        dz = target_pose[2] - current_pose[2]
        distance = math.sqrt(dx*dx + dy*dy + dz*dz)

        if distance < 1e-6:
            self.get_logger().warn("当前位姿与目标位姿重合，无法生成替代目标。")
            return []

        # 沿方向在不同比例处生成候选
        fractions = [0.8, 0.6, 0.5, 0.4, 0.3, 0.2]  # 从远到近尝试
        candidates = []
        for frac in fractions:
            cand = (
                current_pose[0] + dx * frac,
                current_pose[1] + dy * frac,
                current_pose[2] + dz * frac,
                target_pose[3],  # 保持目标姿态
                target_pose[4],
                target_pose[5],
            )
            candidates.append((frac, cand))

        # 逐个测试
        solvable = []
        for frac, cand in candidates:
            # 用当前关节角做种子
            result = self._try_ik_quiet(cand, self.current_joint_state)
            if result is not None:
                solvable.append((frac, cand, result))

        # 打印结果
        self.get_logger().info("")
        if solvable:
            self.get_logger().info(
                f"找到 {len(solvable)} 个可达替代目标（按距离从近到远排列）:"
            )
            self.get_logger().info("-" * 60)
            for frac, cand, joints in solvable:
                self.get_logger().info(
                    f"  {frac*100:.0f}% 距离 → "
                    f"x={cand[0]:.4f} y={cand[1]:.4f} z={cand[2]:.4f} "
                    f"roll={cand[3]:.4f} pitch={cand[4]:.4f} yaw={cand[5]:.4f}"
                )
                self.get_logger().info(
                    f"         关节角 deg: "
                    f"{[round(math.degrees(q), 2) for q in joints]}"
                )
            self.get_logger().info("-" * 60)
            self.get_logger().info(
                "用法示例（复制上面的位姿参数）:"
            )
            # 打印第一个推荐目标的完整命令
            best_frac, best_cand, _ = solvable[0]
            self.get_logger().info(
                f"  ros2 run erobot_motion_client linear_interpolation_client \\"
            )
            self.get_logger().info(
                f"    --x {best_cand[0]:.4f} --y {best_cand[1]:.4f} --z {best_cand[2]:.4f} \\"
            )
            self.get_logger().info(
                f"    --roll {best_cand[3]:.4f} --pitch {best_cand[4]:.4f} --yaw {best_cand[5]:.4f} \\"
            )
            self.get_logger().info(
                f"    --step-size 0.005 --execute 1"
            )
        else:
            self.get_logger().warn(
                "所有候选目标均不可达。当前位姿附近可能存在奇异点，"
                "建议手动调整目标位姿或使用 TRAC-IK 替代 KDL 求解器。"
            )

        self.get_logger().info("=" * 60)
        self.get_logger().info("")

        return solvable

    def build_trajectory(self, all_joint_solutions, waypoints):
        """
        将 IK 解和路径点组装成多点 JointTrajectory。
        时间自动按关节速度限制分配，确保最慢的关节也能跟上。
        如果指定了 --total-time，则作为时间下限（若小于关节空间最小时间会警告）。
        """
        traj = JointTrajectory()
        traj.joint_names = JOINT_NAMES

        joint_vel_limit = float(self.args.joint_velocity_limit)

        # 计算各段笛卡尔距离
        total_distance = 0.0
        segment_distances = []

        for i in range(1, len(waypoints)):
            dx = waypoints[i][0] - waypoints[i-1][0]
            dy = waypoints[i][1] - waypoints[i-1][1]
            dz = waypoints[i][2] - waypoints[i-1][2]
            dist = math.sqrt(dx*dx + dy*dy + dz*dz)
            segment_distances.append(dist)
            total_distance += dist

        # 计算各段关节空间最小时间
        joint_segment_times = []
        total_joint_time = 0.0
        max_joint_name = None
        max_joint_disp = 0.0

        for i in range(1, len(all_joint_solutions)):
            displacements = [
                abs(all_joint_solutions[i][j] - all_joint_solutions[i-1][j])
                for j in range(len(JOINT_NAMES))
            ]
            max_disp = max(displacements)
            max_j = displacements.index(max_disp)
            min_seg_time = max_disp / joint_vel_limit if joint_vel_limit > 0 else 0.0
            joint_segment_times.append(min_seg_time)
            total_joint_time += min_seg_time

            if max_disp > max_joint_disp:
                max_joint_disp = max_disp
                max_joint_name = JOINT_NAMES[max_j]

        # 计算总时间
        cartesian_time = total_distance / DEFAULT_MAX_CARTESIAN_VEL

        if self.args.total_time is not None and self.args.total_time > 0:
            total_time = float(self.args.total_time)
            if total_time < total_joint_time and total_joint_time > 0:
                self.get_logger().warn(
                    f"--total-time = {total_time:.2f} s < 关节空间最小时间 "
                    f"{total_joint_time:.2f} s，"
                    f"最慢关节 ({max_joint_name}) 可能跟不上！"
                )
        else:
            total_time = max(cartesian_time, total_joint_time, DEFAULT_MIN_DURATION)

        self.get_logger().info(
            f"总时间: {total_time:.2f} s "
            f"(笛卡尔下限: {cartesian_time:.2f} s, "
            f"关节空间下限: {total_joint_time:.2f} s, "
            f"瓶颈关节: {max_joint_name}, "
            f"关节速度限制: {joint_vel_limit:.3f} rad/s)"
        )

        # 构建轨迹点 — 时间按关节空间需求比例分配
        cumulative_time = 0.0

        # 第一个路径点 (t=0): 不设速度约束，允许平滑通过
        point0 = JointTrajectoryPoint()
        point0.positions = all_joint_solutions[0]
        point0.velocities = []  # 空 = 不强制速度为 0，控制器平滑插值
        point0.time_from_start.sec = 0
        point0.time_from_start.nanosec = 0
        traj.points.append(point0)

        # 后续路径点
        last_idx = len(all_joint_solutions) - 1
        for i in range(1, len(all_joint_solutions)):
            if total_joint_time > 0:
                segment_time = (joint_segment_times[i-1] / total_joint_time) * total_time
            elif total_distance > 0:
                segment_time = (segment_distances[i-1] / total_distance) * total_time
            else:
                segment_time = total_time / (len(all_joint_solutions) - 1)

            cumulative_time += segment_time

            point = JointTrajectoryPoint()
            point.positions = all_joint_solutions[i]

            # 只有最后一个路径点才强制速度为 0（停下来），
            # 中间路径点不设速度约束，让控制器平滑通过
            if i == last_idx:
                point.velocities = [0.0] * len(JOINT_NAMES)
            else:
                point.velocities = []

            point.time_from_start.sec = int(cumulative_time)
            point.time_from_start.nanosec = int(
                (cumulative_time - int(cumulative_time)) * 1e9
            )

            traj.points.append(point)

        return traj, total_time, total_distance

    def export_trajectory(self, waypoints, all_joint_solutions, traj, csv_path):
        """
        将路径点和关节解导出为 CSV 文件。
        导出两个文件:
          - {csv_path}_cartesian.csv : 笛卡尔路径点
          - {csv_path}_joints.csv    : 关节角 + 速度 + 加速度
        """
        base = csv_path.replace(".csv", "")

        # 笛卡尔路径点
        cart_path = base + "_cartesian.csv"
        with open(cart_path, "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(["index", "x", "y", "z", "roll", "pitch", "yaw"])
            for i, wp in enumerate(waypoints):
                writer.writerow([i, wp[0], wp[1], wp[2], wp[3], wp[4], wp[5]])

        self.get_logger().info(f"笛卡尔路径点已导出: {cart_path}")

        # 提取时间序列
        N = len(traj.points)
        times = []
        for pt in traj.points:
            times.append(pt.time_from_start.sec + pt.time_from_start.nanosec * 1e-9)

        # 计算规划速度和加速度（段间有限差分）
        vel_cols = [name + "_vel" for name in JOINT_NAMES]
        acc_cols = [name + "_acc" for name in JOINT_NAMES]
        all_vels = []
        all_accs = []

        for j in range(len(JOINT_NAMES)):
            pos = [all_joint_solutions[i][j] for i in range(N)]
            vel = [0.0] * N
            acc = [0.0] * N
            for i in range(N - 1):
                dt = times[i+1] - times[i]
                if dt > 1e-9:
                    vel[i] = (pos[i+1] - pos[i]) / dt
            vel[N-1] = vel[N-2] if N >= 2 else 0.0
            for i in range(N - 2):
                dt = times[i+1] - times[i]
                if dt > 1e-9:
                    acc[i] = (vel[i+1] - vel[i]) / dt
            acc[N-2] = acc[N-3] if N >= 3 else 0.0
            acc[N-1] = acc[N-2] if N >= 2 else 0.0
            all_vels.append(vel)
            all_accs.append(acc)

        # 关节角 CSV（位置 + 速度 + 加速度）
        joint_path = base + "_joints.csv"
        with open(joint_path, "w", newline="") as f:
            writer = csv.writer(f)
            header = ["index", "time_from_start_s"] + list(JOINT_NAMES) + vel_cols + acc_cols
            writer.writerow(header)
            for i in range(N):
                row = [i, times[i]]
                row += [all_joint_solutions[i][j] for j in range(len(JOINT_NAMES))]
                row += [all_vels[j][i] for j in range(len(JOINT_NAMES))]
                row += [all_accs[j][i] for j in range(len(JOINT_NAMES))]
                writer.writerow(row)

        self.get_logger().info(f"关节轨迹已导出: {joint_path} (含位置/速度/加速度)")

    def verify_cartesian_path(self, all_joint_solutions, csv_path, samples_per_segment=10):
        """
        验证实际笛卡尔路径：在关节轨迹的每两个相邻节点之间做关节空间线性插值
        （模拟控制器实际行为），对每个采样点跑 FK，得到末端真实笛卡尔路径。
        导出到 {csv_path}_actual_cartesian.csv
        """
        self.wait_for_fk_service()

        actual = []
        total = (len(all_joint_solutions) - 1) * samples_per_segment + 1

        self.get_logger().info(
            f"开始 FK 验证：{len(all_joint_solutions)} 个关节节点, "
            f"每段 {samples_per_segment} 个采样点, 共 {total} 个 FK 点..."
        )

        for seg in range(len(all_joint_solutions) - 1):
            q_start = all_joint_solutions[seg]
            q_end = all_joint_solutions[seg + 1]

            for s in range(samples_per_segment):
                alpha = s / samples_per_segment
                q_interp = [
                    q_start[j] + alpha * (q_end[j] - q_start[j])
                    for j in range(len(JOINT_NAMES))
                ]

                # 构建 JointState
                from sensor_msgs.msg import JointState as JS
                js = JS()
                js.name = list(JOINT_NAMES)
                js.position = [float(v) for v in q_interp]

                pose_stamped = self.compute_fk(js)
                p = pose_stamped.pose.position
                o = pose_stamped.pose.orientation
                r, pi, y = euler_from_quaternion([o.x, o.y, o.z, o.w])

                actual.append((p.x, p.y, p.z, r, pi, y))

        # 最后一个点
        q_last = all_joint_solutions[-1]
        from sensor_msgs.msg import JointState as JS
        js = JS()
        js.name = list(JOINT_NAMES)
        js.position = [float(v) for v in q_last]
        pose_stamped = self.compute_fk(js)
        p = pose_stamped.pose.position
        o = pose_stamped.pose.orientation
        r, pi, y = euler_from_quaternion([o.x, o.y, o.z, o.w])
        actual.append((p.x, p.y, p.z, r, pi, y))

        # 导出
        actual_path = csv_path.replace(".csv", "") + "_actual_cartesian.csv"
        with open(actual_path, "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(["index", "x", "y", "z", "roll", "pitch", "yaw"])
            for i, pt in enumerate(actual):
                writer.writerow([i, pt[0], pt[1], pt[2], pt[3], pt[4], pt[5]])

        # 计算偏离直线的最大误差
        if len(actual) >= 2:
            p0 = np.array(actual[0][:3])
            p1 = np.array(actual[-1][:3])
            direction = p1 - p0
            direction_norm = np.linalg.norm(direction)
            max_dev = 0.0
            for pt in actual:
                p = np.array(pt[:3])
                # 点到直线的距离
                if direction_norm > 1e-9:
                    dev = np.linalg.norm(np.cross(p - p0, direction)) / direction_norm
                    max_dev = max(max_dev, dev)
            self.get_logger().info(
                f"实际笛卡尔路径已导出: {actual_path} "
                f"(共 {len(actual)} 个 FK 采样点, 最大偏离直线: {max_dev*1000:.4f} mm)"
            )
        else:
            self.get_logger().info(f"实际笛卡尔路径已导出: {actual_path}")

    def send_trajectory(self, traj, record_path=None):
        """
        将轨迹发送给 arm_controller 执行。
        如果 record_path 不为 None，则在执行过程中录制 /joint_states 到 CSV。
        """
        self.get_logger().info(f"等待控制器 action: {self.controller_action}")

        if not self.traj_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error(
                f"没有找到 {self.controller_action}。请确认 arm_controller 已 active。"
            )
            rclpy.shutdown()
            sys.exit(1)

        self.get_logger().warn("准备下发轨迹到真实机器人，请确认周围安全。")

        # ---- 录制准备 ----
        recorded_data = []  # list of (timestamp, [joint_positions])
        record_sub = None

        if record_path is not None:
            self.get_logger().info(f"将在执行过程中录制关节状态 -> {record_path}")

            def record_callback(msg):
                name_to_pos = dict(zip(msg.name, msg.position))
                try:
                    positions = [name_to_pos[name] for name in JOINT_NAMES]
                    recorded_data.append((
                        msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
                        positions
                    ))
                except KeyError:
                    pass  # 消息不完整，跳过

            record_sub = self.create_subscription(
                JointState, "/joint_states", record_callback, 10
            )

        # ---- 下发轨迹 ----
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = traj

        send_future = self.traj_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future)

        goal_handle = send_future.result()

        if not goal_handle.accepted:
            self.get_logger().error("轨迹目标被控制器拒绝。")
            rclpy.shutdown()
            sys.exit(1)

        # 记录运动起始时间，用于后续时间对齐（t=0 对应运动开始时刻）
        motion_start_sec = self.get_clock().now().nanoseconds * 1e-9

        self.get_logger().info("轨迹目标已接受，等待执行完成...")

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)

        # 录制停止
        if record_sub is not None:
            self.destroy_subscription(record_sub)
            record_sub = None

        result = result_future.result().result
        self.get_logger().info(f"轨迹执行完成，error_code = {result.error_code}")

        # ---- 导出录制数据 ----
        if record_path is not None and recorded_data:
            # 提取时间戳和位置
            rec_ts = np.array([ts - motion_start_sec for ts, _ in recorded_data])
            rec_pos = np.array([pos for _, pos in recorded_data])  # shape: (M, 6)

            # 计算实际速度和加速度（numpy.gradient 二阶中心差分）
            vel_cols = [name + "_vel" for name in JOINT_NAMES]
            acc_cols = [name + "_acc" for name in JOINT_NAMES]
            rec_vel = np.zeros_like(rec_pos)
            rec_acc = np.zeros_like(rec_pos)
            for j in range(len(JOINT_NAMES)):
                rec_vel[:, j] = np.gradient(rec_pos[:, j], rec_ts)
                rec_acc[:, j] = np.gradient(rec_vel[:, j], rec_ts)

            with open(record_path, "w", newline="") as f:
                writer = csv.writer(f)
                writer.writerow(
                    ["timestamp_s"] + list(JOINT_NAMES) + vel_cols + acc_cols
                )
                for i in range(len(rec_ts)):
                    row = [rec_ts[i]] + list(rec_pos[i]) + list(rec_vel[i]) + list(rec_acc[i])
                    writer.writerow(row)

            self.get_logger().info(
                f"关节状态已录制: {record_path} "
                f"({len(recorded_data)} 个采样点, 含位置/速度/加速度)"
            )

    def run(self):
        """
        编排整个直线插补流程。
        """
        self.wait_for_joint_state()
        self.wait_for_ik_service()

        # Step 1: 获取起始位姿
        start_pose = self.get_start_pose()

        # Step 2: 构建目标位姿
        target_pose = (
            float(self.args.x),
            float(self.args.y),
            float(self.args.z),
            float(self.args.roll),
            float(self.args.pitch),
            float(self.args.yaw),
        )

        self.get_logger().info("========== 目标末端位姿 ==========")
        self.get_logger().info(f"x = {target_pose[0]:.6f} m")
        self.get_logger().info(f"y = {target_pose[1]:.6f} m")
        self.get_logger().info(f"z = {target_pose[2]:.6f} m")
        self.get_logger().info(f"roll  = {target_pose[3]:.6f} rad")
        self.get_logger().info(f"pitch = {target_pose[4]:.6f} rad")
        self.get_logger().info(f"yaw   = {target_pose[5]:.6f} rad")
        self.get_logger().info("===================================")

        # Step 3: 确定路径点数量
        N = self.compute_num_waypoints(start_pose[:3], target_pose[:3])

        # Step 4: 生成路径点
        waypoints = self.generate_waypoints(start_pose, target_pose, N)

        self.get_logger().info("========== 笛卡尔路径点 ==========")
        for i, wp in enumerate(waypoints):
            self.get_logger().info(
                f"路径点 {i+1}/{N}: "
                f"x={wp[0]:.6f} y={wp[1]:.6f} z={wp[2]:.6f} "
                f"roll={wp[3]:.6f} pitch={wp[4]:.6f} yaw={wp[5]:.6f}"
            )
        self.get_logger().info("===================================")

        # Step 5: 对每个路径点做 IK 反解
        self.get_logger().info("开始逐个路径点 IK 反解...")

        all_joint_solutions = []

        # 第一个路径点用当前关节角做种子
        seed_joint_state = self.current_joint_state

        for i, wp in enumerate(waypoints):
            self.get_logger().info(f"路径点 {i+1}/{N} IK 反解中...")

            joint_solution = self.compute_ik_for_waypoint(wp, seed_joint_state)

            if joint_solution is None:
                # IK 失败 — 优雅退出并推荐替代目标
                self.get_logger().error("")
                self.get_logger().error("=" * 60)
                self.get_logger().error(
                    f"  ❌ 规划无解：第 {i+1}/{N} 个路径点 IK 反解失败"
                )
                self.get_logger().error(
                    f"  失败位姿: x={wp[0]:.4f} y={wp[1]:.4f} z={wp[2]:.4f} "
                    f"roll={wp[3]:.4f} pitch={wp[4]:.4f} yaw={wp[5]:.4f}"
                )
                self.get_logger().error(
                    f"  已成功求解: {len(all_joint_solutions)}/{N} 个路径点"
                )
                self.get_logger().error("=" * 60)

                # 读取当前末端位姿用于推荐替代目标
                # 直接通过 FK 获取，不受 --start-x 等参数影响
                fk_ok = False
                if self.fk_client.wait_for_service(timeout_sec=3.0):
                    try:
                        pose_stamped = self.compute_fk(self.current_joint_state)
                        cx = pose_stamped.pose.position.x
                        cy = pose_stamped.pose.position.y
                        cz = pose_stamped.pose.position.z
                        q = pose_stamped.pose.orientation
                        cr, cp, cyaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
                        current_pose = (cx, cy, cz, cr, cp, cyaw)
                        fk_ok = True
                        self.get_logger().info(
                            f"当前末端位姿 (FK): "
                            f"x={cx:.4f} y={cy:.4f} z={cz:.4f} "
                            f"roll={cr:.4f} pitch={cp:.4f} yaw={cyaw:.4f}"
                        )
                    except Exception:
                        pass

                if not fk_ok:
                    # 如果 FK 也失败，用 start_pose 作为近似
                    current_pose = start_pose
                    self.get_logger().warn(
                        "FK 获取当前位姿失败，使用规划起始位姿作为替代参考。"
                    )

                self.suggest_alternative_targets(current_pose, target_pose)
                return  # 优雅退出，不崩溃

            self.get_logger().info(
                f"路径点 {i+1}/{N} IK 反解成功, "
                f"关节角 deg: {[round(math.degrees(q), 3) for q in joint_solution]}"
            )

            all_joint_solutions.append(joint_solution)

            # 分支跳跃检测：相邻路径点 IK 解关节空间距离过大 → KDL 跳到了不同分支
            if len(all_joint_solutions) >= 2:
                jumps = [
                    abs(all_joint_solutions[-1][j] - all_joint_solutions[-2][j])
                    for j in range(len(JOINT_NAMES))
                ]
                max_jump = max(jumps)
                if max_jump > 0.5:  # 0.5 rad ≈ 29°，远超正常 5mm 步长产生的关节位移
                    jump_joint = JOINT_NAMES[jumps.index(max_jump)]
                    self.get_logger().warn(
                        f"⚠ 分支跳跃: 路径点 {i+1} 与 {i} 之间 {jump_joint} 跳变了 "
                        f"{math.degrees(max_jump):.1f}°。KDL 求解器可能切换了 IK 分支。"
                    )

            # 更新种子：用本次 IK 解作为下一个路径点的种子
            seed_joint_state = copy.deepcopy(self.current_joint_state)
            for j, name in enumerate(JOINT_NAMES):
                seed_joint_state.position[
                    list(seed_joint_state.name).index(name)
                ] = joint_solution[j]

        # Step 6: 组装轨迹
        traj, total_time, total_distance = self.build_trajectory(
            all_joint_solutions, waypoints
        )

        self.get_logger().info("========== 轨迹总览 ==========")
        self.get_logger().info(f"总路径点: {len(traj.points)}, 总时间: {total_time:.2f} s, 总距离: {total_distance:.4f} m")
        self.get_logger().info("================================")

        # Step 6.5: 导出 CSV + 验证实际笛卡尔路径（如果指定了 --export-csv）
        if self.args.export_csv is not None:
            # 解析路径：绝对路径直接用，相对路径则拼接 export-dir + 时间戳
            csv_base = self.args.export_csv
            if os.path.isabs(csv_base):
                csv_path = csv_base
            else:
                os.makedirs(self.args.export_dir, exist_ok=True)
                ts = datetime.now().strftime("%Y%m%d_%H%M%S")
                csv_path = os.path.join(self.args.export_dir, f"{ts}_{csv_base}")
            self.export_trajectory(waypoints, all_joint_solutions, traj, csv_path)
            self.verify_cartesian_path(all_joint_solutions, csv_path)

        # Step 7: 执行或打印
        if self.args.execute == 1:
            # 确定录制路径
            record_path = None
            if self.args.record_joint_states is not None:
                record_path = self.args.record_joint_states
            elif self.args.export_csv is not None:
                # 如果指定了 --export-csv，自动录制到同目录（使用解析后的 csv_path）
                record_path = csv_path.replace(".csv", "") + "_recorded_joints.csv"
            self.send_trajectory(traj, record_path=record_path)
        else:
            self.get_logger().info("当前 execute=0，只完成路径规划与IK反解，没有下发给真实机器人。")
            self.get_logger().info("确认关节角合理后，把 --execute 0 改成 --execute 1。")


def parse_args():
    parser = argparse.ArgumentParser(
        description="Cartesian linear interpolation client for erobot"
    )

    parser.add_argument("--group", type=str, default="arm")
    parser.add_argument("--base", type=str, default="base_link")
    parser.add_argument("--tip", type=str, default="end_Link")

    parser.add_argument("--ik-service", type=str, default="/compute_ik")
    parser.add_argument("--fk-service", type=str, default="/compute_fk")
    parser.add_argument(
        "--controller-action",
        type=str,
        default="/arm_controller/follow_joint_trajectory"
    )

    # 目标位姿（必填）
    parser.add_argument("--x", type=float, required=True)
    parser.add_argument("--y", type=float, required=True)
    parser.add_argument("--z", type=float, required=True)

    parser.add_argument("--roll", type=float, default=0.0)
    parser.add_argument("--pitch", type=float, default=0.0)
    parser.add_argument("--yaw", type=float, default=0.0)

    # 起始位姿（可选，不指定则通过 FK 获取）
    parser.add_argument("--start-x", type=float, default=None)
    parser.add_argument("--start-y", type=float, default=None)
    parser.add_argument("--start-z", type=float, default=None)
    parser.add_argument("--start-roll", type=float, default=0.0)
    parser.add_argument("--start-pitch", type=float, default=0.0)
    parser.add_argument("--start-yaw", type=float, default=0.0)

    # 路径点生成策略
    parser.add_argument(
        "--step-size",
        type=float,
        default=None,
        help="最大笛卡尔步长 (m)，默认 0.005"
    )
    parser.add_argument(
        "--num-waypoints",
        type=int,
        default=None,
        help="固定路径点数量（--step-size 优先）"
    )

    # 轨迹时间
    parser.add_argument(
        "--total-time",
        type=float,
        default=None,
        help="总运动时间 (s)，默认根据关节速度限制自动计算"
    )
    parser.add_argument(
        "--joint-velocity-limit",
        type=float,
        default=DEFAULT_JOINT_VELOCITY,
        help=f"关节最大速度限制 (rad/s)，默认 {DEFAULT_JOINT_VELOCITY}"
    )

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
        help="0: 只规划不执行；1: 规划后下发给 arm_controller"
    )

    parser.add_argument(
        "--export-dir",
        type=str,
        default=os.path.expanduser("~/codex_scripts/exports"),
        help="CSV 导出目录，默认 ~/codex_scripts/exports/；传绝对路径时 --export-csv 优先"
    )

    parser.add_argument(
        "--export-csv",
        type=str,
        default=None,
        help="导出路径点到 CSV（如传入 'test' → {export_dir}/{timestamp}_test.csv）；"
             "传入绝对路径则直接使用（兼容旧用法）"
    )

    parser.add_argument(
        "--record-joint-states",
        type=str,
        default=None,
        help="执行时录制实际 /joint_states 到指定 CSV 文件（如 /tmp/recorded.csv），"
             "用于事后对比规划 vs 实际关节轨迹"
    )

    args, _ = parser.parse_known_args()
    return args


def main():
    args = parse_args()

    rclpy.init()

    node = LinearInterpolationClient(args)
    node.run()

    rclpy.shutdown()


if __name__ == "__main__":
    main()
