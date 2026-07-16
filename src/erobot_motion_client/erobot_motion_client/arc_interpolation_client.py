#!/usr/bin/env python3
import argparse
import copy
import csv
import math
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
    "shoulder_joint",
    "arm1_joint",
    "arm2_joint",
    "wrist1_joint",
    "wrist2_joint",
    "end_joint",
]

DEFAULT_STEP_SIZE = 0.005          # m
DEFAULT_MAX_CARTESIAN_VEL = 0.02   # m/s
DEFAULT_JOINT_VELOCITY = 0.3       # rad/s (10% scaling)
DEFAULT_MIN_DURATION = 3.0         # s


class ArcInterpolationClient(Node):
    def __init__(self, args):
        super().__init__("arc_interpolation_client")

        self.args = args

        self.group_name = args.group
        self.base_frame = args.base
        self.tip_link = args.tip

        self.ik_service_name = args.ik_service
        self.fk_service_name = args.fk_service
        self.controller_action = args.controller_action

        self.current_joint_state = None

        self.create_subscription(
            JointState, "/joint_states", self.joint_state_callback, 10
        )

        self.fk_client = self.create_client(GetPositionFK, self.fk_service_name)
        self.ik_client = self.create_client(GetPositionIK, self.ik_service_name)
        self.traj_client = ActionClient(
            self, FollowJointTrajectory, self.controller_action
        )

    # ================================================================
    # 基础服务（与 linear_interpolation_client 相同）
    # ================================================================

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
        current_deg = [math.degrees(name_to_pos[name]) for name in JOINT_NAMES]
        self.get_logger().info(
            "当前关节角 deg: " + str([round(v, 3) for v in current_deg])
        )

    def wait_for_fk_service(self):
        self.get_logger().info(f"等待 FK 服务: {self.fk_service_name}")
        if not self.fk_client.wait_for_service(timeout_sec=10.0):
            self.get_logger().error("没有找到 FK 服务。请确认 move_group / MoveIt 已经启动。")
            rclpy.shutdown(); sys.exit(1)
        self.get_logger().info("FK 服务已连接。")

    def wait_for_ik_service(self):
        self.get_logger().info(f"等待 IK 服务: {self.ik_service_name}")
        if not self.ik_client.wait_for_service(timeout_sec=10.0):
            self.get_logger().error("没有找到 IK 服务。请确认 move_group / MoveIt 已经启动。")
            rclpy.shutdown(); sys.exit(1)
        self.get_logger().info("IK 服务已连接。")

    def compute_fk(self, joint_state):
        req = GetPositionFK.Request()
        req.header.frame_id = self.base_frame
        req.header.stamp = self.get_clock().now().to_msg()
        req.fk_link_names = [self.tip_link]
        req.robot_state.joint_state = joint_state

        future = self.fk_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is None:
            self.get_logger().error("FK 服务调用失败。")
            rclpy.shutdown(); sys.exit(1)
        res = future.result()
        if res.error_code.val != MoveItErrorCodes.SUCCESS:
            self.get_logger().error(f"FK 失败, error_code = {res.error_code.val}")
            rclpy.shutdown(); sys.exit(1)
        return res.pose_stamped[0]

    def get_start_pose(self):
        if (self.args.start_x is not None and
            self.args.start_y is not None and
            self.args.start_z is not None):
            self.get_logger().info("========== 起始末端位姿 (用户指定) ==========")
            x, y, z = float(self.args.start_x), float(self.args.start_y), float(self.args.start_z)
            roll, pitch, yaw = float(self.args.start_roll), float(self.args.start_pitch), float(self.args.start_yaw)
            self.get_logger().info(f"x = {x:.6f}  y = {y:.6f}  z = {z:.6f}")
            self.get_logger().info(f"roll = {roll:.6f}  pitch = {pitch:.6f}  yaw = {yaw:.6f}")
            self.get_logger().info("=============================================")
            return (x, y, z, roll, pitch, yaw)

        self.wait_for_fk_service()
        pose_stamped = self.compute_fk(self.current_joint_state)
        x = pose_stamped.pose.position.x
        y = pose_stamped.pose.position.y
        z = pose_stamped.pose.position.z
        q = pose_stamped.pose.orientation
        roll, pitch, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])

        self.get_logger().info("========== 起始末端位姿 (FK) ==========")
        self.get_logger().info(f"x = {x:.6f}  y = {y:.6f}  z = {z:.6f}")
        self.get_logger().info(f"roll = {roll:.6f}  pitch = {pitch:.6f}  yaw = {yaw:.6f}")
        self.get_logger().info("=======================================")
        return (x, y, z, roll, pitch, yaw)

    # ================================================================
    # 圆弧路径点生成
    # ================================================================

    def compute_num_waypoints(self):
        angle_span = abs(self.args.arc_end_angle_rad - self.args.arc_start_angle_rad)
        arc_length = self.args.arc_radius * angle_span

        if self.args.step_size is not None:
            step = self.args.step_size
            if step <= 0:
                self.get_logger().error("--step-size 必须大于 0")
                rclpy.shutdown(); sys.exit(1)
            N = max(2, int(math.ceil(arc_length / step)) + 1)
        elif self.args.num_waypoints is not None:
            N = self.args.num_waypoints
            if N < 2:
                self.get_logger().warn(f"--num-waypoints = {N}，至少需要 2 个路径点，已强制设为 2")
                N = 2
        else:
            N = max(2, int(math.ceil(arc_length / DEFAULT_STEP_SIZE)) + 1)

        if N > 100:
            self.get_logger().warn(f"路径点数量 {N} 较多，建议增大 --step-size")

        self.get_logger().info(
            f"圆弧: 半径={self.args.arc_radius:.4f} m, "
            f"角度跨度={math.degrees(angle_span):.1f}°, "
            f"弧长={arc_length:.4f} m, "
            f"生成 {N} 个路径点"
        )
        return N

    def generate_arc_waypoints(self, start_orientation, N):
        """
        在指定平面内沿圆弧采样 N 个点。
        保持起始姿态不变。
        返回 list of (x, y, z, roll, pitch, yaw)
        """
        cx = self.args.arc_center_x
        cy = self.args.arc_center_y
        cz = self.args.arc_center_z
        r = self.args.arc_radius
        theta_start = self.args.arc_start_angle_rad
        theta_end = self.args.arc_end_angle_rad
        plane = self.args.arc_plane

        roll, pitch, yaw = start_orientation
        waypoints = []

        for i in range(N):
            alpha = i / (N - 1)  # 0.0 ~ 1.0
            theta = theta_start + alpha * (theta_end - theta_start)

            cos_t = math.cos(theta)
            sin_t = math.sin(theta)

            if plane == "xy":
                x = cx + r * cos_t
                y = cy + r * sin_t
                z = cz
            elif plane == "xz":
                x = cx + r * cos_t
                y = cy
                z = cz + r * sin_t
            elif plane == "yz":
                x = cx
                y = cy + r * cos_t
                z = cz + r * sin_t
            else:
                self.get_logger().error(f"不支持的平面: {plane}，请使用 xy / xz / yz")
                rclpy.shutdown(); sys.exit(1)

            waypoints.append((x, y, z, roll, pitch, yaw))

        return waypoints

    # ================================================================
    # IK / 轨迹 / 执行（与 linear_interpolation_client 相同）
    # ================================================================

    def _call_ik_once(self, waypoint, seed_joint_state, timeout_sec=2.0):
        """单次 IK 调用，返回 (success, joint_positions_list_or_None)。"""
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
                self.get_logger().error(f"IK 解里没有关节 {joint_name}")
                rclpy.shutdown(); sys.exit(1)
            target_positions.append(name_to_pos[joint_name])
        return True, target_positions

    def compute_ik_for_waypoint(self, waypoint, seed_joint_state):
        """
        对单个路径点调用 /compute_ik 反解关节角。
        成功返回 6 个关节角 (rad) 的列表，失败返回 None。
        如果 IK 失败，会用微扰动种子重试（最多 3 次）。
        """
        import random
        MAX_RETRIES = 3
        IK_TIMEOUT = 2.0
        PERTURBATION_RAD = 0.1

        success, positions = self._call_ik_once(waypoint, seed_joint_state, IK_TIMEOUT)
        if success:
            return positions

        self.get_logger().warn("IK 首次尝试失败，将用微扰动种子重试...")
        for attempt in range(1, MAX_RETRIES + 1):
            perturbed_seed = copy.deepcopy(seed_joint_state)
            for j in range(len(perturbed_seed.position)):
                if perturbed_seed.name[j] in JOINT_NAMES:
                    perturbed_seed.position[j] += random.uniform(
                        -PERTURBATION_RAD * attempt, PERTURBATION_RAD * attempt
                    )
            self.get_logger().info(
                f"IK 重试 {attempt}/{MAX_RETRIES} (扰动幅度 {PERTURBATION_RAD * attempt:.2f} rad)..."
            )
            success, positions = self._call_ik_once(waypoint, perturbed_seed, IK_TIMEOUT)
            if success:
                self.get_logger().info(f"IK 重试 {attempt} 成功！")
                return positions

        self.get_logger().error(
            f"IK 失败：{MAX_RETRIES} 次重试后仍未找到解。"
            f"KDL 求解器可能在奇异点附近无法收敛。"
        )
        return None

    def _try_ik_quiet(self, waypoint, seed_joint_state):
        """静默版 IK 检查：尝试单次 IK + 最多 2 次扰动重试，不打印日志。"""
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
                        -PERTURBATION_RAD * attempt, PERTURBATION_RAD * attempt
                    )
            success, positions = self._call_ik_once(waypoint, perturbed, 2.0)
            if success:
                return positions
        return None

    def suggest_alternative_targets(self, current_pose, target_pose):
        """
        当规划无解时，生成一组候选目标位姿并逐个测试 IK 可达性。
        """
        self.get_logger().info("")
        self.get_logger().info("=" * 60)
        self.get_logger().info("  规划无解 — 正在生成替代目标位姿推荐...")
        self.get_logger().info("=" * 60)

        dx = target_pose[0] - current_pose[0]
        dy = target_pose[1] - current_pose[1]
        dz = target_pose[2] - current_pose[2]
        distance = math.sqrt(dx*dx + dy*dy + dz*dz)

        if distance < 1e-6:
            self.get_logger().warn("当前位姿与目标位姿重合，无法生成替代目标。")
            return []

        fractions = [0.8, 0.6, 0.5, 0.4, 0.3, 0.2]
        candidates = []
        for frac in fractions:
            cand = (
                current_pose[0] + dx * frac,
                current_pose[1] + dy * frac,
                current_pose[2] + dz * frac,
                target_pose[3], target_pose[4], target_pose[5],
            )
            candidates.append((frac, cand))

        solvable = []
        for frac, cand in candidates:
            result = self._try_ik_quiet(cand, self.current_joint_state)
            if result is not None:
                solvable.append((frac, cand, result))

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
            self.get_logger().info("用法示例（复制上面的位姿参数）:")
            best_frac, best_cand, _ = solvable[0]
            self.get_logger().info(
                f"  ros2 run erobot_motion_client arc_interpolation_client \\"
            )
            self.get_logger().info(
                f"    --arc-plane {self.args.arc_plane} \\"
            )
            self.get_logger().info(
                f"    --arc-center-x {self.args.arc_center_x:.4f} "
                f"--arc-center-y {self.args.arc_center_y:.4f} "
                f"--arc-center-z {self.args.arc_center_z:.4f} \\"
            )
            self.get_logger().info(
                f"    --arc-radius {self.args.arc_radius:.4f} \\"
            )
            self.get_logger().info(
                f"    --arc-start-angle {self.args.arc_start_angle:.1f} "
                f"--arc-end-angle {self.args.arc_end_angle:.1f} \\"
            )
            self.get_logger().info(
                f"    --step-size 0.005 --execute 1"
            )
        else:
            self.get_logger().warn(
                "所有候选目标均不可达。建议手动调整圆弧参数或使用 TRAC-IK。"
            )
        self.get_logger().info("=" * 60)
        self.get_logger().info("")
        return solvable

    def build_trajectory(self, all_joint_solutions, waypoints):
        """
        将 IK 解和路径点组装成多点 JointTrajectory。
        时间自动按关节速度限制分配，确保最慢的关节也能跟上。
        """
        traj = JointTrajectory()
        traj.joint_names = JOINT_NAMES

        joint_vel_limit = float(self.args.joint_velocity_limit)

        total_distance = 0.0
        segment_distances = []
        for i in range(1, len(waypoints)):
            dx = waypoints[i][0] - waypoints[i-1][0]
            dy = waypoints[i][1] - waypoints[i-1][1]
            dz = waypoints[i][2] - waypoints[i-1][2]
            dist = math.sqrt(dx*dx + dy*dy + dz*dz)
            segment_distances.append(dist)
            total_distance += dist

        # 关节空间最小时间
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

        cumulative_time = 0.0
        point0 = JointTrajectoryPoint()
        point0.positions = all_joint_solutions[0]
        point0.velocities = []
        point0.time_from_start.sec = 0
        point0.time_from_start.nanosec = 0
        traj.points.append(point0)

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
            point.velocities = [] if i != last_idx else [0.0] * len(JOINT_NAMES)
            point.time_from_start.sec = int(cumulative_time)
            point.time_from_start.nanosec = int((cumulative_time - int(cumulative_time)) * 1e9)
            traj.points.append(point)

        return traj, total_time, total_distance

    def send_trajectory(self, traj, record_path=None):
        """
        将轨迹发送给 arm_controller 执行。
        如果 record_path 不为 None，则在执行过程中录制 /joint_states 到 CSV。
        """
        self.get_logger().info(f"等待控制器 action: {self.controller_action}")
        if not self.traj_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error("没有找到控制器 action。请确认 arm_controller 已 active。")
            rclpy.shutdown(); sys.exit(1)

        self.get_logger().warn("准备下发轨迹到真实机器人，请确认周围安全。")

        recorded_data = []
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
                    pass
            record_sub = self.create_subscription(
                JointState, "/joint_states", record_callback, 10
            )

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = traj
        send_future = self.traj_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()
        if not goal_handle.accepted:
            self.get_logger().error("轨迹目标被控制器拒绝。")
            rclpy.shutdown(); sys.exit(1)

        motion_start_sec = self.get_clock().now().nanoseconds * 1e-9
        self.get_logger().info("轨迹目标已接受，等待执行完成...")
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)

        if record_sub is not None:
            self.destroy_subscription(record_sub)
            record_sub = None

        result = result_future.result().result
        self.get_logger().info(f"轨迹执行完成，error_code = {result.error_code}")

        if record_path is not None and recorded_data:
            rec_ts = np.array([ts - motion_start_sec for ts, _ in recorded_data])
            rec_pos = np.array([pos for _, pos in recorded_data])
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

    def export_trajectory(self, waypoints, all_joint_solutions, traj, csv_path):
        """
        导出笛卡尔路径点 + 关节轨迹（位置+速度+加速度）到 CSV。
        """
        base = csv_path.replace(".csv", "")
        cart_path = base + "_cartesian.csv"
        with open(cart_path, "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(["index", "x", "y", "z", "roll", "pitch", "yaw"])
            for i, wp in enumerate(waypoints):
                writer.writerow([i, wp[0], wp[1], wp[2], wp[3], wp[4], wp[5]])
        self.get_logger().info(f"笛卡尔路径点已导出: {cart_path}")

        N = len(traj.points)
        times = []
        for pt in traj.points:
            times.append(pt.time_from_start.sec + pt.time_from_start.nanosec * 1e-9)

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
        self.wait_for_fk_service()
        actual = []
        total = (len(all_joint_solutions) - 1) * samples_per_segment + 1
        self.get_logger().info(
            f"开始 FK 验证：{len(all_joint_solutions)} 个关节节点, "
            f"每段 {samples_per_segment} 个采样点, 共 {total} 个 FK 点..."
        )

        from sensor_msgs.msg import JointState as JS
        for seg in range(len(all_joint_solutions) - 1):
            q_start = all_joint_solutions[seg]
            q_end = all_joint_solutions[seg + 1]
            for s in range(samples_per_segment):
                alpha = s / samples_per_segment
                q_interp = [q_start[j] + alpha * (q_end[j] - q_start[j]) for j in range(len(JOINT_NAMES))]
                js = JS()
                js.name = list(JOINT_NAMES)
                js.position = [float(v) for v in q_interp]
                pose_stamped = self.compute_fk(js)
                p = pose_stamped.pose.position
                o = pose_stamped.pose.orientation
                r, pi, y = euler_from_quaternion([o.x, o.y, o.z, o.w])
                actual.append((p.x, p.y, p.z, r, pi, y))

        js = JS()
        js.name = list(JOINT_NAMES)
        js.position = [float(v) for v in all_joint_solutions[-1]]
        pose_stamped = self.compute_fk(js)
        p = pose_stamped.pose.position
        o = pose_stamped.pose.orientation
        r, pi, y = euler_from_quaternion([o.x, o.y, o.z, o.w])
        actual.append((p.x, p.y, p.z, r, pi, y))

        actual_path = base = csv_path.replace(".csv", "") + "_actual_cartesian.csv"
        with open(actual_path, "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(["index", "x", "y", "z", "roll", "pitch", "yaw"])
            for i, pt in enumerate(actual):
                writer.writerow([i, pt[0], pt[1], pt[2], pt[3], pt[4], pt[5]])

        # 计算偏离理想圆弧的最大误差
        # 跳过第一段（当前位置→圆弧起点），只验证圆弧部分
        arc_start_idx = samples_per_segment  # 第一段是 approach，跳过
        arc_actual = actual[arc_start_idx:]

        if len(arc_actual) >= 3:
            cx = self.args.arc_center_x
            cy = self.args.arc_center_y
            cz = self.args.arc_center_z
            r = self.args.arc_radius
            plane = self.args.arc_plane

            max_dev = 0.0
            for pt in arc_actual:
                if plane == "xy":
                    dist = math.sqrt((pt[0] - cx)**2 + (pt[1] - cy)**2)
                elif plane == "xz":
                    dist = math.sqrt((pt[0] - cx)**2 + (pt[2] - cz)**2)
                elif plane == "yz":
                    dist = math.sqrt((pt[1] - cy)**2 + (pt[2] - cz)**2)
                else:
                    dist = 0
                dev = abs(dist - r)
                max_dev = max(max_dev, dev)

            self.get_logger().info(
                f"实际笛卡尔路径已导出: {actual_path} "
                f"(共 {len(actual)} 个 FK 采样点, 最大偏离圆弧: {max_dev*1000:.4f} mm)"
            )
        else:
            self.get_logger().info(f"实际笛卡尔路径已导出: {actual_path}")

    # ================================================================
    # 主流程
    # ================================================================

    def run(self):
        self.wait_for_joint_state()
        self.wait_for_ik_service()

        # Step 1: 获取起始位姿
        start_pose = self.get_start_pose()
        start_orientation = start_pose[3:]  # (roll, pitch, yaw)

        # Step 2: 确定 N
        N = self.compute_num_waypoints()

        # Step 3: 生成圆弧路径点
        waypoints = self.generate_arc_waypoints(start_orientation, N)

        self.get_logger().info("========== 圆弧路径点 (笛卡尔) ==========")
        self.get_logger().info(
            f"平面: {self.args.arc_plane}, "
            f"圆心: ({self.args.arc_center_x:.4f}, {self.args.arc_center_y:.4f}, {self.args.arc_center_z:.4f}), "
            f"半径: {self.args.arc_radius:.4f} m"
        )
        for i, wp in enumerate(waypoints):
            self.get_logger().info(
                f"路径点 {i+1}/{N}: "
                f"x={wp[0]:.6f} y={wp[1]:.6f} z={wp[2]:.6f} "
                f"roll={wp[3]:.6f} pitch={wp[4]:.6f} yaw={wp[5]:.6f}"
            )
        self.get_logger().info("=========================================")

        # Step 4: IK 反解
        self.get_logger().info("开始逐个路径点 IK 反解...")
        all_joint_solutions = []
        seed_joint_state = self.current_joint_state

        for i, wp in enumerate(waypoints):
            self.get_logger().info(f"路径点 {i+1}/{N} IK 反解中...")
            joint_solution = self.compute_ik_for_waypoint(wp, seed_joint_state)

            if joint_solution is None:
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
                    current_pose = start_pose
                    self.get_logger().warn(
                        "FK 获取当前位姿失败，使用规划起始位姿作为替代参考。"
                    )
                self.suggest_alternative_targets(current_pose, waypoints[-1])
                return

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
                if max_jump > 0.5:
                    jump_joint = JOINT_NAMES[jumps.index(max_jump)]
                    self.get_logger().warn(
                        f"⚠ 分支跳跃: 路径点 {i+1} 与 {i} 之间 {jump_joint} 跳变了 "
                        f"{math.degrees(max_jump):.1f}°。KDL 求解器可能切换了 IK 分支。"
                    )

            seed_joint_state = copy.deepcopy(self.current_joint_state)
            for j, name in enumerate(JOINT_NAMES):
                seed_joint_state.position[
                    list(seed_joint_state.name).index(name)
                ] = joint_solution[j]

        # Step 4.5: 在轨迹最前面插入当前关节角，确保轨迹从当前状态开始
        name_to_pos = dict(
            zip(self.current_joint_state.name, self.current_joint_state.position)
        )
        current_joint_positions = [name_to_pos[name] for name in JOINT_NAMES]
        all_joint_solutions.insert(0, current_joint_positions)
        waypoints.insert(0, start_pose)
        self.get_logger().info(
            f"轨迹起点: 当前关节角 deg={[round(math.degrees(q), 3) for q in current_joint_positions]}"
        )

        # 检查当前位姿到第一个圆弧路径点的关节空间距离
        approach_jumps = [
            abs(all_joint_solutions[1][j] - all_joint_solutions[0][j])
            for j in range(len(JOINT_NAMES))
        ]
        max_approach_jump = max(approach_jumps)
        if max_approach_jump > 0.5:
            jump_joint = JOINT_NAMES[approach_jumps.index(max_approach_jump)]
            self.get_logger().warn(
                f"⚠ 当前位姿到圆弧起点关节距离过大: {jump_joint} "
                f"{math.degrees(max_approach_jump):.1f}°。建议先用 PTP 把机械臂移到圆弧起点附近。"
            )

        # Step 5: 组装轨迹
        traj, total_time, total_distance = self.build_trajectory(
            all_joint_solutions, waypoints
        )

        self.get_logger().info("========== 轨迹总览 ==========")
        self.get_logger().info(
            f"总路径点: {len(traj.points)} (含起点), 总时间: {total_time:.2f} s, "
            f"弧长(弦长): {total_distance:.4f} m"
        )
        self.get_logger().info("================================")

        # Step 5.5: 导出 CSV + 验证
        if self.args.export_csv is not None:
            self.export_trajectory(waypoints, all_joint_solutions, traj, self.args.export_csv)
            self.verify_cartesian_path(all_joint_solutions, self.args.export_csv)

        # Step 6: 执行或打印
        if self.args.execute == 1:
            record_path = None
            if self.args.record_joint_states is not None:
                record_path = self.args.record_joint_states
            elif self.args.export_csv is not None:
                record_path = self.args.export_csv.replace(".csv", "") + "_recorded_joints.csv"
            self.send_trajectory(traj, record_path=record_path)
        else:
            self.get_logger().info("当前 execute=0，只完成路径规划与IK反解，没有下发给真实机器人。")
            self.get_logger().info("确认关节角合理后，把 --execute 0 改成 --execute 1。")


def parse_args():
    parser = argparse.ArgumentParser(
        description="Cartesian arc interpolation client for erobot"
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

    # 圆弧定义
    parser.add_argument(
        "--arc-plane", type=str, required=True,
        choices=["xy", "xz", "yz"],
        help="圆弧所在平面: xy / xz / yz"
    )
    parser.add_argument("--arc-center-x", type=float, required=True)
    parser.add_argument("--arc-center-y", type=float, required=True)
    parser.add_argument("--arc-center-z", type=float, required=True)
    parser.add_argument("--arc-radius", type=float, required=True)
    parser.add_argument(
        "--arc-start-angle", type=float, required=True,
        help="起始角度 (deg)"
    )
    parser.add_argument(
        "--arc-end-angle", type=float, required=True,
        help="终止角度 (deg)"
    )

    # 起始位姿（可选，不指定则通过 FK 获取）
    parser.add_argument("--start-x", type=float, default=None)
    parser.add_argument("--start-y", type=float, default=None)
    parser.add_argument("--start-z", type=float, default=None)
    parser.add_argument("--start-roll", type=float, default=0.0)
    parser.add_argument("--start-pitch", type=float, default=0.0)
    parser.add_argument("--start-yaw", type=float, default=0.0)

    # 路径点密度
    parser.add_argument(
        "--step-size", type=float, default=None,
        help="最大弧长步长 (m)，默认 0.005"
    )
    parser.add_argument("--num-waypoints", type=int, default=None)

    # 时间
    parser.add_argument(
        "--total-time", type=float, default=None,
        help="总运动时间 (s)，默认根据关节速度限制自动计算"
    )
    parser.add_argument(
        "--joint-velocity-limit", type=float, default=DEFAULT_JOINT_VELOCITY,
        help=f"关节最大速度限制 (rad/s)，默认 {DEFAULT_JOINT_VELOCITY}"
    )

    parser.add_argument("--avoid-collisions", type=int, default=0)
    parser.add_argument("--execute", type=int, default=0)

    parser.add_argument(
        "--export-csv", type=str, default=None,
        help="导出路径点 CSV（含位置/速度/加速度），执行时自动录制"
    )
    parser.add_argument(
        "--record-joint-states", type=str, default=None,
        help="执行时录制实际 /joint_states 到指定 CSV"
    )

    args, _ = parser.parse_known_args()

    # 角度从度转弧度
    args.arc_start_angle_rad = math.radians(args.arc_start_angle)
    args.arc_end_angle_rad = math.radians(args.arc_end_angle)

    return args


def main():
    args = parse_args()
    rclpy.init()
    node = ArcInterpolationClient(args)
    node.run()
    rclpy.shutdown()


if __name__ == "__main__":
    main()