#!/usr/bin/env python3
"""关节力矩监控节点

订阅 /joint_states,读取 6 个关节的 effort(电机侧实际力矩,N·m),
按指定频率打印,可选导出 CSV。

effort 来源:zeroerr_ethercat_hardware_6dof 硬件接口把 EtherCAT 对象字典
0x6077(转矩实际值,int16,额定转矩千分比)换算成 N·m 后,经 ros2_control
effort state_interface 暴露,joint_state_broadcaster 自动发布到 /joint_states。

注意:effort 是电机侧力矩(基于电流环估算),换算到关节侧需乘减速比。
"""
import argparse
import csv

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import JointState


class TorqueMonitorNode(Node):
    def __init__(self, export_csv=None, print_hz=10.0):
        super().__init__("torque_monitor")

        # 关节顺序对应 J1~J6(与 URDF ros2_control 一致)
        self.joint_names = [
            "shoulder_joint",   # J1
            "arm1_joint",       # J2
            "arm2_joint",       # J3
            "wrist1_joint",     # J4
            "wrist2_joint",     # J5
            "end_joint",        # J6
        ]

        self.latest_effort = None
        self.latest_stamp = 0.0

        # 打印限频
        self.print_period = 1.0 / print_hz if print_hz > 0 else 0.0
        self.last_print_sec = 0.0

        # CSV 录制
        self.csv_file = None
        self.csv_writer = None
        if export_csv:
            self.csv_file = open(export_csv, "w", encoding="utf-8", newline="")
            self.csv_writer = csv.writer(self.csv_file)
            self.csv_writer.writerow(["stamp_sec"] + self.joint_names)
            self.get_logger().info(f"力矩录制到: {export_csv}")

        self.create_subscription(
            JointState,
            "/joint_states",
            self.joint_state_callback,
            10,
        )

        self.get_logger().info("力矩监控已启动,订阅 /joint_states")

    def joint_state_callback(self, msg: JointState):
        name_to_effort = dict(zip(msg.name, msg.effort))

        efforts = []
        for joint_name in self.joint_names:
            if joint_name not in name_to_effort:
                return
            efforts.append(name_to_effort[joint_name])

        self.latest_effort = efforts
        self.latest_stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

        # CSV 每帧都写
        if self.csv_writer is not None:
            row = [f"{self.latest_stamp:.6f}"] + [f"{v:.6f}" for v in efforts]
            self.csv_writer.writerow(row)
            self.csv_file.flush()

        # 打印限频
        now_sec = self.get_clock().now().nanoseconds * 1e-9
        if self.print_period <= 0.0 or (now_sec - self.last_print_sec) >= self.print_period:
            self.last_print_sec = now_sec
            text = "  ".join(f"J{i+1}={v:+.4f}" for i, v in enumerate(efforts))
            self.get_logger().info(f"[N·m] {text}")

    def close(self):
        if self.csv_file is not None:
            self.csv_file.close()
            self.get_logger().info("CSV 录制结束")


def main(args=None):
    parser = argparse.ArgumentParser(description="关节力矩监控节点")
    parser.add_argument(
        "--export-csv", default=None,
        help="CSV 输出路径,录制每帧 6 关节力矩(N·m)")
    parser.add_argument(
        "--print-hz", type=float, default=10.0,
        help="打印频率 Hz,0=不打印(默认 10)")
    parsed = parser.parse_args(args)

    rclpy.init(args=args)
    node = TorqueMonitorNode(
        export_csv=parsed.export_csv,
        print_hz=parsed.print_hz,
    )
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
