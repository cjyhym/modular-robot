#!/usr/bin/env python3
"""画直线插补轨迹，对比规划 vs 实际（笛卡尔路径 + 关节角）"""
import argparse
import csv
import glob
import os
import matplotlib
matplotlib.use("TkAgg")
import matplotlib.pyplot as plt
import numpy as np


def load_csv(path):
    """读取 CSV 文件，返回 dict-of-lists。"""
    data = {}
    with open(path) as f:
        for row in csv.DictReader(f):
            for key, val in row.items():
                data.setdefault(key, []).append(float(val))
    return data


EXPORT_DIR = os.path.expanduser("~/codex_scripts/exports")


def _resolve(path, suffix):
    """如果 path 的文件存在直接返回；否则在 EXPORT_DIR 找最新匹配文件。"""
    if path and os.path.exists(path):
        return path
    # 在 exports 目录搜索 *{suffix} 文件，取最新
    if os.path.isdir(EXPORT_DIR):
        matches = sorted(glob.glob(os.path.join(EXPORT_DIR, f"*{suffix}")))
        if matches:
            return matches[-1]
    return path  # 回退原路径（后续 os.path.exists 会跳过）


def main():
    parser = argparse.ArgumentParser(
        description="Plot linear interpolation trajectory verification"
    )
    parser.add_argument(
        "--cartesian", type=str,
        default="/tmp/linear_path_cartesian.csv",
        help="规划笛卡尔路径点 CSV (默认 /tmp/linear_path_cartesian.csv)"
    )
    parser.add_argument(
        "--actual-cartesian", type=str,
        default="/tmp/linear_path_actual_cartesian.csv",
        help="FK 验证笛卡尔路径 CSV (默认 /tmp/linear_path_actual_cartesian.csv)"
    )
    parser.add_argument(
        "--joints", type=str,
        default="/tmp/linear_path_joints.csv",
        help="规划关节轨迹 CSV (默认 /tmp/linear_path_joints.csv)"
    )
    parser.add_argument(
        "--recorded", type=str,
        default="/tmp/linear_path_recorded_joints.csv",
        help="实际录制关节状态 CSV (默认 /tmp/linear_path_recorded_joints.csv)"
    )
    args = parser.parse_args()

    # 自动在 exports 目录搜索（优先 /tmp 原始路径，找不到再搜 exports）
    args.cartesian = _resolve(args.cartesian, "_cartesian.csv")
    args.actual_cartesian = _resolve(args.actual_cartesian, "_actual_cartesian.csv")
    args.joints = _resolve(args.joints, "_joints.csv")
    args.recorded = _resolve(args.recorded, "_recorded_joints.csv")

    JOINT_NAMES = [
        "shoulder_joint", "arm1_joint", "arm2_joint",
        "wrist1_joint", "wrist2_joint", "end_joint",
    ]

    has_planned = os.path.exists(args.cartesian)
    has_actual = os.path.exists(args.actual_cartesian)
    has_joints = os.path.exists(args.joints)
    has_recorded = os.path.exists(args.recorded)

    if not has_planned and not has_recorded:
        print("错误: 没有找到任何数据文件。请先运行 linear_interpolation_client 并指定 --export-csv 或 --record-joint-states")
        print(f"  查找路径: {args.cartesian}")
        print(f"  查找路径: {args.recorded}")
        return

    # 确定子图布局
    nrows = 2
    ncols = 3
    fig = plt.figure(figsize=(18, 12))

    # ---- 笛卡尔路径 (3D + 投影) ----
    if has_planned:
        wp = load_csv(args.cartesian)
        wp_x, wp_y, wp_z = np.array(wp["x"]), np.array(wp["y"]), np.array(wp["z"])
    else:
        wp_x = wp_y = wp_z = np.array([])

    if has_actual:
        ac = load_csv(args.actual_cartesian)
        ax_x, ax_y, ax_z = np.array(ac["x"]), np.array(ac["y"]), np.array(ac["z"])

        # 计算偏离直线
        p0 = np.array([ax_x[0], ax_y[0], ax_z[0]])
        p1 = np.array([ax_x[-1], ax_y[-1], ax_z[-1]])
        direction = p1 - p0
        dir_norm = np.linalg.norm(direction)
        deviations = []
        for i in range(len(ax_x)):
            p = np.array([ax_x[i], ax_y[i], ax_z[i]])
            if dir_norm > 1e-9:
                dev = np.linalg.norm(np.cross(p - p0, direction)) / dir_norm
                deviations.append(dev * 1000)  # mm
        max_dev = max(deviations) if deviations else 0
    else:
        ax_x = ax_y = ax_z = np.array([])
        deviations = []
        max_dev = 0

    title_parts = [f"Cartesian Linear Interpolation"]
    if has_planned:
        title_parts.append(f"Waypoints: {len(wp_x)}")
    if has_actual:
        title_parts.append(f"FK samples: {len(ax_x)}")
        title_parts.append(f"Max deviation: {max_dev:.4f} mm")
    fig.suptitle("  |  ".join(title_parts), fontsize=13)

    # 3D 路径
    ax3d = fig.add_subplot(nrows, 3, 1, projection="3d")
    if has_actual:
        ax3d.plot(ax_x, ax_y, ax_z, "b-", linewidth=1.5, alpha=0.7, label="actual FK path")
    if has_planned:
        ax3d.plot(wp_x, wp_y, wp_z, "ro-", linewidth=2, markersize=5, label="planned waypoints")
    ax3d.set_xlabel("X (m)"); ax3d.set_ylabel("Y (m)"); ax3d.set_zlabel("Z (m)")
    ax3d.set_title("3D Path (blue=actual, red=planned)")
    if has_planned or has_actual:
        ax3d.legend()

    # XY 投影
    ax = fig.add_subplot(nrows, 3, 2)
    if has_actual:
        ax.plot(ax_x, ax_y, "b-", linewidth=1.5, alpha=0.7, label="actual")
    if has_planned:
        ax.plot(wp_x, wp_y, "ro-", linewidth=2, markersize=5, label="planned")
    ax.set_xlabel("X (m)"); ax.set_ylabel("Y (m)")
    ax.set_title("XY Projection")
    ax.axis("equal"); ax.grid(True)
    if has_planned or has_actual:
        ax.legend()

    # XZ 投影
    ax = fig.add_subplot(nrows, 3, 3)
    if has_actual:
        ax.plot(ax_x, ax_z, "b-", linewidth=1.5, alpha=0.7, label="actual")
    if has_planned:
        ax.plot(wp_x, wp_z, "ro-", linewidth=2, markersize=5, label="planned")
    ax.set_xlabel("X (m)"); ax.set_ylabel("Z (m)")
    ax.set_title("XZ Projection")
    ax.grid(True)
    if has_planned or has_actual:
        ax.legend()
    ax.invert_yaxis()

    # YZ 投影
    ax = fig.add_subplot(nrows, 3, 4)
    if has_actual:
        ax.plot(ax_y, ax_z, "b-", linewidth=1.5, alpha=0.7, label="actual")
    if has_planned:
        ax.plot(wp_y, wp_z, "ro-", linewidth=2, markersize=5, label="planned")
    ax.set_xlabel("Y (m)"); ax.set_ylabel("Z (m)")
    ax.set_title("YZ Projection")
    ax.grid(True)
    if has_planned or has_actual:
        ax.legend()
    ax.invert_yaxis()

    # 偏离直线误差
    ax = fig.add_subplot(nrows, 3, 5)
    if deviations:
        ax.plot(deviations, "b.-", linewidth=1.5, markersize=3)
    ax.axhline(y=0, color="gray", linestyle="--")
    ax.set_xlabel("FK sample index"); ax.set_ylabel("Deviation (mm)")
    ax.set_title(f"Deviation from straight line (max={max_dev:.4f} mm)")
    ax.grid(True)

    # 位置分量
    ax = fig.add_subplot(nrows, 3, 6)
    if has_actual:
        t = np.arange(len(ax_x))
        ax.plot(t, ax_x, "r-", linewidth=1, alpha=0.7, label="X")
        ax.plot(t, ax_y, "g-", linewidth=1, alpha=0.7, label="Y")
        ax.plot(t, ax_z, "b-", linewidth=1, alpha=0.7, label="Z")
    ax.set_xlabel("FK sample index"); ax.set_ylabel("Position (m)")
    ax.set_title("Actual Position vs Sample")
    ax.legend(); ax.grid(True)

    # ---- 关节角对比 (planned vs recorded) ----
    if has_recorded:
        rec = load_csv(args.recorded)
        rec_ts = np.array(rec["timestamp_s"])
        # 时间归零
        if len(rec_ts) > 0:
            rec_ts = rec_ts - rec_ts[0]

        if has_joints:
            jp = load_csv(args.joints)
            jp_ts = np.array(jp["time_from_start_s"])

        colors = ["#e74c3c", "#2ecc71", "#3498db", "#9b59b6", "#f39c12", "#1abc9c"]

        fig2 = plt.figure(figsize=(16, 10))
        fig2.suptitle(
            "Joint Trajectory: Planned vs Actual  |  "
            f"Recorded samples: {len(rec_ts)}",
            fontsize=13
        )

        for i, name in enumerate(JOINT_NAMES):
            ax = fig2.add_subplot(2, 3, i + 1)

            # 规划轨迹
            if has_joints and name in jp:
                ax.plot(jp_ts, np.degrees(jp[name]), "k--",
                        linewidth=1.5, alpha=0.6, label="planned")

            # 实际录制
            if name in rec:
                ax.plot(rec_ts, np.degrees(rec[name]), "-",
                        color=colors[i], linewidth=2, alpha=0.85, label="actual")

            ax.set_xlabel("Time (s)")
            ax.set_ylabel(f"{name}\n(deg)")
            ax.set_title(name)
            ax.legend(fontsize=7)
            ax.grid(True)

        plt.tight_layout()

    # ---- 关节速度对比 (planned vs recorded) ----
    if has_recorded and has_joints:
        fig3 = plt.figure(figsize=(16, 10))
        fig3.suptitle(
            "Joint Velocity: Planned vs Actual  |  "
            f"Recorded samples: {len(rec_ts)}",
            fontsize=13
        )

        for i, name in enumerate(JOINT_NAMES):
            ax = fig3.add_subplot(2, 3, i + 1)
            vel_name = name + "_vel"

            # 规划速度
            if has_joints and vel_name in jp:
                ax.plot(jp_ts, np.degrees(jp[vel_name]), "k--",
                        linewidth=1.5, alpha=0.6, label="planned")

            # 实际速度
            if vel_name in rec:
                ax.plot(rec_ts, np.degrees(rec[vel_name]), "-",
                        color=colors[i], linewidth=2, alpha=0.85, label="actual")

            ax.set_xlabel("Time (s)")
            ax.set_ylabel(f"{name} vel\n(deg/s)")
            ax.set_title(name)
            ax.legend(fontsize=7)
            ax.grid(True)

        plt.tight_layout()

        # ---- 关节加速度对比 (planned vs recorded) ----
        fig4 = plt.figure(figsize=(16, 10))
        fig4.suptitle(
            "Joint Acceleration: Planned vs Actual  |  "
            f"Recorded samples: {len(rec_ts)}",
            fontsize=13
        )

        for i, name in enumerate(JOINT_NAMES):
            ax = fig4.add_subplot(2, 3, i + 1)
            acc_name = name + "_acc"

            # 规划加速度
            if has_joints and acc_name in jp:
                ax.plot(jp_ts, np.degrees(jp[acc_name]), "k--",
                        linewidth=1.5, alpha=0.6, label="planned")

            # 实际加速度
            if acc_name in rec:
                ax.plot(rec_ts, np.degrees(rec[acc_name]), "-",
                        color=colors[i], linewidth=2, alpha=0.85, label="actual")

            ax.set_xlabel("Time (s)")
            ax.set_ylabel(f"{name} acc\n(deg/s²)")
            ax.set_title(name)
            ax.legend(fontsize=7)
            ax.grid(True)

        plt.tight_layout()

    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    main()
