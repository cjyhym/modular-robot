#!/usr/bin/env python3
"""分析规划 vs 实际关节轨迹的偏差，输出逐点位置/速度/加速度误差和统计摘要。"""
import argparse
import csv
import glob
import os
import sys
import numpy as np


JOINT_NAMES = [
    "shoulder_joint", "arm1_joint", "arm2_joint",
    "wrist1_joint", "wrist2_joint", "end_joint",
]


EXPORT_DIR = os.path.expanduser("~/codex_scripts/exports")


def _resolve(path, suffix):
    """如果 path 的文件存在直接返回；否则在 EXPORT_DIR 找最新匹配文件。"""
    if path and os.path.exists(path):
        return path
    if os.path.isdir(EXPORT_DIR):
        matches = sorted(glob.glob(os.path.join(EXPORT_DIR, f"*{suffix}")))
        if matches:
            return matches[-1]
    return path


def load_csv(path):
    """读取 CSV，返回 dict-of-lists（全部 float）。"""
    data = {}
    with open(path) as f:
        for row in csv.DictReader(f):
            for key, val in row.items():
                data.setdefault(key, []).append(float(val))
    return data


def smooth(data, window):
    """对一维数组做移动平均平滑，window 为窗口大小（奇数更好）。"""
    if window <= 1 or len(data) < window:
        return np.array(data)
    kernel = np.ones(window) / window
    return np.convolve(data, kernel, mode="same")


def interpolate_planned(t_planned, planned_dict, t_recorded, col_names):
    """
    将规划轨迹在录制时间点上做线性插值。
    col_names: 需要插值的列名列表
    返回 dict {col_name: [M]} 插值后的规划值
    """
    interp = {}
    for name in col_names:
        if name in planned_dict:
            interp[name] = np.interp(t_recorded, t_planned, planned_dict[name])
    return interp


def compute_errors(planned_interp, recorded, col_names):
    """
    计算逐点误差。
    返回:
      errors: dict {col_name: [M]}  (recorded - planned)
      abs_errors: dict {col_name: [M]}  absolute errors
    """
    errors = {}
    abs_errors = {}
    for name in col_names:
        if name in planned_interp and name in recorded:
            err = np.array(recorded[name]) - np.array(planned_interp[name])
            errors[name] = err
            abs_errors[name] = np.abs(err)
    return errors, abs_errors


def print_summary(abs_errors, errors, title, unit, scale=1.0, col_names=None):
    """打印统计摘要。col_names 为要查的列名列表，默认 JOINT_NAMES。"""
    if col_names is None:
        col_names = JOINT_NAMES
    print()
    print("=" * 72)
    print(f"  {title}")
    print("=" * 72)

    header = f"{'Joint':<18} {'Max err':>10} {'RMS err':>10} {'Mean err':>10} {'Samples':>8}"
    print(header)
    print("-" * 72)

    for name in col_names:
        if name not in abs_errors:
            continue
        ae = abs_errors[name]
        e = errors[name]
        max_val = np.max(ae) * scale
        rms_val = np.sqrt(np.mean(ae ** 2)) * scale
        mean_val = np.mean(e) * scale
        display = name.replace("_vel", "").replace("_acc", "")
        print(f"{display:<18} {max_val:>10.4f} {rms_val:>10.4f} {mean_val:>10.4f} {len(ae):>8}")

    print("-" * 72)

    all_ae = np.concatenate([abs_errors[n] for n in col_names if n in abs_errors])
    if len(all_ae) > 0:
        overall_max = np.max(all_ae) * scale
        overall_rms = np.sqrt(np.mean(all_ae ** 2)) * scale
        print(f"{'OVERALL':<18} {overall_max:>10.4f} {overall_rms:>10.4f} {'---':>10} {len(all_ae):>8}")
        print()

        if unit == "°":
            if overall_max < 1.0:
                grade = "EXCELLENT — 跟踪精度 < 1°"
            elif overall_max < 3.0:
                grade = "GOOD — 跟踪精度 < 3°"
            elif overall_max < 5.0:
                grade = "FAIR — 跟踪精度 < 5°，考虑放宽容差或降低速度"
            else:
                grade = "POOR — 跟踪精度 > 5°，需排查硬件或控制器参数"
            print(f"  Verdict: {grade}")
    print("=" * 72)
    print()


def export_errors(recorded_ts, planned_interp, recorded, output_path):
    """
    导出逐点误差 CSV（位置 + 速度 + 加速度）。
    """
    with open(output_path, "w", newline="") as f:
        cols = ["timestamp_s"]
        for suffix, unit_scale in [("", np.pi/180), ("_vel", np.pi/180), ("_acc", np.pi/180)]:
            for name in JOINT_NAMES:
                cn = name + suffix
                cols += [
                    f"{cn}_planned",
                    f"{cn}_actual",
                    f"{cn}_error",
                ]
        writer = csv.writer(f)
        writer.writerow(cols)

        n = len(recorded_ts)
        for i in range(n):
            row = [recorded_ts[i]]
            for suffix, unit_scale in [("", np.pi/180), ("_vel", np.pi/180), ("_acc", np.pi/180)]:
                for name in JOINT_NAMES:
                    cn = name + suffix
                    if cn in planned_interp and cn in recorded:
                        p = planned_interp[cn][i] / unit_scale
                        a = recorded[cn][i] / unit_scale
                        e = (recorded[cn][i] - planned_interp[cn][i]) / unit_scale
                        row += [round(p, 6), round(a, 6), round(e, 6)]
                    else:
                        row += ["", "", ""]
            writer.writerow(row)

    print(f"逐点误差已导出: {output_path}")


def main():
    parser = argparse.ArgumentParser(
        description="Analyze planned vs actual joint trajectory tracking errors (position + velocity + acceleration)"
    )
    parser.add_argument(
        "--joints", type=str,
        default="/tmp/linear_path_joints.csv",
        help="规划关节轨迹 CSV (默认 /tmp/linear_path_joints.csv)"
    )
    parser.add_argument(
        "--recorded", type=str,
        default="/tmp/linear_path_recorded_joints.csv",
        help="录制关节状态 CSV (默认 /tmp/linear_path_recorded_joints.csv)"
    )
    parser.add_argument(
        "--output", type=str,
        default="/tmp/trajectory_error.csv",
        help="导出逐点误差 CSV 路径 (默认 /tmp/trajectory_error.csv)"
    )
    parser.add_argument(
        "--smooth-window", type=int, default=0,
        help="平滑滤波窗口大小（采样点数），0=不平滑。建议 5~15，"
             "用于抑制加速度/加速度计算中的微分噪声放大"
    )
    args = parser.parse_args()

    # 自动在 exports 目录搜索（优先 /tmp 原始路径，找不到再搜 exports）
    args.joints = _resolve(args.joints, "_joints.csv")
    args.recorded = _resolve(args.recorded, "_recorded_joints.csv")
    # output 不自动重定向 —— 保持用户指定或默认 /tmp/

    # 检查文件
    if not os.path.exists(args.joints):
        print(f"错误: 规划文件不存在: {args.joints}")
        print(f"  请先运行 linear_interpolation_client 并指定 --export-csv，"
              f"或检查 {EXPORT_DIR}")
        sys.exit(1)
    if not os.path.exists(args.recorded):
        print(f"错误: 录制文件不存在: {args.recorded}")
        print("  请先运行 linear_interpolation_client 并指定 --record-joint-states")
        sys.exit(1)

    # 加载数据
    planned = load_csv(args.joints)
    recorded = load_csv(args.recorded)

    t_planned = np.array(planned["time_from_start_s"])
    t_recorded = np.array(recorded["timestamp_s"])

    # 录制时间归零
    if len(t_recorded) > 0:
        t_recorded = t_recorded - t_recorded[0]

    # 只在录制时间范围内插值
    mask = (t_recorded >= 0) & (t_recorded <= t_planned[-1] + 1.0)
    if not np.any(mask):
        print("错误: 录制数据的时间戳不在规划轨迹的时间范围内。")
        sys.exit(1)
    t_recorded = t_recorded[mask]
    # 过滤所有列
    for key in list(recorded.keys()):
        recorded[key] = np.array(recorded[key])[mask]

    # ---- 位置误差 ----
    pos_names = list(JOINT_NAMES)
    planned_pos_interp = interpolate_planned(t_planned, planned, t_recorded, pos_names)
    pos_errors, pos_abs = compute_errors(planned_pos_interp, recorded, pos_names)
    DEG = 180.0 / np.pi
    print_summary(pos_abs, pos_errors, "Position Tracking Error", "°", scale=DEG)

    # ---- 速度/加速度误差 ----
    # 用与录制数据相同的方法 (numpy.gradient) 从插值位置计算规划速度/加速度，
    # 避免规划侧分段常数速度与录制侧光滑速度之间的"假误差"
    # 可选平滑：抑制微分噪声放大
    sw = args.smooth_window
    vel_names = [n + "_vel" for n in JOINT_NAMES]
    acc_names = [n + "_acc" for n in JOINT_NAMES]
    planned_vel_interp = {}
    planned_acc_interp = {}

    for name in JOINT_NAMES:
        if name in planned_pos_interp:
            p_plan = smooth(planned_pos_interp[name], sw)
            p_rec = smooth(recorded[name], sw)
            # 规划侧：从平滑插值位置计算
            planned_vel_interp[name + "_vel"] = np.gradient(p_plan, t_recorded)
            planned_acc_interp[name + "_acc"] = np.gradient(planned_vel_interp[name + "_vel"], t_recorded)
            # 录制侧：同样从平滑位置重新计算，覆盖原始值
            rec_vel = np.gradient(p_rec, t_recorded)
            recorded[name + "_vel"] = rec_vel
            recorded[name + "_acc"] = np.gradient(rec_vel, t_recorded)

    vel_errors, vel_abs = compute_errors(planned_vel_interp, recorded, vel_names)
    if vel_errors:
        print_summary(vel_abs, vel_errors, "Velocity Tracking Error", "°/s", scale=DEG, col_names=vel_names)

    acc_errors, acc_abs = compute_errors(planned_acc_interp, recorded, acc_names)
    if acc_errors:
        print_summary(acc_abs, acc_errors, "Acceleration Tracking Error", "°/s²", scale=DEG, col_names=acc_names)

    if not pos_errors:
        print("错误: 没有可匹配的关节数据。请检查 CSV 列名是否正确。")
        sys.exit(1)

    # 合并所有插值结果用于导出
    all_planned_interp = {}
    all_planned_interp.update(planned_pos_interp)
    all_planned_interp.update(planned_vel_interp)
    all_planned_interp.update(planned_acc_interp)

    export_errors(t_recorded, all_planned_interp, recorded, args.output)


if __name__ == "__main__":
    main()
