#!/usr/bin/env python3
"""画圆弧插补轨迹，对比规划节点 vs 实际笛卡尔路径 vs 理想圆弧"""
import csv
import matplotlib
matplotlib.use("TkAgg")
import matplotlib.pyplot as plt
import numpy as np

# ---- 读规划节点 ----
wp_x, wp_y, wp_z = [], [], []
with open("/tmp/arc_path_cartesian.csv") as f:
    for row in csv.DictReader(f):
        wp_x.append(float(row["x"]))
        wp_y.append(float(row["y"]))
        wp_z.append(float(row["z"]))

# ---- 读实际笛卡尔路径 ----
ax_x, ax_y, ax_z = [], [], []
with open("/tmp/arc_path_actual_cartesian.csv") as f:
    for row in csv.DictReader(f):
        ax_x.append(float(row["x"]))
        ax_y.append(float(row["y"]))
        ax_z.append(float(row["z"]))

# 跳过 approach 段（当前位置→圆弧起点）
# 规划节点: wp[0] 是当前位姿, wp[1:] 是圆弧
# 实际路径: 前 SKIP 个 FK 采样点是 approach 段
SKIP = 10
wp_x_arc = wp_x[1:]   # 跳过第一个（当前位姿）
wp_y_arc = wp_y[1:]
wp_z_arc = wp_z[1:]
ax_x_arc = ax_x[SKIP:]  # 跳过 approach 段
ax_y_arc = ax_y[SKIP:]
ax_z_arc = ax_z[SKIP:]

# 用圆弧部分拟合圆心和半径
pts = np.column_stack([ax_x_arc[:3], ax_y_arc[:3], ax_z_arc[:3]])
A = np.column_stack([2*pts[:, 0], 2*pts[:, 1], np.ones(3)])
b = pts[:, 0]**2 + pts[:, 1]**2
sol = np.linalg.lstsq(A, b, rcond=None)[0]
fit_cx, fit_cy = sol[0], sol[1]
fit_r = np.sqrt(sol[2] + fit_cx**2 + fit_cy**2)

# 计算偏离
deviations = []
for x, y in zip(ax_x_arc, ax_y_arc):
    dist = np.sqrt((x - fit_cx)**2 + (y - fit_cy)**2)
    deviations.append(abs(dist - fit_r) * 1000)

max_dev = max(deviations) if deviations else 0

fig = plt.figure(figsize=(16, 10))
fig.suptitle(
    f"Arc Interpolation Verification (approach segment hidden)\n"
    f"Arc waypoints: {len(wp_x_arc)}  |  FK samples: {len(ax_x_arc)}  |  "
    f"Fit center: ({fit_cx:.4f}, {fit_cy:.4f})  |  "
    f"Max deviation: {max_dev:.4f} mm",
    fontsize=13
)

# === 3D 路径 ===
ax3d = fig.add_subplot(2, 3, 1, projection="3d")
ax3d.plot(ax_x_arc, ax_y_arc, ax_z_arc, "b-", linewidth=1.5, alpha=0.7, label="actual FK path")
ax3d.plot(wp_x_arc, wp_y_arc, wp_z_arc, "ro-", linewidth=2, markersize=4, label="planned waypoints")
ax3d.set_xlabel("X (m)"); ax3d.set_ylabel("Y (m)"); ax3d.set_zlabel("Z (m)")
ax3d.set_title("3D Path")
ax3d.legend()

# === XY 投影 (主要视图) ===
ax = fig.add_subplot(2, 3, 2)
ax.plot(ax_x_arc, ax_y_arc, "b-", linewidth=1.5, alpha=0.7, label="actual")
ax.plot(wp_x_arc, wp_y_arc, "ro-", linewidth=2, markersize=4, label="planned")
theta = np.linspace(0, 2*np.pi, 200)
ax.plot(fit_cx + fit_r * np.cos(theta), fit_cy + fit_r * np.sin(theta),
        "g--", linewidth=1, alpha=0.5, label=f"fit circle (r={fit_r:.4f})")
ax.plot(fit_cx, fit_cy, "g+", markersize=10, label=f"fit center")
ax.set_xlabel("X (m)"); ax.set_ylabel("Y (m)")
ax.set_title("XY Projection (main view)")
ax.axis("equal"); ax.grid(True); ax.legend()

# === XZ 投影 ===
ax = fig.add_subplot(2, 3, 3)
ax.plot(ax_x_arc, ax_z_arc, "b-", linewidth=1.5, alpha=0.7, label="actual")
ax.plot(wp_x_arc, wp_z_arc, "ro-", linewidth=2, markersize=4, label="planned")
ax.set_xlabel("X (m)"); ax.set_ylabel("Z (m)")
ax.set_title("XZ Projection")
ax.grid(True); ax.legend(); ax.invert_yaxis()

# === YZ 投影 ===
ax = fig.add_subplot(2, 3, 4)
ax.plot(ax_y_arc, ax_z_arc, "b-", linewidth=1.5, alpha=0.7, label="actual")
ax.plot(wp_y_arc, wp_z_arc, "ro-", linewidth=2, markersize=4, label="planned")
ax.set_xlabel("Y (m)"); ax.set_ylabel("Z (m)")
ax.set_title("YZ Projection")
ax.grid(True); ax.legend(); ax.invert_yaxis()

# === 偏离圆弧误差 (mm) ===
ax = fig.add_subplot(2, 3, 5)
ax.plot(deviations, "b.-", linewidth=1.5, markersize=3)
ax.axhline(y=0, color="gray", linestyle="--")
ax.set_xlabel("FK sample index"); ax.set_ylabel("Deviation (mm)")
ax.set_title(f"Deviation from ideal arc (max={max_dev:.4f} mm)")
ax.grid(True)

# === 位置分量 vs 采样点 ===
ax = fig.add_subplot(2, 3, 6)
t = np.arange(len(ax_x_arc))
ax.plot(t, ax_x_arc, "r-", linewidth=1, alpha=0.7, label="X")
ax.plot(t, ax_y_arc, "g-", linewidth=1, alpha=0.7, label="Y")
ax.plot(t, ax_z_arc, "b-", linewidth=1, alpha=0.7, label="Z")
ax.set_xlabel("FK sample index"); ax.set_ylabel("Position (m)")
ax.set_title("Actual Position vs Sample")
ax.legend(); ax.grid(True)

plt.tight_layout()
plt.show()