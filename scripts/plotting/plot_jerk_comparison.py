#!/usr/bin/env python3
"""Generate full trajectory dynamics comparison figures for 6 joints."""

from __future__ import annotations

import argparse
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd


def load_csv(path: Path) -> pd.DataFrame:
    if not path.exists():
        raise FileNotFoundError(f"CSV not found: {path}")
    return pd.read_csv(path)


def build_linear_baseline(raw_waypoints: np.ndarray, t_grid: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    n_points, n_joints = raw_waypoints.shape
    total_time = float(t_grid[-1])

    knot_times = np.linspace(0.0, total_time, n_points)

    linear_pos = np.zeros((t_grid.size, n_joints), dtype=float)
    for j in range(n_joints):
        linear_pos[:, j] = np.interp(t_grid, knot_times, raw_waypoints[:, j])

    segment_dt = np.diff(knot_times)
    segment_dt[segment_dt <= 0.0] = 1e-6
    segment_vel = np.diff(raw_waypoints, axis=0) / segment_dt[:, None]

    linear_vel = np.zeros_like(linear_pos)
    for j in range(n_joints):
        for i in range(n_points - 1):
            if i < n_points - 2:
                mask = (t_grid >= knot_times[i]) & (t_grid < knot_times[i + 1])
            else:
                mask = (t_grid >= knot_times[i]) & (t_grid <= knot_times[i + 1])
            linear_vel[mask, j] = segment_vel[i, j]

    # Piecewise linear interpolation has zero acceleration inside each segment.
    linear_acc = np.zeros_like(linear_pos)

    pulse_acc = np.zeros((max(n_points - 2, 0), n_joints), dtype=float)
    if n_points >= 3:
        jump_dt = np.minimum(segment_dt[1:], segment_dt[:-1])
        jump_dt[jump_dt <= 0.0] = 1e-6
        vel_jump = segment_vel[1:, :] - segment_vel[:-1, :]
        pulse_acc = vel_jump / jump_dt[:, None]

    return knot_times, linear_pos, linear_vel, pulse_acc


def _format_axes(ax: plt.Axes, title: str, y_label: str) -> None:
    ax.set_title(title)
    ax.set_xlabel("时间(sec)")
    ax.set_ylabel(y_label)
    ax.grid(True, linestyle=":", alpha=0.45)


def plot_position(
    t_quintic: np.ndarray,
    q_quintic: np.ndarray,
    knot_times: np.ndarray,
    raw_waypoints: np.ndarray,
    output_svg: Path,
) -> None:
    fig, axes = plt.subplots(2, 3, figsize=(18, 9), sharex=True)
    axes = axes.flatten()

    for j in range(6):
        ax = axes[j]
        ax.plot(
            t_quintic,
            q_quintic[:, j],
            color="blue",
            linestyle="-",
            linewidth=2.0,
            label="五次多项式平滑(Quintic)",
        )
        ax.scatter(
            knot_times,
            raw_waypoints[:, j],
            color="black",
            s=22,
            marker="o",
            label="原始路标点(RRT*)",
            zorder=3,
        )
        _format_axes(ax, f"关节{j + 1}", "角度(rad)")
        ax.legend(fontsize=8, loc="best")

    fig.suptitle("位置对比(PositionComparison)", fontsize=15)
    fig.tight_layout()
    fig.savefig(output_svg, format="svg", dpi=300)


def plot_velocity(
    t_quintic: np.ndarray,
    v_quintic: np.ndarray,
    v_linear: np.ndarray,
    output_svg: Path,
) -> None:
    fig, axes = plt.subplots(2, 3, figsize=(18, 9), sharex=True)
    axes = axes.flatten()

    for j in range(6):
        ax = axes[j]
        ax.plot(
            t_quintic,
            v_quintic[:, j],
            color="blue",
            linestyle="-",
            linewidth=2.0,
            label="五次多项式平滑速度(Quintic)",
        )
        ax.step(
            t_quintic,
            v_linear[:, j],
            where="post",
            color="red",
            linestyle="--",
            linewidth=1.6,
            label="线性插值速度(Linear)",
        )
        _format_axes(ax, f"关节{j + 1}", "速度(rad/s)")
        ax.legend(fontsize=8, loc="best")

    fig.suptitle("速度对比(VelocityComparison)", fontsize=15)
    fig.tight_layout()
    fig.savefig(output_svg, format="svg", dpi=300)


def plot_acceleration(
    t_quintic: np.ndarray,
    a_quintic: np.ndarray,
    pulse_times: np.ndarray,
    pulse_acc: np.ndarray,
    output_svg: Path,
) -> None:
    fig, axes = plt.subplots(2, 3, figsize=(18, 9), sharex=True)
    axes = axes.flatten()

    for j in range(6):
        ax = axes[j]
        ax.plot(
            t_quintic,
            np.zeros_like(t_quintic),
            color="red",
            linestyle="--",
            linewidth=1.5,
            label="线性插值加速度(Linear)",
        )

        if pulse_acc.shape[0] > 0:
            ax.vlines(
                pulse_times,
                np.zeros(pulse_acc.shape[0]),
                pulse_acc[:, j],
                colors="red",
                linestyles="--",
                linewidth=2.0,
                alpha=0.95,
            )

        ax.plot(
            t_quintic,
            a_quintic[:, j],
            color="blue",
            linestyle="-",
            linewidth=2.1,
            label="五次多项式平滑加速度(Quintic)",
        )

        _format_axes(ax, f"关节{j + 1}", "加速度(rad/s²)")
        ax.legend(fontsize=8, loc="best")

    fig.suptitle("加速度对比(AccelerationComparison)", fontsize=15)
    fig.tight_layout()
    fig.savefig(output_svg, format="svg", dpi=300)


def main() -> None:
    parser = argparse.ArgumentParser(description="Generate position/velocity/acceleration comparison SVG figures.")
    parser.add_argument("--quintic-csv", type=Path, default=Path("quintic_trajectory_data.csv"))
    parser.add_argument("--raw-csv", type=Path, default=Path("raw_rrt_waypoints.csv"))
    parser.add_argument("--output-dir", type=Path, default=Path("."))
    args = parser.parse_args()

    quintic_df = load_csv(args.quintic_csv)
    raw_df = load_csv(args.raw_csv)

    t_quintic = quintic_df["time_from_start"].to_numpy(dtype=float)
    if t_quintic.size < 2:
        raise ValueError("quintic_trajectory_data.csv must contain at least 2 points")

    q_quintic = np.column_stack([quintic_df[f"pos_j{i}"] for i in range(1, 7)]).astype(float)
    v_quintic = np.column_stack([quintic_df[f"vel_j{i}"] for i in range(1, 7)]).astype(float)
    a_quintic = np.column_stack([quintic_df[f"acc_j{i}"] for i in range(1, 7)]).astype(float)

    raw_waypoints = np.column_stack([raw_df[f"pos_j{i}"] for i in range(1, 7)]).astype(float)
    if raw_waypoints.shape[0] < 2:
        raise ValueError("raw_rrt_waypoints.csv must contain at least 2 waypoints")

    knot_times, _, v_linear, pulse_acc = build_linear_baseline(raw_waypoints, t_quintic)
    pulse_times = knot_times[1:-1]

    output_dir = args.output_dir.resolve()
    output_dir.mkdir(parents=True, exist_ok=True)

    pos_svg = output_dir / "Position_Comparison.svg"
    vel_svg = output_dir / "Velocity_Comparison.svg"
    acc_svg = output_dir / "Acceleration_Comparison.svg"

    plot_position(t_quintic, q_quintic, knot_times, raw_waypoints, pos_svg)
    plot_velocity(t_quintic, v_quintic, v_linear, vel_svg)
    plot_acceleration(t_quintic, a_quintic, pulse_times, pulse_acc, acc_svg)

    print(f"Saved:{pos_svg}")
    print(f"Saved:{vel_svg}")
    print(f"Saved:{acc_svg}")


if __name__ == "__main__":
    main()
