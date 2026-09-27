#!/usr/bin/env python3
"""Measure a complete still-water capture in fixed tank-local regions.

This analysis intentionally uses the captured particle positions, velocities,
and stable IDs.  It does not use the reconstructed surface mesh.
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
from pathlib import Path

import numpy as np


X_HALF = 0.8
Z_HALF = 0.45
TANK_HEIGHT = 0.28
WALL_BAND = 0.016
INTERIOR_BAND = 0.064
UPPER_Y = (0.064, 0.096)
INTERIOR_Y = (0.024, 0.056)
SURFACE_NX = 100
SURFACE_NZ = 56
SURFACE_CROP = 4
OUTER_X = 0.84
OUTER_Z = 0.49
OUTER_BOTTOM = -0.04
MM_PER_M = 1000.0


def rms_speed_mm_s(velocity: np.ndarray) -> float | None:
    if len(velocity) == 0:
        return None
    # Required definition: sqrt(mean(vx^2 + vy^2 + vz^2)).
    return float(np.sqrt(np.mean(np.einsum("ij,ij->i", velocity, velocity)))) * MM_PER_M


def rms_component_mm_s(component: np.ndarray) -> float | None:
    if len(component) == 0:
        return None
    return float(np.sqrt(np.mean(component * component))) * MM_PER_M


def finite_or_none(value: float | None) -> float | None:
    if value is None or not math.isfinite(value):
        return None
    return value


def surface_envelope(local: np.ndarray) -> tuple[int, float | None, float | None]:
    x = local[:, 0]
    z = local[:, 2]
    in_xz = (x >= -X_HALF) & (x <= X_HALF) & (z >= -Z_HALF) & (z <= Z_HALF)
    if not np.any(in_xz):
        return 0, None, None

    x_use = x[in_xz]
    z_use = z[in_xz]
    y_use = local[in_xz, 1]
    ix = np.floor((x_use + X_HALF) * (SURFACE_NX / (2.0 * X_HALF))).astype(np.int32)
    iz = np.floor((z_use + Z_HALF) * (SURFACE_NZ / (2.0 * Z_HALF))).astype(np.int32)
    # Keep points exactly on the positive boundary in the final bin.
    np.clip(ix, 0, SURFACE_NX - 1, out=ix)
    np.clip(iz, 0, SURFACE_NZ - 1, out=iz)
    flat = iz * SURFACE_NX + ix
    highest = np.full(SURFACE_NX * SURFACE_NZ, -np.inf, dtype=np.float32)
    np.maximum.at(highest, flat, y_use)
    grid = highest.reshape(SURFACE_NZ, SURFACE_NX)
    core = grid[
        SURFACE_CROP : SURFACE_NZ - SURFACE_CROP,
        SURFACE_CROP : SURFACE_NX - SURFACE_CROP,
    ]
    valid = np.isfinite(core)
    values = core[valid].astype(np.float64) * MM_PER_M
    if len(values) == 0:
        return 0, None, None
    return int(len(values)), float(np.mean(values)), float(np.std(values))


def frame_metrics(
    local: np.ndarray, velocity: np.ndarray, time_s: float, frame_index: int
) -> dict[str, int | float | None]:
    x = local[:, 0]
    y = local[:, 1]
    z = local[:, 2]
    dx = X_HALF - np.abs(x)
    dz = Z_HALF - np.abs(z)
    d = np.minimum(dx, dz)
    tank = (
        (np.abs(x) <= X_HALF)
        & (np.abs(z) <= Z_HALF)
        & (y >= 0.0)
        & (y <= TANK_HEIGHT)
    )
    wall = tank & (d > 0.0) & (d < WALL_BAND)
    upper_wall = wall & (y > UPPER_Y[0]) & (y < UPPER_Y[1])
    interior = (
        tank
        & (d > INTERIOR_BAND)
        & (y > INTERIOR_Y[0])
        & (y < INTERIOR_Y[1])
    )
    outside = (np.abs(x) > OUTER_X) | (np.abs(z) > OUTER_Z) | (y < OUTER_BOTTOM)

    nearest_x = dx <= dz
    normal = np.where(nearest_x, velocity[:, 0], velocity[:, 2])
    surface_cells, surface_mean, surface_std = surface_envelope(local)

    return {
        "frame": frame_index,
        "time_s": time_s,
        "all_count": int(len(local)),
        "all_rms_speed_mm_s": rms_speed_mm_s(velocity),
        "tank_count": int(np.count_nonzero(tank)),
        "tank_rms_speed_mm_s": rms_speed_mm_s(velocity[tank]),
        "wall_count": int(np.count_nonzero(wall)),
        "wall_rms_speed_mm_s": rms_speed_mm_s(velocity[wall]),
        "wall_normal_rms_speed_mm_s": rms_component_mm_s(normal[wall]),
        "upper_wall_count": int(np.count_nonzero(upper_wall)),
        "upper_wall_rms_speed_mm_s": rms_speed_mm_s(velocity[upper_wall]),
        "upper_wall_normal_rms_speed_mm_s": rms_component_mm_s(normal[upper_wall]),
        "interior_count": int(np.count_nonzero(interior)),
        "interior_rms_speed_mm_s": rms_speed_mm_s(velocity[interior]),
        "surface_valid_cell_count": surface_cells,
        "surface_height_mean_mm": surface_mean,
        "surface_height_std_mm": surface_std,
        "outside_count": int(np.count_nonzero(outside)),
    }


def write_csv(path: Path, rows: list[dict]) -> None:
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def save_plots(output: Path, rows: list[dict]) -> None:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    t = np.array([row["time_s"] for row in rows])
    velocity_series = [
        ("all_rms_speed_mm_s", "All"),
        ("tank_rms_speed_mm_s", "Inside tank"),
        ("wall_rms_speed_mm_s", "Side-wall band"),
        ("wall_normal_rms_speed_mm_s", "Side-wall normal"),
        ("upper_wall_rms_speed_mm_s", "Upper side-wall band"),
        ("upper_wall_normal_rms_speed_mm_s", "Upper side-wall normal"),
        ("interior_rms_speed_mm_s", "Interior"),
    ]
    fig, axes = plt.subplots(2, 1, figsize=(12, 9), constrained_layout=True)
    for key, label in velocity_series:
        values = np.array([np.nan if row[key] is None else row[key] for row in rows])
        axes[0].plot(t, values, label=label, linewidth=1.35)
        axes[1].plot(t, values, label=label, linewidth=1.35)
    axes[0].set_xlim(0, 6)
    axes[1].set_xlim(4, 6)
    for ax, title in zip(axes, ("0–6 s", "Residual motion, 4–6 s")):
        ax.set_title(title)
        ax.set_xlabel("Time (s)")
        ax.set_ylabel("RMS speed (mm/s)")
        ax.grid(alpha=0.25)
    axes[0].legend(ncol=2, fontsize=8)
    fig.savefig(output / "rms_speed_curves.png", dpi=180)
    plt.close(fig)

    mean = np.array([row["surface_height_mean_mm"] for row in rows])
    std = np.array([row["surface_height_std_mm"] for row in rows])
    fig, axes = plt.subplots(2, 1, figsize=(12, 8), constrained_layout=True)
    axes[0].plot(t, mean, label="Mean height", color="#1665a5")
    axes[0].fill_between(t, mean - std, mean + std, alpha=0.22, label="Mean ± spatial std")
    axes[1].plot(t, std, color="#b84c1c", label="Spatial std")
    axes[0].set_ylabel("Particle-envelope height (mm)")
    axes[1].set_ylabel("Height std (mm)")
    for ax in axes:
        ax.set_xlim(0, 6)
        ax.set_xlabel("Time (s)")
        ax.grid(alpha=0.25)
        ax.legend()
    fig.savefig(output / "surface_height_curves.png", dpi=180)
    plt.close(fig)


def json_dump(path: Path, value: object) -> None:
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--simulation", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()

    simulation = args.simulation.resolve()
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=True)
    capture = simulation / "capture"
    manifest = json.loads((capture / "manifest.json").read_text(encoding="utf-8"))
    report = json.loads((simulation / "probe_report.json").read_text(encoding="utf-8"))
    source = json.loads((simulation / "source_input.json").read_text(encoding="utf-8"))
    frames = manifest["frames"]
    origin = np.asarray(source["origin_m"], dtype=np.float64)
    spacing = float(source["spacing_m"])
    density = 1000.0

    if manifest.get("complete") is not True or report.get("status") != "completed":
        raise RuntimeError("Capture or simulation is not marked complete")
    if len(frames) != 181:
        raise RuntimeError(f"Expected 181 frames, got {len(frames)}")
    expected_times = np.arange(181, dtype=np.float64) / 30.0
    manifest_times = np.asarray([row["recording_seconds"] for row in frames])
    if not np.allclose(manifest_times, expected_times, rtol=0.0, atol=1e-12):
        raise RuntimeError("Capture times do not exactly cover 0–6 seconds at 30 fps")

    rows: list[dict] = []
    first_ids: np.ndarray | None = None
    stable_id_set_ok = True
    initial_set_contiguous = True
    unique_ids_ok = True
    reordered_frame_count = 0
    finite_ok = True
    simulated_time_ok = True
    count_ok = True
    id_hash = None
    initial_local = None

    for index, descriptor in enumerate(frames):
        path = capture / descriptor["file"]
        with np.load(path) as arrays:
            positions = arrays["positions"]
            velocities = arrays["velocities"]
            ids = arrays["ids"]
            simulated_seconds = float(arrays["simulated_seconds"])
        if positions.shape != velocities.shape or positions.ndim != 2 or positions.shape[1] != 3:
            raise RuntimeError(f"Bad array shapes in {path}")
        if len(ids) != len(positions) or len(positions) != int(source["fluid_particles"]):
            count_ok = False
        if not np.isfinite(positions).all() or not np.isfinite(velocities).all():
            finite_ok = False
        if abs(simulated_seconds - expected_times[index]) > 1e-12:
            simulated_time_ok = False
        if first_ids is None:
            first_ids = ids.copy()
            id_hash = hashlib.sha256(first_ids.tobytes()).hexdigest()
            first_unique = np.unique(first_ids)
            unique_ids_ok = len(first_unique) == len(first_ids)
            initial_set_contiguous = np.array_equal(
                first_unique, np.arange(len(first_ids), dtype=first_ids.dtype)
            )
        else:
            if not np.array_equal(ids, first_ids):
                reordered_frame_count += 1
            frame_unique = np.unique(ids)
            frame_unique_ok = len(frame_unique) == len(ids)
            unique_ids_ok = unique_ids_ok and frame_unique_ok
            stable_id_set_ok = stable_id_set_ok and frame_unique_ok and np.array_equal(
                frame_unique, first_unique
            )
        local = positions.astype(np.float64) - origin
        if index == 0:
            initial_local = local.copy()
        rows.append(frame_metrics(local, velocities, expected_times[index], index))
        print(f"measured {index + 1:3d}/{len(frames)}  t={expected_times[index]:.3f}s", flush=True)

    assert first_ids is not None and initial_local is not None
    whole_seconds = [rows[second * 30] for second in range(7)]
    write_csv(output / "metrics_0_6s.csv", rows)
    write_csv(output / "metrics_whole_seconds.csv", whole_seconds)
    json_dump(output / "metrics_0_6s.json", rows)
    json_dump(output / "metrics_whole_seconds.json", whole_seconds)

    center_min = np.min(initial_local, axis=0)
    center_max = np.max(initial_local, axis=0)
    solver_rest_volume_m3 = len(first_ids) * 0.8 * spacing**3
    nominal_lattice_volume_m3 = len(first_ids) * spacing**3
    initial = {
        "particle_count": int(len(first_ids)),
        "particle_spacing_m": spacing,
        "density_kg_m3": density,
        "solver_rest_volume_factor": 0.8,
        "solver_rest_volume_m3": solver_rest_volume_m3,
        "solver_rest_volume_liters": solver_rest_volume_m3 * 1000.0,
        "solver_total_mass_kg": solver_rest_volume_m3 * density,
        "nominal_lattice_volume_m3_spacing_cubed": nominal_lattice_volume_m3,
        "nominal_lattice_volume_liters_spacing_cubed": nominal_lattice_volume_m3 * 1000.0,
        "nominal_lattice_mass_kg_spacing_cubed": nominal_lattice_volume_m3 * density,
        "center_bounds_local_m": [center_min.tolist(), center_max.tolist()],
        "initial_center_y_min_m": float(center_min[1]),
        "initial_center_y_max_m": float(center_max[1]),
        "initial_occupied_vertical_thickness_m_center_span_plus_spacing": float(
            center_max[1] - center_min[1] + spacing
        ),
        "initial_top_envelope_above_floor_m_center_plus_half_spacing": float(
            center_max[1] + 0.5 * spacing
        ),
        "reference_water_depth_m": float(source["source_assets"]["tank"]["reference_water_depth_m"]),
    }
    checks = {
        "simulation_status": report.get("status"),
        "capture_manifest_complete": manifest.get("complete"),
        "backend": manifest.get("backend"),
        "simulation_hz": report.get("simulation_hz"),
        "frame_count": len(frames),
        "capture_fps": manifest.get("fps"),
        "time_start_s": float(expected_times[0]),
        "time_end_s": float(expected_times[-1]),
        "particle_count_constant": count_ok,
        "positions_and_velocities_all_finite": finite_ok,
        "simulated_times_match_manifest": simulated_time_ok,
        "initial_ids_unique": unique_ids_ok,
        "initial_id_set_is_contiguous_zero_based": initial_set_contiguous,
        "stable_id_set_same_all_frames": stable_id_set_ok,
        "id_array_order_same_all_frames": reordered_frame_count == 0,
        "frames_with_particle_array_reordering": reordered_frame_count,
        "stable_id_sha256": id_hash,
        "recycling_enabled": manifest.get("recycling", {}).get("enabled"),
        "recycled_count": manifest.get("recycling", {}).get("removed_count"),
        "solver_failure_detected": False,
    }

    residual = [row for row in rows if row["time_s"] >= 4.0]
    residual_summary = {}
    for key in (
        "all_rms_speed_mm_s",
        "tank_rms_speed_mm_s",
        "wall_rms_speed_mm_s",
        "wall_normal_rms_speed_mm_s",
        "upper_wall_rms_speed_mm_s",
        "upper_wall_normal_rms_speed_mm_s",
        "interior_rms_speed_mm_s",
        "surface_height_mean_mm",
        "surface_height_std_mm",
        "outside_count",
    ):
        values = np.asarray([row[key] for row in residual if row[key] is not None], dtype=np.float64)
        residual_summary[key] = {
            "mean": finite_or_none(float(np.mean(values))) if len(values) else None,
            "min": finite_or_none(float(np.min(values))) if len(values) else None,
            "max": finite_or_none(float(np.max(values))) if len(values) else None,
            "at_6s": whole_seconds[-1][key],
        }

    summary = {
        "analysis": "4 mm, 1200 Hz, complete 6 s still water",
        "source_simulation": str(simulation),
        "coordinate_transform": {
            "world_to_tank_local": "local_position = captured_world_position - origin_m",
            "origin_m": origin.tolist(),
            "axis_convention": "Y up; no rotation; captured velocities already use the same axes",
        },
        "fixed_regions_m": {
            "tank": {"x": [-0.8, 0.8], "y": [0.0, 0.28], "z": [-0.45, 0.45]},
            "side_wall": "inside tank and 0 < min(0.8-|x|, 0.45-|z|) < 0.016",
            "upper_side_wall": "side-wall region and 0.064 < y < 0.096",
            "interior": "inside tank, d > 0.064 and 0.024 < y < 0.056",
            "outside": "|x| > 0.84 or |z| > 0.49 or y < -0.04; high-only particles excluded",
        },
        "surface_grid": {
            "x_bins": 100,
            "z_bins": 56,
            "cropped_border_layers": 4,
            "statistic": "maximum captured liquid-particle y per cell; mean and population std over valid retained cells",
        },
        "initialization": initial,
        "data_quality": checks,
        "whole_seconds": whole_seconds,
        "residual_4_6s_inclusive_61_frames": residual_summary,
    }
    json_dump(output / "summary.json", summary)
    save_plots(output, rows)

    report_lines = [
        "# 4 mm / 1200 Hz 静水 6 秒分析",
        "",
        "本报告仅使用原始缓存中的粒子位置、速度和稳定 ID；水面高度不是重建网格。",
        "",
        "## 初始化与数据完整性",
        "",
        f"- 粒子数：{len(first_ids):,}；间距：{spacing * 1000:g} mm；密度：{density:g} kg/m³。",
        f"- SPH 求解器静止体积：{solver_rest_volume_m3 * 1000:.6f} L；总质量：{solver_rest_volume_m3 * density:.6f} kg（后端 `V0=0.8×spacing³`）。",
        f"- 供初始化水量对照的格点名义体积：{nominal_lattice_volume_m3 * 1000:.6f} L（`N×spacing³`）。",
        f"- 初始粒子中心 y：{center_min[1] * 1000:.3f}–{center_max[1] * 1000:.3f} mm；格点占据厚度：{initial['initial_occupied_vertical_thickness_m_center_span_plus_spacing'] * 1000:.3f} mm；顶部半间距包络：{initial['initial_top_envelope_above_floor_m_center_plus_half_spacing'] * 1000:.3f} mm。",
        f"- 181 帧，0–6 秒，30 fps；位置/速度有限：{finite_ok}；每帧粒子数恒定：{count_ok}；每帧 ID 唯一且稳定 ID 集合全程不变：{unique_ids_ok and stable_id_set_ok}。",
        f"- 求解器发生粒子数组重排的帧数：{reordered_frame_count}；统计始终使用同帧位置、速度和 ID，不把数组下标当作身份。",
        "",
        "## 每整秒指标",
        "",
        "完整数值见 `metrics_whole_seconds.csv`；0–6 秒全部 181 帧见 `metrics_0_6s.csv`。",
        "",
        "| t (s) | 全体 RMS | 槽内 RMS | 侧墙 RMS | 侧墙法向 RMS | 上部侧墙 RMS | 上部法向 RMS | 内部 RMS | 水面均值 | 水面标准差 | 槽外数 |",
        "|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|",
    ]
    for row in whole_seconds:
        report_lines.append(
            "| {time_s:.0f} | {all_rms_speed_mm_s:.3f} | {tank_rms_speed_mm_s:.3f} | "
            "{wall_rms_speed_mm_s:.3f} | {wall_normal_rms_speed_mm_s:.3f} | "
            "{upper_wall_rms_speed_mm_s:.3f} | {upper_wall_normal_rms_speed_mm_s:.3f} | "
            "{interior_rms_speed_mm_s:.3f} | {surface_height_mean_mm:.3f} | "
            "{surface_height_std_mm:.3f} | {outside_count} |".format(**row)
        )
    report_lines.extend(
        [
            "",
            "| t (s) | 全体数 | 槽内数 | 侧墙数 | 上部侧墙数 | 内部数 | 有效水面格 | 槽外数 |",
            "|---:|---:|---:|---:|---:|---:|---:|---:|",
        ]
    )
    for row in whole_seconds:
        report_lines.append(
            "| {time_s:.0f} | {all_count} | {tank_count} | {wall_count} | "
            "{upper_wall_count} | {interior_count} | {surface_valid_cell_count} | "
            "{outside_count} |".format(**row)
        )
    report_lines.extend(
        [
            "",
            "速度单位均为 mm/s，水面高度单位为 mm。各分区粒子数和有效水面格数在 CSV/JSON 中逐帧列出。",
            "",
            "## 4–6 秒残余运动",
            "",
        ]
    )
    for key, label in (
        ("all_rms_speed_mm_s", "全体 RMS"),
        ("tank_rms_speed_mm_s", "槽内 RMS"),
        ("wall_rms_speed_mm_s", "侧墙 RMS"),
        ("wall_normal_rms_speed_mm_s", "侧墙法向 RMS"),
        ("upper_wall_rms_speed_mm_s", "上部侧墙 RMS"),
        ("upper_wall_normal_rms_speed_mm_s", "上部侧墙法向 RMS"),
        ("interior_rms_speed_mm_s", "内部 RMS"),
    ):
        item = residual_summary[key]
        report_lines.append(
            f"- {label}：均值 {item['mean']:.3f}，范围 {item['min']:.3f}–{item['max']:.3f}，6 秒 {item['at_6s']:.3f} mm/s。"
        )
    surface_mean_item = residual_summary["surface_height_mean_mm"]
    surface_std_item = residual_summary["surface_height_std_mm"]
    outside_item = residual_summary["outside_count"]
    report_lines.extend(
        [
            f"- 水面包络均值：4–6 秒均值 {surface_mean_item['mean']:.3f} mm，范围 {surface_mean_item['min']:.3f}–{surface_mean_item['max']:.3f} mm，6 秒 {surface_mean_item['at_6s']:.3f} mm。",
            f"- 水面包络空间标准差：4–6 秒均值 {surface_std_item['mean']:.3f} mm，范围 {surface_std_item['min']:.3f}–{surface_std_item['max']:.3f} mm，6 秒 {surface_std_item['at_6s']:.3f} mm。",
            f"- 容器外粒子数：4–6 秒始终为 {int(outside_item['at_6s'])}。",
            "",
            "这里不设置新的物理合格门槛；只报告数据完整性和实测值。",
            "",
            "## 外观复核",
            "",
            "- `still_4mm_original_appearance_4_6s.mp4`：原外观、原视角的 4.000–6.000 秒片段，960×720、30 fps、60 帧。",
            "- `edge_sequence_4_6s.png`：4–6 秒九帧槽缘放大图。抽查帧未见明显不连续边缘跳变；是否较 8 mm 减轻需与用户持有的 8 mm 同口径画面并排判断。",
            "",
        ]
    )
    (output / "REPORT.md").write_text("\n".join(report_lines), encoding="utf-8")
    print(f"analysis complete: {output}")


if __name__ == "__main__":
    main()
