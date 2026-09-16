"""Low-cost paired design videos from causal caches; no physics or sensor claims."""
from __future__ import annotations

import argparse
import html
import json
import subprocess
from collections import defaultdict
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

from .causal_runner import ROOT
from .io import read_json, write_json, file_hash


def load_run(record):
    root = ROOT / record["episode"]
    rows = [json.loads(line) for line in (root / "body_state_trace.jsonl").read_text().splitlines()]
    inputs = read_json(root / "resolved_inputs.json")
    manifest = read_json(root / "episode.physics.json")
    efforts = {}
    if (root / "actuator_effort_trace.jsonl").exists():
        efforts = {r["physics_step"]: r for r in map(json.loads, (root / "actuator_effort_trace.jsonl").read_text().splitlines())}
    return dict(record=record, root=root, rows=rows, inputs=inputs, manifest=manifest, efforts=efforts)


def meshes(run):
    import trimesh
    result = {}
    for oid, body in run["inputs"]["bodies"].items():
        geometry = body["geometry"]
        kind = geometry["shape"]
        if kind == "soft_box":
            first = run["rows"][0]["body_states"][oid]["geometry"]["path"]
            with np.load(run["root"] / first, allow_pickle=False) as data:
                tets = data["simulation_tets"]
                points = data["simulation_world_m"]
                faces = np.concatenate([tets[:, slots] for slots in ((0,1,2),(0,1,3),(0,2,3),(1,2,3))])
                opposite = np.concatenate([tets[:, i] for i in (3,2,1,0)])
                _, indices, counts = np.unique(np.sort(faces, axis=1), axis=0, return_index=True, return_counts=True)
                selected = indices[counts==1]
                faces, opposite = faces[selected].copy(), opposite[selected]
                tri = points[faces]
                inward = np.einsum("ij,ij->i", np.cross(tri[:,1]-tri[:,0], tri[:,2]-tri[:,0]), points[opposite]-tri[:,0]) > 0
                faces[inward] = faces[inward][:, [0,2,1]]
            result[oid] = {"tet_boundary_faces": faces}
            continue
        if kind == "box":
            mesh = trimesh.creation.box(geometry["size_m"])
        elif kind == "sphere":
            mesh = trimesh.creation.icosphere(subdivisions=3, radius=geometry["radius_m"])
        elif kind == "mesh":
            with np.load(run["root"] / geometry["mesh"]["path"], allow_pickle=False) as data:
                mesh = trimesh.Trimesh(data["vertices"], data["triangles"], process=False)
        else:
            raise ValueError(kind)
        colors = np.tile(body["appearance"]["color"], (len(mesh.faces), 1)).astype(float)
        if kind == "sphere":
            # Paint only, fixed to the material frame: makes actual rotation visible.
            colors[mesh.triangles_center[:, 0] > 0] *= 0.45
        result[oid] = (mesh.vertices.copy(), mesh.faces.copy(), colors)
    return result


def world_mesh(run, geometry, row, oid):
    state = row["body_states"][oid]
    if isinstance(geometry[oid], dict):
        with np.load(run["root"] / state["geometry"]["path"], allow_pickle=False) as data:
            vertices, faces = data["simulation_world_m"].copy(), geometry[oid]["tet_boundary_faces"]
        colors = np.tile(run["inputs"]["bodies"][oid]["appearance"]["color"], (len(faces), 1))
        return vertices, faces, colors
    vertices, faces, colors = geometry[oid]
    vertices = Rotation.from_quat(state["orientation_xyzw"]).apply(vertices) + state["position_m"]
    return vertices, faces, colors


def signals(run, kind):
    rows = run["rows"]
    def values(oid, key, axis=0):
        return np.array([r["body_states"][oid][key][axis] for r in rows])
    if kind == "sliding":
        return [("物体速度 m/s", values("left", "linear_velocity_m_s"), "#dd8735")]
    if kind == "rolling":
        radius = run["inputs"]["bodies"]["left"]["geometry"]["radius_m"]
        v = values("left", "linear_velocity_m_s")
        rw = values("left", "angular_velocity_rad_s", 1) * radius
        return [("平移 v (m/s)", v, "#dd8735"), ("转动 rω (m/s)", rw, "#3488bd")]
    if kind == "collision":
        return [("入射球 vx", values("left", "linear_velocity_m_s"), "#dd8735"),
                ("目标球 vx (m/s)", values("right", "linear_velocity_m_s"), "#3488bd")]
    if kind == "compression":
        height = np.array([r["body_states"]["soft"]["metrics"]["height_m"]*1000 for r in rows])
        return [("软体实际高度 mm", height, "#3488bd")]
    if kind in ("falling", "support_edge"):
        return [("质心高度 m",values("left","position_m",2),"#dd8735")]
    if kind == "balance":
        rotations = Rotation.from_quat([r["body_states"]["load"]["orientation_xyzw"] for r in rows]).as_matrix()
        tilt = np.rad2deg(np.arccos(np.clip(rotations[:,2,2],-1,1)))
        return [("物体倾角 °",tilt,"#3488bd")]
    if kind == "collision_chain":
        return [(f"球{i+1} vx (m/s)",values("ball"+str(i),"linear_velocity_m_s"),color)
                for i,color in enumerate(("#dd8735","#3488bd","#33996a","#9562af"))]
    if kind == "soft_confinement":
        return [("软体整体横宽 mm",np.array([r["body_states"]["soft"]["metrics"]["axis_extents_m"][1]*1000 for r in rows]),"#3488bd")]
    if kind == "soft_impact":
        return [("最大局部非刚性位移 mm",np.array([r["body_states"]["soft"]["metrics"]["maximum_local_displacement_m"]*1000 for r in rows]),"#3488bd")]
    return [("物体位移 m", values("load", "position_m")-values("load", "position_m")[0], "#3488bd"),
            ("执行体位移 m", values("pusher", "position_m")-values("pusher", "position_m")[0], "#dd8735")]


def describe(run, row, kind):
    states = row["body_states"]
    effort = run["efforts"].get(row["physics_step"])
    if kind == "compression":
        t = row["time_s"]
        phase = "初始保持/沉降" if t < 1 else "加载" if t < 2 else "保持" if t < 2.75 else "回撤" if t < 3.75 else "恢复"
        text = f"{phase}  |  高度 {states['soft']['metrics']['height_m']*1000:.1f} mm"
    elif kind in ("pushing", "confinement", "balance"):
        text = f"物体速度 {states['load']['linear_velocity_m_s'][0]:.2f} m/s"
    elif kind == "rolling":
        r = run["inputs"]["bodies"]["left"]["geometry"]["radius_m"]
        slip = states["left"]["linear_velocity_m_s"][0] - r*states["left"]["angular_velocity_rad_s"][1]
        text = f"接触点滑移速度 v−rω = {slip:.3f} m/s"
    elif kind == "collision_chain":
        text = "球速："+" / ".join(f"{states['ball'+str(i)]['linear_velocity_m_s'][0]:.2f}" for i in range(4))+" m/s"
    elif kind in ("soft_confinement", "soft_impact"):
        soft = states["soft"]
        text = f"软体 COM x={soft['centre_of_mass_m'][0]:.3f} m"
        if kind=="soft_impact":
            text += f"  |  刚球 vx={states['projectile']['linear_velocity_m_s'][0]:.2f} m/s"
    elif kind in ("falling", "support_edge"):
        text = f"高度 {states['left']['position_m'][2]:.2f} m  |  竖直速度 {states['left']['linear_velocity_m_s'][2]:+.2f} m/s"
    else:
        text = f"物体速度 {states['left']['linear_velocity_m_s'][0]:.2f} m/s"
    if effort:
        text += f"  |  执行器外力 {effort['applied_force_n']:+.1f} N"
        if effort["saturated"]:
            text += "（限幅）"
    return text


def preview_pair(records, destination, rolling_detail=False):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.font_manager import FontProperties, fontManager
    from mpl_toolkits.mplot3d.art3d import Poly3DCollection
    from PIL import Image

    if len(records) != 2:
        raise ValueError("Preview expects two declared conditions, not arbitrary pair inference")
    font = FontProperties(fname="/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc")
    fontManager.addfont(font.get_file())
    plt.rcParams.update({"font.family": font.get_name(), "axes.unicode_minus": False, "font.size": 10})
    runs = [load_run(record) for record in records]
    kind = records[0]["kind"]
    local = [meshes(run) for run in runs]
    dt = runs[0]["manifest"]["timing"]["physics_hz"]
    if any(run["manifest"]["timing"] != runs[0]["manifest"]["timing"] for run in runs[1:]):
        raise ValueError("Paired playback needs aligned durations and physics rates")
    steps = list(range(0, len(runs[0]["rows"]), dt//30))
    if kind == "soft_impact":
        end_focus = round(.6*dt)
        steps = list(range(0,end_focus,2))+list(range(end_focus,len(runs[0]["rows"]),dt//30))
    if rolling_detail:
        if kind != "rolling":
            raise ValueError("Rolling close-up only applies to rolling records")
        steps = list(range(min(len(runs[0]["rows"]),round(.35*dt)+1)))
    bounds, focus_bounds = [], []
    for run, geometry in zip(runs, local):
        for index in steps:
            row = run["rows"][index]
            for body in run["manifest"]["system"]["bodies"]:
                if body["instance_id"] == "floor":
                    continue
                vertices, _, _ = world_mesh(run, geometry, row, body["instance_id"])
                bounds.extend([vertices.min(0), vertices.max(0)])
                if kind == "soft_impact" and row["time_s"] <= .6:
                    focus_bounds.extend([vertices.min(0), vertices.max(0)])
    bounds = np.array(bounds)
    lo, hi = bounds.min(0), bounds.max(0)
    span = hi-lo
    pad = np.maximum(span*.12, .035)
    lo, hi = lo-pad, hi+pad
    lo[2] = -.06 if kind != "compression" else -.02
    # Reserve enough transverse space for shape/readable orientation, without enlarging geometry.
    yspan = max(hi[1]-lo[1], (hi[0]-lo[0])*.4)
    mid_y = (hi[1]+lo[1])/2
    lo[1], hi[1] = mid_y-yspan/2, mid_y+yspan/2
    full_lo, full_hi = lo.copy(), hi.copy()
    if focus_bounds:
        focus_bounds = np.array(focus_bounds)
        focus_lo, focus_hi = focus_bounds.min(0), focus_bounds.max(0)
        focus_pad = np.maximum((focus_hi-focus_lo)*.12,.035)
        focus_lo, focus_hi = focus_lo-focus_pad, focus_hi+focus_pad
        focus_lo[2] = -.06
        focus_yspan = max(focus_hi[1]-focus_lo[1],(focus_hi[0]-focus_lo[0])*.4)
        focus_y = (focus_hi[1]+focus_lo[1])/2
        focus_lo[1], focus_hi[1] = focus_y-focus_yspan/2, focus_y+focus_yspan/2
    fig = plt.figure(figsize=(12.8, 7.2), dpi=100, facecolor="#f6f7f9")
    fig.text(.045, .951, records[0]["design_id"]+"  "+records[0]["title"], fontsize=20, weight="bold", fontproperties=font)
    question = "起始 0.35 秒特写 · 白色标记随球实际旋转 · 每个物理步直接回放" if rolling_detail else records[0]["question"]
    fig.text(.045, .909, question, fontsize=12, color="#4d5966", fontproperties=font)
    clock = fig.text(.95, .948, "", ha="right", fontsize=12, fontproperties=font)
    fig.text(.5, .025, "实际物理缓存 · 软体显示 Tet 外表面、不放大形变 · 地面按观察窗口裁切 · 非正式传感器观测", ha="center", fontsize=10, color="#657181", fontproperties=font)
    views, polys, traces, cursors, infos, markers = [], [], [], [], [], []
    all_signals = [signals(run, kind) for run in runs]
    combined = np.concatenate([values for group in all_signals for _, values, _ in group])
    bottom, top = min(0, combined.min()), combined.max()
    if kind == "compression":
        bottom = combined.min()-8
    margin = max((top-bottom)*.12, .025)
    for column, (run, geometry) in enumerate(zip(runs, local)):
        left = .025 + column*.5
        fig.text(left+.225, .86, records[column]["variant"]["label"], ha="center", fontsize=13, fontproperties=font)
        ax = fig.add_axes([left, .31, .46, .52], projection="3d", computed_zorder=False)
        ax.set_proj_type("ortho")
        ax.set(xlim=(lo[0], hi[0]), ylim=(lo[1], hi[1]), zlim=(lo[2], hi[2]))
        ax.set_box_aspect(hi-lo)
        ax.view_init(elev=42 if kind in ("confinement","soft_confinement") else 18 if kind in ("compression","soft_impact") else 28, azim=-67)
        ax.set_axis_off()
        ax.set_facecolor("#f6f7f9")
        pieces = {}
        for oid in geometry:
            polygon = Poly3DCollection([], linewidths=0,
                                       edgecolor="none", zsort="average", antialiaseds=False)
            # Explicit approximate body order is updated from camera depth below.
            ax.add_collection3d(polygon)
            pieces[oid] = polygon
        views.append(ax)
        polys.append(pieces)
        markers.append(ax.plot([],[],[],marker="o",markersize=10,markerfacecolor="white",markeredgecolor="#111827",linestyle="none",zorder=100)[0] if rolling_detail else None)
        grid_step = .05 if kind=="compression" else .2
        for x in np.arange(np.ceil(lo[0]/grid_step)*grid_step, hi[0], grid_step):
            ax.plot([x,x], [lo[1],hi[1]], [0.0001]*2, color="#b4bdc7", linewidth=.5, zorder=-50)
        for y in np.arange(np.ceil(lo[1]/grid_step)*grid_step, hi[1], grid_step):
            ax.plot([lo[0],hi[0]], [y,y], [0.0001]*2, color="#b4bdc7", linewidth=.5, zorder=-50)
        infos.append(fig.text(left+.23, .295, "", ha="center", fontsize=10, fontproperties=font))
        graph = fig.add_axes([left+.045, .11, .405, .135], facecolor="#ffffff")
        graph.set(xlim=(0, run["rows"][steps[-1]]["time_s"]), ylim=(bottom-margin, top+margin))
        graph.spines[["top", "right"]].set_visible(False)
        graph.grid(alpha=.15)
        graph.set_xlabel("物理时间 / s", fontproperties=font, fontsize=9, labelpad=1)
        lines = [graph.plot([], [], color=color, linewidth=1.8, label=label)[0] for label, _, color in all_signals[column]]
        graph.legend(loc="best", fontsize=8, prop=font, framealpha=.7)
        traces.append(lines)
        cursors.append(graph.axvline(0, color="#8a94a4", linewidth=.7))
    output = destination / (records[0]["design_id"]+".mp4")
    process = subprocess.Popen(["ffmpeg", "-hide_banner", "-loglevel", "error", "-n", "-f", "rawvideo", "-pix_fmt", "rgb24",
                                "-s", "1280x720", "-r", "30", "-i", "-", "-an", "-c:v", "libx264", "-preset", "fast",
                                "-crf", "20", "-pix_fmt", "yuv420p", "-movflags", "+faststart", str(output)], stdin=subprocess.PIPE)
    repeat = 1 if kind in ("compression","soft_confinement","soft_impact") else 2
    light = np.array([-.3, -.5, .8]); light /= np.linalg.norm(light)
    elevation = np.deg2rad(42 if kind in ("confinement","soft_confinement") else 18 if kind in ("compression","soft_impact") else 28)
    azimuth = np.deg2rad(-67)
    camera = np.array([np.cos(elevation)*np.cos(azimuth), np.cos(elevation)*np.sin(azimuth), np.sin(elevation)])
    samples = []
    try:
        for index, step in enumerate(steps):
            time = runs[0]["rows"][step]["time_s"]
            speed_label = "1/16 慢放" if rolling_detail else "原速" if repeat==1 else "0.5× 慢放"
            if kind == "soft_impact" and time < .6:
                speed_label = "0.25× 撞击特写"
            if kind == "soft_impact":
                lo, hi = (focus_lo,focus_hi) if time < .6 else (full_lo,full_hi)
                for ax in views:
                    ax.set(xlim=(lo[0],hi[0]),ylim=(lo[1],hi[1]),zlim=(lo[2],hi[2]))
                    ax.set_box_aspect(hi-lo)
            clock.set_text(f"t = {time:.3f} s  |  {speed_label}")
            for column, (run, geometry) in enumerate(zip(runs, local)):
                row = run["rows"][step]
                for oid, polygon in polys[column].items():
                    vertices, faces, colors = world_mesh(run, geometry, row, oid)
                    if oid == "floor":
                        # Clip only the diagnostic ground drawing, not the cached collider.
                        vertices[:,0] = np.clip(vertices[:,0],lo[0],hi[0])
                        vertices[:,1] = np.clip(vertices[:,1],lo[1],hi[1])
                        colors = np.tile([.85,.87,.9], (len(faces),1))
                    triangles = vertices[faces]
                    normal = np.cross(triangles[:,1]-triangles[:,0], triangles[:,2]-triangles[:,0])
                    normal /= np.maximum(np.linalg.norm(normal, axis=1, keepdims=True), 1e-12)
                    # Cull rear faces to avoid painter-order artifacts on thin convex slabs.
                    visible = normal@camera > 0
                    triangles, normal, colors = triangles[visible], normal[visible], colors[visible]
                    shading = .58 + .42*np.maximum(normal@light, 0)
                    polygon.set_verts(triangles)
                    polygon.set_facecolor(np.clip(colors*shading[:,None], 0, 1))
                    # Floor drawn first; other surfaces use the camera-depth sort key.
                    polygon.set_zorder(-100 if oid=="floor" else float(vertices.mean(0)@camera))
                if rolling_detail:
                    state = row["body_states"]["left"]
                    radius = run["inputs"]["bodies"]["left"]["geometry"]["radius_m"]
                    offset = Rotation.from_quat(state["orientation_xyzw"]).apply(np.array([0,-.8,.6])*radius*1.005)
                    point = offset+state["position_m"]
                    # A material-fixed dot, hidden if it rotates onto the rear hemisphere.
                    markers[column].set_visible(float(offset@camera)>0)
                    markers[column].set_data_3d([point[0]],[point[1]],[point[2]])
                infos[column].set_text(describe(run, row, kind))
                ts = [r["time_s"] for r in run["rows"][:step+1]]
                for line, (_, values, _) in zip(traces[column], all_signals[column]):
                    line.set_data(ts, values[:step+1])
                cursors[column].set_xdata([time, time])
            fig.canvas.draw()
            pixels = np.asarray(fig.canvas.buffer_rgba())[..., :3].copy()
            copies = repeat + (15 if index==0 or index==len(steps)-1 else 0)
            for _ in range(copies):
                process.stdin.write(pixels.tobytes())
            samples.append({"physics_step": step, "time_s": time, "video_frame_repetitions": copies})
            if index in (0, len(steps)//2, len(steps)-1):
                Image.fromarray(pixels).save(destination / f"{records[0]['design_id']}_{index:03d}.png")
            if index%30 == 0:
                print("PREVIEW", records[0]["design_id"], index+1, "/", len(steps), flush=True)
    finally:
        process.stdin.close()
        code = process.wait()
        plt.close(fig)
    if code:
        raise RuntimeError(f"Video encoding failed: {output}")
    return {"design_id": records[0]["design_id"], "title": records[0]["title"], "video": output.name,
            "frames": samples, "source_episodes": [r["episode"] for r in records],
            "source_state_sha256": [file_hash(run["root"] / "body_state_trace.jsonl") for run in runs],
            "duration_s": sum(s["video_frame_repetitions"] for s in samples)/30,
            "actual_trajectories": True, "physics_rerun": False, "sensor_observation": False,
            "rolling_closeup": rolling_detail,
            "display_representation": "rigid geometry; soft simulation-tet boundary; ground drawing clipped to view window; body-fixed hemisphere paint on balls"}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--batch", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--only", help="Render one design before the complete overview")
    parser.add_argument("--rolling-detail", action="store_true", help="Replay every physics step of the first 0.35 s at 1/16 speed")
    args = parser.parse_args()
    if args.rolling_detail:
        args.only = "P02"
    args.output.mkdir(parents=True, exist_ok=True)
    grouped = defaultdict(list)
    for record in read_json(args.batch)["episodes"]:
        # Asset slots may expand; each asset gets its own independent paired preview.
        grouped[(record["design_id"], record["asset_id"])].append(record)
    completed = []
    for (design_id, asset_id), records in grouped.items():
        if args.only and args.only != design_id:
            continue
        folder = args.output / asset_id if asset_id else args.output
        folder.mkdir(exist_ok=True)
        index = folder / (design_id+".json")
        if not index.exists():
            write_json(index, preview_pair(records, folder, args.rolling_detail))
        item = read_json(index)
        item["video"] = (folder / item["video"]).relative_to(args.output).as_posix()
        completed.append(item)
    if args.only:
        return
    # Encoding again avoids container timestamp discontinuities at clip boundaries.
    import tempfile
    with tempfile.TemporaryDirectory(prefix="phenomena_concat_") as tmp:
        listing = Path(tmp) / "clips.txt"
        listing.write_text("".join("file '"+str((args.output/c["video"]).resolve()).replace("'", "'\\''")+"'\n" for c in completed))
        subprocess.run(["ffmpeg", "-hide_banner", "-loglevel", "error", "-n", "-f", "concat", "-safe", "0", "-i", str(listing),
                        "-an", "-c:v", "libx264", "-preset", "fast", "-crf", "20", "-pix_fmt", "yuv420p", "-movflags", "+faststart",
                        str(args.output / "overview.mp4")], check=True)
    write_json(args.output / "index.json", {"scope": "low-cost phenomenon design overview, not training observations", "clips": completed})
    cards = "".join(f'<section><h2>{html.escape(c["design_id"]+" "+c["title"])}</h2><video controls preload="metadata" src="{html.escape(c["video"])}"></video></section>' for c in completed)
    page = '<!doctype html><meta charset="utf-8"><title>物理现象代表实验</title><style>body{font:16px sans-serif;background:#eef1f5;color:#202c39;margin:32px auto;max-width:1100px}video{width:100%}section{background:white;padding:18px;margin:24px 0;border-radius:12px}</style><h1>目标物理现象 · 代表实验</h1><p>每组两条独立 episode。直接回放实际缓存；播放倍率与物理时间在画面中标明。不是正式传感器观测。</p><video controls src="overview.mp4"></video>'+cards
    with (args.output / "index.html").open("x", encoding="utf-8") as stream:
        stream.write(page)
    print("OVERVIEW_COMPLETE", args.output, flush=True)


if __name__ == "__main__":
    main()
