"""Blender-side M5A cache replay renderer; never runs or advances physics."""
from __future__ import annotations

import argparse
import hashlib
import json
import math
import sys
import time
from pathlib import Path

import bpy
import numpy as np
from mathutils import Vector


def read(path: Path):
    return json.loads(path.read_text(encoding="utf-8"))


def material(name, color, roughness=0.42, metallic=0.0):
    value = bpy.data.materials.new(name)
    value.use_nodes = True
    nodes = value.node_tree.nodes
    nodes.clear()
    node = nodes.new("ShaderNodeBsdfPrincipled")
    output = nodes.new("ShaderNodeOutputMaterial")
    value.node_tree.links.new(node.outputs["BSDF"], output.inputs["Surface"])
    node.inputs["Base Color"].default_value = (*color, 1.0)
    node.inputs["Roughness"].default_value = roughness
    node.inputs["Metallic"].default_value = metallic
    return value


def look_at(obj, eye, target):
    obj.location = eye
    obj.rotation_euler = (Vector(target) - Vector(eye)).to_track_quat("-Z", "Y").to_euler()


def geometry_paths(frame, object_ids):
    if "geometries" in frame:
        return frame["geometries"]
    if len(object_ids) != 1 or "geometry" not in frame:
        raise ValueError("Cache frame has no unambiguous object geometry mapping")
    return {object_ids[0]: frame["geometry"]}


def select_frames(frames, start_s, end_s, fps):
    times = np.asarray([float(row["time_s"]) for row in frames])
    if start_s < times[0] - 1e-8 or end_s > times[-1] + 1e-8 or end_s <= start_s:
        raise ValueError(f"Slice {start_s}..{end_s}s outside cache {times[0]}..{times[-1]}s")
    output_times = np.arange(start_s, end_s + 0.25 / fps, 1.0 / fps)
    indices = np.searchsorted(times, output_times, side="left")
    indices = np.clip(indices, 0, len(times) - 1)
    previous = np.maximum(indices - 1, 0)
    use_previous = np.abs(times[previous] - output_times) < np.abs(times[indices] - output_times)
    indices[use_previous] = previous[use_previous]
    return [(int(i), float(t)) for i, t in zip(indices, output_times)]


def add_area(name, location, energy, size, color, target):
    data = bpy.data.lights.new(name, "AREA")
    data.energy = energy
    data.shape = "DISK"
    data.size = size
    data.color = color
    obj = bpy.data.objects.new(name, data)
    bpy.context.collection.objects.link(obj)
    look_at(obj, location, target)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, required=True)
    parser.add_argument("--slice-id", required=True)
    parser.add_argument("--episode", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--view", default="hero")
    parser.add_argument("--start-frame", type=int, default=0)
    parser.add_argument("--frame-count", type=int, default=0)
    argv = sys.argv[sys.argv.index("--") + 1 :] if "--" in sys.argv else []
    args = parser.parse_args(argv)

    config = read(args.config)
    selection = next(row for row in config["slices"] if row["id"] == args.slice_id)
    manifest = read(args.episode / "episode.prepared.json")
    validation = read(args.episode / "physics_validation.json")
    if not validation["passed"]:
        raise ValueError("M5A refuses a physics cache that failed physical validation")
    if manifest["spec"]["event_id"] != selection["event_id"]:
        raise ValueError("Slice event and episode manifest disagree")
    state = read(args.episode / "state/index.json")
    fixture = read(args.episode / "fixture.json")
    fps = int(config["output"]["fps"])
    selected = select_frames(state["frames"], float(selection["start_s"]), float(selection["end_s"]), fps)
    if args.start_frame:
        selected = selected[args.start_frame :]
    if args.frame_count:
        selected = selected[: args.frame_count]
    if not selected:
        raise ValueError("No frames selected")
    args.output.mkdir(parents=True, exist_ok=True)

    bpy.ops.wm.read_factory_settings(use_empty=True)
    scene = bpy.context.scene
    # Blender 5 exposes the Eevee Next implementation through the historical
    # BLENDER_EEVEE enum name.
    scene.render.engine = "BLENDER_EEVEE"
    scene.render.resolution_x, scene.render.resolution_y = config["output"]["resolution"]
    scene.render.resolution_percentage = 100
    scene.render.image_settings.file_format = "PNG"
    scene.render.image_settings.color_mode = "RGB"
    scene.render.film_transparent = False
    scene.render.use_file_extension = True
    scene.render.image_settings.color_depth = "8"
    scene.render.image_settings.compression = 35
    scene.render.fps = fps
    scene.render.film_transparent = False
    scene.view_settings.look = "AgX - Medium High Contrast"

    environment = config["environment"]
    scene.world = bpy.data.worlds.new("M5A_world")
    scene.world.use_nodes = True
    world_nodes = scene.world.node_tree.nodes
    world_nodes.clear()
    background = world_nodes.new("ShaderNodeBackground")
    world_output = world_nodes.new("ShaderNodeOutputWorld")
    scene.world.node_tree.links.new(background.outputs["Background"], world_output.inputs["Surface"])
    background.inputs["Color"].default_value = (*environment["world_color"], 1.0)
    background.inputs["Strength"].default_value = 0.32

    floor_mat = material("M5A_floor", environment["floor_color"], roughness=0.7)
    fixture_mat = material("M5A_fixture", environment["fixture_color"], roughness=0.5, metallic=0.08)
    actuator_mat = material("M5A_actuator", environment["actuator_color"], roughness=0.38)
    boxes = {}
    for box in fixture["boxes"]:
        bpy.ops.mesh.primitive_cube_add(size=1.0, location=box["position_m"])
        obj = bpy.context.object
        obj.name = box["id"]
        obj.dimensions = box["size_m"]
        obj.rotation_mode = "QUATERNION"
        q = box["orientation_xyzw"]
        obj.rotation_quaternion = (q[3], q[0], q[1], q[2])
        obj.data.materials.append(
            floor_mat if box["id"] == "floor" else actuator_mat if box["id"] in {"gate", "pusher", "upper_plate", "support"} else fixture_mat
        )
        boxes[box["id"]] = obj

    objects = manifest["inputs"]["objects"]
    object_ids = [row["instance_id"] for row in objects]
    first_paths = geometry_paths(state["frames"][selected[0][0]], object_ids)
    subjects = {}
    palette = config.get("asset_palette", {})
    for ordinal, row in enumerate(objects):
        oid = row["instance_id"]
        with np.load(args.episode / first_paths[oid], allow_pickle=False) as geometry:
            vertices = np.asarray(geometry["surface_world_m"])
            triangles = np.asarray(geometry["surface_triangles"])
        mesh = bpy.data.meshes.new(oid)
        mesh.from_pydata(vertices.tolist(), [], triangles.tolist())
        mesh.update()
        mesh.polygons.foreach_set("use_smooth", [True] * len(mesh.polygons))
        obj = bpy.data.objects.new(oid, mesh)
        bpy.context.collection.objects.link(obj)
        base = palette.get(row["object_id"], row["appearance"]["color"])
        if row["object_id"] == "asset_banana" and ordinal:
            base = [min(1.0, base[0] * 0.78), min(1.0, base[1] * 1.12), min(1.0, base[2] * 0.8)]
        obj.data.materials.append(material("subject_" + oid, base, row["appearance"]["roughness"], row["appearance"]["metallic"]))
        subjects[oid] = obj

    d = float(fixture["D_m"])
    framing = config["event_framing"][selection["event_id"]]
    target = np.asarray(framing["target_D"], dtype=float) * d
    radius = float(framing["radius_D"]) * d
    camera_standard = config["camera_standard"]
    view = camera_standard["views"][args.view]
    direction = np.asarray(framing.get(args.view + "_direction", view["direction"]), dtype=float)
    direction /= np.linalg.norm(direction)
    focal = float(camera_standard["focal_length_mm"])
    sensor = float(camera_standard["sensor_width_mm"])
    width, height = config["output"]["resolution"]
    vertical_fov = 2.0 * math.atan((sensor * height / width) / (2.0 * focal))
    distance = radius * float(camera_standard["safe_margin"]) / math.tan(vertical_fov / 2.0)
    camera_data = bpy.data.cameras.new(args.view)
    camera_data.lens = focal
    camera_data.sensor_width = sensor
    camera_data.dof.use_dof = False
    camera = bpy.data.objects.new(args.view, camera_data)
    bpy.context.collection.objects.link(camera)
    look_at(camera, target + direction * distance, target)
    scene.camera = camera

    lights = environment["lighting"]
    light_scale = max(1.0, 5.0 * radius)
    add_area("key", target + np.asarray([-1.2, -1.5, 2.2]) * light_scale, lights["key_energy_w"], 2.4 * light_scale, (1.0, 0.88, 0.72), target)
    add_area("fill", target + np.asarray([1.7, -0.3, 1.0]) * light_scale, lights["fill_energy_w"], 2.0 * light_scale, (0.65, 0.78, 1.0), target)
    add_area("rim", target + np.asarray([0.2, 1.8, 2.0]) * light_scale, lights["rim_energy_w"], 1.6 * light_scale, (0.78, 0.87, 1.0), target)
    sun_data = bpy.data.lights.new("sun", "SUN")
    sun_data.energy = lights["sun_energy"]
    sun = bpy.data.objects.new("sun", sun_data)
    bpy.context.collection.objects.link(sun)
    sun.rotation_euler = (math.radians(30), math.radians(-20), math.radians(-35))

    rendered = []
    started = time.perf_counter()
    for output_index, (source_index, output_time) in enumerate(selected, start=args.start_frame):
        frame = state["frames"][source_index]
        for oid, relative in geometry_paths(frame, object_ids).items():
            with np.load(args.episode / relative, allow_pickle=False) as geometry:
                points = np.asarray(geometry["surface_world_m"], dtype=np.float64)
            mesh = subjects[oid].data
            if len(mesh.vertices) != len(points):
                raise ValueError(f"Topology changed for {oid} at source frame {source_index}")
            mesh.vertices.foreach_set("co", points.ravel())
            mesh.update()
        for name, position in frame.get("fixture_positions", {}).items():
            if name in boxes:
                boxes[name].location = position
        action = manifest["spec"]["action_parameters"]
        event_id = manifest["spec"]["event_id"]
        if event_id == "R01" and "gate" in boxes:
            boxes["gate"].hide_render = float(frame["time_s"]) > float(action["release_time_s"])
        if event_id == "R05" and "support" in boxes:
            boxes["support"].hide_render = float(frame["time_s"]) >= float(action["remove_time_s"])
        relative = f"frames/{args.view}_{output_index:04d}.png"
        path = args.output / relative
        path.parent.mkdir(exist_ok=True)
        scene.render.filepath = str(path)
        bpy.ops.render.render(write_still=True)
        rendered.append({"output_index": output_index, "output_time_s": output_time, "source_frame": source_index, "source_time_s": frame["time_s"], "path": relative})
        print(f"M5A_RENDER {args.slice_id} {output_index - args.start_frame + 1}/{len(selected)} source={source_index}", flush=True)

    report = {
        "schema_version": "0.1.0",
        "valid": len(rendered) == len(selected),
        "slice_id": args.slice_id,
        "event_id": selection["event_id"],
        "source_episode_id": manifest["spec"]["episode_id"],
        "source_episode_manifest_sha256": hashlib.sha256((args.episode / "episode.prepared.json").read_bytes()).hexdigest(),
        "physics_rerun": False,
        "renderer": "Blender EEVEE Next",
        "view": args.view,
        "fps": fps,
        "resolution": config["output"]["resolution"],
        "selected_source_frames": [row["source_frame"] for row in rendered],
        "render_seconds": time.perf_counter() - started,
        "renders": rendered
    }
    (args.output / "render_report.json").write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print("M5A_RENDER_COMPLETE " + str(args.output), flush=True)


if __name__ == "__main__":
    main()
