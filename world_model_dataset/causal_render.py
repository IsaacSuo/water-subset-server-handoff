"""Two-view v0.2 RTX sensor replay; reconstruct all bodies and never advance physics."""
from __future__ import annotations

import argparse
import json
import sys
import traceback
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from world_model_dataset.io import file_hash, read_json, write_json


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--episode", type=Path, required=True)
    args = parser.parse_args()
    output = args.episode
    manifest = read_json(output / "episode.physics.json")
    resolved = read_json(output / "resolved_inputs.json")
    frames = [json.loads(line) for line in (output / "body_state_trace.jsonl").read_text(encoding="utf-8").splitlines()]
    stride = manifest["timing"]["physics_hz"] // manifest["timing"]["capture_hz"]
    frames = [row for row in frames if row["physics_step"] % stride == 0]
    destination = output / "observations"
    destination.mkdir(exist_ok=False)
    from isaacsim import SimulationApp
    camera_set = resolved["camera_set"]
    width, height = camera_set["resolution"]
    app = SimulationApp({"headless": True, "renderer": "RayTracedLighting", "width": width, "height": height})
    try:
        import numpy as np
        import omni.usd
        import omni.replicator.core as rep
        from PIL import Image
        from pxr import Gf, Sdf, UsdGeom, UsdLux, UsdShade, Semantics

        rep.orchestrator.set_capture_on_play(False)
        omni.usd.get_context().new_stage()
        stage = omni.usd.get_context().get_stage()
        UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
        UsdGeom.SetStageMetersPerUnit(stage, 1.0)
        UsdGeom.Xform.Define(stage, "/World")
        # Fresh render stage: no PhysicsScene, rigid-body, collider, or deformable APIs.
        dome = UsdLux.DomeLight.Define(stage, "/World/Light")
        dome.CreateIntensityAttr(700.0)
        key = UsdLux.DistantLight.Define(stage, "/World/Key")
        key.CreateIntensityAttr(1600.0)
        key.AddRotateXYZOp().Set(Gf.Vec3f(-35, -25, -15))
        body_ids = {body["instance_id"]: index + 1 for index, body in enumerate(manifest["system"]["bodies"])}
        operations = {}
        for oid, definition in resolved["bodies"].items():
            geometry = definition["geometry"]
            if geometry["shape"] == "sphere":
                shape = UsdGeom.Sphere.Define(stage, "/World/" + oid)
                shape.CreateRadiusAttr(geometry["radius_m"])
            elif geometry["shape"] == "box":
                shape = UsdGeom.Cube.Define(stage, "/World/" + oid)
                shape.CreateSizeAttr(1.0)
            else:
                raise ValueError(f"No renderer for {geometry['shape']}")
            translate = shape.AddTranslateOp()
            orient = shape.AddOrientOp()
            if geometry["shape"] == "box":
                shape.AddScaleOp().Set(Gf.Vec3f(*geometry["size_m"]))
            operations[oid] = (translate, orient)
            semantics = Semantics.SemanticsAPI.Apply(shape.GetPrim(), "Semantics")
            semantics.CreateSemanticTypeAttr("class")
            semantics.CreateSemanticDataAttr(oid)
            appearance = definition["appearance"]
            material = UsdShade.Material.Define(stage, "/World/Appearance_" + oid)
            shader = UsdShade.Shader.Define(stage, "/World/Appearance_" + oid + "/Shader")
            shader.CreateIdAttr("UsdPreviewSurface")
            shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*appearance["color"]))
            shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(appearance["roughness"])
            shader.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(appearance["metallic"])
            material.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")
            UsdShade.MaterialBindingAPI.Apply(shape.GetPrim()).Bind(material)

        streams = {}
        calibration = {}
        for camera in camera_set["cameras"]:
            name = camera["id"]
            cam = UsdGeom.Camera.Define(stage, "/World/Camera_" + name)
            matrix = Gf.Matrix4d().SetLookAt(Gf.Vec3d(*camera["position_m"]),
                                          Gf.Vec3d(*camera["target_m"]), Gf.Vec3d(0, 0, 1)).GetInverse()
            cam.AddTransformOp().Set(matrix)
            cam.CreateFocalLengthAttr(camera_set["focal_length_mm"])
            cam.CreateHorizontalApertureAttr(camera_set["horizontal_aperture_mm"])
            cam.CreateVerticalApertureAttr(camera_set["horizontal_aperture_mm"] * height / width)
            cam.CreateClippingRangeAttr(Gf.Vec2f(0.01, 100.0))
            product = rep.create.render_product(str(cam.GetPath()), (width, height))
            annotators = {name: rep.AnnotatorRegistry.get_annotator(name) for name in
                          ("rgb", "distance_to_image_plane", "semantic_segmentation")}
            for annotator in annotators.values():
                annotator.attach([product])
            streams[name] = annotators
            (destination / name).mkdir()
            focal_px = width * camera_set["focal_length_mm"] / camera_set["horizontal_aperture_mm"]
            calibration[name] = {
                "world_from_camera_usd": np.asarray(matrix).tolist(), "matrix_convention": "USD row-vector",
                "intrinsic_opencv": [[focal_px, 0, (width - 1) / 2], [0, focal_px, (height - 1) / 2], [0, 0, 1]],
                "pixel_centres": "integer coordinates; optical centre is ((width-1)/2, (height-1)/2)",
                "optical_axis_conversion": "USD (x,y,z) to OpenCV (x,-y,-z)", "resolution": [width, height],
            }
        outputs = []
        for index, frame in enumerate(frames):
            if set(frame["body_states"]) != set(operations):
                raise ValueError("Physics and render participant sets differ")
            for oid, value in frame["body_states"].items():
                translate, orient = operations[oid]
                translate.Set(Gf.Vec3d(*value["position_m"]))
                q = value["orientation_xyzw"]
                orient.Set(Gf.Quatf(q[3], Gf.Vec3f(*q[:3])))
            rep.orchestrator.step(rt_subframes=1, delta_time=0.0, pause_timeline=True)
            if index == 0:
                rep.orchestrator.step(rt_subframes=1, delta_time=0.0, pause_timeline=True)
            for name, annotations in streams.items():
                rgb = np.asarray(annotations["rgb"].get_data())
                depth = np.asarray(annotations["distance_to_image_plane"].get_data())
                semantic = annotations["semantic_segmentation"].get_data()
                if rgb.shape[:2] != (height, width) or depth.shape != (height, width):
                    raise ValueError("Missing or misaligned RGB-D")
                raw_seg = np.asarray(semantic["data"], dtype=np.uint32)
                segmentation = np.zeros(raw_seg.shape, dtype=np.uint16)
                for raw_id, labels in semantic["info"]["idToLabels"].items():
                    oid = labels.get("class")
                    if oid in body_ids:
                        segmentation[raw_seg == int(raw_id)] = body_ids[oid]
                valid = np.isfinite(depth) & (depth > 0) & (depth < 100)
                rgb_name = f"{name}/frame_{index:04d}.png"
                data_name = f"{name}/frame_{index:04d}.npz"
                Image.fromarray(rgb[..., :3].astype(np.uint8)).save(destination / rgb_name)
                with (destination / data_name).open("xb") as stream:
                    np.savez_compressed(stream, depth_m=np.where(valid, depth, 0).astype(np.float32),
                                        depth_valid=valid, segmentation=segmentation, time_s=frame["time_s"],
                                        physics_step=frame["physics_step"])
                outputs.append({"time_s": frame["time_s"], "physics_step": frame["physics_step"],
                                "camera_id": name, "rgb": rgb_name, "data": data_name,
                                "rgb_sha256": file_hash(destination / rgb_name), "data_sha256": file_hash(destination / data_name)})
            print(f"CAUSAL_OBSERVATION {index+1}/{len(frames)}", flush=True)
        stage.GetRootLayer().Export(str(destination / "replay_final.usda"))
        write_json(destination / "source_snapshot.json", {
            "renderer": Path(__file__).read_text(encoding="utf-8"), "renderer_sha256": file_hash(Path(__file__)),
            "source_state_sha256": file_hash(output / "body_state_trace.jsonl"),
            "resolved_inputs_sha256": file_hash(output / "resolved_inputs.json"),
        })
        write_json(destination / "index.json", {
            "schema_version": "0.2.0-draft", "complete": True, "frames": outputs,
            "cameras": calibration, "body_instance_ids": body_ids,
            "physics_rerun": False, "render_backend": "Isaac RTX RayTracedLighting",
            "depth_units": "metres along the OpenCV optical z axis", "segmentation": "stable body instance ID; zero is background",
            "source_state_sha256": file_hash(output / "body_state_trace.jsonl"),
            "geometry_representation": "USD primitives tessellated by RTX; visible sphere triangles can differ from the exact PhysX sphere collider",
        })
        print("CAUSAL_OBSERVATIONS_COMPLETE", output, flush=True)
    except Exception as exc:
        traceback.print_exc()
        write_json(destination / "failure.json", {"error": f"{type(exc).__name__}: {exc}", "traceback": traceback.format_exc()})
        raise
    finally:
        app.close()


if __name__ == "__main__":
    main()
