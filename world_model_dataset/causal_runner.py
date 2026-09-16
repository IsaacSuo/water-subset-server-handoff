"""Prepare and run v0.2 causal prototypes; no mutations of frozen v0.1 episodes."""
from __future__ import annotations

import argparse
import copy
import subprocess
from pathlib import Path

from .causal_contract import audit_causal_manifest
from .io import file_hash, read_json, write_json
from .local import windows_path
from .probe_episode import artifact

ROOT = Path(__file__).resolve().parents[1]


def load_config(path):
    """Small variant files inherit a recipe and change only the specified values."""
    def merge(base, changes):
        for key, value in changes.items():
            if isinstance(value, dict) and isinstance(base.get(key), dict):
                merge(base[key], value)
            else:
                base[key] = copy.deepcopy(value)
        return base
    config = read_json(path)
    if "base_config" in config:
        config = merge(load_config(ROOT / config["base_config"]), config.get("overrides", {}))
    return config


def prepare(config_path, output):
    config = load_config(config_path)
    manifest = read_json(ROOT / config["manifest_template"])
    manifest["episode_id"] = output.name
    manifest["timing"] = config["timing"]
    manifest["counterfactual"]["family_id"] = config["prototype_id"]
    for oid, changes in config["initial_state_overrides"].items():
        manifest["initial_state"]["body_states"][oid].update(changes)
    for body in manifest["system"]["bodies"]:
        body.update(config.get("body_profile_overrides", {}).get(body["instance_id"], {}))
    for controller in manifest["control_program"]["controllers"]:
        controller.update(config.get("controller_overrides", {}).get(controller["controller_id"], {}))
        for command in manifest["control_program"]["commands"]:
            if command["controller_id"] == controller["controller_id"]:
                command["limits"]["max_force_n"] = controller["max_force_n"]
    manifest["counterfactual"].update(config.get("counterfactual", {}))
    resolved = {key: copy.deepcopy(config[key]) for key in
                ("geometry_profiles", "physics_profiles", "appearance_profiles", "numerics", "camera_set")}
    resolved["bodies"] = {}
    for body in manifest["system"]["bodies"]:
        resolved["bodies"][body["instance_id"]] = {
            "geometry": copy.deepcopy(resolved["geometry_profiles"][body["geometry_id"]]),
            "physics": resolved["physics_profiles"][body["physics_profile_id"]],
            "appearance": resolved["appearance_profiles"][body["appearance_profile_id"]],
        }
    meshes = {}
    for oid, definition in resolved["bodies"].items():
        geometry = definition["geometry"]
        if geometry["shape"] != "mesh":
            continue
        import numpy as np
        from .geometry import make_geometry
        entry = copy.deepcopy(read_json(ROOT / "configs/dataset/m4_extensions.json")["objects"][geometry["asset_id"]])
        entry["characteristic_size_m"] = geometry["characteristic_size_m"]
        mesh = make_geometry(entry)
        mass = definition["physics"]["mass_kg"]
        inertia = np.asarray(mesh.moment_inertia) * mass / mesh.volume
        geometry.update(source_path=entry["source_path"], source_sha256=entry["source_sha256"],
                        collision_approximation=geometry.get("collision_approximation", entry["rigid_collision"]), bounds_m=mesh.bounds.tolist(),
                        inertia_tensor_kg_m2=inertia.tolist(), rest_volume_m3=float(mesh.volume))
        if oid in config.get("place_on_floor", []):
            initial = manifest["initial_state"]["body_states"][oid]
            from scipy.spatial.transform import Rotation
            rotated = Rotation.from_quat(initial["orientation_xyzw"]).apply(mesh.vertices)
            initial["position_m"][2] = -float(rotated[:,2].min())
        meshes[oid] = mesh
    audit = audit_causal_manifest(manifest)
    if not audit["accepted"]:
        raise ValueError(audit["errors"])
    output.mkdir(parents=True, exist_ok=False)
    for oid, mesh in meshes.items():
        folder = output / "geometry"
        folder.mkdir(exist_ok=True)
        name = "geometry/" + oid + ".npz"
        with (output / name).open("xb") as stream:
            np.savez_compressed(stream, vertices=np.asarray(mesh.vertices,dtype=np.float32),
                                triangles=np.asarray(mesh.faces,dtype=np.int32))
        resolved["bodies"][oid]["geometry"]["mesh"] = artifact(output,name,"existing STL normalized to declared size and centered at mass centroid")
    write_json(output / "input_config.json", config)
    write_json(output / "resolved_inputs.json", resolved)
    manifest["system"]["resolved_inputs"] = artifact(output, "resolved_inputs.json", "resolved independent geometry, physics, appearance, numerics and camera profiles")
    write_json(output / "episode.prepared.json", manifest)
    source_paths = ("world_model_dataset/native_causal_rigid.py", "world_model_dataset/causal_runner.py",
                    "world_model_dataset/causal_loader.py", "world_model_dataset/controllers.py",
                    "world_model_dataset/causal_control.py", "world_model_dataset/causal_soft.py", "soft_body/tet_quality.py",
                    "world_model_dataset/io.py", "world_model_dataset/geometry.py", "configs/dataset/v0_2/schema.json")
    write_json(output / "source_snapshot.json", {
        "files": {name: (ROOT / name).read_text(encoding="utf-8") for name in source_paths},
        "sha256": {name: file_hash(ROOT / name) for name in source_paths},
    })
    return manifest


def invoke(script, output, logname):
    values = [r"Y:\isaacsim\python.bat", windows_path(ROOT / "world_model_dataset" / script),
              "--episode", windows_path(output)]
    command = "& " + " ".join("'" + v.replace("'", "''") + "'" for v in values) + "; exit $LASTEXITCODE"
    with (output / logname).open("x", encoding="utf-8") as stream:
        result = subprocess.run(["powershell.exe", "-NoProfile", "-Command", command], cwd=ROOT,
                                stdout=stream, stderr=subprocess.STDOUT)
    if result.returncode or (output / "native_failure.json").exists():
        raise RuntimeError(f"Native step failed; see {output / logname}")


def package_physics(output):
    manifest = read_json(output / "episode.prepared.json")
    report = read_json(output / "native_report.json")
    initial = read_json(output / "initial_state.json")
    manifest["initial_state"]["body_states"] = initial["body_states"]
    manifest["initial_state"]["state"] = artifact(output, "initial_state.json", "native authored t0 capture after PhysX load, before the first simulate")
    manifest["trajectory"]["states"] = artifact(output, "body_state_trace.jsonl", "all physical bodies at t0 and every explicit physics step; flexible geometry in nested hashed records when present")
    manifest["trajectory"]["contacts"] = artifact(output, "contacts.jsonl", "rigid-only native PhysX point contact reports with vector impulse; flexible pairs are not substituted")
    manifest["capabilities"]["rigid_contact_impulse"] = {
        "status": "native", "source": report["runtime"] + " native contact reports", "reason": None}
    manifest["capabilities"]["rigid_state"] = {"status": "native", "source": report.get("state_source", "PhysX native body state"), "reason": None}
    if any(b["physics_kind"] == "volumetric" for b in manifest["system"]["bodies"]):
        evidence = read_json(output / "capability_probes/soft_contact_impulse.json")
        manifest["capabilities"]["soft_contact_impulse"] = {"status": "unavailable",
            "source": evidence["runtime"] + "; capability_probes/soft_contact_impulse.json sha256=" + file_hash(output / "capability_probes/soft_contact_impulse.json"),
            "reason": evidence["reason"]}
        manifest["capabilities"]["soft_state"] = {"status": "native", "source": "native surface, simulation tet, collision nodes and nodal velocities in per-step hashed geometry records", "reason": None}
        manifest["capabilities"]["soft_material"] = {"status": "native", "source": "native_soft_material_readback.json sha256=" + file_hash(output / "native_soft_material_readback.json"), "reason": None}
        manifest["capabilities"]["soft_angular_velocity"] = {"status": "unavailable", "source": "volume deformable has nodal velocity, not a unique native angular velocity", "reason": "No fabricated rigid angular state for deformed bodies"}
        manifest["capabilities"]["soft_geometric_contact"] = {"status": "derived", "source": "native collision-node signed distance to guided axis-aligned plate", "reason": None}
    if manifest["control_program"]["primitive"] != "none":
        manifest["control_program"]["command_trace"] = artifact(output, "command_trace.jsonl", "declared command and active interval")
        for controller in manifest["control_program"]["controllers"]:
            controller["state_trace"] = artifact(output, "actuator_state_trace.jsonl", "native rigid actuator state at t0 and every step")
            controller["effort_trace"] = artifact(output, "actuator_effort_trace.jsonl", "requested and bounded applied external force over explicit step intervals; derived F*dx work")
        manifest["capabilities"]["actuator_applied_effort"] = {"status": "native", "source": "recorded PhysxForceAPI external input, not a contact reaction measurement", "reason": None}
    write_json(output / "episode.physics.json", manifest)
    audit = audit_causal_manifest(manifest)
    write_json(output / "contract_review.json", audit)
    if not audit["accepted"]:
        raise ValueError(audit["errors"])
    return manifest


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    prepare(args.config, args.output)
    invoke("native_causal_rigid.py", args.output, "simulation.log")
    package_physics(args.output)
    print("CAUSAL_PHYSICS_COMPLETE", args.output, flush=True)


if __name__ == "__main__":
    main()
