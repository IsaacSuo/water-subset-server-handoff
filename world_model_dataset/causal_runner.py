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


def prepare(config_path, output):
    config = read_json(config_path)
    manifest = read_json(ROOT / config["manifest_template"])
    manifest["episode_id"] = output.name
    manifest["timing"] = config["timing"]
    manifest["counterfactual"]["family_id"] = config["prototype_id"]
    for oid, changes in config["initial_state_overrides"].items():
        manifest["initial_state"]["body_states"][oid].update(changes)
    for body in manifest["system"]["bodies"]:
        body.update(config.get("body_profile_overrides", {}).get(body["instance_id"], {}))
    resolved = {key: copy.deepcopy(config[key]) for key in
                ("geometry_profiles", "physics_profiles", "appearance_profiles", "numerics", "camera_set")}
    resolved["bodies"] = {}
    for body in manifest["system"]["bodies"]:
        resolved["bodies"][body["instance_id"]] = {
            "geometry": resolved["geometry_profiles"][body["geometry_id"]],
            "physics": resolved["physics_profiles"][body["physics_profile_id"]],
            "appearance": resolved["appearance_profiles"][body["appearance_profile_id"]],
        }
    audit = audit_causal_manifest(manifest)
    if not audit["accepted"]:
        raise ValueError(audit["errors"])
    output.mkdir(parents=True, exist_ok=False)
    write_json(output / "input_config.json", config)
    write_json(output / "resolved_inputs.json", resolved)
    manifest["system"]["resolved_inputs"] = artifact(output, "resolved_inputs.json", "resolved independent geometry, physics, appearance, numerics and camera profiles")
    write_json(output / "episode.prepared.json", manifest)
    source_paths = ("world_model_dataset/native_causal_rigid.py", "world_model_dataset/causal_runner.py",
                    "world_model_dataset/causal_loader.py", "world_model_dataset/controllers.py",
                    "world_model_dataset/io.py", "configs/dataset/v0_2/schema.json")
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
    manifest["trajectory"]["states"] = artifact(output, "body_state_trace.jsonl", "all physical bodies at t0 and every explicit physics step")
    manifest["trajectory"]["contacts"] = artifact(output, "contacts.jsonl", "native PhysX point contact reports with vector impulse")
    manifest["capabilities"]["rigid_contact_impulse"] = {
        "status": "native", "source": report["runtime"] + " native contact reports", "reason": None}
    manifest["capabilities"]["rigid_state"] = {"status": "native", "source": report.get("state_source", "PhysX native body state"), "reason": None}
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
