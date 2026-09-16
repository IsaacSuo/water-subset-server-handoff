"""Package independent C1 physics runs; these are not C2 observation-complete episodes."""
from __future__ import annotations

from pathlib import Path

from .io import file_hash, read_json, write_json

ROOT = Path(__file__).resolve().parents[1]


def artifact(root, name, source):
    path = root / name
    return {"status": "available", "path": name, "sha256": file_hash(path),
            "bytes": path.stat().st_size, "source": source, "reason": None}


def package_probe_episode(root):
    root = Path(root)
    config = read_json(root / "resolved_config.json")
    scenario = config["scenarios"][0]
    controller = config["controller"]
    impedance = controller["law"] in ("linear_impedance", "angular_impedance")
    rotary = config["actuator"].get("joint_kind") == "revolute"
    manifest = read_json(ROOT / "configs/dataset/v0_2/examples/effort_pusher_overload.json")
    manifest["episode_id"] = root.name
    # Physics packaged; observations and derived labels belong to later work.
    manifest["lifecycle"] = "draft"
    initial = read_json(root / "initial_state.json")
    ids = set(initial["body_states"])
    manifest["system"]["bodies"] = [b for b in manifest["system"]["bodies"] if b["instance_id"] in ids]
    for body in manifest["system"]["bodies"]:
        if body["instance_id"] == "load":
            body["geometry_id"] = "probe_cube"
        elif body["instance_id"] == "floor":
            body["geometry_id"] = "probe_floor_box"
        elif rotary and body["instance_id"] == "pusher":
            body["geometry_id"] = "probe_rotor_bar"
    joint = manifest["system"]["joints"][0]
    if rotary:
        joint.update(joint_id="rotor_hinge", kind="revolute", axis="Z",
                     lower_limit=config["actuator"]["lower_limit_rad"], upper_limit=config["actuator"]["upper_limit_rad"])
    else:
        joint["upper_limit"] = config["actuator"]["travel_limit_m"]
    manifest["environment"]["environment_id"] = "canonical_" + scenario["id"] + "_lane"
    manifest["environment"]["environment_body_ids"] = [i for i in ("floor", "wall") if i in ids]
    manifest["initial_state"].update(initial)
    manifest["initial_state"]["participant_ids"] = list(initial["body_states"])
    manifest["initial_state"]["state"] = artifact(root, "initial_state.json", "USD native authored t0 state before first simulate")
    manifest["timing"] = config["timing"]
    program = manifest["control_program"]
    primitive = "impedance_control" if impedance else "effort_control"
    program["primitive"] = primitive
    ctrl = program["controllers"][0]
    ctrl.update(primitive=primitive, implementation="dynamic_body_impedance" if impedance else "dynamic_body_effort",
                max_force_n=controller.get("max_force_n"), max_torque_nm=controller.get("max_torque_nm"),
                stiffness_n_m=controller.get("stiffness_n_m"),
                damping_n_s_m=controller.get("damping_n_s_m"))
    if rotary:
        ctrl.update(controller_id="rotor_" + controller["law"],
                    stiffness_nm_rad=controller.get("stiffness_nm_rad"), damping_nm_s_rad=controller.get("damping_nm_s_rad"))
    ctrl["state_trace"] = artifact(root, "actuator_state_trace.jsonl", "PhysX native actuator state at each physics step")
    ctrl["effort_trace"] = artifact(root, "actuator_effort_trace.jsonl", "bounded external force/torque input; not joint reaction sensing")
    command = program["commands"][0]
    command.update(start_time_s=controller["start_time_s"], end_time_s=controller["end_time_s"],
                   controller_id=ctrl["controller_id"], command_id="rotate" if rotary else "push_velocity",
                   target={k: v for k, v in controller.items() if k not in ("start_time_s", "end_time_s", "max_force_n", "max_torque_nm")},
                   limits={k: controller[k] for k in ("max_force_n", "max_torque_nm") if k in controller})
    manifest["implementation"]["post_t0_operations"][0].update(
        kind="apply_torque" if rotary else "apply_force", time_s=controller["start_time_s"],
        reason="bounded feedback effort is applied only to the physical actuator")
    manifest["capabilities"]["actuator_applied_effort"]["source"] = "bounded PhysxForceAPI external force/torque input"
    program["command_trace"] = artifact(root, "command_trace.jsonl", "declared finite controller command")
    trajectory = manifest["trajectory"]
    trajectory["states"] = artifact(root, "body_state_trace.jsonl", "all physical bodies, t0 and every physics step; radians for angular velocity")
    trajectory["contacts"] = artifact(root, "contacts.jsonl", "PhysX native point, normal, vector impulse and separation")
    for cap in manifest["capabilities"].values():
        cap.update(status="native", reason=None)
    manifest["counterfactual"]["family_id"] = "c1_" + controller["law"] + "_load_comparison"
    write_json(root / "episode.json", manifest)
    write_json(root / "source_snapshot.json", {
        name: (ROOT / name).read_text(encoding="utf-8") for name in
        ("world_model_dataset/causal_effort_probe.py", "world_model_dataset/controllers.py",
         "world_model_dataset/probe_episode.py", "configs/dataset/v0_2/examples/effort_pusher_overload.json")
    })
    write_json(root / "packaging_review.json", {
        "milestone": "C1", "scope": "independent physics episode, not a C2 prototype",
        "static_contract": "pending host review; native Isaac Python needs no jsonschema dependency",
        "observation_complete": False,
        "reproduce": {"entrypoint": "world_model_dataset/causal_effort_probe.py",
                      "config": "resolved_config.json", "scenario": scenario["id"],
                      "initial_stage": "probe_initial.usda"},
        "snapshots": {name: artifact(root, name, "reproduction snapshot") for name in
                      ("resolved_config.json", "probe_initial.usda", "probe_final.usda", "probe_report.json")},
        "source_hashes": {name: file_hash(ROOT / name) for name in
                          ("world_model_dataset/causal_effort_probe.py", "world_model_dataset/controllers.py",
                           "world_model_dataset/probe_episode.py")},
    })
