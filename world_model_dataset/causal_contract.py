"""Draft v0.2 causal contract and static admission audit.

This module is deliberately separate from the frozen v0.1 contract.  It checks whether a
manifest describes a continuous physical episode; it never launches simulation or edits an
episode directory.
"""
from __future__ import annotations

import argparse
import json
import math
from functools import lru_cache
from pathlib import Path, PurePosixPath

from .io import finite, read_json


ROOT = Path(__file__).resolve().parents[1]
SCHEMA = ROOT / "configs/dataset/v0_2/schema.json"

FORBIDDEN_OPERATIONS = {
    "direct_state_write",
    "set_pose",
    "set_velocity",
    "spawn_body",
    "delete_body",
    "activate_body",
    "deactivate_body",
    "enable_collision",
    "disable_collision",
    "show_body",
    "hide_body",
    "unbounded_kinematic_target",
}


@lru_cache(maxsize=1)
def _validator():
    import jsonschema

    schema = read_json(SCHEMA)
    jsonschema.Draft202012Validator.check_schema(schema)
    return jsonschema.Draft202012Validator(dict(schema, **{"$ref": "#/$defs/manifest"}))


def validate_causal_schema(value):
    """Validate only the structural v0.2 draft schema."""
    finite(value)
    _validator().validate(value)


def _portable_path(value):
    if value is None:
        return True
    path = PurePosixPath(value)
    return bool(value) and "\\" not in value and ":" not in value and not path.is_absolute() and \
        ".." not in path.parts and str(path) == value and value != "."


def _record_errors(name, record):
    status = record["status"]
    payload = (record["path"], record["sha256"], record["bytes"])
    errors = []
    if status == "available":
        if any(value is None for value in payload):
            errors.append(f"{name}: available record requires path, sha256 and bytes")
        if record["reason"] is not None:
            errors.append(f"{name}: available record cannot carry an unavailability reason")
    else:
        if any(value is not None for value in payload):
            errors.append(f"{name}: {status} record cannot claim an artifact payload")
        if status == "unavailable" and record["reason"] is None:
            errors.append(f"{name}: unavailable record requires a reason")
    if not _portable_path(record["path"]):
        errors.append(f"{name}: non-portable artifact path")
    return errors


def audit_causal_manifest(value, require_complete=False):
    """Return a deterministic causal-admission report for one v0.2 manifest.

    Schema errors are raised because the audit cannot safely interpret malformed input.
    Causal violations are returned as failed checks so rejected designs remain inspectable.
    """
    validate_causal_schema(value)
    errors = []
    warnings = []

    bodies = value["system"]["bodies"]
    body_ids = [body["instance_id"] for body in bodies]
    body_by_id = {body["instance_id"]: body for body in bodies}
    if len(body_by_id) != len(body_ids):
        errors.append("system: duplicate body instance_id")
    if set(value["initial_state"]["participant_ids"]) != set(body_ids) or \
            len(value["initial_state"]["participant_ids"]) != len(body_ids):
        errors.append("initial_state: participant_ids must equal the complete system body set")
    state_by_id = value["initial_state"]["body_states"]
    if set(state_by_id) != set(body_ids):
        errors.append("initial_state: body_states must describe the complete system body set")

    environment_ids = value["environment"]["environment_body_ids"]
    if len(set(environment_ids)) != len(environment_ids):
        errors.append("environment: duplicate environment body reference")
    for instance_id in environment_ids:
        if instance_id not in body_by_id or body_by_id[instance_id]["role"] != "environment":
            errors.append(f"environment: {instance_id} is not a declared environment body")

    for instance_id, state in state_by_id.items():
        norm = sum(component * component for component in state["orientation_xyzw"])
        if abs(norm - 1.0) > 1e-6:
            errors.append(f"initial_state/{instance_id}: orientation quaternion is not normalized")
    for body in bodies:
        for field in ("physics_presence", "render_presence"):
            if body[field] != "continuous":
                errors.append(f"system/{body['instance_id']}: {field} is not continuous")
        if body["collision_participation"] == "time_varying":
            errors.append(f"system/{body['instance_id']}: collision participation changes after t0")

    joints = value["system"]["joints"]
    joint_ids = [joint["joint_id"] for joint in joints]
    if len(set(joint_ids)) != len(joint_ids):
        errors.append("system: duplicate joint_id")
    for joint in joints:
        for field in ("body0_id", "body1_id"):
            body_id = joint[field]
            if body_id is not None and body_id not in body_by_id:
                errors.append(f"joint/{joint['joint_id']}: unknown {field} {body_id}")
        if joint["kind"] == "fixed":
            if any(joint[field] is not None for field in ("axis", "lower_limit", "upper_limit")):
                errors.append(f"joint/{joint['joint_id']}: fixed joint cannot declare an axis or limits")
        else:
            if joint["axis"] is None or joint["lower_limit"] is None or joint["upper_limit"] is None:
                errors.append(f"joint/{joint['joint_id']}: movable joint requires axis and finite limits")
            elif joint["lower_limit"] >= joint["upper_limit"]:
                errors.append(f"joint/{joint['joint_id']}: lower limit must be below upper limit")

    timing = value["timing"]
    if timing["physics_hz"] % timing["capture_hz"]:
        errors.append("timing: capture_hz must divide physics_hz")
    for rate in (timing["physics_hz"], timing["capture_hz"]):
        if abs(timing["duration_s"] * rate - round(timing["duration_s"] * rate)) > 1e-8:
            errors.append(f"timing: duration_s does not land on the {rate} Hz grid")

    lineage = value["lineage"]
    if value["initial_state"]["source"] != lineage["derivation"]:
        errors.append("lineage: initial-state source and derivation disagree")
    if lineage["derivation"] == "fresh_simulation":
        if lineage["parents"]:
            errors.append("lineage: fresh simulation cannot declare parent caches")
    else:
        if len(lineage["parents"]) != 1:
            errors.append("lineage: a derived crop requires exactly one parent cache")
        for parent in lineage["parents"]:
            if parent["crop_start_time_s"] <= 0:
                errors.append("lineage: derived crop must begin after the parent time origin")
            if parent["crop_end_time_s"] <= parent["crop_start_time_s"]:
                errors.append("lineage: crop end must follow crop start")
            if not _portable_path(parent["manifest_path"]):
                errors.append("lineage: parent manifest path is not portable")

    implementation = value["implementation"]
    if implementation["state_update_authority"] != "solver_only":
        errors.append("implementation: subject state is not solver-only after t0")
    for field in ("participant_set", "collision_participation", "render_presence"):
        if implementation[field] != "constant":
            errors.append(f"implementation: {field} changes after t0")
    for operation in implementation["post_t0_operations"]:
        if operation["kind"] in FORBIDDEN_OPERATIONS:
            errors.append(
                f"implementation: forbidden {operation['kind']} on {operation['target']} "
                f"at {operation['time_s']:.9g}s"
            )
        if operation["time_s"] > timing["duration_s"]:
            errors.append(f"implementation: operation after episode end ({operation['time_s']:.9g}s)")

    program = value["control_program"]
    controllers = program["controllers"]
    controller_ids = [item["controller_id"] for item in controllers]
    controller_by_id = {item["controller_id"]: item for item in controllers}
    if len(controller_by_id) != len(controller_ids):
        errors.append("control_program: duplicate controller_id")
    command_ids = [item["command_id"] for item in program["commands"]]
    if len(set(command_ids)) != len(command_ids):
        errors.append("control_program: duplicate command_id")

    if program["primitive"] == "none":
        if controllers or program["commands"]:
            errors.append("control_program: none cannot contain controllers or commands")
        if program["command_trace"]["status"] != "not_applicable":
            errors.append("control_program: none requires a not_applicable command trace")
    else:
        if not controllers or not program["commands"]:
            errors.append("control_program: controlled episode requires a controller and command")
        if program["command_trace"]["status"] in ("unavailable", "not_applicable"):
            errors.append("control_program: commanded control requires a command trace")

    expected_implementation = {
        "effort_control": {"dynamic_body_effort", "joint_effort"},
        "impedance_control": {"dynamic_body_impedance", "joint_impedance"},
        "field_control": {"continuous_field"},
    }
    for controller in controllers:
        cid = controller["controller_id"]
        primitive = controller["primitive"]
        if primitive != program["primitive"]:
            errors.append(f"controller/{cid}: primitive differs from the control program")
        if controller["implementation"] not in expected_implementation.get(primitive, set()):
            errors.append(f"controller/{cid}: implementation is not admitted for {primitive}")
        if not controller["reaction_observable"]:
            errors.append(f"controller/{cid}: physical reaction is not observable")
        if primitive == "field_control":
            if controller["actuator_instance_id"] is not None:
                errors.append(f"controller/{cid}: field control cannot name a rigid actuator body")
        else:
            actuator = controller["actuator_instance_id"]
            if actuator not in body_by_id or body_by_id.get(actuator, {}).get("role") != "actuator":
                errors.append(f"controller/{cid}: actuator is not a declared actuator body")
            if controller["max_force_n"] is None and controller["max_torque_nm"] is None:
                errors.append(f"controller/{cid}: no finite force or torque authority is declared")
        if primitive == "impedance_control":
            linear = (controller["max_force_n"] is not None and bool(controller["stiffness_n_m"]) and
                      controller["damping_n_s_m"] is not None)
            angular = (controller["max_torque_nm"] is not None and bool(controller.get("stiffness_nm_rad")) and
                       controller.get("damping_nm_s_rad") is not None)
            if not (linear or angular):
                errors.append(f"controller/{cid}: impedance gains are incomplete")
        for field in ("state_trace", "effort_trace"):
            if controller[field]["status"] in ("unavailable", "not_applicable"):
                errors.append(f"controller/{cid}: {field} must be recorded")

    dt = 1.0 / timing["physics_hz"]
    for command in program["commands"]:
        cid = command["controller_id"]
        if cid not in controller_by_id:
            errors.append(f"command/{command['command_id']}: unknown controller {cid}")
        if command["end_time_s"] <= command["start_time_s"]:
            errors.append(f"command/{command['command_id']}: end time must follow start time")
        if command["end_time_s"] > timing["duration_s"]:
            errors.append(f"command/{command['command_id']}: command extends beyond episode")
        for field in ("start_time_s", "end_time_s"):
            value_s = command[field]
            if abs(value_s / dt - round(value_s / dt)) > 1e-8:
                errors.append(f"command/{command['command_id']}: {field} is off the physics grid")
        if not command["limits"]:
            errors.append(f"command/{command['command_id']}: command has no explicit limits")

    actuator_ids = {body["instance_id"] for body in bodies if body["role"] == "actuator"}
    for operation in implementation["post_t0_operations"]:
        kind, target = operation["kind"], operation["target"]
        if kind in ("apply_force", "apply_torque") and target not in actuator_ids:
            errors.append(f"implementation: {kind} may target only a declared actuator body")
        if kind == "controller_command" and target not in controller_by_id:
            errors.append(f"implementation: controller command targets unknown controller {target}")
        if kind == "field_update" and (target not in controller_by_id or
                                       controller_by_id[target]["primitive"] != "field_control"):
            errors.append(f"implementation: field update targets non-field controller {target}")

    records = [("initial_state/state", value["initial_state"]["state"]),
               ("control_program/command_trace", program["command_trace"])]
    if "resolved_inputs" in value["system"]:
        records.append(("system/resolved_inputs", value["system"]["resolved_inputs"]))
    records.extend((f"controller/{controller['controller_id']}/{field}", controller[field])
                   for controller in controllers for field in ("state_trace", "effort_trace"))
    records.extend((f"trajectory/{name}", record) for name, record in value["trajectory"].items())
    for name, record in records:
        errors.extend(_record_errors(name, record))

    counterfactual = value["counterfactual"]
    paired = counterfactual["baseline_episode_id"] is not None
    if paired != (counterfactual["changed_pointer"] is not None):
        errors.append("counterfactual: baseline and changed pointer must be declared together")

    if require_complete:
        if value["lifecycle"] != "completed":
            errors.append("completion: lifecycle is not completed")
        required_records = [value["initial_state"]["state"], value["trajectory"]["states"],
                            value["trajectory"]["observations"],
                            value["trajectory"]["interaction_annotations"],
                            value["trajectory"]["outcomes"]]
        if program["primitive"] != "none":
            required_records.append(program["command_trace"])
            required_records.extend(controller[field] for controller in controllers
                                    for field in ("state_trace", "effort_trace"))
        if any(record["status"] != "available" for record in required_records):
            errors.append("completion: required state, control, observation or outcome records are unavailable")

    if value["lifecycle"] == "rejected" and not errors:
        warnings.append("lifecycle is rejected although the static causal audit found no violation")

    return {
        "schema_version": "0.2.0-draft",
        "episode_id": value["episode_id"],
        "accepted": not errors,
        "checks": {
            "schema": "pass",
            "causal_continuity": "pass" if not errors else "fail",
            "complete_records": "checked" if require_complete else "not_requested",
        },
        "errors": errors,
        "warnings": warnings,
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("manifest")
    parser.add_argument("--require-complete", action="store_true")
    args = parser.parse_args()
    report = audit_causal_manifest(read_json(args.manifest), require_complete=args.require_complete)
    print(json.dumps(report, ensure_ascii=False, indent=2))
    if not report["accepted"]:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
