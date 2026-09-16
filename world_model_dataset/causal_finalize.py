"""Finalize a manually reviewed causal smoke candidate, not an entire C2 matrix or release."""
from __future__ import annotations

import argparse
import copy
from pathlib import Path

from .causal_contract import audit_causal_manifest
from .causal_loader import open_episode
from .io import file_hash, read_json, write_json
from .probe_episode import artifact


def finalize_smoke(output):
    output = Path(output)
    review = read_json(output / "human_review.json")
    if not review["physics_accepted"] or not review["observations_accepted"]:
        raise ValueError("This candidate has not received manual physics and observation acceptance")
    if (output / "episode.json").exists():
        raise FileExistsError("Never overwrite a finalized manifest")
    episode = open_episode(output, require_complete=False)
    manifest = copy.deepcopy(episode.manifest)
    primitive = manifest["control_program"]["primitive"]
    if primitive not in ("none", "effort_control", "impedance_control"):
        raise ValueError("Unsupported causal smoke control primitive")
    states = list(episode.states())
    contacts = list(episode.contacts())
    raw_index = read_json(output / "observations/index.json")
    calibrated = copy.deepcopy(raw_index)
    for camera in calibrated["cameras"].values():
        if "pixel_centres" not in camera:
            # The first local capture used edge-origin intrinsics. Preserve raw metadata.
            camera["intrinsic_opencv"][0][2] -= 0.5
            camera["intrinsic_opencv"][1][2] -= 0.5
            camera["pixel_centres"] = "integer coordinates; corrected from edge-origin calibration by -0.5 principal-point pixels"
    calibrated["calibration_provenance"] = {
        "raw_index_sha256": file_hash(output / "observations/index.json"),
        "review": "Integer-pixel convention retained or legacy edge-origin corrected; see human_review.json for actual sensor checks",
    }
    calibrated.setdefault("geometry_representation", "USD spheres tessellated by RTX; render depth is visible triangulated geometry, not exact analytical collider depth")
    write_json(output / "observations/index.calibrated.json", calibrated)
    manifest["trajectory"]["observations"] = artifact(output, "observations/index.calibrated.json", "native cache-only RGB-D and segmentation with reviewed integer-pixel camera calibration")
    subject_ids = {b["instance_id"] for b in manifest["system"]["bodies"] if b["role"] == "subject"}
    actuator_ids = {b["instance_id"] for b in manifest["system"]["bodies"] if b["role"] == "actuator"}
    soft_ids = {b["instance_id"] for b in manifest["system"]["bodies"] if b["physics_kind"] == "volumetric"}
    # Preserve actual actor pairs. A chain contact must not label all subjects as
    # simultaneously touching; environment contacts matter for support changes.
    contact_pairs = {}
    for row in contacts:
        if sum(v*v for v in row['impulse_ns']) > 0:
            contact_pairs.setdefault(tuple(sorted(row['actor_ids'])), []).append(row)
    labels = ([{"label": "free_motion", "start_time_s": 0, "end_time_s": states[-1]["time_s"],
                "source": "native state under fixed environment and no control"}] if primitive == "none" else [])
    for pair, impact_points in sorted(contact_pairs.items()):
        # Preserve distinct runs rather than inventing contact across an observed gap.
        steps = sorted({r["physics_step"] for r in impact_points})
        runs = [[steps[0]]]
        for step in steps[1:]:
            if step == runs[-1][-1] + 1:
                runs[-1].append(step)
            else:
                runs.append([step])
        for run in runs:
            pair_ids = set(pair)
            label = ("actuator_subject_contact" if pair_ids & actuator_ids and pair_ids & subject_ids else
                     "transient_impact" if len(pair_ids & subject_ids) == 2 else "environment_contact")
            labels.append({"label": label,
                "start_time_s": run[0] / manifest["timing"]["physics_hz"],
                "end_time_s": run[-1] / manifest["timing"]["physics_hz"],
                "body_ids": list(pair), "source": "nonzero native pair contact impulse reports; sampled report runs, not exact continuous contact intervals"})
    for oid in sorted(soft_ids):
        steps = [row["physics_step"] for row in states if row["body_states"][oid]["metrics"]["geometric_contact"]]
        runs = []
        for step in steps:
            if runs and step == runs[-1][-1] + 1:
                runs[-1].append(step)
            else:
                runs.append([step])
        for run in runs:
            labels.append({"label": "sampled_actuator_soft_contact", "start_time_s": run[0]/manifest["timing"]["physics_hz"],
                "end_time_s": run[-1]/manifest["timing"]["physics_hz"], "body_ids": sorted({oid}|actuator_ids),
                "source": "derived native collision-node signed distance to guided plate within configured contact offset; not a solver contact report"})
    write_json(output / "interaction_annotations.json", {"labels": labels, "source_contact_sha256": file_hash(output / "contacts.jsonl")})
    initial, final = states[0]["body_states"], states[-1]["body_states"]
    def momentum_x(frame):
        return sum(frame[oid]["mass_kg"] * frame[oid]["linear_velocity_m_s"][0] for oid in subject_ids)
    def horizontal_ke(frame):
        return sum(0.5 * frame[oid]["mass_kg"] * sum(v*v for v in frame[oid]["linear_velocity_m_s"][:2]) for oid in subject_ids)
    outcomes = {
        "duration_s": states[-1]["time_s"], "initial_momentum_x_kg_m_s": momentum_x(initial),
        "final_momentum_x_kg_m_s": momentum_x(final),
        "horizontal_kinetic_energy_ratio": horizontal_ke(final) / horizontal_ke(initial) if horizontal_ke(initial) else None,
        "final_subject_states": {oid: final[oid] for oid in sorted(subject_ids)},
        "right_censored": any(sum(v*v for v in final[oid]["linear_velocity_m_s"]) > 0.0001 for oid in subject_ids),
        "source": "derived native-state diagnostics at the recorded horizon; not final resting outcome",
    }
    if primitive != "none":
        efforts = list(episode.actuator_efforts())
        active = [row for row in efforts if row["command_active"]]
        outcomes["actuator_control"] = {
            "controller_id": manifest["control_program"]["controllers"][0]["controller_id"],
            "work_j": efforts[-1]["cumulative_work_j"],
            "peak_applied_force_n": max(abs(row["applied_force_n"]) for row in efforts),
            "active_saturated_fraction": sum(row["saturated"] for row in active) / len(active) if active else None,
            "final_actuator_states": {oid: final[oid] for oid in sorted(actuator_ids)},
            "source": "recorded external force inputs and derived signed F*dx; not contact work or contact-force truth",
        }
        outcomes["subject_displacements_m"] = {oid: [final[oid].get("centre_of_mass_m",final[oid]["position_m"])[i] -
                                                        initial[oid].get("centre_of_mass_m",initial[oid]["position_m"])[i]
                                                        for i in range(3)] for oid in sorted(subject_ids)}
    if soft_ids:
        loading_start = next((c["start_time_s"] for c in manifest["control_program"]["commands"]
                              if c["target"].get("trajectory") == "smoothstep"),0.0)
        baseline_row = max((row for row in states if row["time_s"] <= loading_start),key=lambda row:row["time_s"])
        outcomes["soft_deformation"] = {}
        for oid in sorted(soft_ids):
            metrics = [row["body_states"][oid]["metrics"] for row in states]
            before, after = baseline_row["body_states"][oid]["metrics"],metrics[-1]
            minimum_height = min(m["height_m"] for m in metrics)
            height_loss = before["height_m"]-minimum_height
            outcomes["soft_deformation"][oid] = {
                "preload_reference_time_s": baseline_row["time_s"],
                "preload_height_m": before["height_m"], "minimum_height_m": minimum_height,
                "final_height_m": after["height_m"],
                "height_recovery_fraction": (after["height_m"]-minimum_height)/height_loss if height_loss>0 else None,
                "maximum_axis_z_compression_fraction": max(m["axis_z_compression_fraction"] for m in metrics),
                "maximum_nonrigid_rms_m": max(m["nonrigid_rms_m"] for m in metrics),
                "final_nonrigid_rms_m": after["nonrigid_rms_m"],
                "minimum_j_all_steps": min(m["minimum_j"] for m in metrics),
                "maximum_inverted_tets": max(m["inverted_tets"] for m in metrics),
                "maximum_sampled_actuator_penetration_m": max(m["sampled_actuator_penetration_m"] for m in metrics),
                "final_volume_ratio": after["volume_ratio"],
                "source": "derived native tet and collision-node geometry at every physics step; sampled penetration is not exact mesh intersection",
            }
        outcomes["soft_contact_impulse_supervision"] = "unavailable; exclude episode from flexible point-impulse tasks"
        outcomes["right_censored"] = outcomes["right_censored"] or any(final[oid]["metrics"]["maximum_nodal_speed_m_s"] > .01 for oid in soft_ids)
    write_json(output / "outcomes.json", outcomes)
    for name in ("interaction_annotations", "outcomes"):
        manifest["trajectory"][name] = artifact(output, name + ".json", "derived from manually reviewed native trajectory")
    manifest["capabilities"]["interaction_annotations"] = {"status": "derived", "source": "native state, rigid-only point reports and explicitly derived soft geometry when present", "reason": None}
    manifest["lifecycle"] = "completed"
    audit = audit_causal_manifest(manifest, require_complete=True)
    if not audit["accepted"]:
        raise ValueError(audit["errors"])
    write_json(output / "episode.json", manifest)
    write_json(output / "candidate_completion.json", {
        "complete_record_audit": audit, "manual_review_sha256": file_hash(output / "human_review.json"),
        "c2_single_smoke_completed": True, "c2_prototype_matrix_accepted": False,
        "dataset_release_admitted": False,
    })


def finalize_none(output):
    """Compatibility wrapper for the first no-control smoke."""
    return finalize_smoke(output)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--episode", type=Path, required=True)
    args = parser.parse_args()
    finalize_smoke(args.episode)
    print("CAUSAL_SMOKE_CANDIDATE_COMPLETE", args.episode, flush=True)


if __name__ == "__main__":
    main()
