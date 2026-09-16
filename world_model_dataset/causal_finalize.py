"""Finalize a manually reviewed causal smoke candidate, not an entire C2 matrix or release."""
from __future__ import annotations

import argparse
import copy
from pathlib import Path

from .causal_contract import audit_causal_manifest
from .causal_loader import open_episode
from .io import file_hash, read_json, write_json
from .probe_episode import artifact


def finalize_none(output):
    output = Path(output)
    review = read_json(output / "human_review.json")
    if not review["physics_accepted"] or not review["observations_accepted"]:
        raise ValueError("This candidate has not received manual physics and observation acceptance")
    if (output / "episode.json").exists():
        raise FileExistsError("Never overwrite a finalized manifest")
    episode = open_episode(output, require_complete=False)
    manifest = copy.deepcopy(episode.manifest)
    if manifest["control_program"]["primitive"] != "none":
        raise ValueError("This initial finalizer derives no-control rigid collision smoke outputs only")
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
        "review": "Ground-plane ray intersections verified against optical-axis depth; native sensor pixels unchanged",
    }
    calibrated.setdefault("geometry_representation", "USD spheres tessellated by RTX; render depth is visible triangulated geometry, not exact analytical collider depth")
    write_json(output / "observations/index.calibrated.json", calibrated)
    manifest["trajectory"]["observations"] = artifact(output, "observations/index.calibrated.json", "native cache-only RGB-D and segmentation with reviewed integer-pixel camera calibration")
    subject_ids = {b["instance_id"] for b in manifest["system"]["bodies"] if b["role"] == "subject"}
    impact_points = [r for r in contacts if set(r["actor_ids"]) == subject_ids and
                     sum(v*v for v in r["impulse_ns"]) > 0]
    labels = [{"label": "free_motion", "start_time_s": 0, "end_time_s": states[-1]["time_s"],
               "source": "native state under fixed environment and no control"}]
    if impact_points:
        labels.append({"label": "transient_impact", "start_time_s": min(r["time_s"] for r in impact_points),
                       "end_time_s": max(r["time_s"] for r in impact_points),
                       "body_ids": sorted(subject_ids), "source": "nonzero native interbody contact impulse reports"})
    write_json(output / "interaction_annotations.json", {"labels": labels, "source_contact_sha256": file_hash(output / "contacts.jsonl")})
    initial, final = states[0]["body_states"], states[-1]["body_states"]
    def momentum_x(frame):
        return sum(frame[oid]["mass_kg"] * frame[oid]["linear_velocity_m_s"][0] for oid in subject_ids)
    def horizontal_ke(frame):
        return sum(0.5 * frame[oid]["mass_kg"] * sum(v*v for v in frame[oid]["linear_velocity_m_s"][:2]) for oid in subject_ids)
    write_json(output / "outcomes.json", {
        "duration_s": states[-1]["time_s"], "initial_momentum_x_kg_m_s": momentum_x(initial),
        "final_momentum_x_kg_m_s": momentum_x(final),
        "horizontal_kinetic_energy_ratio": horizontal_ke(final) / horizontal_ke(initial) if horizontal_ke(initial) else None,
        "final_subject_states": {oid: final[oid] for oid in sorted(subject_ids)},
        "right_censored": any(sum(v*v for v in final[oid]["linear_velocity_m_s"]) > 0.0001 for oid in subject_ids),
        "source": "derived native-state diagnostics at the recorded horizon; not final resting outcome",
    })
    for name in ("interaction_annotations", "outcomes"):
        manifest["trajectory"][name] = artifact(output, name + ".json", "derived from manually reviewed native trajectory")
    manifest["capabilities"]["interaction_annotations"] = {"status": "derived", "source": "native state and point-contact reports", "reason": None}
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


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--episode", type=Path, required=True)
    args = parser.parse_args()
    finalize_none(args.episode)
    print("CAUSAL_SMOKE_CANDIDATE_COMPLETE", args.episode, flush=True)


if __name__ == "__main__":
    main()
