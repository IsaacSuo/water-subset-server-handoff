"""Read-only inventory of historical v0.1 caches for v0.2 causal migration."""
from __future__ import annotations

import argparse
import json
from collections import Counter
from pathlib import Path

from .io import file_hash, read_json


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_SOURCES = ROOT / "configs/dataset/v0_2/historical_sources.json"


def classify(event_id, commands):
    """Classify reuse conservatively; no returned class means automatic admission."""
    kinds = {command["kind"] for command in commands}
    methods = {command.get("parameters", {}).get("method") for command in commands}
    if not commands:
        return "direct_candidate"
    if kinds == {"initial_velocity"}:
        return "post_write_crop_candidate"
    if "deactivate_actor" in methods or ("release" in kinds and "remove_support" in kinds):
        return "post_deactivation_recovery_review"
    if kinds <= {"release", "remove_support"} and "disable_collision" in methods:
        return "post_removal_crop_review"
    if "kinematic_trajectory" in kinds:
        return "post_actuation_passive_review"
    return "regression_only"


def crop_boundary(commands, frames):
    """Return the first fully captured state after the last historical action mutation."""
    if not commands:
        return 0.0
    last_mutation = max(command["end_time_s"] for command in commands)
    return next((frame["time_s"] for frame in frames if frame["time_s"] > last_mutation), None)


def _episode_row(matrix_path, episode_id):
    root = matrix_path.parent / "episodes" / episode_id
    manifest_path = root / "episode.prepared.json"
    action_path = root / "action.json"
    state_index_path = root / "state/index.json"
    for path in (manifest_path, action_path, state_index_path):
        if not path.is_file():
            raise FileNotFoundError(path)
    manifest = read_json(manifest_path)
    action = read_json(action_path)
    state_index = read_json(state_index_path)
    commands = action["commands"]
    frames = state_index["frames"]
    event_id = manifest["spec"]["event_id"]
    boundary = crop_boundary(commands, frames)
    duration = manifest["spec"]["timing"]["duration_s"]
    command_summary = [{
        "kind": command["kind"],
        "target": command["target"],
        "start_time_s": command["start_time_s"],
        "end_time_s": command["end_time_s"],
        "method": command.get("parameters", {}).get("method"),
    } for command in commands]
    relative_root = root.relative_to(ROOT).as_posix()
    return {
        "episode_id": episode_id,
        "event_id": event_id,
        "classification": classify(event_id, commands),
        "source_root": relative_root,
        "manifest_sha256": file_hash(manifest_path),
        "action_sha256": file_hash(action_path),
        "state_index_sha256": file_hash(state_index_path),
        "historical_commands": command_summary,
        "candidate_crop_start_time_s": boundary,
        "candidate_remaining_duration_s": None if boundary is None else duration - boundary,
        "captured_state_at_boundary": boundary is not None,
        "automatic_admission": False,
        "manual_review_required": True,
    }


def inventory(sources_path=DEFAULT_SOURCES):
    definition = read_json(sources_path)
    rows = []
    seen = set()
    source_matrices = []
    for source in definition["sources"]:
        matrix_path = ROOT / source["matrix"]
        matrix = read_json(matrix_path)
        included = set(source["include_event_ids"])
        accepted_here = 0
        for episode_id in matrix["episodes"]:
            row = _episode_row(matrix_path, episode_id)
            if row["event_id"] not in included:
                continue
            if episode_id in seen:
                raise ValueError(f"Duplicate historical episode: {episode_id}")
            seen.add(episode_id)
            rows.append(row)
            accepted_here += 1
        source_matrices.append({
            "path": source["matrix"],
            "sha256": file_hash(matrix_path),
            "included_event_ids": sorted(included),
            "episode_count": accepted_here,
        })
    rows.sort(key=lambda row: (row["event_id"], row["episode_id"]))
    counts = Counter(row["classification"] for row in rows)
    event_counts = Counter(row["event_id"] for row in rows)
    return {
        "schema_version": "0.2.0-draft",
        "kind": "historical_causal_migration_inventory",
        "source_definition": Path(sources_path).relative_to(ROOT).as_posix(),
        "source_matrices": source_matrices,
        "episode_count": len(rows),
        "classification_counts": dict(sorted(counts.items())),
        "event_counts": dict(sorted(event_counts.items())),
        "admission_policy": "candidate_only; every row requires manual cache and observation review",
        "episodes": rows,
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--sources", default=str(DEFAULT_SOURCES))
    parser.add_argument("--output")
    args = parser.parse_args()
    result = inventory(Path(args.sources))
    payload = json.dumps(result, ensure_ascii=False, indent=2) + "\n"
    if args.output:
        output = Path(args.output)
        if output.exists():
            raise FileExistsError(f"Refusing to replace migration inventory: {output}")
        output.write_text(payload, encoding="utf-8")
    print(json.dumps({
        "episode_count": result["episode_count"],
        "event_counts": result["event_counts"],
        "classification_counts": result["classification_counts"],
    }, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
