"""Read v0.2 causal state/contact/observation records, with an explicit v0.1 bridge."""
from __future__ import annotations

import json
from pathlib import Path

from .causal_contract import audit_causal_manifest
from .io import file_hash, finite, inside, read_json


class CausalEpisode:
    def __init__(self, root, require_complete=True):
        self.root = Path(root)
        path = next((self.root / name for name in ("episode.json", "episode.observed.json", "episode.physics.json", "episode.prepared.json")
                     if (self.root / name).is_file()), None)
        if path is None:
            raise FileNotFoundError("No causal episode manifest")
        self.manifest = read_json(path)
        audit = audit_causal_manifest(self.manifest, require_complete=require_complete)
        if not audit["accepted"]:
            raise ValueError(audit["errors"])
        self._checked = set()

    def capability(self, name, allowed=("native", "derived")):
        value = self.manifest["capabilities"].get(name)
        if value is None or value["status"] not in allowed:
            raise RuntimeError(f"Capability {name}: {value}")
        return value

    def record_path(self, record):
        if record["status"] != "available":
            raise RuntimeError(f"Record {record['status']}: {record['source']}; {record['reason']}")
        path = inside(self.root, record["path"])
        if path not in self._checked:
            if path.stat().st_size != record["bytes"] or file_hash(path) != record["sha256"]:
                raise ValueError(f"Artifact changed: {record['path']}")
            self._checked.add(path)
        return path

    def jsonl(self, record):
        with self.record_path(record).open(encoding="utf-8") as stream:
            for line in stream:
                row = json.loads(line)
                finite(row)
                yield row

    def states(self):
        previous = -1.0
        ids = set(self.manifest["initial_state"]["participant_ids"])
        for row in self.jsonl(self.manifest["trajectory"]["states"]):
            if row["time_s"] <= previous or set(row["body_states"]) != ids:
                raise ValueError("State time axis or physical participant set is inconsistent")
            previous = row["time_s"]
            yield row

    def contacts(self):
        self.capability("rigid_contact_impulse", ("native",))
        return self.jsonl(self.manifest["trajectory"]["contacts"])

    def controls(self):
        if self.manifest["control_program"]["primitive"] == "none":
            return iter(())
        return self.jsonl(self.manifest["control_program"]["command_trace"])

    def actuator_trace(self, field, controller_id=None):
        controllers = self.manifest["control_program"]["controllers"]
        if controller_id is None:
            if len(controllers) != 1:
                raise ValueError("Specify a controller_id when there is not exactly one controller")
            controller_id = controllers[0]["controller_id"]
        controller = next((c for c in controllers if c["controller_id"] == controller_id), None)
        if controller is None:
            raise KeyError(controller_id)
        return (row for row in self.jsonl(controller[field]) if row["controller_id"] == controller_id)

    def actuator_states(self, controller_id=None):
        return self.actuator_trace("state_trace", controller_id)

    def actuator_efforts(self, controller_id=None):
        return self.actuator_trace("effort_trace", controller_id)

    def soft_geometries(self, instance_id):
        import numpy as np
        body = next((b for b in self.manifest["system"]["bodies"] if b["instance_id"] == instance_id), None)
        if body is None or body["physics_kind"] != "volumetric":
            raise ValueError("Expected a declared volumetric body")
        for row in self.states():
            path = self.record_path(row["body_states"][instance_id]["geometry"])
            with np.load(path, allow_pickle=False) as data:
                if float(data["time_s"]) != row["time_s"] or int(data["physics_step"]) != row["physics_step"]:
                    raise ValueError("Soft geometry time metadata mismatch")
                yield row, {key: data[key].copy() for key in data.files}

    def soft_topology(self, instance_id):
        first = next(self.states())
        record = first["body_states"][instance_id].get("topology")
        if record is None:
            raise RuntimeError("This diagnostic cache has no hashed soft bind/topology record")
        return read_json(self.record_path(record))

    def resolved_inputs(self):
        record = self.manifest["system"].get("resolved_inputs")
        if record is None:
            raise RuntimeError("This capability-only manifest has no resolved registry; inspect its initial USD snapshot")
        return read_json(self.record_path(record))

    def annotations(self):
        return read_json(self.record_path(self.manifest["trajectory"]["interaction_annotations"]))

    def outcomes(self):
        return read_json(self.record_path(self.manifest["trajectory"]["outcomes"]))

    def observations(self):
        import numpy as np
        from PIL import Image
        record = self.manifest["trajectory"]["observations"]
        path = self.record_path(record)
        index = read_json(path)
        for row in index["frames"]:
            data_path = inside(path.parent, row["data"])
            rgb_path = inside(path.parent, row["rgb"])
            if file_hash(data_path) != row["data_sha256"] or file_hash(rgb_path) != row["rgb_sha256"]:
                raise ValueError("Observation artifact changed")
            with np.load(data_path, allow_pickle=False) as data:
                if float(data["time_s"]) != row["time_s"] or int(data["physics_step"]) != row["physics_step"]:
                    raise ValueError("Observation time metadata mismatch")
                values = {key: data[key].copy() for key in data.files}
            with Image.open(rgb_path) as image:
                values["rgb"] = np.asarray(image.convert("RGB")).copy()
            if values["rgb"].shape[:2] != values["depth_m"].shape or values["segmentation"].shape != values["depth_m"].shape:
                raise ValueError("RGB/depth/segmentation shape mismatch")
            yield row, values


def open_episode(root, require_complete=True):
    root = Path(root)
    path = next((root / name for name in ("episode.json", "episode.observed.json", "episode.physics.json", "episode.prepared.json") if (root / name).exists()), None)
    if path is None:
        raise FileNotFoundError("No episode manifest")
    if read_json(path)["schema_version"].startswith("0.2."):
        return CausalEpisode(root, require_complete=require_complete)
    from .loader import Episode
    return Episode(root, require_complete=require_complete)
