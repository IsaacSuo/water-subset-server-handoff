import json
import tempfile
import unittest
from pathlib import Path

from world_model_dataset.c1_reproduction import compare_traces
from world_model_dataset.causal_contract import audit_causal_manifest
from world_model_dataset.io import read_json, write_json
from world_model_dataset.probe_episode import ROOT, package_probe_episode


class C1PackagingTests(unittest.TestCase):
    def test_independent_packaging_is_physics_only_and_has_complete_initial_set(self):
        for law in ("effort", "impedance"):
            for sid in ("free", "resisted", "overload"):
                with self.subTest(law=law, scenario=sid), tempfile.TemporaryDirectory() as tmp:
                    root = Path(tmp) / f"c1_{law}_{sid}"
                    root.mkdir()
                    config = read_json(ROOT / f"configs/dataset/v0_2/c1_{law}_probe.json")
                    config["scenarios"] = [s for s in config["scenarios"] if s["id"] == sid]
                    initial = read_json(ROOT / "configs/dataset/v0_2/examples/effort_pusher_overload.json")["initial_state"]["body_states"]
                    if sid == "free":
                        initial.pop("load")
                    if sid != "overload":
                        initial.pop("wall")
                    write_json(root / "resolved_config.json", config)
                    write_json(root / "initial_state.json", {"time_s": 0.0, "body_states": initial})
                    for name in ("actuator_state_trace.jsonl", "actuator_effort_trace.jsonl", "command_trace.jsonl",
                                 "body_state_trace.jsonl", "contacts.jsonl", "probe_initial.usda", "probe_final.usda", "probe_report.json"):
                        (root / name).touch()
                    package_probe_episode(root)
                    manifest = read_json(root / "episode.json")
                    report = audit_causal_manifest(manifest)
                    self.assertTrue(report["accepted"], report["errors"])
                    self.assertEqual(set(initial), {b["instance_id"] for b in manifest["system"]["bodies"]})
                    self.assertEqual(manifest["trajectory"]["observations"]["status"], "pending")
                    self.assertFalse(audit_causal_manifest(manifest, require_complete=True)["accepted"])
                    self.assertEqual(manifest["capabilities"]["rigid_contact_impulse"]["status"], "native")
                    self.assertIn("world_model_dataset/causal_effort_probe.py", read_json(root / "source_snapshot.json"))

    def test_comparison_checks_entire_trajectory_and_time_grid(self):
        with tempfile.TemporaryDirectory() as tmp:
            old, new = Path(tmp) / "old.jsonl", Path(tmp) / "new.jsonl"
            rows = [{"scenario_id": "free", "time_s": t, "position_m": [t, 0.0, 0.0],
                     "linear_velocity_m_s": [1.0, 0.0, 0.0]} for t in (0.1, 0.2)]
            def save(path, values):
                with path.open("w", encoding="utf-8") as stream:
                    stream.write("\n".join(json.dumps(r) for r in values) + "\n")
            save(old, rows)
            save(new, rows)
            self.assertTrue(compare_traces(old, new, "free")["reproduced"])
            rows[0]["position_m"][0] += 0.02
            save(new, rows)
            self.assertFalse(compare_traces(old, new, "free")["reproduced"])
            rows[0]["time_s"] = 0.11
            save(new, rows)
            self.assertFalse(compare_traces(old, new, "free")["matched_time_grid"])
            save(old, [])
            save(new, [])
            self.assertFalse(compare_traces(old, new, "free")["reproduced"])


if __name__ == "__main__":
    unittest.main()
