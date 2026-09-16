import json
import tempfile
import unittest
from pathlib import Path

from world_model_dataset.c1_reproduction import compare_traces
from world_model_dataset.causal_contract import audit_causal_manifest
from world_model_dataset.io import read_json, write_json
from world_model_dataset.probe_episode import ROOT, package_probe_episode
from world_model_dataset.controllers import bounded_angular_impedance, bounded_angular_velocity_effort, signed_work_increment


class C1PackagingTests(unittest.TestCase):
    def test_rotational_feedback_uses_torque_and_radians(self):
        decision = bounded_angular_velocity_effort(0.8, 0.0, 0.6, 0.2)
        self.assertEqual(decision["requested_torque_nm"], 0.48)
        self.assertEqual(decision["applied_torque_nm"], 0.2)
        self.assertTrue(decision["saturated"])
        brake = bounded_angular_velocity_effort(0.0, 1.0, 0.6, 0.2)
        self.assertEqual(brake["applied_torque_nm"], -0.2)
        impedance = bounded_angular_impedance(1.2, 0.0, 0.5, 0.0, 1.0, 0.3, 0.2)
        self.assertAlmostEqual(impedance["angle_error_rad"], 0.7)
        self.assertEqual(impedance["applied_torque_nm"], 0.2)
        self.assertAlmostEqual(signed_work_increment(0.2, 0.5, 0.6), 0.02)
        with self.assertRaises(ValueError):
            bounded_angular_impedance(1.0, 0.0, 0.0, 0.0, 1.0, -0.3, 0.2)

    def test_rotary_manifest_declares_rad_gains_and_torque_authority(self):
        for law in ("effort", "impedance"):
            with self.subTest(law=law), tempfile.TemporaryDirectory() as tmp:
                root = Path(tmp) / f"c1_rotary_{law}"
                root.mkdir()
                config = read_json(ROOT / f"configs/dataset/v0_2/c1_rotary_{law}_probe.json")
                config["scenarios"] = [config["scenarios"][2]]
                initial = read_json(ROOT / "configs/dataset/v0_2/examples/effort_pusher_overload.json")["initial_state"]["body_states"]
                write_json(root / "resolved_config.json", config)
                write_json(root / "initial_state.json", {"time_s": 0.0, "body_states": initial})
                for name in ("actuator_state_trace.jsonl", "actuator_effort_trace.jsonl", "command_trace.jsonl",
                             "body_state_trace.jsonl", "contacts.jsonl", "probe_initial.usda", "probe_final.usda", "probe_report.json"):
                    (root / name).touch()
                package_probe_episode(root)
                manifest = read_json(root / "episode.json")
                controller = manifest["control_program"]["controllers"][0]
                joint = manifest["system"]["joints"][0]
                self.assertEqual(joint["kind"], "revolute")
                self.assertEqual(joint["axis"], "Z")
                self.assertAlmostEqual(joint["upper_limit"], 2.6179938779914944)
                self.assertIsNone(controller["max_force_n"])
                self.assertEqual(controller["max_torque_nm"], 0.2)
                report = audit_causal_manifest(manifest)
                self.assertTrue(report["accepted"], report["errors"])
                if law == "impedance":
                    self.assertEqual(controller["stiffness_nm_rad"], 1.0)
                    self.assertIsNone(controller["stiffness_n_m"])
                    controller["max_torque_nm"] = None
                    self.assertFalse(audit_causal_manifest(manifest)["accepted"])

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
