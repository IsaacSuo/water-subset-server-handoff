import tempfile
import unittest
from pathlib import Path

from world_model_dataset.causal_contract import audit_causal_manifest
from world_model_dataset.causal_loader import CausalEpisode, open_episode
from world_model_dataset.causal_runner import ROOT, prepare, package_physics
from world_model_dataset.io import read_json, write_json
from world_model_dataset.causal_observe import package_observations
from world_model_dataset.causal_finalize import finalize_none


class CausalEpisodeTests(unittest.TestCase):
    def test_push_preparation_and_controlled_stream_packaging(self):
        import json
        with tempfile.TemporaryDirectory() as tmp:
            output = Path(tmp) / "c2_push_test"
            manifest = prepare(ROOT / "configs/dataset/v0_2/c2_rigid_push.json", output)
            self.assertTrue(audit_causal_manifest(manifest)["accepted"])
            self.assertIsNone(manifest["system"]["joints"][0]["body0_id"])
            self.assertEqual(manifest["initial_state"]["participant_ids"], ["pusher", "load", "floor"])
            write_json(output / "initial_state.json", {"time_s": 0, "body_states": manifest["initial_state"]["body_states"]})
            write_json(output / "native_report.json", {"runtime": "synthetic packaging test"})
            (output / "contacts.jsonl").touch()
            (output / "body_state_trace.jsonl").touch()
            for name in ("command_trace", "actuator_state_trace", "actuator_effort_trace"):
                with (output / (name + ".jsonl")).open("x") as stream:
                    stream.write(json.dumps({"time_s": 0.2, "controller_id": "pusher_velocity_effort"}) + "\n")
            package_physics(output)
            episode = open_episode(output, require_complete=False)
            self.assertEqual(len(list(episode.controls())), 1)
            self.assertEqual(len(list(episode.actuator_states())), 1)
            self.assertEqual(len(list(episode.actuator_efforts())), 1)
            with self.assertRaises(KeyError):
                episode.actuator_efforts("unknown")

    def test_controlled_finalizer_derives_external_work_not_contact_force(self):
        import json
        from world_model_dataset.causal_finalize import finalize_smoke
        with tempfile.TemporaryDirectory() as tmp:
            output = Path(tmp) / "c2_push_finalize_test"
            manifest = prepare(ROOT / "configs/dataset/v0_2/c2_rigid_push.json", output)
            initial = manifest["initial_state"]["body_states"]
            write_json(output / "initial_state.json", {"time_s": 0, "body_states": initial})
            write_json(output / "native_report.json", {"runtime": "synthetic finalizer unit test"})
            (output / "contacts.jsonl").touch()
            rows = {
                "body_state_trace": {"time_s": 0, "physics_step": 0,
                    "body_states": {oid: dict(value, **({"mass_kg": 1} if oid != "floor" else {})) for oid, value in initial.items()}},
                "command_trace": dict(manifest["control_program"]["commands"][0], time_s=0.2),
                "actuator_state_trace": {"time_s": 0, "controller_id": "pusher_velocity_effort"},
                "actuator_effort_trace": {"time_s": 1/240, "controller_id": "pusher_velocity_effort",
                    "command_active": False, "saturated": False, "applied_force_n": 0, "cumulative_work_j": 0},
            }
            for name, row in rows.items():
                with (output / (name + ".jsonl")).open("x") as stream:
                    stream.write(json.dumps(row) + "\n")
            package_physics(output)
            write_json(output / "observations/index.json", {"render_backend": "synthetic test", "frames": [], "cameras": {}})
            package_observations(output)
            write_json(output / "human_review.json", {"physics_accepted": True, "observations_accepted": True,
                                                      "scope": "synthetic unit test, not physical evidence"})
            finalize_smoke(output)
            episode = open_episode(output)
            self.assertEqual(episode.outcomes()["actuator_control"]["work_j"], 0)
            self.assertIsNone(episode.outcomes()["horizontal_kinetic_energy_ratio"])
            self.assertEqual(episode.annotations()["labels"], [])
            self.assertFalse(read_json(output / "candidate_completion.json")["c2_prototype_matrix_accepted"])

    def test_completed_candidate_reads_all_streams_without_release_admission(self):
        import json
        import numpy as np
        from PIL import Image
        from world_model_dataset.io import file_hash
        with tempfile.TemporaryDirectory() as tmp:
            config = read_json(ROOT / "configs/dataset/v0_2/c2_none_collision.json")
            config["timing"]["duration_s"] = 1 / 30
            config["camera_set"]["resolution"] = [2, 2]
            config_path = Path(tmp) / "isolated.json"
            write_json(config_path, config)
            output = Path(tmp) / "c2_synthetic_loader_test"
            manifest = prepare(config_path, output)
            initial = manifest["initial_state"]["body_states"]
            write_json(output / "initial_state.json", {"time_s": 0, "body_states": initial})
            write_json(output / "native_report.json", {"runtime": "synthetic loader test; not physical evidence"})
            (output / "contacts.jsonl").touch()
            with (output / "body_state_trace.jsonl").open("x", encoding="utf-8") as stream:
                for step in range(9):
                    bodies = {oid: dict(value, **({"mass_kg": 1.0} if oid != "floor" else {})) for oid, value in initial.items()}
                    stream.write(json.dumps({"time_s": step / 240, "physics_step": step, "body_states": bodies}) + "\n")
            package_physics(output)
            observation_dir = output / "observations"
            observation_dir.mkdir()
            rows = []
            for camera in ("front", "rear"):
                (observation_dir / camera).mkdir()
                for index, step in enumerate((0, 8)):
                    rgb = observation_dir / camera / f"{index}.png"
                    Image.fromarray(np.zeros((2, 2, 3), dtype=np.uint8)).save(rgb)
                    path = observation_dir / camera / f"{index}.npz"
                    with path.open("xb") as stream:
                        np.savez_compressed(stream, depth_m=np.ones((2, 2)), segmentation=np.ones((2, 2)),
                                            time_s=step/240, physics_step=step)
                    rows.append({"camera_id": camera, "time_s": step/240, "physics_step": step,
                                 "data": f"{camera}/{index}.npz", "rgb": f"{camera}/{index}.png",
                                 "data_sha256": file_hash(path), "rgb_sha256": file_hash(rgb)})
            write_json(observation_dir / "index.json", {
                "complete": True, "render_backend": "synthetic isolated test", "frames": rows,
                "cameras": {camera: {"intrinsic_opencv": [[1, 0, 1], [0, 1, 1], [0, 0, 1]]} for camera in ("front", "rear")},
            })
            package_observations(output)
            write_json(output / "human_review.json", {"physics_accepted": True, "observations_accepted": True,
                                                       "scope": "synthetic loader test; no real physical acceptance"})
            finalize_none(output)
            ep = open_episode(output)
            self.assertEqual(len(list(ep.states())), 9)
            self.assertEqual(len(list(ep.observations())), 4)
            self.assertEqual(next(ep.observations())[1]["rgb"].shape, (2, 2, 3))
            self.assertEqual(ep.outcomes()["horizontal_kinetic_energy_ratio"], 1.0)
            self.assertEqual(ep.annotations()["labels"][0]["label"], "free_motion")
            self.assertFalse(read_json(output / "candidate_completion.json")["dataset_release_admitted"])
            self.assertEqual(read_json(observation_dir / "index.calibrated.json")["cameras"]["front"]["intrinsic_opencv"][0][2], 0.5)

    def test_prepare_resolves_inputs_without_mutating_template(self):
        template = ROOT / "configs/dataset/v0_2/examples/rigid_collision_none.json"
        before = template.read_bytes()
        with tempfile.TemporaryDirectory() as tmp:
            output = Path(tmp) / "c2_none_test"
            manifest = prepare(ROOT / "configs/dataset/v0_2/c2_none_collision.json", output)
            self.assertTrue(audit_causal_manifest(manifest)["accepted"])
            self.assertEqual(manifest["initial_state"]["body_states"]["left"]["linear_velocity_m_s"], [1.0, 0.0, 0.0])
            self.assertEqual(manifest["initial_state"]["body_states"]["right"]["linear_velocity_m_s"], [0.0, 0.0, 0.0])
            ep = open_episode(output, require_complete=False)
            self.assertIsInstance(ep, CausalEpisode)
            self.assertEqual(ep.resolved_inputs()["bodies"]["floor"]["physics"]["dynamic_friction"], 0.0)
            self.assertEqual(list(ep.controls()), [])
            with self.assertRaises(ValueError):
                open_episode(output)
            with self.assertRaises(FileExistsError):
                prepare(ROOT / "configs/dataset/v0_2/c2_none_collision.json", output)
        self.assertEqual(before, template.read_bytes())

    def test_physics_packaging_and_changed_artifact_rejection(self):
        with tempfile.TemporaryDirectory() as tmp:
            output = Path(tmp) / "c2_none_test"
            manifest = prepare(ROOT / "configs/dataset/v0_2/c2_none_collision.json", output)
            write_json(output / "initial_state.json", {"time_s": 0, "body_states": manifest["initial_state"]["body_states"]})
            write_json(output / "native_report.json", {"runtime": "isolated test, not native physics evidence"})
            (output / "contacts.jsonl").touch()
            with (output / "body_state_trace.jsonl").open("x", encoding="utf-8") as stream:
                import json
                stream.write(json.dumps({"time_s": 0, "body_states": manifest["initial_state"]["body_states"]}) + "\n")
            packaged = package_physics(output)
            ep = open_episode(output, require_complete=False)
            self.assertEqual(len(list(ep.states())), 1)
            self.assertEqual(list(ep.contacts()), [])
            self.assertEqual(packaged["trajectory"]["observations"]["status"], "pending")
            with (output / "body_state_trace.jsonl").open("a", encoding="utf-8") as stream:
                stream.write("{}\n")
            with self.assertRaises(ValueError):
                list(open_episode(output, require_complete=False).states())


if __name__ == "__main__":
    unittest.main()
