import copy
import unittest

from world_model_dataset.causal_contract import SCHEMA, audit_causal_manifest, validate_causal_schema
from world_model_dataset.io import read_json
from world_model_dataset.migration_v02 import classify, crop_boundary


EXAMPLE = SCHEMA.parent / "examples/rigid_collision_none.json"


def pending(source):
    return {
        "status": "pending",
        "path": None,
        "sha256": None,
        "bytes": None,
        "source": source,
        "reason": None,
    }


class CausalContractTests(unittest.TestCase):
    def setUp(self):
        self.manifest = read_json(EXAMPLE)

    def test_no_control_initial_velocity_is_causal(self):
        validate_causal_schema(self.manifest)
        report = audit_causal_manifest(self.manifest)
        self.assertTrue(report["accepted"], report["errors"])
        self.assertEqual(self.manifest["system"]["bodies"][0]["linear_velocity_m_s"], [1.0, 0.0, 0.0])
        self.assertEqual(self.manifest["control_program"]["primitive"], "none")

    def test_mid_timeline_velocity_write_is_rejected(self):
        value = copy.deepcopy(self.manifest)
        value["implementation"]["state_update_authority"] = "mixed"
        value["implementation"]["post_t0_operations"].append({
            "kind": "set_velocity",
            "target": "left",
            "time_s": 0.5,
            "reason": "historical initial_velocity action",
        })
        report = audit_causal_manifest(value)
        self.assertFalse(report["accepted"])
        self.assertTrue(any("set_velocity" in error for error in report["errors"]))
        self.assertTrue(any("solver-only" in error for error in report["errors"]))

    def test_collision_or_visibility_switch_is_rejected(self):
        for kind, policy in (("disable_collision", "collision_participation"),
                             ("hide_body", "render_presence")):
            with self.subTest(kind=kind):
                value = copy.deepcopy(self.manifest)
                value["implementation"][policy] = "time_varying"
                value["implementation"]["post_t0_operations"].append({
                    "kind": kind,
                    "target": "floor",
                    "time_s": 0.75,
                    "reason": "historical removal shortcut",
                })
                report = audit_causal_manifest(value)
                self.assertFalse(report["accepted"])
                self.assertTrue(any(kind in error for error in report["errors"]))

    def test_unbounded_kinematic_controller_is_rejected(self):
        value = copy.deepcopy(self.manifest)
        value["system"]["bodies"][0]["role"] = "actuator"
        value["control_program"] = {
            "primitive": "effort_control",
            "controllers": [{
                "controller_id": "pusher_controller",
                "primitive": "effort_control",
                "implementation": "unbounded_kinematic",
                "actuator_instance_id": "left",
                "max_force_n": 20.0,
                "max_torque_nm": None,
                "stiffness_n_m": None,
                "damping_n_s_m": None,
                "reaction_observable": False,
                "state_trace": pending("native actuator state"),
                "effort_trace": pending("native applied effort"),
            }],
            "commands": [{
                "command_id": "push",
                "controller_id": "pusher_controller",
                "start_time_s": 0.0,
                "end_time_s": 1.0,
                "target": {"velocity_m_s": 1.0},
                "limits": {"max_force_n": 20.0},
            }],
            "command_trace": pending("control command stream"),
        }
        report = audit_causal_manifest(value)
        self.assertFalse(report["accepted"])
        self.assertTrue(any("not admitted" in error for error in report["errors"]))
        self.assertTrue(any("reaction" in error for error in report["errors"]))

    def test_finite_effort_controller_is_admitted_as_draft(self):
        value = copy.deepcopy(self.manifest)
        value["system"]["bodies"][0]["role"] = "actuator"
        value["control_program"] = {
            "primitive": "effort_control",
            "controllers": [{
                "controller_id": "pusher_controller",
                "primitive": "effort_control",
                "implementation": "dynamic_body_effort",
                "actuator_instance_id": "left",
                "max_force_n": 20.0,
                "max_torque_nm": None,
                "stiffness_n_m": None,
                "damping_n_s_m": None,
                "reaction_observable": True,
                "state_trace": pending("native actuator state"),
                "effort_trace": pending("native applied effort"),
            }],
            "commands": [{
                "command_id": "push",
                "controller_id": "pusher_controller",
                "start_time_s": 0.0,
                "end_time_s": 1.0,
                "target": {"force_n": [10.0, 0.0, 0.0]},
                "limits": {"max_force_n": 20.0},
            }],
            "command_trace": pending("control command stream"),
        }
        value["implementation"]["post_t0_operations"] = [
            {
                "kind": "apply_force",
                "target": "left",
                "time_s": 0.0,
                "reason": "bounded force is applied to the physical actuator",
            }
        ]
        report = audit_causal_manifest(value)
        self.assertTrue(report["accepted"], report["errors"])

    def test_derived_crop_requires_exact_parent_and_positive_boundary(self):
        value = copy.deepcopy(self.manifest)
        value["initial_state"]["source"] = "derived_crop"
        value["lineage"] = {"derivation": "derived_crop", "parents": []}
        report = audit_causal_manifest(value)
        self.assertFalse(report["accepted"])
        self.assertTrue(any("exactly one parent" in error for error in report["errors"]))

        value["lineage"]["parents"] = [{
            "episode_id": "r03_historical",
            "manifest_path": "parents/r03_historical/episode.json",
            "manifest_sha256": "0" * 64,
            "crop_start_time_s": 0.5,
            "crop_end_time_s": 1.5,
        }]
        report = audit_causal_manifest(value)
        self.assertTrue(report["accepted"], report["errors"])

    def test_completed_admission_requires_actual_records(self):
        value = copy.deepcopy(self.manifest)
        value["lifecycle"] = "completed"
        report = audit_causal_manifest(value, require_complete=True)
        self.assertFalse(report["accepted"])
        self.assertTrue(any("required state" in error for error in report["errors"]))

    def test_historical_action_classification_is_conservative(self):
        initial_velocity = [{"kind": "initial_velocity"}]
        removal = [{"kind": "remove_support", "parameters": {"method": "disable_collision"}}]
        trajectory = [{"kind": "kinematic_trajectory"}]
        deactivation = [
            {"kind": "release", "parameters": {"method": "set_dynamic"}},
            {"kind": "remove_support", "parameters": {"method": "deactivate_actor"}},
        ]
        self.assertEqual(classify("V01", []), "direct_candidate")
        self.assertEqual(classify("R03", initial_velocity), "post_write_crop_candidate")
        self.assertEqual(classify("R05", removal), "post_removal_crop_review")
        self.assertEqual(classify("V02", trajectory), "post_actuation_passive_review")
        self.assertEqual(classify("V03", deactivation), "post_deactivation_recovery_review")

    def test_crop_starts_at_first_complete_state_after_mutation(self):
        commands = [{"end_time_s": 0.5}]
        frames = [{"time_s": 0.5}, {"time_s": 0.5166666666666667}, {"time_s": 0.5333333333333333}]
        self.assertEqual(crop_boundary(commands, frames), 0.5166666666666667)
        self.assertEqual(crop_boundary([], frames), 0.0)

    def test_generated_historical_inventory_is_complete_but_not_admitted(self):
        inventory = read_json(SCHEMA.parent / "historical_cache_inventory.json")
        self.assertEqual(inventory["episode_count"], 121)
        self.assertEqual(sum(inventory["event_counts"].values()), 121)
        self.assertEqual(inventory["classification_counts"], {
            "direct_candidate": 11,
            "post_actuation_passive_review": 32,
            "post_deactivation_recovery_review": 11,
            "post_removal_crop_review": 32,
            "post_write_crop_candidate": 35,
        })
        self.assertTrue(all(not row["automatic_admission"] for row in inventory["episodes"]))
        self.assertTrue(all(row["captured_state_at_boundary"] for row in inventory["episodes"]))


if __name__ == "__main__":
    unittest.main()
