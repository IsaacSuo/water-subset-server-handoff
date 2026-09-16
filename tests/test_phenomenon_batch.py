import unittest

from world_model_dataset.causal_contract import audit_causal_manifest
from world_model_dataset.phenomenon_batch import recipe


class PhenomenonRecipeTests(unittest.TestCase):
    def test_second_batch_preserves_causal_contract(self):
        from world_model_dataset.causal_runner import ROOT
        from world_model_dataset.io import read_json
        spec = read_json(ROOT/"configs/dataset/v0_2/phenomena_batch02.json")
        for design in spec["designs"]:
            for variant in design["variants"]:
                config, manifest = recipe(design,variant,spec["assets"][design["assets"][0]])
                self.assertTrue(audit_causal_manifest(manifest)["accepted"],design["id"])
                if design["kind"].startswith("soft_"):
                    self.assertEqual(manifest["capabilities"]["soft_contact_impulse"]["status"],"unavailable")
                    self.assertTrue(config["numerics"]["gpu_dynamics"])

    def test_geometric_distances_are_derived_and_respect_rotation(self):
        import numpy as np
        from scipy.spatial.transform import Rotation
        from world_model_dataset.causal_soft import sampled_rigid_contacts
        state = {"position_m":[0,0,0],"orientation_xyzw":Rotation.from_euler("z",90,degrees=True).as_quat()}
        shapes = {"wall":(state,{"shape":"box","size_m":[2,.2,1]}),
                  "ball":(state,{"shape":"sphere","radius_m":.1})}
        result = sampled_rigid_contacts(np.array([[0,.5,0]]),shapes,.002)
        self.assertAlmostEqual(result["wall"]["sampled_penetration_m"],.1)
        self.assertAlmostEqual(result["ball"]["minimum_node_gap_m"],.4)
        self.assertEqual(result["wall"]["status"],"derived")
        self.assertNotIn("impulse",result["wall"])

    def test_single_body_keeps_complete_participant_set(self):
        config, manifest = recipe({"kind": "sliding"}, {"friction": .15},
                                  {"shape": "box", "size_m": [.25,.2,.15], "mass_kg": 2})
        self.assertTrue(audit_causal_manifest(manifest)["accepted"])
        self.assertEqual(set(manifest["initial_state"]["body_states"]), {"left", "floor"})
        self.assertEqual(manifest["initial_state"]["body_states"]["left"]["position_m"][2], .075)
        self.assertEqual(config["physics_profiles"]["rigid_reference"]["mass_kg"], 2)

    def test_gap_and_start_positions_follow_asset_size(self):
        config, manifest = recipe({"kind": "confinement"}, {"gap_ratio": .8},
                                  {"shape": "box", "size_m": [1,.6,.4], "mass_kg": 2})
        self.assertTrue(audit_causal_manifest(manifest)["accepted"])
        states = manifest["initial_state"]["body_states"]
        gap = states["wall_right"]["position_m"][1]-states["wall_left"]["position_m"][1]-.3
        self.assertAlmostEqual(gap,.48)
        self.assertLess(states["load"]["position_m"][0]+.5,.31)
        self.assertLess(states["pusher"]["position_m"][0]+.1, states["load"]["position_m"][0]-.5)
        self.assertEqual(config["geometry_profiles"]["pusher_pad"]["size_m"][1],.36)

    def test_rolling_sets_only_initial_spin(self):
        _, manifest = recipe({"kind": "rolling"}, {"rolling_ratio": 1},
                             {"shape": "sphere", "radius_m": .2, "mass_kg": 1})
        self.assertEqual(manifest["initial_state"]["body_states"]["left"]["angular_velocity_rad_s"],[0,5,0])
        self.assertEqual(manifest["control_program"]["primitive"],"none")
        self.assertEqual(manifest["implementation"]["post_t0_operations"][0]["kind"],"solver_step")


if __name__ == "__main__":
    unittest.main()
