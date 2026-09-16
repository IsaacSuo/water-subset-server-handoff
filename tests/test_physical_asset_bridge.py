"""Local integration checks; skip when the exploration delivery is not installed."""
from pathlib import Path
import tempfile
import unittest

import numpy as np

from world_model_dataset.causal_runner import ROOT, prepare
from world_model_dataset.io import read_json, write_json
from world_model_dataset.physical_asset_bridge import resolve_asset
from world_model_dataset.physical_asset_smoke import make_config


EXPLORE = Path("/home/fangsuo/isaacsim_work_flexible_explore")


@unittest.skipUnless((EXPLORE/"output/physical_assets/library_v1/index.json").exists(),
                     "Optional exploration asset delivery is not installed")
class PhysicalAssetBridgeTests(unittest.TestCase):
    def inputs(self, name="banana"):
        config = make_config(EXPLORE,name)
        return config,config["geometry_profiles"]["exploration_asset"],config["physics_profiles"]["asset_development"]

    def test_all_four_keep_source_triangles_complete_inertia_and_sdf(self):
        for name,count in (("banana",43622),("elephant",50000),("chair",16954),("carrot",39398)):
            _,geometry,physics = self.inputs(name)
            mesh = resolve_asset(geometry,physics,ROOT)
            self.assertEqual(len(mesh.faces),count)
            self.assertAlmostEqual(max(mesh.extents),.2,places=7)
            np.testing.assert_allclose(mesh.center_mass,0,atol=1e-8)
            np.testing.assert_allclose(geometry["inertia_tensor_kg_m2"],
                                      mesh.moment_inertia*physics["mass_kg"]/mesh.volume,rtol=3e-6,atol=1e-11)
            self.assertAlmostEqual(physics["mass_kg"]/geometry["rest_volume_m3"],700)
            self.assertEqual((geometry["collision_approximation"],geometry["sdf_resolution"],
                              geometry["sdf_subgrid_resolution"],geometry["sdf_bits_per_subgrid_pixel"],
                              geometry["sdf_triangle_count_reduction_factor"]),('sdf',256,6,'BitsPerPixel16',1.))
            self.assertEqual(geometry["physical_asset_provenance"]["status"],"development_integration_not_asset_admission")

    def test_explicit_xyz_and_mass_reuse_source_loader(self):
        _,geometry,physics = self.inputs()
        geometry["physical_asset_source"]["size"] = {"extents_m":[.2,.1,.15]}
        physics.update(mass_kg=.5,density_kg_m3=None)
        mesh = resolve_asset(geometry,physics,ROOT)
        np.testing.assert_allclose(mesh.extents,[.2,.1,.15],rtol=1e-6)
        self.assertEqual(physics["mass_kg"],.5)
        np.testing.assert_allclose(geometry["inertia_tensor_kg_m2"],mesh.moment_inertia*.5/mesh.volume,rtol=3e-6,atol=1e-11)

    def test_reject_changed_loader(self):
        _,geometry,physics = self.inputs()
        geometry["physical_asset_source"]["loader_sha256"] = '0'*64
        with self.assertRaisesRegex(ValueError,"loader changed"):
            resolve_asset(geometry,physics,ROOT)

    def test_reject_ambiguous_mass_density(self):
        _,geometry,physics = self.inputs()
        physics["mass_kg"] = .5
        with self.assertRaisesRegex(ValueError,"mass or density"):
            resolve_asset(geometry,physics,ROOT)

    def test_prepared_episode_keeps_c2_control_and_local_geometry(self):
        config,_,_ = self.inputs()
        base = read_json(ROOT/config["manifest_template"])
        with tempfile.TemporaryDirectory() as tmp:
            path=Path(tmp)/"config.json";write_json(path,config)
            output=Path(tmp)/"episode";manifest=prepare(path,output)
            self.assertEqual(manifest["control_program"],base["control_program"])
            self.assertEqual(manifest["system"]["joints"],base["system"]["joints"])
            resolved=read_json(output/"resolved_inputs.json")
            load=resolved["bodies"]["load"]
            self.assertEqual(resolved["bodies"]["pusher"]["physics"]["mass_kg"],1.)
            self.assertIsNone(resolved["physics_profiles"]["asset_development"]["mass_kg"])
            with np.load(output/load["geometry"]["mesh"]["path"],allow_pickle=False) as mesh:
                z=manifest["initial_state"]["body_states"]["load"]["position_m"][2]
                self.assertAlmostEqual(float(mesh['vertices'][:,2].min())+z,0,places=7)

    def test_sdf_never_falls_back_on_cpu(self):
        config,_,_=self.inputs();config["numerics"]["gpu_dynamics"]=False
        with tempfile.TemporaryDirectory() as tmp:
            path=Path(tmp)/"config.json";write_json(path,config)
            with self.assertRaisesRegex(ValueError,"no convex fallback"):
                prepare(path,Path(tmp)/"episode")


if __name__ == "__main__":
    unittest.main()
