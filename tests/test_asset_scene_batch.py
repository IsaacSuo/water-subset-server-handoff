import tempfile
import unittest
from pathlib import Path

import numpy as np
import trimesh

from world_model_dataset.asset_scene_batch import select_geometry,recipe
from world_model_dataset.causal_contract import audit_causal_manifest
from world_model_dataset.causal_runner import prepare
from world_model_dataset.io import read_json,write_json


class AssetSelectionTests(unittest.TestCase):
    def test_edge_selection_uses_support_not_name(self):
        box=trimesh.creation.box([.2,.2,.2])
        self.assertTrue(select_geometry(box,'support_edge')['selected'])
        cone=trimesh.creation.cone(radius=.1,height=.2)
        cone.apply_transform(trimesh.transformations.rotation_matrix(np.pi,[1,0,0]))
        self.assertFalse(select_geometry(cone,'support_edge')['selected'])

    def test_collision_aim_stays_above_ground_and_hits_mesh(self):
        mesh=trimesh.creation.box([.2,.2,.2])
        selected=select_geometry(mesh,'collision')
        self.assertTrue(selected['selected'])
        self.assertGreaterEqual(selected['local_aim_yz_m'][1]-mesh.bounds[0,2],selected['projectile_radius_m'])


EXPLORE=Path('/home/fangsuo/isaacsim_work_flexible_explore')


@unittest.skipUnless((EXPLORE/'output/physical_assets/library_v1/index.json').exists(),'Exploration asset delivery not installed')
class AssetSceneIntegrationTests(unittest.TestCase):
    def test_batch_layouts_preserve_none_control_and_nonintersecting_initial_bounds(self):
        for kind in ('falling','support_edge','collision'):
            for name in ('banana','elephant','chair','carrot'):
                config,manifest,selection=recipe(EXPLORE,name,kind)
                if config is None:
                    self.assertEqual(kind,'support_edge')
                    continue
                self.assertTrue(audit_causal_manifest(manifest)['accepted'])
                self.assertEqual(manifest['control_program']['primitive'],'none')
                with tempfile.TemporaryDirectory() as tmp:
                    folder=Path(tmp)
                    write_json(folder/'manifest.json',manifest)
                    config['manifest_template']=str(folder/'manifest.json')
                    write_json(folder/'config.json',config)
                    output=folder/'episode';prepared=prepare(folder/'config.json',output)
                    resolved=read_json(output/'resolved_inputs.json')
                    oid='load' if kind=='collision' else 'left'
                    geometry=resolved['bodies'][oid]['geometry']
                    self.assertEqual(geometry['collision_approximation'],'sdf')
                    self.assertEqual(geometry['sdf_resolution'],256)
                    state=prepared['initial_state']['body_states'][oid]
                    lo,hi=np.array(geometry['bounds_m'])+state['position_m']
                    if kind=='falling':self.assertAlmostEqual(lo[2],.4,places=6)
                    if kind=='support_edge':
                        self.assertAlmostEqual(lo[2],.45,places=6)
                        self.assertLess(hi[0],0)
                    if kind=='collision':
                        self.assertGreater(resolved['bodies']['floor']['geometry']['size_m'][0],8.)
                        projectile=prepared['initial_state']['body_states']['projectile']
                        radius=resolved['bodies']['projectile']['geometry']['radius_m']
                        self.assertAlmostEqual(lo[0]-projectile['position_m'][0]-radius,.1,places=6)
                        self.assertGreaterEqual(projectile['position_m'][2],radius)


if __name__=='__main__':unittest.main()
