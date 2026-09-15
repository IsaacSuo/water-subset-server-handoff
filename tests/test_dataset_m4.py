import tempfile
import unittest
from pathlib import Path

from world_model_dataset.m4 import (ensure_r02_specs,ensure_r03_specs,ensure_v01_specs,
                                    generate_r02,generate_r03,generate_v01,
                                    generate_v02_corrected,ensure_v02_corrected_specs)
from world_model_dataset.contract import CONFIG,resolve,validate_pair
from world_model_dataset.actions import compile_actions
from world_model_dataset.fixtures import build_fixture
from world_model_dataset.geometry import make_geometry
from world_model_dataset.io import read_json


class M4Tests(unittest.TestCase):
    def test_r03_matrix(self):
        rows,pairs=generate_r03()
        self.assertEqual(len(rows),11);self.assertEqual(len(pairs),8)
        self.assertEqual({row['objects'][0]['object_id'] for row in rows},{'sphere','rounded_cube','cylinder'})
        self.assertEqual(sum(row['episode_id'].endswith('lateral_miss') for row in rows),1)

    def test_r03_specs_are_immutable(self):
        with tempfile.TemporaryDirectory() as temp:
            root=Path(temp)/'r03';self.assertEqual(ensure_r03_specs(root),ensure_r03_specs(root))

    def test_v01_free_drop_contract(self):
        spec=read_json(CONFIG/'examples/v01_rounded_cube_drop.json');resolved=resolve(spec)
        self.assertEqual(resolved['event']['id'],'V01');self.assertEqual(compile_actions(spec,resolved)['commands'],[])
        vertices=make_geometry(resolved['objects'][0]['geometry']).vertices
        fixture=build_fixture(spec,resolved,vertices)
        self.assertAlmostEqual(fixture['drop_clearance_m'],.3)
        self.assertAlmostEqual(fixture['subject_position_m'][2]+vertices[:,2].min(),.3)

    def test_v01_matrix(self):
        rows,pairs=generate_v01();self.assertEqual(len(rows),11);self.assertEqual(len(pairs),8)
        self.assertEqual({row['objects'][0]['object_id'] for row in rows},{'rounded_cube','sphere','capsule'})
        with tempfile.TemporaryDirectory() as temp:
            root=Path(temp)/'v01';self.assertEqual(ensure_v01_specs(root),ensure_v01_specs(root))

    def test_r02_stair_smoke_contract(self):
        spec=read_json(CONFIG/'examples/r02_rounded_cube_stairs.json');resolved=resolve(spec)
        vertices=make_geometry(resolved['objects'][0]['geometry']).vertices
        fixture=build_fixture(spec,resolved,vertices);actions=compile_actions(spec,resolved,fixture)
        self.assertEqual(resolved['event']['id'],'R02');self.assertEqual(len(fixture['step_ids']),5)
        self.assertAlmostEqual(fixture['top_surface_z_m'],.3)
        self.assertAlmostEqual(fixture['start_edge_clearance_m'],.01)
        self.assertEqual(actions['commands'][0]['kind'],'initial_velocity')
        self.assertEqual(actions['commands'][0]['parameters']['linear_m_s'],[1.4,0.,0.])

    def test_r02_matrix(self):
        rows,pairs=generate_r02();self.assertEqual(len(rows),11);self.assertEqual(len(pairs),8)
        self.assertEqual({row['objects'][0]['object_id'] for row in rows},{'rounded_cube','sphere','capsule'})
        with tempfile.TemporaryDirectory() as temp:
            root=Path(temp)/'r02';self.assertEqual(ensure_r02_specs(root),ensure_r02_specs(root))

    def test_v05_mixed_impact_contract(self):
        spec=read_json(CONFIG/'examples/v05_sphere_soft_cube_impact.json');resolved=resolve(spec)
        vertices={o['instance_id']:make_geometry(o['geometry']).vertices for o in resolved['objects']}
        fixture=build_fixture(spec,resolved,vertices);actions=compile_actions(spec,resolved,fixture)
        self.assertEqual(resolved['event']['id'],'V05')
        self.assertEqual(fixture['target_id'],'target');self.assertEqual(fixture['projectile_id'],'projectile')
        self.assertEqual(actions['commands'][0]['target'],'projectile')
        self.assertEqual(actions['commands'][0]['parameters']['linear_m_s'],[1.5,0.,0.])
        miss=read_json(CONFIG/'examples/v05_sphere_soft_cube_miss.json')
        self.assertEqual(validate_pair(spec,miss),['/fixture_parameters/impact_offset_D'])
        miss_fixture=build_fixture(miss,resolve(miss),vertices)
        self.assertEqual(fixture['boxes'],miss_fixture['boxes'])
        self.assertGreater(fixture['boxes'][0]['size_m'][0]/2,1.5*3.)

    def test_corrected_v02_matrix(self):
        rows,pairs=generate_v02_corrected()
        self.assertEqual(len(rows),9);self.assertEqual(len(pairs),6)
        self.assertEqual({r['fixture_parameters']['compression_fraction'] for r in rows},{.1,.25,.4})
        self.assertEqual({r['objects'][0]['physics_profile_id'] for r in rows},
                         {'elastic_reference','elastic_soft','elastic_stiff'})
        self.assertTrue(all(r['numerics_profile_id']=='contact_convergence_128' for r in rows))
        self.assertTrue(all(r['episode_id'].endswith('_material_corrected') for r in rows))
        with tempfile.TemporaryDirectory() as temp:
            root=Path(temp)/'v02';self.assertEqual(ensure_v02_corrected_specs(root),ensure_v02_corrected_specs(root))

    def test_v05_miss_velocity_excludes_launch_from_rest(self):
        from world_model_dataset.audit_v05 import velocity_comparison_indices
        times=[0.,.25,.5,.75,1.,1.25]
        self.assertEqual(velocity_comparison_indices(times,.5,None,None),(3,5))
        self.assertEqual(velocity_comparison_indices(times,.5,.8,1.05),(3,5))


if __name__=='__main__':unittest.main()
