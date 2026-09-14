import tempfile
import unittest
from pathlib import Path

from world_model_dataset.m4 import ensure_r03_specs,generate_r03,ensure_v01_specs,generate_v01
from world_model_dataset.contract import CONFIG,resolve
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


if __name__=='__main__':unittest.main()
