import copy
import json
from pathlib import Path
import tempfile
import unittest

from world_model_dataset.phenomenon_templates import expand
from world_model_dataset.phenomenon_pilot_batch import run


class TemplateTests(unittest.TestCase):
    def setUp(self):
        self.base=dict(scene_export='unchanged_scene.json',shots=[dict(id='base',group=1,camera={},
            bodies=[dict(id='subject',asset='original',size_m=.1,mass_kg=.2,xy_m=[0,0],
                         material=dict(static_friction=.4,dynamic_friction=.3))])])
        self.template=dict(id='roll',family='rolling_friction',subject_id='subject',title='roll',
            assets=['onion','potato'],comparison_variable='friction',
            conditions=[dict(id='low',body=dict(material=dict(dynamic_friction=.1))),
                        dict(id='high',body=dict(material=dict(dynamic_friction=.6)))])

    def test_asset_list_uses_same_logic_and_preserves_scene_and_other_inputs(self):
        original=copy.deepcopy(self.base);expanded=expand(self.template,self.base)
        self.assertEqual(len(expanded),4);self.assertEqual(self.base,original)
        for spec,job in expanded:
            self.assertEqual(spec['scene_export'],original['scene_export'])
            body=spec['shots'][0]['bodies'][0]
            self.assertEqual(body['mass_kg'],.2);self.assertEqual(body['material']['static_friction'],.4)
            self.assertIn(body['asset'],self.template['assets'])
        expanded[0][0]['shots'][0]['bodies'][0]['xy_m'][0]=99
        self.assertEqual(expanded[1][0]['shots'][0]['bodies'][0]['xy_m'][0],0)

    def test_refuses_environment_edits_invalid_physics_and_duplicate_ids(self):
        for changes in (dict(scene_export='other'),dict(size_m=-1),dict(velocity_m_s=[1,0]),
                        dict(material=dict(dynamic_friction=float('nan')))):
            template=copy.deepcopy(self.template);template['conditions'][0]['body']=changes
            with self.assertRaises(ValueError):expand(template,self.base)
        self.template['assets']=['onion','onion']
        with self.assertRaisesRegex(ValueError,'Duplicate'):expand(self.template,self.base)

    def test_changed_input_refuses_before_output_or_gpu(self):
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp);source=root/'mesh.npz';source.write_bytes(b'changed')
            plan=root/'plan.json';plan.write_text(json.dumps(dict(input_sources={str(source):'old'},jobs=[])))
            with self.assertRaisesRegex(ValueError,'Pinned experiment input changed'):run(plan,root/'output')
            self.assertFalse((root/'output').exists())


if __name__=='__main__':unittest.main()
