import copy
import tempfile
from pathlib import Path
import unittest

from world_model_dataset.causal_runner import ROOT
from world_model_dataset.causal_subset import freeze_sources, observation_cache_matches
from world_model_dataset.io import read_json, write_json, file_hash
from world_model_dataset.native_scene_batch import recipe


class SubsetTests(unittest.TestCase):
    def test_reuse_rejects_changed_geometry_or_appearance_even_with_same_state(self):
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory)
            (root/'observations').mkdir()
            write_json(root/'observations/index.json',dict(complete=True,source_state_sha256='state'))
            profile={'camera_set': {'id': 'test'}}
            write_json(root/'observation_profile.json',profile)
            write_json(root/'resolved_inputs.json',{'geometry': 'original', 'appearance': 'orange'})
            source=dict(source_state_sha256='state',source_resolved_sha256=file_hash(root/'resolved_inputs.json'))
            self.assertTrue(observation_cache_matches(root,profile,source))
            self.assertFalse(observation_cache_matches(root,dict(profile,camera_lights={}),source))
            source['source_resolved_sha256']='changed-appearance-or-geometry'
            self.assertFalse(observation_cache_matches(root,profile,source))

    @unittest.skipUnless((ROOT/'output/world_model_dataset/v0_2/blue_wall_native01/edge_speed12/episode/episode.physics.json').exists(), 'local source caches not installed')
    def test_family_variants_cannot_cross_splits(self):
        spec=read_json(ROOT/'configs/dataset/v0_2/c4_rigid_micro01.json')
        small=copy.deepcopy(spec)
        small['episodes']=[small['episodes'][4],small['episodes'][5]]
        small['episodes'][1]['group']='other'
        small['groups']['other']='test'
        with self.assertRaisesRegex(ValueError,'leakage'):
            freeze_sources(small)

    @unittest.skipUnless((ROOT/'output/world_model_dataset/v0_2/native_scene_batch02/chain_control/warehouse_chain_ball20_far/episode/episode.physics.json').exists(), 'local source caches not installed')
    def test_selected_sources_have_one_split_each(self):
        spec=read_json(ROOT/'configs/dataset/v0_2/c4_rigid_micro01.json')
        frozen=freeze_sources(spec)
        self.assertEqual(len(frozen),len(spec['episodes']))
        self.assertEqual({r['split'] for r in frozen},{'train','validation','test'})

    @unittest.skipUnless((ROOT/'output/world_model_dataset/v0_2/native_scene_batch02/warehouse_geometry/scene.json').exists(), 'local scene export not installed')
    def test_native_scene_keeps_finite_actuator_and_static_originals(self):
        spec=read_json(ROOT/'configs/dataset/v0_2/native_scene_batch02.json')
        ex=spec['experiments'][0]
        c,m=recipe(spec,ex,spec['assets']['block20'],ex['conditions'][0])
        self.assertEqual(m['control_program']['primitive'],'effort_control')
        self.assertEqual(m['control_program']['controllers'][0]['max_force_n'],8.)
        self.assertEqual(len(m['environment']['environment_body_ids']),3)
        for oid in m['environment']['environment_body_ids']:
            self.assertEqual(m['initial_state']['body_states'][oid]['position_m'],[0,0,0])
            self.assertIn('static_scene_source',c['geometry_profiles'][oid])


if __name__=='__main__':unittest.main()
