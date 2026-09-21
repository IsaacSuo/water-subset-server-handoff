import json
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest

import numpy as np
from scipy.spatial.transform import Rotation

from world_model_dataset.io import file_hash
from world_model_dataset.phenomenon_collect import apply_quality_evidence, effort_summary
from world_model_dataset.phenomenon_observations import GeometryView, posed


class RopeLoadTests(unittest.TestCase):
    def test_native_box_half_extents_shape_transform_and_body_pose(self):
        with tempfile.TemporaryDirectory() as tmp:
            native=Path(tmp)/'native.npz'
            np.savez(native,body_ids=[0,1],segment_body_ids=[0],load_body_ids=[1],
                shape_body=[0,1],shape_type=[4,7],shape_scale=[[.003,.01,0],[.03,.04,.05]],
                shape_transform=[[0,0,.01,0,0,0,1],[.1,.2,.3,0,0,0,1]])
            pose=dict(position_m=[1,2,3],orientation_xyzw=Rotation.from_euler('z',90,degrees=True).as_quat())
            ep=SimpleNamespace(manifest=dict(system=dict(bodies=[dict(instance_id='segment_0',role='subject'),dict(instance_id='load',role='actuator')])),
                resolved_inputs=lambda:dict(config=dict(kind='rope',loads=[dict(id='load')]),static_geometries={},raw_native={}),
                states=lambda:iter([dict(body_states={'load':pose})]),record_path=lambda r:native)
            view=GeometryView(ep);vertices,_=view.local['load']
            np.testing.assert_allclose(vertices.min(0),[.07,.16,.25])
            np.testing.assert_allclose(vertices.max(0),[.13,.24,.35])
            world=posed(vertices,pose)
            np.testing.assert_allclose(world.mean(0),[.8,2.1,3.3])
            # Earlier passive caches have only body_ids, no load metadata.
            np.savez(native,body_ids=[0],shape_body=[0],shape_scale=[[.003,.01,0]],shape_transform=[[0,0,.01,0,0,0,1]])
            ep.resolved_inputs=lambda:dict(config=dict(kind='rope'),static_geometries={},raw_native={})
            self.assertEqual(set(GeometryView(ep).local),{'segment_0'})

    def test_native_force_vector_and_cap(self):
        self.assertEqual(effort_summary([dict(applied_force_world_n=[-1.5,0,0])],1.5)['peak_applied_force_n'],1.5)
        for vector in ([0,2,0],[float('nan'),0,0],[1,2]):
            with self.assertRaises(ValueError):effort_summary([dict(applied_force_world_n=vector)],1.5)

    def test_functional_evidence_does_not_grant_use_admission(self):
        ep=SimpleNamespace(manifest={'trajectory':{'states':{'sha256':'current'}}})
        with tempfile.TemporaryDirectory() as tmp:
            p=Path(tmp)/'review.json'
            data=dict(format='rope-load-adoption/1',source_states_sha256=['current'],status='functional_cache_checked',
                checks={'native':True},use_review=dict(status='human_use_review_pending',training_admission=False))
            p.write_text(json.dumps(data))
            review=apply_quality_evidence(ep,{'warnings':[]},dict(path=str(p),sha256=file_hash(p)))
            self.assertFalse(review['fixed_configuration_state_use']['training_admission'])
            self.assertEqual(review['fixed_configuration_state_use']['status'],'human_use_review_pending')
            data['checks']['native']=False;p.write_text(json.dumps(data))
            with self.assertRaisesRegex(ValueError,'functional cache check failed'):
                apply_quality_evidence(ep,{'warnings':[]},dict(path=str(p),sha256=file_hash(p)))


if __name__=='__main__':unittest.main()
