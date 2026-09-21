import json
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest

import numpy as np

from world_model_dataset.io import file_hash
from world_model_dataset.phenomenon_collect import apply_quality_evidence
from world_model_dataset.phenomenon_observations import GeometryView


class GripperAdoptionTests(unittest.TestCase):
    def test_visual_and_functional_acceptance_does_not_clear_stability_warning(self):
        ep=SimpleNamespace(manifest={'trajectory':{'states':{'sha256':'free'}}})
        with tempfile.TemporaryDirectory() as tmp:
            path=Path(tmp)/'review.json'
            data=dict(format='cloth-gripper-adoption/1',status='functional_cache_checked',
                      source_states_sha256=['free','blocked','low'],checks={'functional':True},training_admission=False)
            path.write_text(json.dumps(data));record=dict(path=str(path),sha256=file_hash(path))
            review=apply_quality_evidence(ep,dict(warnings=['Cloth contact stability pending']),record)
            self.assertIn('Cloth contact stability pending',review['warnings'])
            self.assertFalse(review['cloth_gripper_adoption']['training_admission'])
            data['checks']['functional']=False;path.write_text(json.dumps(data));record['sha256']=file_hash(path)
            with self.assertRaisesRegex(ValueError,'functional cache check failed'):apply_quality_evidence(ep,{},record)

    def test_gripper_fixture_and_native_cloth_all_use_recorded_geometry(self):
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp);nodes=np.array([[1,0,0],[2,0,0],[1,1,0]],float)
            np.savez(root/'cloth.npz',surface_world_m=nodes,surface_triangles=[[0,1,2]])
            descriptors=[dict(instance_id=k,role=r,physics_kind=p) for k,r,p in
                [('cloth','subject','surface'),('Gripper','actuator','rigid'),('OpposingFixture','environment','rigid')]]
            states={b['instance_id']:dict(position_m=[0,0,0],orientation_xyzw=[0,0,0,1]) for b in descriptors}
            states['cloth']['geometry']='cloth.npz';states['Gripper']['position_m']=[3,0,0]
            states['OpposingFixture']['position_m']=[1,0,.1]
            resolved=dict(config=dict(kind='cloth'),bodies={k:dict(geometry=dict(shape='box',size_m=[.02,.08,.01])) for k in ('Gripper','OpposingFixture')})
            ep=SimpleNamespace(manifest={'system':{'bodies':descriptors}},resolved_inputs=lambda:resolved,
                               states=lambda:iter([dict(body_states=states)]),record_path=lambda record:root/record)
            view=GeometryView(ep);captured={}
            view.render(SimpleNamespace(mesh=lambda v,f,oid,color:captured.update({oid:np.asarray(v)})),dict(body_states=states))
            np.testing.assert_array_equal(captured[view.body_ids['cloth']],nodes)
            np.testing.assert_allclose(captured[view.body_ids['Gripper']].mean(0),[3,0,0])
            np.testing.assert_allclose(captured[view.body_ids['OpposingFixture']].mean(0),[1,0,.1])


if __name__=='__main__':unittest.main()
