import json
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch

import numpy as np

from world_model_dataset.io import file_hash
from world_model_dataset.phenomenon_catalog import build
from world_model_dataset.phenomenon_collect import apply_quality_evidence, effort_summary
from world_model_dataset.phenomenon_observations import GeometryView, tet_boundary


class BeamAdapterTests(unittest.TestCase):
    def test_pair_evidence_accepts_both_members_but_rejects_other_cache_and_failed_review(self):
        with tempfile.TemporaryDirectory() as tmp:
            path=Path(tmp)/'review.json'
            evidence=dict(format='beam-pair-review/1',source_states_sha256=['load','hold'],
                          status='paired_bending_checked',checks={'clearance':True})
            path.write_text(json.dumps(evidence))
            record=dict(path=str(path),sha256=file_hash(path))
            for state_hash in ('load','hold','other'):
                ep=SimpleNamespace(manifest={'trajectory':{'states':{'sha256':state_hash}}})
                if state_hash=='other':
                    with self.assertRaisesRegex(ValueError,'different physical cache'):apply_quality_evidence(ep,{},record)
                else:self.assertIn('beam_pair_review',apply_quality_evidence(ep,{},record))
            evidence['checks']['clearance']=False
            path.write_text(json.dumps(evidence));record['sha256']=file_hash(path)
            ep=SimpleNamespace(manifest={'trajectory':{'states':{'sha256':'load'}}})
            with self.assertRaisesRegex(ValueError,'did not pass'):apply_quality_evidence(ep,{},record)

    def test_nested_catalog_preserves_supersession_and_does_not_revive_old_cache(self):
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp);(root/'episode.json').write_text('{}')
            item=dict(episode=str(root),manifest_sha256=file_hash(root/'episode.json'),
                      source_states_sha256='state',states=0,observations=0,group=3)
            old=dict(item,id='unstable');new=dict(item,id='stable')
            history=dict(old_episode=old,replacement_id='stable',reason='explicit selection')
            (root/'first.json').write_text(json.dumps(dict(episodes=[old,new])))
            (root/'selected.json').write_text(json.dumps(dict(episodes=[new],superseded=[history])))
            ep=SimpleNamespace(manifest={'trajectory':{'states':{'sha256':'state'}}},
                               states=lambda:iter([]),observations=lambda:iter([]))
            with patch('world_model_dataset.phenomenon_catalog.open_episode',return_value=ep):
                result=build([root/'first.json',root/'selected.json'],[],root/'result.json',True)
            self.assertEqual([r['id'] for r in result['episodes']],['stable'])
            self.assertEqual(result['superseded'],[history])

    def test_tet_surface_removes_internal_face_and_orients_outwards(self):
        points=np.array([[0,0,0],[1,0,0],[0,1,0],[0,0,1],[0,0,-1]],float)
        faces=tet_boundary([[0,1,2,3],[0,2,1,4]],points)
        self.assertEqual(len(faces),6)
        self.assertNotIn((0,1,2),[tuple(sorted(f)) for f in faces])
        for f in faces:
            a,b,c=points[f]
            self.assertGreater(np.dot(np.cross(b-a,c-a),(a+b+c)/3-points.mean(0)),0)

    def test_force_vector_bound_and_missing_saturation_are_explicit(self):
        result=effort_summary([dict(force_world_n=[-3,4,0])],5)
        self.assertEqual(result['peak_applied_force_n'],5)
        self.assertIsNone(result['saturated_step_fraction'])
        self.assertEqual(result['saturation_record_status'],'unavailable')
        with self.assertRaisesRegex(ValueError,'exceeds'):
            effort_summary([dict(force_world_n=[3,4,1])],5)
        old=effort_summary([dict(applied_force_n=-3,saturated=True)],3)
        self.assertEqual(old['saturated_step_fraction'],1)

    def test_beam_renderer_preserves_fixture_and_actual_actuator_pose(self):
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp)
            points=np.array([[0,0,0],[1,0,0],[0,1,0],[0,0,1]],float)
            np.savez(root/'beam.npz',simulation_world_m=points,simulation_tets=[[0,1,2,3]],
                     surface_world_m=points+99,surface_triangles=[[0,1,2]])
            np.savez(root/'scene.npz',vertices=points[:3],triangles=[[0,1,2]])
            descriptors=[dict(instance_id=k,role=r,physics_kind=p) for k,r,p in
                [('Beam','subject','volumetric'),('Plate','actuator','rigid'),('Fixture','environment','rigid'),('Table','environment','static')]]
            states={b['instance_id']:dict(position_m=[0,0,0],orientation_xyzw=[0,0,0,1]) for b in descriptors}
            states['Beam']['geometry']='beam.npz';states['Plate']['position_m']=[2,0,0]
            resolved=dict(config=dict(kind='beam'),bodies={
                'Beam':dict(geometry=dict(shape='soft_box',size_m=[1,1,1])),
                'Plate':dict(geometry=dict(shape='box',size_m=[.1,.1,.1])),
                'Fixture':dict(geometry=dict(shape='box',size_m=[.1,.1,.1])),
                'Table':dict(geometry=dict(shape='mesh',mesh='scene.npz'))})
            ep=SimpleNamespace(manifest={'system':{'bodies':descriptors}},resolved_inputs=lambda:resolved,
                states=lambda:iter([dict(body_states=states)]),record_path=lambda name:root/name)
            view=GeometryView(ep)
            self.assertEqual(set(view.local),{'Plate','Fixture'})
            self.assertEqual(set(view.fixed),{'Table'})
            captured={}
            raster=SimpleNamespace(mesh=lambda v,f,oid,color:captured.update({oid:np.asarray(v).copy()}))
            view.render(raster,dict(body_states=states))
            np.testing.assert_allclose(captured[view.body_ids['Plate']].mean(0),[2,0,0])
            np.testing.assert_array_equal(captured[view.body_ids['Beam']],points)


if __name__=='__main__':unittest.main()
