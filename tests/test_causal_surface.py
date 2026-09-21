import copy
import json
from pathlib import Path
import tempfile
import unittest

import numpy as np

from world_model_dataset.causal_loader import open_episode
from world_model_dataset.causal_runner import ROOT
from world_model_dataset.io import read_json, write_json
from world_model_dataset.probe_episode import artifact


class NativeRepresentationTests(unittest.TestCase):
    def fixture(self, kind='surface', geometry_time=0., fields=None):
        temp=tempfile.TemporaryDirectory();self.addCleanup(temp.cleanup);root=Path(temp.name)
        m=read_json(ROOT/'configs/dataset/v0_2/examples/rigid_collision_none.json')
        body=dict(m['system']['bodies'][0],instance_id='sample',physics_kind=kind)
        m['system']['bodies']=[body];m['environment']['environment_body_ids']=[]
        pose=dict(position_m=[0,0,0],orientation_xyzw=[0,0,0,1],linear_velocity_m_s=[0,0,0],angular_velocity_rad_s=[0,0,0])
        m['initial_state'].update(participant_ids=['sample'],body_states={'sample':pose})
        np.savez(root/'geometry.npz',time_s=geometry_time,physics_step=0,**(fields or dict(
            surface_world_m=np.array([[0.,0,0],[1,0,0],[0,1,0]]),surface_nodal_velocities_m_s=np.zeros((3,3)),surface_triangles=[[0,1,2]])))
        write_json(root/'topology.json',{'representation':kind,'connectivity':None if kind=='volumetric' else [[0,1,2]]})
        state=dict(pose,geometry=artifact(root,'geometry.npz','synthetic native field fixture'),topology=artifact(root,'topology.json','fixture topology'))
        (root/'states.jsonl').write_text(json.dumps(dict(time_s=0.,physics_step=0,body_states={'sample':state}))+'\n')
        m['trajectory']['states']=artifact(root,'states.jsonl','test states')
        write_json(root/'episode.json',m)
        return root

    def test_surface_uses_core_loader_without_tets(self):
        ep=open_episode(self.fixture(),require_complete=False)
        row,g=next(ep.soft_geometries('sample'))
        self.assertEqual(g['surface_triangles'].shape,(1,3))
        self.assertNotIn('simulation_tets',g)
        self.assertEqual(ep.soft_topology('sample')['representation'],'surface')

    def test_time_and_artifact_mutations_fail(self):
        ep=open_episode(self.fixture(geometry_time=.1),require_complete=False)
        with self.assertRaisesRegex(ValueError,'time metadata'):list(ep.soft_geometries('sample'))
        root=self.fixture();(root/'geometry.npz').write_bytes(b'changed')
        with self.assertRaisesRegex(ValueError,'Artifact changed'):list(open_episode(root,require_complete=False).soft_geometries('sample'))

    def test_material_points_and_cable_stay_distinct(self):
        ep=open_episode(self.fixture(kind='volumetric',fields=dict(particle_world_m=np.zeros((4,3)),material_Jp=np.ones(4))),require_complete=False)
        g=next(ep.soft_geometries('sample'))[1]
        self.assertIn('material_Jp',g);self.assertNotIn('surface_triangles',g)
        ep=open_episode(self.fixture(kind='rigid',fields=dict(body_q=np.zeros((2,7)),centerline=np.zeros((3,3)))),require_complete=False)
        self.assertIn('body_q',next(ep.geometries('sample'))[1])
        with self.assertRaisesRegex(ValueError,'volumetric or surface'):list(ep.soft_geometries('sample'))

    def test_nonfinite_geometry_rejected(self):
        ep=open_episode(self.fixture(fields=dict(surface_world_m=np.array([[np.nan,0,0]]))),require_complete=False)
        with self.assertRaisesRegex(ValueError,'Nonfinite'):list(ep.geometries('sample'))


if __name__=='__main__':unittest.main()
