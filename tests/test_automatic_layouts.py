import copy
import unittest
import numpy as np
from tests import test_experiment_construction as fixtures
from world_model_dataset.experiment_construct import construct
from world_model_dataset.io import read_json


class AutomaticLayouts(unittest.TestCase):
    setUp=fixtures.ConstructionTests.setUp
    mesh=fixtures.ConstructionTests.mesh
    request=fixtures.ConstructionTests.request

    def test_beam_self_weight_rejects_open_loop_miss(self):
        r=self.request('beam_load_hold_withdraw');r['object']=dict(size_m=[.38,.05,.025])
        r['conditions']=dict(clamp_fraction=.13,deflection_fraction=.2,max_force_n=15.)
        p=read_json(fixtures.PROFILES/'beam.json')
        with self.assertRaisesRegex(ValueError,'beam_self_weight'):construct(r,p)
        r['object']['size_m']=[.22,.05,.03];_,_,report=construct(r,p)
        self.assertLess(report['calculations']['linear_self_weight_tip_estimate_m'],.008)

    def test_wrap_preserves_bar_and_rejects_impossible_length(self):
        r=self.request('rope_wrap');bar=self.mesh('rod',[.4,.006,.006],[0,0,.3])
        from world_model_dataset.io import write_json,file_hash
        source=self.root/'scene.json'
        write_json(source,dict(groups={'rod':dict(sha256=file_hash(bar['path']),objects=[dict(name='bar',triangle_start=0,triangle_count=12)])}))
        r['scene'].update(collision=dict(meshes=[bar]),source_records=dict(scene=str(source)))
        r['support_group']=dict(group='rod',object_index=0)
        r['object']=dict(length_m=.16,radius_m=.0015,density_kg_m3=800)
        r['conditions']=dict(end_condition='first_clamped',left_leg_fraction=.5)
        p=read_json(fixtures.PROFILES/'rope_wrap.json');doc,_,report=construct(r,p)
        self.assertEqual(doc['scene']['collision'],r['scene']['collision'])
        self.assertAlmostEqual(report['calculations']['centerline_length_m'],.16)
        line=np.array(doc['input']['rope']['centerline_m'])
        self.assertGreater(line[:,2].max(),.303+.0015)
        self.assertLess(line[0,2],.3)
        r['object']['length_m']=.02
        with self.assertRaisesRegex(ValueError,'wrap_length'):construct(r,p)

    def test_load_anchors_and_disabled_control(self):
        r=self.request('rope_finite_load');r['object']=dict(length_m=.3,radius_m=.002,density_kg_m3=700,
            loads=[dict(size_m=[.05]*3,mass_kg=.1,friction=.4),dict(size_m=[.06]*3,mass_kg=.2,friction=.4)])
        r['conditions']=dict(slack_fraction=.5,travel_m=.1,max_force_n=1.,enabled=True)
        p=read_json(fixtures.PROFILES/'rope_load.json');a,_,report=construct(r,p)
        line=np.array(a['input']['rope']['centerline_m'])
        self.assertAlmostEqual(np.linalg.norm(np.diff(line,axis=0),axis=1).sum(),.3)
        for load,end in zip(a['input']['loads'],[line[0],line[-1]]):
            np.testing.assert_allclose(end,np.array(load['position_m'])+load['anchor_local_m'],atol=1e-12)
        r['conditions']['enabled']=False;b,_,_=construct(r,p)
        self.assertFalse(b['control']['enabled']);self.assertEqual(a['actuation'],b['actuation'])
        b['input']['load_control']['enabled']=True;self.assertEqual(a['input'],b['input'])

    def test_drag_conditions_preserve_nonintervention_physics(self):
        r=self.request('cloth_drag');r['object']={'size_m':[.4,.28]}
        r['conditions']=dict(grip_fraction=.06,travel_m=.1,max_force_n=4.,mode='free')
        p=read_json(fixtures.PROFILES/'cloth_drag.json');a,_,report=construct(r,p)
        self.assertGreaterEqual(report['calculations']['attachments']['gripper']['node_count'],2)
        low=copy.deepcopy(r);low['conditions'].update(mode='low_force',max_force_n=.2)
        b,_,_=construct(low,p);b['input']['gripper']['max_force_n']=4
        self.assertEqual(a['input'],b['input'])
        blocked=copy.deepcopy(r);blocked['conditions']['mode']='blocked';c,_,_=construct(blocked,p)
        opposing=c['input'].pop('opposing_fixture');self.assertEqual(c['input'],a['input'])
        self.assertLess(opposing['attachment_bounds_world_m'][1][0],a['input']['gripper']['attachment_bounds_world_m'][0][0])
        self.assertEqual(a['input']['material']['youngs_modulus_Pa'],200000)
        too_far=copy.deepcopy(r);too_far['conditions']['travel_m']=5
        with self.assertRaises(ValueError):construct(too_far,p)


if __name__=='__main__':unittest.main()
