import copy
import unittest
from tests import test_experiment_construction as fixtures
from world_model_dataset.experiment_construct import construct
from world_model_dataset.io import read_json


class AutomaticLayouts(unittest.TestCase):
    setUp=fixtures.ConstructionTests.setUp
    mesh=fixtures.ConstructionTests.mesh
    request=fixtures.ConstructionTests.request

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
