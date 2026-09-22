"""Review corrections: names, control/device independence and actual mesh reuse."""
import copy
import unittest
import numpy as np

from tests import test_experiment_construction as fixtures
PROFILES,ROOT=fixtures.PROFILES,fixtures.ROOT
from world_model_dataset.experiment_construct import construct
from world_model_dataset.experiment_construct import passage_camera
from world_model_dataset.experiment_geometry import Geometry
from world_model_dataset.experiment_contract import (load_experiment, native_config, validate_common,
                                                    upgrade_experiment, control_and_actuation)
from world_model_dataset.io import read_json


class RevisionTests(unittest.TestCase):
    setUp=fixtures.ConstructionTests.setUp
    request=fixtures.ConstructionTests.request
    mesh=fixtures.ConstructionTests.mesh
    rigid_request=fixtures.ConstructionTests.rigid_request
    def test_camera_rejects_source_wall_occlusion(self):
        r=self.request('cloth_drape')
        r['scene']['collision']['meshes'].append(self.mesh('wall',[.1,2,2],[.6,0,1.]))
        geo=Geometry(r['scene'])
        camera=passage_camera(geo,[0,0,.6],[.2,0,.6],1.)
        self.assertLess(camera['position_m'][0],.55)
        self.assertEqual(camera['target_m'],[.2,0,.6])

    def test_rearrangement_not_alias_for_collision_chain(self):
        r,p=self.rigid_request(); r['phenomenon']='multibody_rearrangement';p['phenomenon']=r['phenomenon']
        r['object']=[dict(r['object'],id='a'),dict(r['object'],id='b')]
        r['conditions']=dict(layer_gap_m=.008,offset_fraction=.2)
        doc,_,report=construct(r,p)
        placements=report['calculations']['placements']
        self.assertGreater(placements[1]['bounds_m'][0][2],placements[0]['bounds_m'][1][2])
        self.assertGreater(placements[1]['projected_overlap_fraction'],.25)
        self.assertEqual(doc['input']['participants'][0]['velocity_m_s'],[0,0,0])
        self.assertEqual(placements[1]['potential_support'],'a')
        legacy=load_experiment(ROOT/'configs/dataset/v0_2/experiment_api_v1/multibody.json')
        self.assertEqual(legacy['phenomenon'],'multibody_rearrangement')

    def test_v1_migration_preserves_native_input_and_device(self):
        old=read_json(ROOT/'configs/dataset/v0_2/experiment_api_v1/beam.json')
        new=upgrade_experiment(old)
        self.assertEqual(new['control'],dict(mode='impedance_control',enabled=True))
        self.assertEqual(new['actuation'],dict(kind='finite_plate',instance_ids=['Plate']))
        self.assertEqual(native_config(old),native_config(new))
        self.assertEqual(old['control'],'finite_plate')
        validate_common(new)
        wrong=copy.deepcopy(new);wrong['control']['mode']='effort_control'
        with self.assertRaisesRegex(ValueError,'Unsupported actuator/control combination'):
            validate_common(wrong)
        self.assertEqual(wrong['actuation'],new['actuation'])
        wrong=copy.deepcopy(new);wrong['actuation']['kind']='finite_load'
        with self.assertRaisesRegex(ValueError,'Unsupported actuator/control combination'):
            validate_common(wrong)

    def test_same_load_entity_disabled_mode_keeps_identity(self):
        active,entity=control_and_actuation('finite_load',True)
        disabled,same=control_and_actuation('finite_load',False)
        self.assertEqual(entity,same);self.assertEqual(active['mode'],disabled['mode'])
        self.assertIs(disabled['enabled'],False)

    def test_existing_mesh_topology_and_inferred_placement(self):
        r=self.request('cloth_drape');p=read_json(PROFILES/'cloth.json')
        vertices=np.array([[-.3,-.15,0],[.3,-.15,0],[.3,.15,0],[-.3,.15,0]],float)
        angle=.3;rotation=np.array([[np.cos(angle),-np.sin(angle),0],[np.sin(angle),np.cos(angle),0],[0,0,1]])
        vertices=vertices@rotation.T+[3,4,2]
        faces=np.array([[0,1,2],[0,2,3]],int);path=self.root/'cloth_asset.npz'
        np.savez(path,vertices=vertices,triangles=faces)
        r['object']=dict(kind='mesh',path=str(path))
        doc,native,report=construct(r,p)
        cloth=doc['input']['cloth'];tf=np.asarray(cloth['world_from_mesh'])
        placed=np.c_[vertices,np.ones(4)]@tf.T
        self.assertAlmostEqual(placed[:,0].max(),.68)
        self.assertAlmostEqual(placed[:,2].min(),.506)
        self.assertEqual(cloth['path'],str(path));self.assertNotIn('cells',cloth)
        np.testing.assert_array_equal(np.load(path)['triangles'],faces)
        self.assertEqual(report['calculations']['mesh_asset']['vertices'],4)

    def test_oriented_support_is_not_rotated_in_backend(self):
        r=self.request('cloth_drape');p=read_json(PROFILES/'cloth.json')
        path=r['scene']['collision']['meshes'][0]['path']
        with np.load(path) as arrays:v=arrays['vertices'];f=arrays['triangles']
        a=.2;rot=np.array([[np.cos(a),-np.sin(a),0],[np.sin(a),np.cos(a),0],[0,0,1]])
        np.savez(path,vertices=v@rot.T,triangles=f)
        scene=copy.deepcopy(r['scene']);r['scene']['region_bounds_m']=[[-1,-1,0],[1,1,1.5]]
        doc,_,report=construct(r,p)
        self.assertEqual(doc['scene']['collision'],scene['collision'])
        self.assertGreater(abs(doc['input']['cloth']['world_from_mesh'][1][0]),.1)
        self.assertAlmostEqual(report['calculations']['supported_area_fraction'],.7)


if __name__=='__main__': unittest.main()
