import unittest
import numpy as np
from scipy.spatial.transform import Rotation

from world_model_dataset.metrics import nonrigid_residual,recovery_time,box_penetration,triangle_box_surface_audit,tet_boundary_faces
from world_model_dataset.geometry import make_geometry
from world_model_dataset.fixtures import box,stairs,obstacles,flat_ground,compression_plates
from world_model_dataset.contract import CONFIG
from world_model_dataset.io import read_json
from world_model_dataset.motion import fixed_topology_motion
from world_model_dataset.finalize import motion_validation_summary


class MetricsTests(unittest.TestCase):
    def test_rigid_motion_removed(self):
        rng=np.random.default_rng(123);x=rng.normal(size=(100,3))
        y=Rotation.from_euler('xyz',[.3,-.6,1.2]).apply(x)+[2,5,-1]
        self.assertLess(nonrigid_residual(x,y),1e-12)
        self.assertGreater(nonrigid_residual(x,1.2*y),.1)

    def test_censored_recovery(self):
        t=np.arange(11)/10
        self.assertIsNone(recovery_time(t,np.ones(11),.2,.1))
        v=np.ones(11);v[5:]=0
        self.assertAlmostEqual(recovery_time(t,v,.2,.1),.3)
        v[8]=1
        self.assertIsNone(recovery_time(t,v,.2,.1))

    def test_fixture_penetration(self):
        b=box('plate',[0,0,0],[2,2,2]);p={'plate':[0,0,0]}
        self.assertAlmostEqual(box_penetration(np.array([[0,0,.8]]),[b],p),.2)
        self.assertEqual(box_penetration(np.array([[0,0,1]]),[b],p),0)
        vertices=np.array([[-.5,-.5,.8],[.5,-.5,.8],[0,.5,.8]])
        report=triangle_box_surface_audit(vertices,[[0,1,2]],[b],p)
        self.assertEqual(report['intersecting_triangles'],1)
        self.assertAlmostEqual(report['maximum_sampled_penetration_m'],.2)

    def test_tet_boundary(self):
        faces=tet_boundary_faces([[0,1,2,3],[0,1,2,4]])
        self.assertEqual(len(faces),6)
        self.assertNotIn(tuple(sorted((0,1,2))),{tuple(sorted(f)) for f in faces})

    def test_fixed_topology_motion_uses_material_correspondence(self):
        calibration={
            'world_from_camera_usd':np.eye(4).tolist(),
            'intrinsic_opencv':[[100.,0.,50.],[0.,100.,50.],[0.,0.,1.]],
        }
        current=np.array([[-.2,.2,-2.],[.2,.2,-2.],[0.,-.2,-2.]])
        previous=current+np.array([-.02,0.,0.])
        segmentation=np.zeros((100,100),dtype=np.uint32);segmentation[40:61,40:61]=7
        depth=np.full((100,100),2.,dtype=np.float32)
        motion,valid=fixed_topology_motion(previous,current,np.array([[0,1,2]]),depth,
                                                   segmentation,[7],calibration)
        self.assertGreater(np.count_nonzero(valid),50)
        self.assertAlmostEqual(float(np.median(motion[valid,0])),-1.,places=5)
        self.assertAlmostEqual(float(np.median(motion[valid,1])),0.,places=5)

    def test_motion_validation_accepts_translation_or_true_stationarity(self):
        moving=motion_validation_summary([(0.95,1.1)]*5,[.9]*10,[1.2]*10,'moving')
        self.assertTrue(moving['passed']);self.assertEqual(moving['mode'],'translating')
        stationary=motion_validation_summary([], [1.]*10, [.001]*10, 'stationary')
        self.assertTrue(stationary['passed']);self.assertEqual(stationary['mode'],'stationary_zero_signal')
        self.assertFalse(motion_validation_summary([], [1.]*10, [1.]*10, 'stationary')['passed'])
        self.assertFalse(motion_validation_summary([], [.2]*10, [.001]*10, 'stationary')['passed'])

    def test_six_diagnostic_shapes(self):
        for name,entry in read_json(CONFIG/'objects.json')['entries'].items():
            with self.subTest(name=name):
                mesh=make_geometry(entry)
                self.assertTrue(mesh.is_watertight);self.assertGreater(mesh.volume,0)
                self.assertAlmostEqual(max(mesh.extents),entry['characteristic_size_m'])
                self.assertTrue(np.allclose(mesh.center_mass,0,atol=1e-8))

    def test_fixtures_scale_and_validity(self):
        for factory in (flat_ground,stairs,obstacles):
            a=factory(.2);b=factory(.4)
            for x,y in zip(a,b):self.assertTrue(np.allclose(np.array(x['size_m'])*2,y['size_m']))
        self.assertEqual(compression_plates(.2,.2)[-1]['id'],'upper_plate')


if __name__=='__main__':unittest.main()
