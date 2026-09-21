import unittest
import numpy as np

from world_model_dataset.phenomenon_observations import Raster


class DiagnosticObservationTests(unittest.TestCase):
    def setUp(self):
        self.cam=dict(resolution=[5,5],intrinsic_opencv=[[4,0,2],[0,4,2],[0,0,1]],
                      world_from_camera_cv=np.eye(3).tolist(),camera_origin_m=[0,0,0],near_m=.01)

    def test_no_implicit_support_and_metric_depth(self):
        raster=Raster(self.cam)
        self.assertFalse(raster.arrays()['depth_valid'].any())
        v=np.array([[-1,-1,2],[1,-1,2],[1,1,2],[-1,1,2]])
        raster.mesh(v,[[0,1,2],[0,2,3]],7,[1,0,0])
        self.assertAlmostEqual(float(raster.arrays()['depth_m'][2,2]),2.)
        self.assertEqual(raster.seg[2,2],7)
        raster.mesh(v+np.array([0,0,1]),[[0,1,2],[0,2,3]],8,[0,0,1])
        self.assertEqual(raster.seg[2,2],7)

    def test_near_plane_crossing_is_clipped(self):
        raster=Raster(self.cam)
        raster.mesh([[-1,-1,-.1],[1,-1,1],[0,1,1]],[[0,1,2]],3,[1,1,1])
        a=raster.arrays()
        self.assertTrue(a['depth_valid'].any())
        self.assertTrue(np.all(a['depth_m'][a['depth_valid']]>=.01))

    def test_particle_glyph_depth_does_not_invent_connectivity(self):
        raster=Raster(self.cam)
        raster.spheres(np.array([[0,0,2.]]),np.array([.25]),4,[1,0,0])
        self.assertAlmostEqual(raster.arrays()['depth_m'][2,2],1.75)
        self.assertEqual(raster.seg[2,2],4)


if __name__=='__main__':unittest.main()
