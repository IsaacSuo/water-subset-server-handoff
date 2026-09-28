import unittest

import numpy as np

from coupled_scene.gpu_dfsph.thin_features import anisotropic_pressure_tensor


class ThinFeaturePressureFilterTests(unittest.TestCase):
    def test_supported_bulk_is_unchanged(self):
        covariance=np.diag([4.,2.,1.])
        np.testing.assert_array_equal(anisotropic_pressure_tensor(covariance,20),np.eye(3))

    def test_isotropic_deficiency_is_unchanged(self):
        np.testing.assert_allclose(anisotropic_pressure_tensor(np.eye(3)*7,12),np.eye(3),atol=1e-14)

    def test_planar_neighborhood_suppresses_only_normal_pressure(self):
        tensor=anisotropic_pressure_tensor(np.diag([3.,3.,0.]),12)
        np.testing.assert_allclose(tensor,np.diag([1.,1.,0.]),atol=1e-14)

    def test_rotated_sheet_is_rotation_covariant(self):
        axis=np.array([1.,2.,3.]);axis/=np.linalg.norm(axis)
        projector=np.eye(3)-np.outer(axis,axis)
        tensor=anisotropic_pressure_tensor(projector*5,8)
        np.testing.assert_allclose(tensor@axis,np.zeros(3),atol=1e-14)
        tangent=np.cross(axis,[0.,0.,1.]);tangent/=np.linalg.norm(tangent)
        np.testing.assert_allclose(tensor@tangent,tangent,atol=1e-14)

    def test_line_neighborhood_retains_axial_pressure(self):
        tensor=anisotropic_pressure_tensor(np.diag([0.,0.,9.]),6)
        np.testing.assert_allclose(tensor,np.diag([0.,0.,1.]),atol=1e-14)

    def test_one_sided_free_surface_retains_more_normal_pressure(self):
        covariance=np.diag([3.,3.,1.])
        symmetric=anisotropic_pressure_tensor(covariance,12,first_moment=[0.,0.,0.],weighted_radius_sum=1.)
        one_sided=anisotropic_pressure_tensor(covariance,12,first_moment=[0.,0.,-.75],weighted_radius_sum=1.)
        self.assertGreater(one_sided[2,2],symmetric[2,2])
        np.testing.assert_allclose(one_sided[:2,:2],np.eye(2),atol=1e-14)

    def test_unreliable_degenerate_neighborhood_falls_back_to_identity(self):
        np.testing.assert_array_equal(anisotropic_pressure_tensor(np.zeros((3,3)),1),np.eye(3))


if __name__=='__main__':unittest.main()
