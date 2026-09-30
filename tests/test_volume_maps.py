import math
import unittest

import numpy as np

from coupled_scene.gpu_dfsph.volume_maps import (
    REFERENCE_VOLUME_SCALE,
    cubic_extension,
    map_cache_key,
    planar_boundary_volume,
    sphere_gauss_legendre_quadrature,
)


class CubicExtensionTests(unittest.TestCase):
    def test_piecewise_values_and_smooth_endpoints(self):
        h = 0.008
        self.assertEqual(cubic_extension(-0.001, h), 1.0)
        self.assertEqual(cubic_extension(0.0, h), 1.0)
        self.assertEqual(cubic_extension(0.5 * h, h), 0.25)
        self.assertEqual(cubic_extension(h, h), 0.0)
        self.assertEqual(cubic_extension(2.0 * h, h), 0.0)
        eps = 1.0e-7 * h
        self.assertAlmostEqual(cubic_extension(eps, h), 1.0, places=12)
        self.assertAlmostEqual(cubic_extension(h - eps, h), 0.0, places=12)

    def test_vectorized_input(self):
        values = cubic_extension(np.asarray([-1.0, 0.0, 0.5, 1.0]), 1.0)
        np.testing.assert_allclose(values, [1.0, 1.0, 0.25, 0.0])


class VolumeQuadratureTests(unittest.TestCase):
    def test_quadrature_points_stay_inside_support(self):
        offsets, weights = sphere_gauss_legendre_quadrature(0.008)
        self.assertGreater(len(offsets), 0)
        self.assertEqual(len(offsets), len(weights))
        self.assertTrue(np.all(np.linalg.norm(offsets, axis=1) <= 0.008 * (1.0 + 1.0e-7)))
        self.assertTrue(np.all(weights > 0.0))

    def test_planar_volume_is_monotone_and_compact(self):
        h = 0.008
        samples = [planar_boundary_volume(d, h) for d in (0.0, 0.25 * h, 0.5 * h, 0.75 * h, h, 2.0 * h)]
        self.assertTrue(all(a >= b for a, b in zip(samples, samples[1:])))
        self.assertGreater(samples[0], 0.0)
        self.assertAlmostEqual(samples[-1], 0.0, delta=1.0e-18)

    def test_volume_scales_with_support_radius_cubed(self):
        h = 0.008
        scale = 1.7
        base = planar_boundary_volume(0.4 * h, h)
        compared = planar_boundary_volume(0.4 * scale * h, scale * h)
        self.assertAlmostEqual(compared, base * scale**3, delta=base * scale**3 * 2.0e-7)

    def test_reference_scale_is_locked(self):
        self.assertEqual(REFERENCE_VOLUME_SCALE, 0.8)


class VolumeMapIdentityTests(unittest.TestCase):
    def test_cache_key_changes_with_geometry_or_discretization(self):
        vertices = np.asarray([[0, 0, 0], [1, 0, 0], [0, 1, 0]], np.float32)
        triangles = np.asarray([[0, 1, 2]], np.int32)
        key = map_cache_key(vertices, triangles, 0.008, 0.002, 16)
        self.assertNotEqual(key, map_cache_key(vertices + 0.1, triangles, 0.008, 0.002, 16))
        self.assertNotEqual(key, map_cache_key(vertices, triangles, 0.009, 0.002, 16))
        self.assertNotEqual(key, map_cache_key(vertices, triangles, 0.008, 0.003, 16))
        self.assertNotEqual(key, map_cache_key(vertices, triangles, 0.008, 0.002, 8))
        self.assertEqual(len(key), 64)


if __name__ == "__main__":
    unittest.main()
