import math
import unittest

import numpy as np

from coupled_scene.gpu_dfsph.akinci2013 import cohesion_kernel, surface_accelerations


class AkinciCohesionKernelTests(unittest.TestCase):
    def setUp(self):
        self.h = 0.008

    def test_short_range_branch_is_repulsive(self):
        self.assertLess(cohesion_kernel(0.25 * self.h, self.h), 0.0)

    def test_half_support_is_continuous_and_positive(self):
        at_half = cohesion_kernel(0.5 * self.h, self.h)
        expected = 1.0 / (2.0 * math.pi * self.h**3)
        self.assertGreater(at_half, 0.0)
        self.assertAlmostEqual(at_half, expected, delta=abs(expected) * 1.0e-14)

        left = cohesion_kernel(math.nextafter(0.5 * self.h, 0.0), self.h)
        right = cohesion_kernel(math.nextafter(0.5 * self.h, self.h), self.h)
        self.assertAlmostEqual(left, right, delta=abs(expected) * 1.0e-14)

    def test_compact_support_and_coincident_guard(self):
        self.assertEqual(cohesion_kernel(0.0, self.h), 0.0)
        self.assertEqual(cohesion_kernel(self.h, self.h), 0.0)
        self.assertEqual(cohesion_kernel(math.nextafter(self.h, math.inf), self.h), 0.0)

    def test_h_cubed_scaling_is_invariant_at_fixed_q(self):
        for q in (0.25, 0.5, 0.75):
            reference = self.h**3 * cohesion_kernel(q * self.h, self.h)
            other_h = 0.013
            compared = other_h**3 * cohesion_kernel(q * other_h, other_h)
            self.assertAlmostEqual(reference, compared, delta=max(abs(reference), 1.0) * 1.0e-14)

    def test_rejects_invalid_support_radius(self):
        for invalid in (0.0, -1.0, math.inf, math.nan):
            with self.assertRaises(ValueError):
                cohesion_kernel(0.001, invalid)


class AkinciDiscreteReferenceTests(unittest.TestCase):
    def test_equal_mass_fluid_internal_force_cancels(self):
        positions = np.asarray(
            [[0.000, 0.000, 0.000], [0.004, 0.000, 0.000],
             [0.002, 0.003, 0.000], [0.005, 0.004, 0.001]],
            dtype=np.float64,
        )
        mass = 6.4e-5
        masses = np.full(len(positions), mass)
        densities = np.asarray([940.0, 980.0, 1020.0, 960.0])
        _, accelerations = surface_accelerations(positions, masses, densities, 0.008, 0.05)
        total_force = np.sum(mass * accelerations, axis=0)
        scale = np.sum(np.linalg.norm(mass * accelerations, axis=1))
        np.testing.assert_allclose(total_force, 0.0, atol=max(scale, 1.0) * 2.0e-15)

    def test_nonfluid_particle_is_not_an_akinci_neighbor(self):
        fluid_positions = np.asarray([[0.000, 0.000, 0.000], [0.004, 0.000, 0.000]])
        masses = np.full(2, 6.4e-5)
        densities = np.full(2, 1000.0)
        expected_normals, expected_accelerations = surface_accelerations(
            fluid_positions, masses, densities, 0.008, 0.05
        )

        positions = np.vstack((fluid_positions, [0.002, 0.001, 0.000]))
        actual_normals, actual_accelerations = surface_accelerations(
            positions,
            np.r_[masses, 100.0],
            np.r_[densities, 5000.0],
            0.008,
            0.05,
            fluid_mask=np.asarray([True, True, False]),
        )
        np.testing.assert_allclose(actual_normals[:2], expected_normals)
        np.testing.assert_allclose(actual_accelerations[:2], expected_accelerations)
        np.testing.assert_array_equal(actual_normals[2], 0.0)
        np.testing.assert_array_equal(actual_accelerations[2], 0.0)


if __name__ == "__main__":
    unittest.main()
