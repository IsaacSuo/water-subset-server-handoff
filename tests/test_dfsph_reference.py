import unittest

import numpy as np

from coupled_scene.gpu_dfsph.dfsph_reference import (
    accumulated_pressure_jacobi_update,
    cubic_kernel_gradient,
    filled_lattice_alpha_reference,
    redistribute_short_frame_tail,
)


class CubicKernelGradientTests(unittest.TestCase):
    def test_gradient_is_antisymmetric(self):
        displacement = np.asarray([0.003, -0.002, 0.001])
        np.testing.assert_allclose(
            cubic_kernel_gradient(displacement, 0.008),
            -cubic_kernel_gradient(-displacement, 0.008),
        )

    def test_gradient_vanishes_at_support(self):
        np.testing.assert_array_equal(cubic_kernel_gradient([0.008, 0.0, 0.0], 0.008), 0.0)


class FilledLatticeAlphaReferenceTests(unittest.TestCase):
    def test_four_mm_configuration_is_symmetric_and_has_26_neighbors(self):
        reference = filled_lattice_alpha_reference(0.004, 0.008, 0.8 * 0.004**3)
        self.assertEqual(reference["neighbor_count"], 26)
        np.testing.assert_allclose(reference["central_gradient_sum_per_m"], 0.0, atol=1.0e-12)
        self.assertGreater(reference["denominator_per_m2"], 0.0)

    def test_uniform_rescaling_has_inverse_length_squared_denominator(self):
        base = filled_lattice_alpha_reference(0.004, 0.008, 0.8 * 0.004**3)
        scale = 1.75
        scaled = filled_lattice_alpha_reference(
            scale * 0.004,
            scale * 0.008,
            scale**3 * 0.8 * 0.004**3,
        )
        self.assertAlmostEqual(
            scaled["denominator_per_m2"],
            base["denominator_per_m2"] / scale**2,
            delta=base["denominator_per_m2"] * 1.0e-13,
        )


class AccumulatedPressureJacobiTests(unittest.TestCase):
    def test_compression_increases_pressure_by_half_residual(self):
        pressure, error = accumulated_pressure_jacobi_update(
            pressure=2.0, source=-0.2, applied_operator=-0.1, alpha=4.0, time_scale=0.5
        )
        self.assertAlmostEqual(pressure, 2.4)
        self.assertAlmostEqual(error, 0.1)

    def test_expansion_can_reduce_but_not_negate_pressure(self):
        pressure, error = accumulated_pressure_jacobi_update(
            pressure=0.1, source=0.2, applied_operator=0.0, alpha=2.0, time_scale=1.0
        )
        self.assertEqual(pressure, 0.0)
        self.assertEqual(error, 0.0)

    def test_density_and_divergence_use_distinct_time_scales(self):
        density, _ = accumulated_pressure_jacobi_update(0.0, -0.1, 0.0, 1.0, 0.25**2)
        divergence, _ = accumulated_pressure_jacobi_update(0.0, -0.1, 0.0, 1.0, 0.25)
        self.assertAlmostEqual(density, 4.0 * divergence)


class FrameTailRedistributionTests(unittest.TestCase):
    def test_short_legal_tail_is_balanced(self):
        candidate, changed = redistribute_short_frame_tail(
            remaining=0.000845, candidate=0.000833, minimum_dt=6.5e-6
        )
        self.assertTrue(changed)
        self.assertAlmostEqual(candidate, 0.0004225)

    def test_large_tail_and_exact_step_are_unchanged(self):
        self.assertEqual(redistribute_short_frame_tail(0.0015, 0.0008, 6.5e-6), (0.0008, False))
        self.assertEqual(redistribute_short_frame_tail(0.0008, 0.0008, 6.5e-6), (0.0008, False))

    def test_never_splits_below_two_hard_minimum_steps(self):
        candidate, changed = redistribute_short_frame_tail(10.0e-6, 7.0e-6, 6.0e-6)
        self.assertFalse(changed)
        self.assertEqual(candidate, 7.0e-6)


if __name__ == "__main__":
    unittest.main()
