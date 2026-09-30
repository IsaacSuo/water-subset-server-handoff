import unittest
from types import SimpleNamespace

import numpy as np
import taichi as ti

from coupled_scene.gpu_dfsph.pressure_projection import (
    divergence_pressure_update,
    pair_pressure_acceleration,
    projection_retry_decision,
)
from coupled_scene.gpu_dfsph.dfsph_reference import redistribute_short_frame_tail


@ti.data_oriented
class PressureUpdateProbe:
    def __init__(self):
        self.result = ti.field(ti.f32, shape=2)
        self.acceleration = ti.Vector.field(3, ti.f32, shape=())

    @ti.kernel
    def update(self, pressure: ti.f32, active: ti.i32):
        current_pressure = pressure
        for _ in ti.static(range(8)):
            current_pressure, error = divergence_pressure_update(current_pressure, -20.0, 0.0, 0.2, 0.001, active)
            self.result[0] = current_pressure
            self.result[1] = error
        self.acceleration[None] = pair_pressure_acceleration(ti.Vector([1., 0., 0.]), current_pressure, 3.)


class DivergenceMaskTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        ti.init(arch=ti.cpu)
        cls.probe = PressureUpdateProbe()

    def test_locked_pressure_stays_zero_despite_compressive_source(self):
        self.probe.update(100.0, 0)
        np.testing.assert_array_equal(self.probe.result.to_numpy(), [0.0, 0.0])

    def test_locked_particle_still_receives_neighbor_pressure(self):
        self.probe.update(0.0, 0)
        np.testing.assert_array_equal(self.probe.acceleration.to_numpy(), [-3.0, 0.0, 0.0])

    def test_active_particle_keeps_positive_compression_update(self):
        self.probe.update(0.0, 1)
        np.testing.assert_allclose(self.probe.result.to_numpy(), [16000.0, 20.0], rtol=2.e-6)


class ProjectionRetryTests(unittest.TestCase):
    def test_converged_final_iteration_is_accepted_even_at_cap(self):
        self.assertEqual(projection_retry_decision(True, 4, 1.e-4, 1.e-5), 'accept')

    def test_four_halvings_then_flagged_acceptance(self):
        dt = 8.e-4
        for retry in range(4):
            self.assertEqual(projection_retry_decision(False, retry, dt, 1.e-6), 'halve')
            dt *= 0.5
        self.assertEqual(projection_retry_decision(False, 4, dt, 1.e-6), 'accept_flagged')

    def test_never_halves_under_floor_and_a_branch_does_not_retry(self):
        self.assertEqual(projection_retry_decision(False, 0, 1.5e-5, 1.e-5), 'accept_flagged')
        self.assertEqual(projection_retry_decision(False, 0, 8.e-4, 1.e-5, 0), 'accept_flagged')

    def test_retry_tail_balancing_cannot_increase_halved_step(self):
        for remaining in (0.001, 0.000845, 0.0008):
            for dt in (0.0008, 0.0004, 0.0002, 0.0001):
                retry = 0.5 * dt
                balanced, _ = redistribute_short_frame_tail(remaining, retry, 1.e-6)
                self.assertLessEqual(balanced, retry)


class NewtonSnapshotTests(unittest.TestCase):
    def test_rejected_trial_restores_wrenches_and_energy_ledgers(self):
        from coupled_scene.gpu_dfsph.backend import NewtonRigidSolver

        class Array:
            def __init__(self, value):
                self.value = np.asarray(value, np.float32)

            def numpy(self):
                return self.value.copy()

            def assign(self, value):
                self.value = np.asarray(value, np.float32).copy()

        solver = NewtonRigidSolver.__new__(NewtonRigidSolver)
        solver.state = SimpleNamespace(body_q=Array([[0., 0., 0., 0., 0., 0., 1.]]),
            body_qd=Array(np.zeros((1, 6))), body_f=Array(np.ones((1, 6))))
        solver.total_time = 1.0
        solver.last_wrenches = {2: np.arange(6, dtype=np.float32)}
        solver.last_applied_wrenches = {2: np.zeros(6, np.float32)}
        solver.last_prescribed_power = {2: 3.0}
        solver.cumulative_prescribed_work = {2: 7.0}
        solver.cumulative_fluid_impulse = {2: np.ones(6)}
        solver.publish = lambda: None
        snapshot = solver.snapshot()
        solver.last_wrenches[2][:] = 900
        solver.last_applied_wrenches[2][:] = 100
        solver.last_prescribed_power[2] = 50.0
        solver.cumulative_prescribed_work[2] = 80.0
        solver.cumulative_fluid_impulse[2][:] = 70.0
        solver.state.body_qd.assign(np.ones((1, 6)))
        solver.total_time = 2.0
        solver.restore(snapshot)
        np.testing.assert_array_equal(solver.last_wrenches[2], np.arange(6))
        np.testing.assert_array_equal(solver.last_applied_wrenches[2], 0.)
        np.testing.assert_array_equal(solver.cumulative_fluid_impulse[2], 1.)
        np.testing.assert_array_equal(solver.state.body_qd.numpy(), 0.)
        self.assertEqual(solver.last_prescribed_power[2], 3.0)
        self.assertEqual(solver.cumulative_prescribed_work[2], 7.0)
        self.assertEqual(solver.total_time, 1.0)


if __name__ == '__main__':
    unittest.main()
