import unittest

from world_model_dataset.causal_control import evaluate, reference


class CausalControlTests(unittest.TestCase):
    def setUp(self):
        self.controller = {"controller_id": "plate", "primitive": "impedance_control",
            "stiffness_n_m": 6000, "damping_n_s_m": 120, "max_force_n": 250}
        self.command = {"command_id": "load", "controller_id": "plate", "start_time_s": 1,
            "end_time_s": 2, "target": {"trajectory": "smoothstep", "position_start_m": .26,
            "position_end_m": .16}, "limits": {"max_force_n": 250}}

    def test_reference_is_continuous_and_has_zero_endpoint_velocity(self):
        self.assertEqual(reference(self.command, 1), (.26, 0))
        end, velocity = reference(self.command, 2)
        self.assertAlmostEqual(end, .16)
        self.assertEqual(velocity, 0)
        position, velocity = reference(self.command, 1.5)
        self.assertAlmostEqual(position, .21)
        self.assertAlmostEqual(velocity, -.15)

    def test_impedance_has_finite_authority_and_observable_tracking_error(self):
        decision = evaluate(self.controller, [self.command], 1.5, .26, 0)
        self.assertEqual(decision["applied_force_n"], -250)
        self.assertTrue(decision["saturated"])
        self.assertAlmostEqual(decision["position_error_m"], -.05)
        self.assertTrue(decision["command_active"])

    def test_uncommanded_interval_is_zero_force_not_frozen_motion(self):
        decision = evaluate(self.controller, [self.command], 2, .19, -.1)
        self.assertEqual(decision["applied_force_n"], 0)
        self.assertFalse(decision["command_active"])

    def test_overlapping_commands_are_not_silently_overwritten(self):
        with self.assertRaises(ValueError):
            evaluate(self.controller, [self.command, self.command], 1.5, .26, 0)

    def test_velocity_effort_remains_force_feedback(self):
        controller = dict(self.controller, primitive="effort_control", max_force_n=8)
        command = dict(self.command, target={"velocity_m_s": .8, "velocity_gain_n_s_m": 20}, limits={"max_force_n": 8})
        self.assertEqual(evaluate(controller, [command], 1.2, -.8, 0)["applied_force_n"], 8)
        self.assertAlmostEqual(evaluate(controller, [command], 1.2, -.8, .75)["applied_force_n"], 1)


if __name__ == "__main__":
    unittest.main()
