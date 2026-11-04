import math
import unittest
from main import simulate


class DynamicsTests(unittest.TestCase):
    def test_rest_at_zero(self):
        self.assertTrue(all(row[1] == 0 for row in simulate(target=0)))

    def test_nominal_reaches_target(self):
        self.assertLess(abs(simulate()[-1][1] - 0.5), 0.001)

    def test_torque_is_bounded(self):
        self.assertLessEqual(max(abs(row[3]) for row in simulate(limit=0.12)), 0.12)

    def test_saturation_prevents_target(self):
        # Holding torque at target is 0.5*sin(0.5) = 0.2397 Nm, above 0.12 Nm.
        self.assertGreater(abs(simulate(limit=0.12)[-1][1] - 0.5), 0.1)

    def test_step_convergence(self):
        a = simulate(dt=0.001)
        b = simulate(dt=0.0005)
        self.assertLess(max(abs(row[1] - b[i*2][1]) for i,row in enumerate(a)), 0.002)

    def test_first_step_matches_newtons_law(self):
        row = simulate(duration=0.001)[1]
        self.assertAlmostEqual(row[2], 2.0 / 0.05 * 0.001)
        self.assertAlmostEqual(row[1], row[2] * 0.001)

    def test_mismatch_has_residual(self):
        self.assertGreater(abs(simulate(inertia=0.08, gravity=0.8)[-1][1] - 0.5), 0.01)

    def test_invalid_parameters(self):
        for kwargs in ({'inertia':0}, {'dt':-1}, {'limit':0}, {'target':math.nan}, {'kp':-1}):
            with self.subTest(kwargs=kwargs), self.assertRaises(ValueError):
                simulate(**kwargs)


if __name__ == '__main__':
    unittest.main()
