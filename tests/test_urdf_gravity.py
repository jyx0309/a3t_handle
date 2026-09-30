"""Offline numerical checks; no SDK or robot connection."""
import importlib.util
from pathlib import Path
import unittest
import json
import numpy as np

PROJECT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location("gravity", PROJECT / "tools/compare_urdf_gravity.py")
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


class GravityTest(unittest.TestCase):
    def setUp(self):
        self.model = module.Model(PROJECT.parent / "carm_a3t/carm_a3t/urdf/carm_a3t.urdf")

    def test_potential_gradient(self):
        rng = np.random.default_rng(42)
        for _ in range(8):
            q = rng.uniform(-0.6, 0.6, 6)
            q[1] += 1.2
            q[2] -= 1.2
            torque = self.model.evaluate(q)[0]
            numerical = []
            for joint in range(6):
                step = np.zeros(6)
                step[joint] = 1e-6
                numerical.append((self.model.evaluate(q+step)[3] - self.model.evaluate(q-step)[3]) / 2e-6)
            np.testing.assert_allclose(torque, numerical, atol=1e-7, rtol=1e-7)

    def test_gravity_reversal_and_zero(self):
        q = [0.1, 0.9, -0.5, -0.3, -0.1, 0.05]
        normal = self.model.evaluate(q)[0]
        np.testing.assert_allclose(self.model.evaluate(q, gravity=(0, 0, 9.81))[0], -normal)
        np.testing.assert_allclose(self.model.evaluate(q, gravity=(0, 0, 0))[0], 0)
        self.assertAlmostEqual(normal[0], 0)

    def test_symmetric_fingers_and_mass(self):
        q = [0.1, 0.9, -0.5, -0.3, -0.1, 0.05]
        # CAD rotations use rounded 1.5708/3.1416, so symmetry is approximate.
        np.testing.assert_allclose(self.model.evaluate(q, 0)[0], self.model.evaluate(q, 0.037)[0], atol=1e-6)
        mass = sum(m for k, (m, _) in self.model.links.items() if k.startswith("gripper_"))
        self.assertAlmostEqual(mass, 0.8258574545394582)
        with self.assertRaises(ValueError):
            self.model.evaluate(q, -0.01)

    def test_configured_model_is_loadable(self):
        config = json.loads((PROJECT / "config/handle.json").read_text())
        path = PROJECT / "config" / config["gravity"]["urdf_path"]
        model = module.Model(path)
        self.assertTrue(np.isfinite(model.evaluate(np.zeros(6))[0]).all())

    def test_alternative_payload_replaces_whole_gripper(self):
        path = PROJECT / "config/gravity_gripper_can.urdf"
        corrected = module.Model(path)
        self.assertAlmostEqual(sum(m for n, (m, _) in corrected.links.items()
                                   if n.startswith("gripper_")), 0.537306)
        np.testing.assert_array_equal(corrected.links["gripper_base"][1], [0, 0, 0.033631])
        for name in self.model.links:
            if not name.startswith("gripper_"):
                self.assertEqual(corrected.links[name][0], self.model.links[name][0])
                np.testing.assert_array_equal(corrected.links[name][1], self.model.links[name][1])
        def signature(element):
            return element.tag, element.attrib, [signature(c) for c in element]
        self.assertEqual([signature(j) for j in corrected.joints],
                         [signature(j) for j in self.model.joints])
        # Independent potential-energy gradient verifies the replacement torque.
        for q in ([0, 0, 0, 0, 0, 0], [0.01438, 0.91213, -0.533493, -0.375944, -0.143625, -0.006676],
                  [0, 1.7665, -0.533493, -0.375944, -0.143625, 0]):
            q = np.array(q, dtype=float)
            numerical = []
            for i in range(6):
                step = np.eye(6)[i] * 1e-6
                numerical.append((corrected.evaluate(q+step)[3] - corrected.evaluate(q-step)[3]) / 2e-6)
            np.testing.assert_allclose(corrected.evaluate(q)[0], numerical, atol=1e-7)
            np.testing.assert_allclose(corrected.evaluate(q, 0)[0], corrected.evaluate(q, 0.037)[0])


if __name__ == "__main__":
    unittest.main()
