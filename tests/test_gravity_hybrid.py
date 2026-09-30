"""Offline regression checks; no SDK imports or robot commands."""
import sys
import unittest
from pathlib import Path
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'tools'))
from fit_gravity_hybrid import LINKS, basis, fit
from compare_urdf_gravity import Model


class HybridTests(unittest.TestCase):
    def test_basis_reconstructs_physical_change(self):
        model = Model(ROOT/'config/gravity_gripper_can.urdf')
        original = dict(model.links)
        q = np.array([.4, 1.59, -1.05, -.1, -.25, -1.1])
        before = model.evaluate(q)[0]
        X = basis(model, q)
        for name in original:
            self.assertEqual(model.links[name][0], original[name][0])
            np.testing.assert_array_equal(model.links[name][1], original[name][1])
        delta = np.array([.001,-.002,.003]*3 + [.04,.002,-.003,.004])
        for i,name in enumerate(LINKS[:3]):
            m,c=original[name]
            model.links[name]=(m,c+delta[3*i:3*i+3])
        m,c=original['gripper_base']; new_mass=m+delta[9]
        model.links['gripper_base']=(new_mass,c+m*delta[10:13]/new_mass)
        np.testing.assert_allclose(model.evaluate(q)[0]-before,X@delta,atol=1e-10)

    def test_zero_residual_leaves_model_unchanged(self):
        X=np.random.default_rng(123).normal(size=(24,13))
        theta,residual,hits=fit(X,np.zeros(24),np.tile(np.arange(6),4))
        np.testing.assert_allclose(theta,0,atol=1e-12)
        np.testing.assert_array_equal(residual,np.zeros(6))
        self.assertEqual(hits,[])


if __name__=='__main__':
    unittest.main()
