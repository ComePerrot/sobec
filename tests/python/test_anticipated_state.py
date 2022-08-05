"""
Simple numdiff test of the COP cost.
COP is a complex function as it depends on the contact forces.
Hence, build a complete DAM with contact, and assert its gradient WRT numdiff.
"""

from turtle import shape
import unittest

import pinocchio as pin
import crocoddyl as croc
import numpy as np
import example_robot_data as robex
from numpy.linalg import norm

# Local imports
import sobec


class TestAnticipatedState(unittest.TestCase):
    def test_anticipated_state(self):

        np.random.seed(0)
        dt = np.random.random()
        robot = robex.load("talos_arm")

        # The pinocchio model is what we are really interested by.
        model = robot.model
        model.q0 = robot.q0

        # Initial config, also used for warm start
        x0 = np.concatenate([model.q0, np.random.random(model.nv) * 20 - 10])

        # #############################################################################

        state = croc.StateMultibody(model)
        actuation = croc.ActuationModelFull(state)

        # Costs
        costs = croc.CostModelSum(state, actuation.nu)

        anticipatedStateResidual = sobec.ResidualModelAnticipatedState(state, state.nq, dt)
        anticipatedStateCost = croc.CostModelResidual(state, anticipatedStateResidual)
        costs.addCost("anticipatedState", anticipatedStateCost, 1)

        # Action
        damodel = croc.DifferentialActionModelFreeFwdDynamics(state, actuation, costs)

        # #############################################################################

        # jid = model.frames[cid].parent

        # ### For easier manipulation at debug time
        dadata = damodel.createData()
        u0 = np.random.rand(actuation.nu) * 20 - 10
        q0 = x0[: state.nq]
        v0 = x0[state.nq :]

        try:
            cosname = "anticipatedState"
            cosdata = dadata.costs.costs[cosname]
            # cosmodel = damodel.costs.costs[cosname].cost
            data = cosdata.shared.pinocchio
        except KeyError:
            pass

        damodel.calc(dadata, x0, u0)
        damodel.calcDiff(dadata, x0, u0)

        # ## MANUAL CHECK
        np.set_printoptions(precision=3, linewidth=300, suppress=True, threshold=10000)
        self.assertGreater(norm(cosdata.residual.r), 0)

        r = q0 + dt * v0
        self.assertLess(norm(cosdata.residual.r - r), 1e-6)

        Rx = np.concatenate((np.diag(np.ones(state.nq)), dt * np.diag(np.ones(state.nq))), axis=1)
        self.assertLess(norm(cosdata.residual.Rx - Rx), 1e-6)

        # ### NUMDIFF TEST
        damnd = croc.DifferentialActionModelNumDiff(damodel, gaussApprox=True)
        dadnd = damnd.createData()
        damnd.calc(dadnd, x0, u0)
        damnd.calcDiff(dadnd, x0, u0)

        self.assertLess(norm(dadnd.Lx - dadata.Lx) / norm(dadata.Lx), 1e-5)
        self.assertLess(norm(dadnd.Lu - dadata.Lu) / norm(dadata.Lx), 1e-5)


if __name__ == "__main__":
    unittest.main()
