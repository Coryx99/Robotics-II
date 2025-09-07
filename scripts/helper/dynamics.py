from pydrake.autodiffutils import AutoDiffXd
from pydrake.multibody.tree import MultibodyForces_
from pydrake.multibody.plant import MultibodyPlant_
from pydrake.symbolic import Expression
import numpy as np
import numpy.typing as npt
from pydrake.multibody.plant import MultibodyPlant
from pydrake.multibody.tree import JointActuatorIndex, MultibodyForces_
from pydrake.systems.framework import Context


def CalcRobotDynamics(
    plant: MultibodyPlant,
    q: npt.NDArray,
    v: npt.NDArray = None,
    context: Context = None,
):
    """Returns the components of the manipulator equations -- M(q), C(q,v), tauG(q), B(q), and tauExt(q, v) -- for a given MultibodyPlant.

    Args:
        plant: The MultibodyPlant for which to compute the manipulator equations.
        q: The generalized positions.
        v: The generalized velocities.  If None, the velocities are taken from the
           context.
        context: The Context to use for the computation.  If None, a new default
                 Context will be created.
    """
    if context is None:
        context = plant.CreateDefaultContext()
    plant.SetPositions(context, q)
    if v is not None:
        plant.SetVelocities(context, v)
    M = plant.CalcMassMatrixViaInverseDynamics(context)
    Cv = plant.CalcBiasTerm(context)
    tauG = plant.CalcGravityGeneralizedForces(context)
    B = plant.MakeActuationMatrix()
    forces = MultibodyForces_(plant)
    plant.CalcForceElementsContribution(context, forces)
    tauExt = plant.CalcGeneralizedForces(context, forces) - tauG     
    return (M, Cv, tauG, B, tauExt)



# check: https://github.com/RussTedrake/underactuated/blob/7faf2721c248e889464fe024a65982a43dd78aff/examples/double_pendulum/dynamics.ipynb

