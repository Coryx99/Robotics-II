"""
Template: Robot with PD+G Controller
Purpose: Control robot joints to reach desired positions using PD+Gravity compensation
Customization Points: ROBOT_PATH, PID_GAINS, DESIRED_POSITIONS
Based on: tutorial_04_ik.py pattern
"""

import os
import numpy as np
from pydrake.geometry import StartMeshcat
from pydrake.multibody.parsing import Parser
from pydrake.multibody.plant import AddMultibodyPlantSceneGraph
from pydrake.systems.analysis import Simulator
from pydrake.systems.framework import DiagramBuilder, LeafSystem
from pydrake.systems.primitives import ConstantVectorSource
from pydrake.visualization import AddDefaultVisualization

# ============================================================================
# SECTION 1: CONFIGURATION
# ============================================================================
ROBOT_PATH = "../models/descriptions/robots/arms/franka_description/urdf/panda_arm_hand.urdf"
TIME_STEP = 0.001
SIM_TIME = 5.0

# PD+G Controller Gains (Kp, Kd for each joint)
# Format: [joint1_kp, joint2_kp, ..., joint_n_kp]
# Higher Kp = stiffer response, Higher Kd = more damping
PID_GAINS = {
    "kp": [120.0, 120.0, 120.0, 100.0, 50.0, 45.0, 15.0, 120.0, 120.0],
    "kd": [8.0, 8.0, 8.0, 5.0, 2.0, 2.0, 2.0, 5.0, 5.0]
}

# Desired joint positions (radians for revolute, meters for prismatic)
DESIRED_POSITIONS = [0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785, 0.0, 0.0]

# ============================================================================
# SECTION 2: PD+G CONTROLLER CLASS
# ============================================================================
class PDGravityController(LeafSystem):
    """
    PD controller with gravity compensation.

    Control law: tau = Kp * (q_desired - q) - Kd * q_dot + g(q)

    where:
        tau = control torque
        Kp = proportional gain
        Kd = derivative gain
        q_desired = desired joint positions
        q = current joint positions
        q_dot = current joint velocities
        g(q) = gravity forces
    """

    def __init__(self, plant, kp, kd):
        """
        Args:
            plant: MultibodyPlant instance
            kp: List of proportional gains (one per joint)
            kd: List of derivative gains (one per joint)
        """
        super().__init__()

        self.plant = plant
        self.plant_context = plant.CreateDefaultContext()
        self.kp = np.array(kp)
        self.kd = np.array(kd)

        num_positions = plant.num_positions()
        num_velocities = plant.num_velocities()
        state_size = num_positions + num_velocities

        # Input port for current robot state [q, q_dot]
        self._current_state_port = self.DeclareVectorInputPort(
            name="current_state",
            size=state_size
        )

        # Input port for desired joint positions [q_desired]
        self._desired_state_port = self.DeclareVectorInputPort(
            name="desired_state",
            size=num_positions
        )

        # Output port for control torques [tau]
        # Using discrete update at 1kHz
        state_index = self.DeclareDiscreteState(num_positions)
        self.DeclareStateOutputPort("control_output", state_index)
        self.DeclarePeriodicDiscreteUpdateEvent(
            period_sec=0.001,  # 1 millisecond
            offset_sec=0.0,
            update=self.compute_control
        )

    def compute_control(self, context, discrete_state):
        """
        Computes PD+G control law.

        Args:
            context: Current context
            discrete_state: Discrete state to update
        """
        # Read current state [q, q_dot]
        current_state = self._current_state_port.Eval(context)
        num_positions = self.plant.num_positions()
        q = current_state[:num_positions]
        q_dot = current_state[num_positions:]

        # Read desired positions
        q_desired = self._desired_state_port.Eval(context)

        # Compute gravity compensation
        self.plant_context.SetDiscreteState(current_state)
        gravity = -self.plant.CalcGravityGeneralizedForces(self.plant_context)

        # PD+G control law
        tau = self.kp * (q_desired - q) - self.kd * q_dot + gravity

        # Update output
        discrete_state.get_mutable_vector().SetFromVector(tau)

# ============================================================================
# SECTION 3: SIMULATION SETUP
# ============================================================================
def create_controlled_simulation():
    """Creates simulation with robot and PD+G controller."""

    meshcat = StartMeshcat()
    meshcat.Delete()
    meshcat.DeleteAddedControls()

    builder = DiagramBuilder()

    # Add plant and scene graph
    plant, scene_graph = AddMultibodyPlantSceneGraph(builder, time_step=TIME_STEP)

    # Load robot
    parser = Parser(plant)
    parser.AddModelsFromUrl("file://" + os.path.abspath(ROBOT_PATH))

    # Weld base to world
    base_link = plant.GetBodyByName("panda_link0")
    plant.WeldFrames(plant.world_frame(), base_link.body_frame())

    # Finalize plant
    plant.Finalize()

    # Add visualization
    AddDefaultVisualization(builder=builder, meshcat=meshcat)

    # Create controller
    controller = builder.AddNamedSystem(
        "PD+G_controller",
        PDGravityController(plant, PID_GAINS["kp"], PID_GAINS["kd"])
    )

    # Create desired position source (constant target)
    desired_pos_source = builder.AddNamedSystem(
        "desired_positions",
        ConstantVectorSource(np.array(DESIRED_POSITIONS))
    )

    # Connect systems
    builder.Connect(
        plant.get_state_output_port(),
        controller.GetInputPort("current_state")
    )
    builder.Connect(
        controller.GetOutputPort("control_output"),
        plant.GetInputPort("applied_generalized_force")
    )
    builder.Connect(
        desired_pos_source.get_output_port(),
        controller.GetInputPort("desired_state")
    )

    # Build diagram
    diagram = builder.Build()
    return diagram, meshcat

# ============================================================================
# SECTION 4: RUN SIMULATION
# ============================================================================
def run_controlled_simulation():
    """Runs the controlled simulation."""

    diagram, meshcat = create_controlled_simulation()

    simulator = Simulator(diagram)
    simulator.set_target_realtime_rate(1.0)
    simulator.Initialize()

    print(f"Running controlled simulation for {SIM_TIME} seconds...")
    print(f"Target joint positions: {DESIRED_POSITIONS}")

    meshcat.StartRecording()
    simulator.AdvanceTo(SIM_TIME)
    meshcat.PublishRecording()

    print("Simulation complete!")

# ============================================================================
# SECTION 5: MAIN ENTRY POINT
# ============================================================================
if __name__ == "__main__":
    run_controlled_simulation()
