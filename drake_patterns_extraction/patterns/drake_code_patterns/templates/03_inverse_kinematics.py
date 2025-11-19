"""
Template: Inverse Kinematics for End-Effector Positioning
Purpose: Use IK to find joint configurations that achieve desired end-effector pose
Customization Points: ROBOT_PATH, TARGET_POSITION, TARGET_ORIENTATION, END_EFFECTOR_FRAME
Based on: tutorial_04_ik.py pattern
"""

import os
import numpy as np
from pydrake.geometry import StartMeshcat
from pydrake.math import RigidTransform, RotationMatrix, RollPitchYaw
from pydrake.multibody.parsing import Parser
from pydrake.multibody.plant import AddMultibodyPlantSceneGraph
from pydrake.systems.analysis import Simulator
from pydrake.systems.framework import DiagramBuilder
from pydrake.visualization import AddDefaultVisualization
from pydrake.all import InverseKinematics, Solve

# ============================================================================
# SECTION 1: CONFIGURATION
# ============================================================================
ROBOT_PATH = "../models/descriptions/robots/arms/franka_description/urdf/panda_arm_hand.urdf"
TIME_STEP = 0.001
END_EFFECTOR_FRAME = "panda_hand"  # Name of end-effector frame in URDF

# Desired end-effector pose
# Position: [x, y, z] in world frame (meters)
TARGET_POSITION = [0.6, 0.0, 0.4]

# Orientation: Roll-Pitch-Yaw angles (radians)
# Roll = rotation about X, Pitch = rotation about Y, Yaw = rotation about Z
TARGET_ORIENTATION = [np.pi, 0, 0]  # [roll, pitch, yaw]

# IK solver tolerance
POSITION_TOLERANCE = 0.001  # meters
ORIENTATION_TOLERANCE = 0.01  # radians

# ============================================================================
# SECTION 2: INVERSE KINEMATICS SOLVER
# ============================================================================
def solve_inverse_kinematics(plant, context, frame_E, X_WE_desired):
    """
    Solves inverse kinematics for desired end-effector pose.

    Args:
        plant: MultibodyPlant instance
        context: Plant context
        frame_E: End-effector frame
        X_WE_desired: RigidTransform of desired world pose

    Returns:
        q_solution: Joint positions if successful, None otherwise
    """
    # Create IK problem
    ik = InverseKinematics(plant, context)

    # Get current joint positions as nominal (starting guess)
    q_nominal = plant.GetPositions(context).reshape((-1, 1))

    # Constrain end-effector position
    p_AQ = X_WE_desired.translation().reshape((3, 1))
    ik.AddPositionConstraint(
        frameB=frame_E,
        p_BQ=np.zeros((3, 1)),  # Origin of end-effector frame
        frameA=plant.world_frame(),
        p_AQ_lower=p_AQ - POSITION_TOLERANCE,  # Lower bound
        p_AQ_upper=p_AQ + POSITION_TOLERANCE   # Upper bound
    )

    # Constrain end-effector orientation
    ik.AddOrientationConstraint(
        frameAbar=plant.world_frame(),
        R_AbarA=X_WE_desired.rotation(),
        frameBbar=frame_E,
        R_BbarB=RotationMatrix(),
        theta_bound=ORIENTATION_TOLERANCE
    )

    # Get underlying optimization program
    prog = ik.prog()
    q_var = ik.q()  # Decision variables (joint angles)

    # Add cost to stay close to nominal configuration
    # cost = (q - q_nominal)^T * W * (q - q_nominal)
    W = np.identity(q_nominal.shape[0])
    prog.AddQuadraticErrorCost(W, q_nominal, q_var)

    # Enforce joint limits
    lower = plant.GetPositionLowerLimits()
    upper = plant.GetPositionUpperLimits()
    prog.AddBoundingBoxConstraint(lower, upper, q_var)

    # Solve optimization problem
    result = Solve(prog, q_nominal)

    # Check if solver succeeded
    if result.is_success():
        q_sol = result.GetSolution(q_var)
        print("✓ IK solved successfully!")
        print(f"  Solution: {q_sol[:7]}")  # Print first 7 joints
        return q_sol
    else:
        print("✗ IK failed to converge!")
        return None

# ============================================================================
# SECTION 3: VISUALIZATION HELPER
# ============================================================================
def visualize_ik_solution(robot_path, q_solution, target_pose):
    """
    Visualizes the IK solution in Meshcat.

    Args:
        robot_path: Path to robot URDF
        q_solution: Joint positions from IK
        target_pose: RigidTransform of target
    """
    meshcat = StartMeshcat()
    meshcat.Delete()

    builder = DiagramBuilder()
    plant, scene_graph = AddMultibodyPlantSceneGraph(builder, time_step=TIME_STEP)

    parser = Parser(plant)
    parser.AddModelsFromUrl("file://" + os.path.abspath(robot_path))

    base_link = plant.GetBodyByName("panda_link0")
    plant.WeldFrames(plant.world_frame(), base_link.body_frame())

    plant.Finalize()

    # Set robot to IK solution
    plant.SetDefaultPositions(q_solution)

    AddDefaultVisualization(builder=builder, meshcat=meshcat)

    diagram = builder.Build()
    simulator = Simulator(diagram)
    simulator.Initialize()

    print("\n" + "="*60)
    print("IK Solution Visualization")
    print("="*60)
    print(f"Target Position: {TARGET_POSITION}")
    print(f"Target Orientation (RPY): {TARGET_ORIENTATION}")
    print(f"Joint Solution: {q_solution[:7]}")  # First 7 joints
    print("="*60)

    # Verify the solution
    context = plant.CreateDefaultContext()
    plant.SetPositions(context, q_solution)
    frame_E = plant.GetFrameByName(END_EFFECTOR_FRAME)
    X_WE_actual = plant.CalcRelativeTransform(context, plant.world_frame(), frame_E)

    print("\nVerification:")
    print(f"  Desired position: {target_pose.translation()}")
    print(f"  Actual position:  {X_WE_actual.translation()}")
    position_error = np.linalg.norm(target_pose.translation() - X_WE_actual.translation())
    print(f"  Position error: {position_error:.6f} m")

    meshcat.StartRecording()
    simulator.AdvanceTo(0.1)  # Just show static pose
    meshcat.PublishRecording()

    return meshcat

# ============================================================================
# SECTION 4: MAIN IK PIPELINE
# ============================================================================
def run_ik_example():
    """
    Complete IK pipeline: setup plant, solve IK, visualize result.
    """
    # Create a temporary plant for IK solving
    builder_temp = DiagramBuilder()
    plant_temp, _ = AddMultibodyPlantSceneGraph(builder_temp, time_step=0.0)

    parser = Parser(plant_temp)
    parser.AddModelsFromUrl("file://" + os.path.abspath(ROBOT_PATH))

    base_link = plant_temp.GetBodyByName("panda_link0")
    plant_temp.WeldFrames(plant_temp.world_frame(), base_link.body_frame())

    plant_temp.Finalize()

    # Create context and set initial configuration
    context_temp = plant_temp.CreateDefaultContext()
    initial_positions = [0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785, 0.0, 0.0]
    plant_temp.SetPositions(context_temp, initial_positions)

    # Get end-effector frame
    frame_E = plant_temp.GetFrameByName(END_EFFECTOR_FRAME)

    # Create desired pose
    X_WE_desired = RigidTransform(
        RollPitchYaw(TARGET_ORIENTATION),
        TARGET_POSITION
    )

    print("Solving inverse kinematics...")
    print(f"Target: position={TARGET_POSITION}, orientation={TARGET_ORIENTATION}")

    # Solve IK
    q_solution = solve_inverse_kinematics(plant_temp, context_temp, frame_E, X_WE_desired)

    if q_solution is not None:
        # Visualize the solution
        visualize_ik_solution(ROBOT_PATH, q_solution, X_WE_desired)
        print("\nCheck Meshcat to see the robot at the IK solution pose.")
    else:
        print("\nFailed to find IK solution. Try adjusting the target pose.")

# ============================================================================
# SECTION 5: MAIN ENTRY POINT
# ============================================================================
if __name__ == "__main__":
    run_ik_example()
