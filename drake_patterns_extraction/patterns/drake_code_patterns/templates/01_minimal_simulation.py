"""
Template: Minimal Drake Simulation
Purpose: Load a robot URDF and visualize it in Meshcat
Customization Points: ROBOT_URDF_PATH, SIM_TIME, TIME_STEP
Based on: tutorial_02.py pattern
"""

import os
from pydrake.geometry import StartMeshcat
from pydrake.multibody.parsing import Parser
from pydrake.multibody.plant import AddMultibodyPlantSceneGraph
from pydrake.systems.analysis import Simulator
from pydrake.systems.framework import DiagramBuilder
from pydrake.visualization import AddDefaultVisualization

# ============================================================================
# SECTION 1: CONFIGURATION - Customize these values
# ============================================================================
ROBOT_URDF_PATH = "../models/descriptions/robots/arms/franka_description/urdf/panda_arm_hand.urdf"
TIME_STEP = 0.001  # Discrete time step for simulation (seconds)
SIM_TIME = 5.0     # Total simulation time (seconds)
REALTIME_RATE = 1.0  # 1.0 = real-time, >1.0 = faster, <1.0 = slower

# ============================================================================
# SECTION 2: SETUP MESHCAT VISUALIZER
# ============================================================================
# Start Meshcat visualizer - this creates a web-based 3D viewer
meshcat = StartMeshcat()
# Clean up any previous visualizations
meshcat.Delete()
meshcat.DeleteAddedControls()

# ============================================================================
# SECTION 3: BUILD SIMULATION DIAGRAM
# ============================================================================
def create_simulation_diagram():
    """
    Creates the Drake system diagram with robot and visualization.

    Returns:
        diagram: Complete system diagram ready for simulation
    """
    # Create a DiagramBuilder - this is the container for all system components
    builder = DiagramBuilder()

    # Add MultibodyPlant (physics engine) and SceneGraph (geometry/collision)
    # time_step > 0 means discrete simulation (recommended for most applications)
    plant, scene_graph = AddMultibodyPlantSceneGraph(builder, time_step=TIME_STEP)

    # Create a parser to load robot models
    parser = Parser(plant)

    # Load the robot from URDF file
    # Returns a list of ModelInstanceIndex (one per model loaded)
    robot_model = parser.AddModelsFromUrl("file://" + os.path.abspath(ROBOT_URDF_PATH))[0]

    # Weld the robot base to the world frame (prevents it from falling)
    # Find the base link (root of the kinematic tree)
    base_link = plant.GetBodyByName("panda_link0")  # Adjust name for your robot
    plant.WeldFrames(plant.world_frame(), base_link.body_frame())

    # IMPORTANT: Finalize the plant - no more bodies/joints can be added after this
    plant.Finalize()

    # Add visualization to Meshcat
    AddDefaultVisualization(builder=builder, meshcat=meshcat)

    # Build and return the complete diagram
    diagram = builder.Build()
    return diagram

# ============================================================================
# SECTION 4: RUN SIMULATION
# ============================================================================
def run_simulation():
    """
    Builds the diagram, initializes simulator, and runs simulation.
    """
    # Build the system diagram
    diagram = create_simulation_diagram()

    # Create simulator
    simulator = Simulator(diagram)

    # Configure simulator
    simulator.set_target_realtime_rate(REALTIME_RATE)
    simulator.Initialize()

    # Optional: Record simulation for replay in Meshcat
    meshcat.StartRecording()

    # Run simulation for specified duration
    print(f"Running simulation for {SIM_TIME} seconds...")
    simulator.AdvanceTo(SIM_TIME)

    # Publish recording to Meshcat (allows scrubbing through time)
    meshcat.PublishRecording()

    print("Simulation complete! Check Meshcat for visualization.")

# ============================================================================
# SECTION 5: MAIN ENTRY POINT
# ============================================================================
if __name__ == "__main__":
    run_simulation()
