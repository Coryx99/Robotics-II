import os
import numpy as np
import pydot
from IPython.display import SVG, display
from pydrake.common import temp_directory
from pydrake.geometry import StartMeshcat
from pydrake.math import RotationMatrix, RigidTransform, RollPitchYaw
from pydrake.multibody.parsing import Parser
from pydrake.multibody.plant import AddMultibodyPlantSceneGraph, MultibodyPlant
from pydrake.systems.analysis import Simulator
from pydrake.systems.framework import DiagramBuilder
from pydrake.visualization import AddDefaultVisualization
from pydrake.systems.framework import LeafSystem
from pydrake.systems.primitives import ConstantVectorSource
from pydrake.all import Variable, MakeVectorVariable

from helper.dynamics import CalcRobotDynamics

# Function to get the path relative to the script's directory
def get_relative_path(path):
    script_dir = os.path.dirname(os.path.abspath(__file__))
    return os.path.normpath(os.path.join(script_dir, path))

# Start the visualizer and clean up previous instances
meshcat = StartMeshcat()
meshcat.Delete()
meshcat.DeleteAddedControls()

# Set the path to your robot model:
robot_path = get_relative_path("../../models/descriptions/robots//husky_description/urdf/husky.urdf")
scene_path = get_relative_path("../../models/objects&scenes/scenes/floor.sdf")

# Function to Create Simulation Scene
def create_sim_scene(sim_time_step):   
    builder = DiagramBuilder()
    plant, scene_graph = AddMultibodyPlantSceneGraph(builder, time_step=sim_time_step)
    parser = Parser(plant)
    parser.AddModelsFromUrl("file://" + robot_path)
    parser.AddModelsFromUrl("file://" + scene_path)
    plant.Finalize()

    # Add visualization to see the geometries in MeshCat
    AddDefaultVisualization(builder=builder, meshcat=meshcat)

    # Build and return the diagram
    diagram = builder.Build()
    return diagram

# Create a function to run the simulation scene and save the block diagram:
def run_simulation(sim_time_step):
    diagram = create_sim_scene(sim_time_step)
    simulator = Simulator(diagram)
    simulator.Initialize()
    simulator.set_target_realtime_rate(1.)
    
    # Run simulation and record for replays in MeshCat
    meshcat.StartRecording()
    simulator.AdvanceTo(10.0)  # Adjust this time as needed
    meshcat.PublishRecording()

# Run the simulation with a specific time step. Try gradually increasing it!
run_simulation(sim_time_step=0.001)
