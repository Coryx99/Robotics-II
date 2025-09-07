import os
import numpy as np
import pydot
from IPython.display import SVG, display
from pydrake.common import temp_directory
from pydrake.geometry import StartMeshcat, SceneGraph, MeshcatVisualizer
from pydrake.math import RotationMatrix, RigidTransform, RollPitchYaw
from pydrake.multibody.parsing import Parser
from pydrake.multibody.plant import AddMultibodyPlantSceneGraph, MultibodyPlant
from pydrake.systems.analysis import Simulator
from pydrake.systems.framework import DiagramBuilder
from pydrake.visualization import AddDefaultVisualization, ModelVisualizer
from pydrake.all import WeldJoint, ConstantVectorSource

# Function to get the path relative to the script's directory
def get_relative_path(path):
    script_dir = os.path.dirname(os.path.abspath(__file__))
    return os.path.normpath(os.path.join(script_dir, path))


# Start the visualizer. The cell will output an HTTP link after the execution.
# Click the link and a MeshCat tab should appear in your browser.
meshcat = StartMeshcat()
visualize = True # Bool to switch the viszualization and simulation
model_path = get_relative_path("../../models/descriptions/robots/panda_fr3/urdf/panda_fr3.urdf")


def create_sim_scene(sim_time_step):   
    """
    This function creates a simulation scene using the Drake and the Panda robot model.

    Args:
        sim_time_step (float): The time step for the simulation.

    Returns:
        Diagram: The created simulation diagram.
    """
    # Clean up the Meshcat instance.
    meshcat.Delete()
    meshcat.DeleteAddedControls()

    builder = DiagramBuilder()
    plant_1, scene_graph = AddMultibodyPlantSceneGraph(builder, time_step=sim_time_step)
    Parser(plant_1).AddModelsFromUrl("file://" + model_path)

    # Finalize the plant after loading the scene.
    plant_1.Finalize()

    # Add visualization to see the geometries.
    AddDefaultVisualization(builder=builder, meshcat=meshcat)
    diagram = builder.Build()
    return diagram


def run_simulation(sim_time_step):
    if visualize:
        # First we will choose our definition file of the panda robot
        model_path = get_relative_path("../../models/descriptions/robots/panda_fr3/urdf/panda_fr3.urdf")
        # Create a model visualizer and add the robot arm.
        visualizer = ModelVisualizer(meshcat=meshcat)
        visualizer.parser().AddModelsFromUrl("file://" + model_path)

        # Start the interactive visualizer.
        # Note: the visualizer will be closed automatically when the script exits or  
        # when the "Stop Running" button in MeshCat is clicked.
        visualizer.Run()
        
    if not visualize:
        diagram = create_sim_scene(sim_time_step)

        # Initialize a simulation with a default context
        simulator = Simulator(diagram)
        simulator.set_target_realtime_rate(1)
        simulator.Initialize()
        simulator.set_publish_every_time_step(True) # Publish every simulation step based on the system definition

        sim_time = 10  # the simulation time

        meshcat.StartRecording() # Records the simulation
        simulator.AdvanceTo(sim_time) # Runs the simulation for sim_time seconds
        meshcat.PublishRecording() # Replays the simulation
            
        # Save the block diagram as an image file
        svg_data = diagram.GetGraphvizString(max_depth=2)
        graph = pydot.graph_from_dot_data(svg_data)[0]
        image_path = "figures/block_diagram_2a.png"  # Change this path as needed
        graph.write_png(image_path)
        print(f"Block diagram saved as: {image_path}")

# Run the simulation with a small time step. Try gradually increasing it!
run_simulation(sim_time_step=0.001) 