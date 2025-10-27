"""
Test file for inspecting models
--------------------
"""

import os
import pydot
from pydrake.geometry import StartMeshcat
from pydrake.multibody.parsing import Parser
from pydrake.multibody.plant import AddMultibodyPlantSceneGraph
from pydrake.systems.framework import DiagramBuilder
from pydrake.visualization import AddDefaultVisualization, ModelVisualizer
from pydrake.systems.analysis import Simulator  

# ------------------ Settings ------------------
visualize = True  # True = only visualize, False = run full simulation
meshcat = StartMeshcat()

# Adjust the path to where the SDF is in your directory
scene_path = os.path.join(
    # "..", "models", "project", "project_01_pick_and_place_sorting_world.sdf"
    # "..", "models", "project", "project_02_drawing_robot_world.sdf"
    # "..", "models", "project", "project_03_diffrentialdrive_robot_tracking_world.sdf"     
    # "..", "models", "project", "project_04_setpoint_regulation_mobile_manipulation.sdf"    
    # "..", "models", "project", "project_05_navigation_world.sdf" ## ---> Remove or change mobilel robot to omnidirectional
    # "..", "models", "project", "project_xx_multirobot_cooperation_world.sdf"    
    # "..", "models", "project", "project_07_two_panda_cooperation_world.sdf"
    # "..", "models", "project", "project_08_task_and_motion_planning.sdf"
    "..", "models", "project", "project_09_two_panda_handover.sdf"
)

# ------------------ Functions ------------------
def create_sim_scene(sim_time_step):
    """Creates a MultibodyPlant + SceneGraph diagram."""
    meshcat.Delete()
    meshcat.DeleteAddedControls()

    builder = DiagramBuilder()
    plant, scene_graph = AddMultibodyPlantSceneGraph(builder, time_step=sim_time_step)
    Parser(plant).AddModelsFromUrl("file://" + os.path.abspath(scene_path))

    plant.Finalize()
    AddDefaultVisualization(builder, meshcat)
    return builder.Build()

def run_visualizer():
    """Minimal visualization using ModelVisualizer."""
    visualizer = ModelVisualizer(meshcat=meshcat)
    visualizer.parser().AddModelsFromUrl("file://" + os.path.abspath(scene_path))
    visualizer.Run()

def run_simulation(sim_time_step=0.001, sim_time = 10.0):
    """Run full simulation if visualize=False."""
    diagram = create_sim_scene(sim_time_step)
    simulator = Simulator(diagram)
    simulator.Initialize()
    simulator.set_target_realtime_rate(1.0)
    simulator.set_publish_every_time_step(True)    
    meshcat.StartRecording()
    simulator.AdvanceTo(sim_time)
    meshcat.PublishRecording()
    # Save diagram imageu
    svg_data = diagram.GetGraphvizString(max_depth=2)
    graph = pydot.graph_from_dot_data(svg_data)[0]
    graph.write_png("figures/block_diagram_project.png")
    print("Block diagram saved as figures/block_diagram_project.png")

# ------------------ Main ------------------
if visualize:
    run_visualizer()
else:
    run_simulation()
