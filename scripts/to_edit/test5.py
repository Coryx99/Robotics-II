import os
import numpy as np
import pydot
from IPython.display import SVG, display
from pydrake.all import (DiagramBuilder, MultibodyPlant, Parser, RollPitchYaw,
                         SceneGraph, Simulator, ConstantVectorSource, AddMultibodyPlantSceneGraph,
                         InverseDynamicsController, WeldJoint, Isometry3, AddDefaultVisualization,
                         FindResourceOrThrow, StartMeshcat, MeshcatVisualizer, PiecewisePolynomial, TrajectorySource, LogVectorOutput, RigidTransform, RotationMatrix)

meshcat = StartMeshcat()

def create_scene(sim_time_step):
    builder = DiagramBuilder()
    plant, scene_graph = AddMultibodyPlantSceneGraph(builder, time_step=sim_time_step)
    parser = Parser(plant)

    # Loading models.
    parser.AddModels(url="file:///home/cory/panda_description/urdf/panda.urdf")
    parser.AddModels(url="file:///home/cory/Documents/object_models/world.sdf")
    # parser.AddModels(url="file:///home/cory/Documents/table.sdf")
    # parser.AddModels(url="package://drake/manipulation/models/ycb/sdf/003_cracker_box.sdf")
    # parser.AddModels(url="package://drake_models/ycb/meshes/004_sugar_box_textured.obj")

    # plant.AddJoint(WeldJoint("weld_arm", plant.world_frame(), plant.GetBodyByName("base_link").body_frame(),
    #                             RigidTransform(RotationMatrix.MakeZRotation(0), [0.8, 0., 5])))
    
    # Finalize the plant after loading the scene.
    plant.Finalize()

    # Set the initial pose for the panda robot and other objects.
    # Adjusted panda spawn position to prevent overlap with the table
    # plant.SetDefaultFreeBodyPose(plant.GetBodyByName("panda_link0"), RigidTransform(RotationMatrix.MakeZRotation(np.pi/2), [0.2, -0.6, 0.3]))

    # table_frame = plant.GetFrameByName("table_top_center")
    crane_frame = plant.GetFrameByName("base_link")
    crane = plant.GetBodyByName("base_link")
    # cracker_box = plant.GetBodyByName("base_link_cracker")
    # sugar_box = plant.GetBodyByName("004_sugar_box_textured")

    X_WorldTable = crane_frame.CalcPoseInWorld(plant.CreateDefaultContext())
    X_TableCrane = RigidTransform(RollPitchYaw(np.asarray([0, 0, 0]) * np.pi / 180), p=[0,0,0.5])
    # X_TableCracker = RigidTransform(RollPitchYaw(np.asarray([45, 30, 0]) * np.pi / 180), p=[0,0,3])
    # X_TableSugar = RigidTransform(p=[0,-0.25,3])

    plant.SetDefaultFreeBodyPose(crane, X_WorldTable.multiply(X_TableCrane))
    # plant.SetDefaultFreeBodyPose(sugar_box, X_WorldTable.multiply(X_TableSugar))

    # Add visualization to see the geometries.
    AddDefaultVisualization(builder=builder, meshcat=meshcat)

    return builder.Build()

def run_simulation(sim_time_step):
    diagram = create_scene(sim_time_step)
    
    simulator = Simulator(diagram)
    simulator.Initialize()
    simulator.set_target_realtime_rate(1.)

    # Save the block diagram as an image file
    svg_data = diagram.GetGraphvizString(max_depth=2)
    graph = pydot.graph_from_dot_data(svg_data)[0]
    image_path = "block_diagram.png"  # Change this path as needed
    graph.write_png(image_path)
    print(f"Block diagram saved as: {image_path}")
    
    meshcat.StartRecording()
    simulator.AdvanceTo(10.0)  # Adjust this time as needed
    meshcat.PublishRecording()

# Run the simulation with a small time step. Try gradually increasing it!
run_simulation(sim_time_step=0.0)
