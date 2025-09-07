import os
# import numpy as np
# import pydot
# from IPython.display import SVG, display
# from pydrake.common import temp_directory
# from pydrake.geometry import StartMeshcat
# from pydrake.math import RotationMatrix, RigidTransform, RollPitchYaw
# from pydrake.multibody.parsing import Parser
# from pydrake.multibody.plant import AddMultibodyPlantSceneGraph, MultibodyPlant
# from pydrake.systems.analysis import Simulator
# from pydrake.systems.framework import DiagramBuilder
# from pydrake.visualization import AddDefaultVisualization
# from pydrake.systems.framework import LeafSystem
# from pydrake.systems.primitives import ConstantVectorSource
# from pydrake.all import Variable, MakeVectorVariable

# from helper.dynamics import CalcRobotDynamics


import numpy as np
from pydrake.all import (DiagramBuilder, Simulator, RigidTransform, RotationMatrix,
                         AddMultibodyPlantSceneGraph, Parser, StartMeshcat,
                         MultibodyPlant, CoulombFriction, UnitInertia, SpatialInertia, 
                         HalfSpace, JointIndex, SpatialVelocity, CoulombFriction, 
                         ConnectMeshcatVisualizer, ContactSolverResults, 
                         SceneGraph, FindResourceOrThrow)



# Function to get the path relative to the script's directory
def get_relative_path(path):
    script_dir = os.path.dirname(os.path.abspath(__file__))
    return os.path.normpath(os.path.join(script_dir, path))

model_path = get_relative_path("../../models/descriptions/robots/panda_fr3/urdf/panda_fr3.urdf")

def create_multibody_plant(time_step):
    """
    Creates a multibody plant with a simple setup for testing continuous or discrete simulations.
    
    Args:
        time_step: If time_step > 0, the plant will use discrete dynamics. If 0, it will use continuous dynamics.
    
    Returns:
        plant: MultibodyPlant with specified time_step and scene graph.
        scene_graph: SceneGraph to handle geometry and visualization.
    """
    builder = DiagramBuilder()
    plant, scene_graph = AddMultibodyPlantSceneGraph(builder, time_step=time_step)
    
    # Add a simple rigid body
    body_frame = plant.AddRigidBody(
        "box",
        SpatialInertia(
            mass=1.0, 
            p_PScm_E=np.array([0.0, 0.0, 0.0]), 
            G_SP_E=UnitInertia.SolidBox(1.0, 1.0, 1.0)
        )
    )
    
    # Add a ground plane (half-space)
    plant.RegisterCollisionGeometry(
        plant.world_body(),
        RigidTransform(), HalfSpace(),
        "ground_collision", CoulombFriction(0.9, 0.5)
    )
    
    # Set gravity
    plant.mutable_gravity_field().set_gravity_vector([0.0, 0.0, -9.81])
    
    # Finalize the plant (required before simulation)
    plant.Finalize()
    
    # Meshcat visualization
    meshcat = StartMeshcat()
    ConnectMeshcatVisualizer(builder, scene_graph, meshcat)
    
    diagram = builder.Build()
    
    return plant, diagram, meshcat

def run_simulation(plant, diagram, meshcat, sim_time, time_step):
    """
    Runs the simulation for a specified duration and records the results.
    
    Args:
        plant: The MultibodyPlant instance to simulate.
        diagram: The system diagram containing plant and scene graph.
        meshcat: The Meshcat instance for visualization.
        sim_time: Duration for running the simulation.
        time_step: The time step used for the plant.
    """
    # Create the simulator and set the integrator parameters
    simulator = Simulator(diagram)
    
    # Select integrator based on whether the system is discrete or continuous
    if time_step > 0:  # Discrete simulation
        integrator = simulator.get_mutable_integrator()
        integrator.set_fixed_step_mode(True)
        simulator.get_context().set_time(0.0)
        simulator.set_target_realtime_rate(1.0)
        print("Running discrete model simulation.")
    else:  # Continuous simulation
        integrator = simulator.get_mutable_integrator()
        integrator.set_maximum_step_size(0.01)
        integrator.set_fixed_step_mode(False)
        integrator.set_target_accuracy(1e-4)
        simulator.get_context().set_time(0.0)
        simulator.set_target_realtime_rate(1.0)
        print("Running continuous model simulation.")
    
    # Start recording and run the simulation
    meshcat.StartRecording()
    simulator.AdvanceTo(sim_time)
    meshcat.PublishRecording()

# Main function to run both discrete and continuous simulations
def main():
    sim_time = 10.0  # Simulation time
    time_step = 1e-3  # Time step for discrete simulation
    
    # Run discrete simulation
    plant_discrete, diagram_discrete, meshcat_discrete = create_multibody_plant(time_step=time_step, is_discrete=True)
    run_simulation(plant_discrete, diagram_discrete, meshcat_discrete, sim_time, time_step)
    
    # Run continuous simulation (time_step = 0)
    plant_continuous, diagram_continuous, meshcat_continuous = create_multibody_plant(time_step=0.0, is_discrete=False)
    run_simulation(plant_continuous, diagram_continuous, meshcat_continuous, sim_time, time_step=0.0)

if __name__ == "__main__":
    main()
