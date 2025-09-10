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
robot_path = get_relative_path("../../models/descriptions/robots/panda_fr3/urdf/panda_fr3.urdf")

######################################################################################################
#                             ########Define PD+G Controller as a LeafSystem #######   
######################################################################################################

class Controller(LeafSystem):
    def __init__(self, plant):
        super().__init__()

        # Declare input ports for desired and current states
        self._current_state_port = self.DeclareVectorInputPort(name="Current_state", size=18)
        self._desired_state_port = self.DeclareVectorInputPort(name="Desired_state", size=9)

        # PD+G gains (Kp and Kd)
        self.Kp_ = [120.0, 120.0, 120.0, 100.0, 50.0, 45.0, 15.0, 120, 120]
        self.Kd_ = [8.0, 8.0, 8.0, 5.0, 2.0, 2.0, 2.0, 5, 5]

        # Store plant and context for dynamics calculations
        self.plant, self.plant_context_ad = plant, plant.CreateDefaultContext()

        # Declare discrete state and output port for control input (tau_u)
        state_index = self.DeclareDiscreteState(9)  # 9 state variables.
        self.DeclareStateOutputPort("tau_u", state_index)  # output: y=x.
        self.DeclarePeriodicDiscreteUpdateEvent(
            period_sec=1/1000,  # One millisecond time step.
            offset_sec=0.0,  # The first event is at time zero.
            update=self.compute_tau_u) # Call the Update method defined below.
        
        # Create a periodic event to publish the Dynamics of our robot.
        self.DeclarePeriodicPublishEvent(1, 10, self.PublishDynamics) 
    
    def compute_tau_u(self, context, discrete_state):
        num_positions = self.plant.num_positions()
        num_velocities = self.plant.num_velocities()

        # Evaluate the input ports
        self.q_d = self._desired_state_port.Eval(context)
        self.q = self._current_state_port.Eval(context)

        # Compute gravity forces for the current state
        self.plant_context_ad.SetDiscreteState(self.q)
        gravity = -self.plant.CalcGravityGeneralizedForces(self.plant_context_ad)      
        
        tau = self.Kp_ * (self.q_d - self.q[:num_positions]) - self.Kd_ * self.q[num_positions:] + gravity

        # Update the output port = state
        discrete_state.get_mutable_vector().SetFromVector(tau)

    def PublishDynamics(self, context, mode='symbolic'):
        print("Publishing event")

        # Get current state
        current_state = self._current_state_port.Eval(context)
        q = current_state[:9]
        qdot = current_state[9:]

        if mode == 'numerical':
            # Evaluate the dynamics numerically
            (M, Cv, tauG, B, tauExt) = CalcRobotDynamics(self.plant, q=q, v=qdot)
        elif mode == 'symbolic':
            # Evaluate the dynamics symbolically
            # Symbolic variables for joint positions and velocities
            self.q_sym = MakeVectorVariable(9, "q")
            self.qdot_sym = MakeVectorVariable(9, "qdot")
            (M, Cv, tauG, B, tauExt) = CalcRobotDynamics(self.plant.ToSymbolic(), q=self.q_sym, v=self.qdot_sym)
        else:
            raise ValueError("Invalid mode. Choose 'numerical' or 'symbolic'.")

        print("M = \n" + str(M))
        print("Cv = " + str(Cv))
        print("tau_G = " + str(tauG))
        print("B = " + str(B))
        print("tau_ext = " + str(tauExt))

# Function to Create Simulation Scene
def create_sim_scene(sim_time_step):   
    builder = DiagramBuilder()
    plant, scene_graph = AddMultibodyPlantSceneGraph(builder, time_step=sim_time_step)
    parser = Parser(plant)
    parser.AddModelsFromUrl("file://" + robot_path)
    plant.Finalize()

    # Set the initial joint position of the robot otherwise it will correspond to zero positions
    plant.SetDefaultPositions([0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785, 0.0, 0.0])
    print(plant.GetDefaultPositions())

    # Add visualization to see the geometries in MeshCat
    AddDefaultVisualization(builder=builder, meshcat=meshcat)

    # Add a PD+G controller to regulate the robot
    controller = builder.AddNamedSystem("PD+G controller", Controller(plant))
    
    # Create a constant source for desired positions
    despos_ = [-0.2, -1.5, 0.2, -2.356, 0.0, 1.571, 0.785, 0.0, 0.0]
    des_pos = builder.AddNamedSystem("Desired position", ConstantVectorSource(despos_))
    
    # Connect systems: plant outputs to controller inputs, and vice versa
    builder.Connect(plant.get_state_output_port(), controller.GetInputPort("Current_state")) 
    builder.Connect(controller.GetOutputPort("tau_u"), plant.GetInputPort("applied_generalized_force"))
    builder.Connect(des_pos.get_output_port(), controller.GetInputPort("Desired_state"))

    # Build and return the diagram
    diagram = builder.Build()
    return diagram

# Create a function to run the simulation scene and save the block diagram:
def run_simulation(sim_time_step):
    diagram = create_sim_scene(sim_time_step)
    simulator = Simulator(diagram)
    simulator.Initialize()
    simulator.set_target_realtime_rate(1.)

    # Save the block diagram as an image file
    # svg_data = diagram.GetGraphvizString(max_depth=2)
    # graph = pydot.graph_from_dot_data(svg_data)[0]
    # image_path = "figures/block_diagram_2b.png"  # Change this path as needed
    # graph.write_png(image_path)
    # print(f"Block diagram saved as: {image_path}")
    
    # Run simulation and record for replays in MeshCat
    meshcat.StartRecording()
    simulator.AdvanceTo(10.0)  # Adjust this time as needed
    meshcat.PublishRecording()

# Run the simulation with a specific time step. Try gradually increasing it!
run_simulation(sim_time_step=0.001)
