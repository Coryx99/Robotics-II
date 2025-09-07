##
#
# Simulation of a Franka Emika Panda manipulator arm
#
##

import time
import numpy as np
import matplotlib.pyplot as plt
import csv  # Import the CSV module
from pydrake.all import *

# Choose what to do
simulate = False   # Run a simple simulation with fixed input
forward = True
optimize = False    # Find an optimal trajectory using ilqr
playback = False   # Visualize the optimal trajectory by playing it back.

# Hydroelastic, Point, or HydroelasticWithFallback
contact_model = ContactModel.kHydroelasticWithFallback
mesh_type = HydroelasticContactRepresentation.kTriangle  # Triangle or Polygon
T = 10
dt = 1e-3
realtime_factor = 1

meshcat_visualisation = True

def create_system_model(plant, scene_graph):
    # Add the panda arm model from urdf
    urdf = "file:///home/cory/Documents/drake_ddp-main/models/panda_fr3/urdf/panda_fr3.urdf"
    arm = Parser(plant).AddModelsFromUrl(urdf)
    plant.set_contact_surface_representation(mesh_type)
    plant.set_contact_model(contact_model)
    plant.Finalize()
    return plant, scene_graph

####################################
# Create system diagram
####################################
builder = DiagramBuilder()
plant, scene_graph = AddMultibodyPlantSceneGraph(builder, dt)
plant, scene_graph = create_system_model(plant, scene_graph)

# Connect to visualizer
if meshcat_visualisation:
    meshcat = StartMeshcat()
    visualizer = MeshcatVisualizer.AddToBuilder( 
        builder, scene_graph, meshcat,
        MeshcatVisualizerParams(role=Role.kPerception, prefix="visual"))

# Finalize the diagram
diagram = builder.Build()
diagram_context = diagram.CreateDefaultContext()
plant_context = diagram.GetMutableSubsystemContext(plant, diagram_context)

####################################
# Calculate system dynamics
####################################
assert diagram.IsDifferenceEquationSystem()[0], "must be a discrete-time system"

def calc_dynamics(x, u):
    plant_context.SetDiscreteState(x)
    plant.get_actuation_input_port().FixValue(plant_context, u)
    state = diagram_context.get_discrete_state()
    diagram.CalcForcedDiscreteVariableUpdate(diagram_context, state)
    x_next = state.get_vector().value().flatten()
    return x_next

# Desired initial state for each joint
num_positions = plant.num_positions()
num_velocities = plant.num_velocities()
x0 = np.zeros(num_positions + num_velocities)  # positions + velocities
x0[0:num_positions] = [0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785, 0.0, 0.0]

# Desired state for the controller
x_desired = np.zeros(num_positions + num_velocities)
x_desired[0:num_positions] = [0.5, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785, 0.0, 0.0]

# PID gains
K_p = np.ones(num_positions) * 150.0  # Proportional gains
K_i = np.ones(num_positions) * 0.1    # Integral gains
K_d = np.ones(num_positions) * 5.0    # Derivative gains

# Initialize PID error terms
integral_error = np.zeros(num_positions)
previous_error = np.zeros(num_positions)

####################################
# Run Simulation with PID Controller
####################################
if forward:
    num_steps = int(T / dt)
    N = num_steps

    # Define state and input sizes
    n = plant_context.get_discrete_state_vector().size()
    m = plant.get_actuation_input_port().size()

    x = np.zeros((n, N))
    u = np.zeros((m, N-1))
    time_steps = np.linspace(0, T, N)  # Define the time variable for plotting
    calc_times = np.zeros(N-1)  # Array to store calculation times

    x[:, 0] = x0
    for t in range(N-1):
        error = x_desired[:num_positions] - x[:num_positions, t]
        integral_error += error * dt
        derivative_error = (error - previous_error) / dt

        u[:, t] = K_p * error + K_i * integral_error + K_d * derivative_error
        previous_error = error    

        try:
            # Measure the time taken by calc_dynamics
            start_time = time.time()
            x[:, t+1] = calc_dynamics(x[:, t], u[:, t])
            calc_times[t] = time.time() - start_time
        except RuntimeError as e:
            print("Warning: encountered infeasible simulation in linesearch")
            print(e)
            break

    # Save data to CSV file
    with open('simulation_data.csv', mode='w', newline='') as file:
        writer = csv.writer(file)
        writer.writerow(['Time'] + [f'Joint_{i}_position' for i in range(1, 8)] + [f'Joint_{i}_velocity' for i in range(1, 8)] + [f'Joint_{i}_input' for i in range(1, 8)] + ['Calc_time'])
        
        for t in range(N):
            row = [time_steps[t]] + list(x[:num_positions, t]) + list(x[num_positions:, t]) + list(u[:, t-1] if t > 0 else [0]*m) + ([calc_times[t-1]] if t > 0 else [0])
            writer.writerow(row)

    # Plot the results for the first 7 joints only.
    joint_names = ['Panda joint 1', 'Panda joint 2', 'Panda joint 3', 'Panda joint 4', 'Panda joint 5', 'Panda joint 6', 'Panda joint 7', 'panda_figer1', 'panda_figer2']

    # Plot input torques
    plt.figure(figsize=(10, 8))
    for i in range(u.shape[0]):
        plt.plot(time_steps[:-1], u[i, :], label=f'{joint_names[i]} input')  # Use time_steps[:-1] to match dimensions

    plt.xlabel('Time [s]')
    plt.ylabel('Input [torque]')
    plt.title('Input Torques for Panda Joints')
    plt.grid(True)
    plt.legend()
    plt.show()

    # Plot positions
    plt.figure(figsize=(10, 8))
    for i in range(num_positions):
        plt.plot(time_steps, x[i, :], label=f'{joint_names[i]} position')
        plt.axhline(y=x_desired[i], color='r', linestyle='--', label=f'{joint_names[i]} desired position' if i == 0 else "")

    plt.xlabel('Time [s]')
    plt.ylabel('Position [rad]')
    plt.title('Positions of Panda Joints')
    plt.grid(True)
    plt.legend()
    plt.show()

    # Plot velocities
    plt.figure(figsize=(10, 8))
    for i in range(num_velocities):
        plt.plot(time_steps, x[num_positions + i, :], label=f'{joint_names[i]} velocity')

    plt.xlabel('Time [s]')
    plt.ylabel('Velocity [rad/s]')
    plt.title('Velocities of Panda Joints')
    plt.grid(True)
    plt.legend()
    plt.show()

    # Plot calculation times
    plt.figure(figsize=(10, 8))
    plt.plot(time_steps[:-1], calc_times, label='Calculation Time', color='g')
    plt.xlabel('Time [s]')
    plt.ylabel('Calculation Time [s]')
    plt.title('Time Taken for calc_dynamics Calculation')
    plt.grid(True)
    plt.legend()
    plt.show()

#####################################
# Playback
#####################################
while True:
    plant.get_actuation_input_port().FixValue(plant_context, 
            np.zeros(plant.num_actuators()))
    # Just keep playing back the trajectory
    for i in range(len(time_steps)):
        t = time_steps[i]
        x_bar = x[:,i]

        diagram_context.SetTime(t)
        plant.SetPositionsAndVelocities(plant_context, x_bar)
        diagram.ForcedPublish(diagram_context)

        time.sleep(1/realtime_factor*dt-4e-4)
    time.sleep(1)


####################################
# Run Simple Simulation
####################################
if simulate:
    plant.get_actuation_input_port().FixValue(plant_context, np.zeros(plant.num_actuators()))
    plant.SetPositionsAndVelocities(plant_context, x0)

    simulator = Simulator(diagram, diagram_context)
    simulator.set_target_realtime_rate(realtime_factor)
    simulator.set_publish_every_time_step(True)
    simulator.AdvanceTo(T)

####################################
# Trajectory-based ERG
####################################
