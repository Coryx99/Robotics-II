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
playback = True   # Visualize the optimal trajectory by playing it back.

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
    # plant.AddForceElement(UniformGravityFieldElement());
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
'''
TamsiSolver uses the Transition-Aware Modified Semi-Implicit (TAMSI) method, [Castro et al., 2019], 
to solve the equations below for mechanical systems in contact with regularized friction:
            q̇ = N(q) v
  (1)  M(q) v̇ = τ + Jₙᵀ(q) fₙ(q, v) + Jₜᵀ(q) fₜ(q, v)

where v ∈ ℝⁿᵛ is the vector of generalized velocities, M(q) ∈ ℝⁿᵛˣⁿᵛ is the mass matrix, Jₙ(q) ∈ ℝⁿᶜˣⁿᵛ is the Jacobian of normal separation velocities, 
Jₜ(q) ∈ ℝ²ⁿᶜˣⁿᵛ is the Jacobian of tangent velocities, fₙ ∈ ℝⁿᶜ is the vector of normal contact forces, fₜ ∈ ℝ²ⁿᶜ is the vector of tangent friction forces 
and τ ∈ ℝⁿᵛ is a vector of generalized forces containing all other applied forces (e.g., Coriolis, gyroscopic terms, actuator forces, etc.) but contact forces. 

This solver assumes a compliant law for the normal forces fₙ(q, v) and therefore the functional dependence of fₙ(q, v) with q and v is stated explicitly.

Since TamsiSolver uses regularized friction, we explicitly emphasize the functional dependence of fₜ(q, v) with the generalized velocities. 
The functional dependence of fₜ(q, v) with the generalized positions stems from its direct dependence with the normal forces fₙ(q, v).
'''
assert diagram.IsDifferenceEquationSystem()[0], "must be a discrete-time system"
def calc_dynamics(x, u):
    plant_context.SetDiscreteState(x)
    # u = u + plant.CalcGravityGeneralizedForces(plant_context)
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
x_desired[0:num_positions] = [1.1, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785, 0.0, 0.0]

# PID gains
# K_p = np.ones(num_positions) * 0.0  # Proportional gains
# K_d = np.ones(num_positions) * 0.0    # Derivative gains

K_p = [120.0, 120.0, 120.0, 100.0, 50.0, 45.0, 15.0, 120, 120]  # Proportional gains
K_d = [8.0, 8.0, 8.0, 5.0, 2.0, 2.0, 1.0, 5, 5]    # Derivative gains

# Initialize PID error terms
integral_error = np.zeros(num_positions)
previous_error = np.zeros(num_positions)

####################################
# Run Simulation with PID Controller
####################################
if forward:
    N = int(T / dt)  # = num_steps

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
        u[:, t] = (K_p * error - K_d * x[num_positions:, t])  - plant.CalcGravityGeneralizedForces(plant_context)

        try:
            # Measure the time taken by calc_dynamics
            start_time = time.time()
            x[:, t+1] = calc_dynamics(x[:, t], u[:, t])
            calc_times[t] = (time.time() - start_time)*1000
        except RuntimeError as e:
            print("Warning: encountered infeasible simulation in linesearch")
            print(e)
            break

    # Save data to CSV file
    with open('simulation_data.csv', mode='w', newline='') as file:
        writer = csv.writer(file)
        writer.writerow(['Time ms'] + [f'Joint_{i}_position' for i in range(1, 8)] + [f'Joint_{i}_velocity' for i in range(1, 8)] + [f'Joint_{i}_input' for i in range(1, 8)] + ['Calc_time'])
        
        for t in range(N):
            row = [time_steps[t]] + list(x[:num_positions, t]) + list(x[num_positions:, t]) + list(u[:, t-1] if t > 0 else [0]*m) + ([calc_times[t-1]] if t > 0 else [0])
            writer.writerow(row)

    # Plot the results for the joints.
    joint_names = ['Panda joint 1', 'Panda joint 2', 'Panda joint 3', 'Panda joint 4', 'Panda joint 5', 'Panda joint 6', 'Panda joint 7', 'panda_figer1', 'panda_figer2']

num_positions = 7
num_velocities = 7

# Plot input torques for each joint in subplots
plt.figure(figsize=(12, 20))
for i in range(7):
    plt.subplot(7, 1, i + 1)
    plt.plot(time_steps[:-1], u[i, :], label=f'{joint_names[i]} input', marker='o', markersize=3)
    plt.xlabel('Time [s]')
    plt.ylabel('Torque [Nm]')
    plt.title(f'{joint_names[i]} Input Torque')
    plt.grid(True)
    plt.legend(loc='best')

plt.tight_layout()
plt.show()

# Plot positions for each joint in subplots
plt.figure(figsize=(15, 20))
for i in range(num_positions):
    plt.subplot(7, 1, i + 1)
    plt.plot(time_steps, x[i, :], label=f'{joint_names[i]} position', marker='o', markersize=3)
    plt.axhline(y=x_desired[i], color='r', linestyle='--', label=f'{joint_names[i]} desired position')
    plt.xlabel('Time [s]')
    plt.ylabel('Position [rad]')
    plt.title(f'{joint_names[i]} Position')
    plt.grid(True)
    plt.legend(loc='best')

plt.tight_layout()
plt.show()

# Plot velocities for each joint in subplots
plt.figure(figsize=(15, 20))
for i in range(num_velocities):
    plt.subplot(7, 1, i + 1)
    plt.plot(time_steps, x[num_positions + i, :], label=f'{joint_names[i]} velocity', marker='o', markersize=3)
    plt.xlabel('Time [s]')
    plt.ylabel('Velocity [rad/s]')
    plt.title(f'{joint_names[i]} Velocity')
    plt.grid(True)
    plt.legend(loc='best')

plt.tight_layout()
plt.show()

# Plot calculation times in a single plot
plt.figure(figsize=(12, 8))
plt.plot(time_steps[:-1], calc_times, label='Calculation Time', color='g', marker='o', markersize=3)
plt.xlabel('Sample [n]')
plt.ylabel('Calculation Time [ms]')
plt.title('Time Taken for calc_dynamics Calculation')
plt.grid(True)
plt.legend(loc='best')
plt.show()
#####################################
# Playback
#####################################
while playback:
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

