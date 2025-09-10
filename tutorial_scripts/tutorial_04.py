import os
import numpy as np
import time
import matplotlib.pyplot as plt
import csv  
from pydrake.all import (DiagramBuilder, Simulator, SimulatorConfig,
                         AddMultibodyPlant, MultibodyPlantConfig, Parser, StartMeshcat,
                         AddDefaultVisualization, ApplySimulatorConfig)



####################################
#         Helper functions         #
####################################
def calc_dynamics(x, u, plant, plant_context, diagram, diagram_context, dt):
    """Calculate the next state based on current state and input."""
    if diagram.IsDifferenceEquationSystem()[0]:  # Discrete-time system
        plant_context.SetDiscreteState(x)
        plant.get_actuation_input_port().FixValue(plant_context, u)

        # Compute the state update step
        state = diagram_context.get_mutable_discrete_state()
        diagram.CalcForcedDiscreteVariableUpdate(diagram_context, state)
        x_next = state.get_vector().value().flatten()
    else:  # Continuous-time system
        plant_context.SetContinuousState(x)
        plant.get_actuation_input_port().FixValue(plant_context, u)
        
        # Compute derivatives
        derivatives = plant.AllocateTimeDerivatives()
        plant.CalcTimeDerivatives(plant_context, derivatives)
        x_next = x + dt * derivatives.get_vector().CopyToVector()  # Euler integration

    return x_next


def create_multibody_plant(time_step):
    """
    Creates a multibody plant with a simple setup for testing continuous or discrete simulations.
    
    Args:
        time_step: If time_step > 0, the plant will use discrete dynamics. If 0, it will use continuous dynamics.
    
    Returns:
        plant: MultibodyPlant with the specified time_step and scene graph.
        scene_graph: SceneGraph to handle geometry and visualization.
    """
    builder = DiagramBuilder()
    plant_config = MultibodyPlantConfig()

    # whether the system is discrete or continuous the contact solver must be congifured
    if time_step > 0:  # Configure the plant for discrete dynamics
        plant_config = MultibodyPlantConfig(
                        time_step=time_step,
                        # Penetration allowance (for point contact) and stiction allowance (for dry friciton) can be configured here
                        contact_model="hydroelastic_with_fallback",  # Options: "point", "hydroelastic", "hydroelastic_with_fallback"
                        discrete_contact_approximation="sap",  # Discrete contact solver. Options: "tamsi", "sap", "lagged", "similar"
                        # You can also set contact surface representation for hydroelastic contact. Options: "triangle", "polygon"
                        adjacent_bodies_collision_filters=True)  # Filters for collisions between adjacent bodies
                
    else:  # Configure the plant for continuous dynamics
        plant_config = MultibodyPlantConfig(
                        time_step=0,
                        # Penetration allowance and stiction allowance can be configured here for point contact
                        contact_model="hydroelastic_with_fallback",  # Options: "point", "hydroelastic", "hydroelastic_with_fallback"
                        adjacent_bodies_collision_filters=True)  # Filters for collisions between adjacent bodies

    # Create the MultibodyPlant and SceneGraph and add them to the builder
    plant, scene_graph = AddMultibodyPlant(plant_config, builder)

    # Parse the URDF model of the robot
    model_path = os.path.join(
        "..", "models", "descriptions", "robots", "arms", "franka_description", "urdf", "panda_arm_hand.urdf"
    )
    Parser(plant).AddModelsFromUrl("file://" + os.path.abspath(model_path))
    
    # Finalize the plant (required before simulation)
    plant.Finalize()

    # Set the initial joint position of the robot otherwise it will correspond to zero positions
    plant.SetDefaultPositions([0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785, 0.0, 0.0])
    print(plant.GetDefaultPositions())
    
    # Initialize Meshcat for visualization
    meshcat = StartMeshcat()
    AddDefaultVisualization(builder=builder, meshcat=meshcat)

    # Build the diagram containing the plant and scene graph    
    diagram = builder.Build()
    
    return plant, diagram, meshcat

def run_simulation(plant, diagram, meshcat, sim_time, time_step, realtime_factor):
    """
    Runs the simulation for a specified duration and records the results.
    
    Args:
        plant: The MultibodyPlant instance to simulate.
        diagram: The system diagram containing plant and scene graph.
        meshcat: The Meshcat instance for visualization.
        sim_time: Duration for running the simulation.
        time_step: The time step used for the plant.
    """
    # Create the simulator
    simulator = Simulator(diagram)
    
    # Select integrator based on whether the system is discrete or continuous
    if time_step > 0:  # Discrete simulation
        print("Running discrete model simulation.")
        # Discrete systems are advanced by the contact solver choosen in the plant config,
        # Therefore, choice of integrators is irrelevent here. 
        simulator_config = SimulatorConfig(
                            target_realtime_rate=realtime_factor, 
                            publish_every_time_step=True)  # Force publishing at each time step
        ApplySimulatorConfig(simulator_config, simulator)

    else:  # Continuous simulation
        print("Running continuous model simulation.")
        # Simulator configuration for continuous system(integrator and publisher parameters).
        simulator_config = SimulatorConfig(integration_scheme="runge_kutta3", # Options are: 'bogacki_shampine3', "explicit_euler", "implicit_euler", "radau1" ,"radau3" ,"runge_kutta2" ,"runge_kutta3" ,"runge_kutta5" ,"semi_explicit_euler", "velocity_implicit_euler"
                                        max_step_size=1e-3, 
                                        accuracy = 1.0e-2, # This may be ignored for fixed-step integration since accuracy control requires variable step integrators.
                                        use_error_control = True, # Use error control for variable step integration.   
                                        target_realtime_rate = realtime_factor, 
                                        publish_every_time_step = True) # Sets whether the simulation should trigger a forced-Publish event on the System under simulation at the end of every trajectory-advancing step.
        ApplySimulatorConfig(simulator_config, simulator)
    
    # Start recording the visualization
    meshcat.StartRecording(frames_per_second=100.0)
    
    # Run the simulation for the specified time
    simulator.AdvanceTo(sim_time)
    
    # Publish the recorded visualization
    meshcat.PublishRecording()

def do_forward(plant, diagram, meshcat, sim_time, time_step, realtime_factor):
    diagram_context = diagram.CreateDefaultContext()
    plant_context = diagram.GetMutableSubsystemContext(plant, diagram_context)

    num_positions = plant.num_positions()
    num_velocities = plant.num_velocities()
    x0 = np.zeros(num_positions + num_velocities)  # positions + velocities
    x_desired = np.zeros(num_positions + num_velocities)  

    x0[0:num_positions] = [0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785, 0.0, 0.0]     # Initial position of each joint              
    x_desired[0:num_positions] = [0.5, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785, 0.0, 0.0] # Desired state for the controller

    # PID gains
    Kp_ = [120.0, 120.0, 120.0, 100.0, 50.0, 45.0, 15.0, 120, 120]   # Proportional gains
    Kd_ = [8.0, 8.0, 8.0, 5.0, 2.0, 2.0, 2.0, 5, 5]                 # Derivative gains
    previous_error = np.zeros(num_positions)     # Initialize PID error terms

    dt = time_step if diagram.IsDifferenceEquationSystem()[0] else 1e-3
    num_steps = int(sim_time / dt)
    
    # Define state and input sizes
    n = plant_context.get_discrete_state_vector().size() if diagram.IsDifferenceEquationSystem()[0] else plant_context.get_continuous_state().size()
    m = plant.get_actuation_input_port().size()

    x = np.zeros((n, num_steps)) # state matrix to store values
    u = np.zeros((m, num_steps-1)) # Inpute matrix to store values
    x[:, 0] = x0 # Sets the initial state as the first element
    
    
    time_steps = np.linspace(0, sim_time, num_steps)  # Define the time variable for plotting
    calc_times = np.zeros(num_steps-1)  # Array to store calculation times

    for t in range(num_steps-1):
        error = x_desired[:num_positions] - x[:num_positions, t]
        derivative_error = (error - previous_error) / dt

        gravity = -plant.CalcGravityGeneralizedForces(plant_context)
        u[:, t] = Kp_ * error + Kd_ * derivative_error + gravity
        previous_error = error    

        try:
            # Measure the time taken by calc_dynamics
            start_time = time.time()
            # print(x[:, t])
            # print(u[:, t])
            x[:, t+1] = calc_dynamics(x[:, t], u[:, t], plant, plant_context, diagram, diagram_context, dt)
            calc_times[t] = time.time() - start_time
        except RuntimeError as e:
            print("Warning: encountered infeasible simulation in linesearch")
            print(e)
            break

    save_results(time_steps, x, u, calc_times, num_positions)
    plot_results(time_steps, x, u, num_positions, calc_times)

    
    while True: # Playback
        playback(plant, diagram, plant_context, diagram_context, time_steps, x, realtime_factor, dt)
        time.sleep(1)


def save_results(time_steps, x, u, calc_times, num_positions):
    """Save simulation data to CSV file."""
    with open('simulation_data.csv', mode='w', newline='') as file:
        writer = csv.writer(file)
        header = ['Time'] + [f'Joint_{i}_position' for i in range(1, 8)] + \
                 [f'Joint_{i}_velocity' for i in range(1, 8)] + \
                 [f'Joint_{i}_input' for i in range(1, 8)] + ['Calc_time']
        writer.writerow(header)

        for t in range(len(time_steps)):
            row = [time_steps[t]] + \
                  list(x[:num_positions, t]) + \
                  list(x[num_positions:, t]) + \
                  list(u[:, t - 1] if t > 0 else [0] * u.shape[0]) + \
                  ([calc_times[t - 1]] if t > 0 else [0])
            writer.writerow(row)


def plot_results(time_steps, x, u, num_positions, calc_times):
    """Plot simulation results."""
    joint_names = [f'Panda joint {i+1}' for i in range(7)]
    
    # Plot joint positions
    fig, axes = plt.subplots(7, 1, figsize=(10, 14), sharex=True)
    fig.suptitle('Positions of Panda Joints', fontsize=16)
    for i in range(7):
        axes[i].plot(time_steps, x[i, :], label=f'{joint_names[i]} position')
        axes[i].set_ylabel('Position [rad]')
        axes[i].grid(True)
        axes[i].legend(loc='upper right')
    axes[-1].set_xlabel('Time [s]')
    plt.tight_layout(rect=[0, 0.03, 1, 0.95])
    plt.show()

    # Plot input torques
    fig, axes = plt.subplots(7, 1, figsize=(10, 14), sharex=True)
    fig.suptitle('Input Torques for Panda Joints', fontsize=16)
    for i in range(7):
        axes[i].plot(time_steps[:-1], u[i, :], label=f'{joint_names[i]} input')
        axes[i].set_ylabel('Input [torque]')
        axes[i].grid(True)
        axes[i].legend(loc='upper right')
    axes[-1].set_xlabel('Time [s]')
    plt.tight_layout(rect=[0, 0.03, 1, 0.95])
    plt.show()

    # Plot joint velocities
    fig, axes = plt.subplots(7, 1, figsize=(10, 14), sharex=True)
    fig.suptitle('Velocities of Panda Joints', fontsize=16)
    for i in range(7):
        axes[i].plot(time_steps, x[num_positions + i, :], label=f'{joint_names[i]} velocity')
        axes[i].set_ylabel('Velocity [rad/s]')
        axes[i].grid(True)
        axes[i].legend(loc='upper right')
    axes[-1].set_xlabel('Time [s]')
    plt.tight_layout(rect=[0, 0.03, 1, 0.95])
    plt.show()


def playback(plant, diagram, plant_context, diagram_context, time_steps, x, realtime_factor, dt):
    """Visualize the playback of the simulation."""

    plant.get_actuation_input_port().FixValue(plant_context, 
            np.zeros(plant.num_actuators()))
    
    # Just keep playing back the trajectory
    for i in range(len(time_steps)):
        t = time_steps[i]
        x_bar = x[:,i]

        diagram_context.SetTime(t)
        plant.SetPositionsAndVelocities(plant_context, x_bar)
        diagram.ForcedPublish(diagram_context)

        sleep_time = max(0, 1/realtime_factor*dt-4e-4)
        time.sleep(sleep_time)

# Main function to run both discrete and continuous simulations
def main():
    sim_time = 10.0  # Simulation time
    time_step = 0.0  # Time step for simulation (0 == continuous, else == Discrete) 
    realtime_factor = 1.0 
    do_forward = True
    # Create the plant and diagram and run the simulation
    plant, diagram, meshcat = create_multibody_plant(time_step=time_step)

    if do_forward:
        do_forward(plant, diagram, meshcat, sim_time, time_step, realtime_factor)
    else:
        run_simulation(plant, diagram, meshcat, sim_time, time_step, realtime_factor)


# Entry point of the script
if __name__ == "__main__":
    main()
