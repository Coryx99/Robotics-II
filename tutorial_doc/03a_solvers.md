# Simulation of multibody systems with frictional contact
The `MultibodyPlant` offers two different modalities to model mechanical systems: continuous and discrete. These models serve different purposes and offer various trade-offs in terms of accuracy, speed, and handling of contact and friction.

In this tutorial, we will explore:

- Continuous simulation models, where dynamics are handled via numerical integration.
- Discrete simulation models, where the system advances in fixed time steps and frictional contact is solved using solvers like SAP.
- Forward dynamic computation 

We will be running the tutorial example [tutorial_3a.py](../python_tutorials/tutorial_3a.py), which can be executed with the command: 
```sh
python3 ./tutorial_3a.py
```
This Python code demonstrates both continuous and discrete simulations, focusing on how to set up a MultibodyPlant with appropriate `configurations` for each type of simulation.
<!-- In the following sections we'll only provide a very limited distinction between a continuous and a discrete system. A complete discussion of the subject, including the modeling of hybrid systems is beyond the scope of this section and thus interested readers are referred to the documentation for Simulator. -->

<!-- In this tutorial, we'll explore:
- Simulation and integration schemes for continuous systems.
- Contact solvers and mechanics for discrete systems. -->

## Explanation of the `main()` function
In the To run both continuous and discrete simulations, the `main()` function sets up the model and invokes the appropriate simulation based on the `time_step` parameter:
```Python
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

if __name__ == "__main__":
    main()
```
- `sim_time:` This variable sets the duration of the simulation, here set to 10 seconds.
- `time_step:` Defines the time step for the simulation. A value of 0 indicates that the simulation will use continuous dynamics, while any positive value indicates discrete dynamics.
- `realtime_factor:` This factor controls the real-time playback speed of the simulation.
- `do_forward:` A boolean flag that determines whether to run the forward dynamics simulation (`True`) or a standard simulation (`False`).

The function then calls `create_multibody_plant()` to initialize the multibody system and visualization. Based on the value of `do_forward`, it either executes the `do_forward` function for forward dynamics or the `run_simulation` function for a standard simulation. 

# Standard simulation of multibody plants
When `run_simulation` is invoked, it performs the standard simulation process. This function initializes the context and executes the simulation for the specified duration using the chosen dynamics configuration.
## Continuous models
When the `time_step` is set to `0` during `MultibodyPlant` construction, the system is modeled as continuous. This means it follows the dynamics of the form:
$$\dot{x} = f(t,x,u)$$
here, $x$ represent the state of the plant, $t$ is time, and $u$ is the set of externally applied inputs (like actuation or forces). Continuous models rely on numerical integrators to advance the system through time. To setup up a continuous plant we use the `MultibodyPlantConfig` as shown below:
```Python
plant_config = MultibodyPlantConfig(
    time_step=0,
    contact_model="hydroelastic_with_fallback",
    adjacent_bodies_collision_filters=True
)
```
This configuration sets the `time_step` to zero, which indicates continuous dynamics. The `contact_model` is set to "hydroelastic_with_fallback," meaning the system will attempt to use hydroelastic contact and fallback to point contact if needed (check [03b_contact.md](./03b_contact.md) for more information about contact models). This configuration is passed to the `AddMultibodyPlant()` function in the code to construct the plant.

### Integrators for continuous models
Drake offers a variety of built-in integrators, with different levels of accuracy and performance. Integrators can be broadly categorized as one of:
- Explicit/Implicit integrators.
- Fixed-step/error-controlled integrators.

Fixed-step integrators often run faster, but they may fail to capture critical transitions, such as slip-stick behavior, when stiction tolerances are very small. On the other hand, error-controlled integrators, like `RungeKutta3Integrator`, adjust the time step to maintain stability and accuracy at the cost of simulation speed.Implicit integrators (i.e `ImplicitEulerIntegrator`) are promising for handling stiff systems with larger time steps. 

In the example code, the `RungeKutta3Integrator` is selected using the `SimulatorConfig` as follows:
```Python
simulator_config = SimulatorConfig(
    integration_scheme="runge_kutta3",
    max_step_size=1e-3,
    accuracy=1e-2,
    use_error_control=True
)
ApplySimulatorConfig(simulator_config, simulator)
```
This configuration specifies:
- `integration_scheme`: The integration scheme to be used. Available options are enumerated in the code, and can be viewed (here)[add/later].
- `max_step_size`: Sets the maximum simulation time step used for integration [s].
- `accuracy`: Sets the simulation accuracy for variable step size integrators with error control.
- `use_error_control`: If `'true'`, the simulator's integrator will use error control if it supports it. Otherwise, the simulator attempts to use fixed steps.

After setting up the plant and simulator, you can run the continuous simulation:

```python
simulator.AdvanceTo(sim_time)
```
This command advances the simulation for the specified duration (`sim_time`).


## Discrete models
For scenarios involving contact and friction, discrete simulation models are often preferred for robustness and computational efficiency. When the `time_step` is greater than zero, Drake uses discrete dynamics with contact approximations.  This means it follows the dynamics of the form:
$$x_{n+1} = x_{n} + f(n,x_{n},u_{n})\Delta t$$

where $x_n$ represents the state of the system at discrete time step $n$, $u_n$ is the input at that time step, and $\Delta t$ is the fixed time interval between steps. Drake provides two primary solvers for discrete contact:

- **TAMSI Solver**: Solves non-linear compliant contact with regularized friction using a Newton-Raphson solver with a custom "transition aware" line search.
- **SAP Solver**: Uses a convex formulation, making it more robust for systems with frictional contact to resolve physical compliance.

To configure a discrete simulation, the following setup is used:
```Python
plant_config = MultibodyPlantConfig(
    time_step=0.001,
    contact_model="hydroelastic_with_fallback",
    discrete_contact_approximation="sap",
    adjacent_bodies_collision_filters=True
)
```
In this case, the `time_step` is non-zero (0.001), indicating discrete dynamics. The discrete_contact_approximation is set to "sap", which selects the Semi-Analytic Primal (SAP) solver for contact. This configuration is passed to the `AddMultibodyPlant()` function, as shown in the tutorial code.


To select a different contact approximation, you can also directly set it from the plant before finalizing the plant, for example:
```Python
plant.set_discrete_contact_solver(DiscreteContactApproximation.kSAP)
```

Discrete simulations run similarly to continuous ones but use a fixed time step to advance the system:
```python
simulator.AdvanceTo(sim_time)
```
This runs the simulation for the given duration using the chosen discrete solver.
## Numerical approximations of dry friction (TOD: Maybe expand or rephrase or remove this section)
Continuous models formulate the multibody physics at the acceleration level and use a regularized model of Coulomb friction. This model is a continuous function of state that can be integrated over time using standard error-controlled numerical methods.

Discrete models, on the other hand, formulate the multibody physics at the velocity level and incorporate friction as constraints. In discrete models, only dynamic friction is considered, as static friction cannot be fully resolved due to the velocity-level formulation.


# Forward dynamics computation
In addition to running simulation we can also propagate our system in time via forward dynamic computation.

Forward dynamics computes the resulting motion of a system based on the applied forces and torques. In the provided code, the `do_forwardd` function is responsible for computing forward dynamics. This function sets up the initial conditions and desired states, applies a PD controller with gravity compensation, and updates the system state based on its dynamics.

## Steps in the `do_forward`
The initial state (positions and velocities) and desired state for the controller are defined, along with controller gains and parameters for forward computation.
```Python
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

```
The dynamics are calculated in a loop over the simulation steps, updating the state based on the computed control input.
```Python
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
```
The `calc_dynamics` function is responsible for updating the state of the multibody system based on the current state and applied input. It handles both discrete and continuous dynamics depending on the configuration of the system.

- **Discrete-time system**: When operating in discrete mode, the function sets the current state in the plant context and applies the actuation input. It then calculates the next state by invoking a discrete variable update, that then computes our dynamics using the selected discrete solvers. 

- **Continuous-time system:** For continuous dynamics, the function sets the continuous state and computes the system's derivatives. It then uses an integration to update the state by adding the product of the derivatives and a time step (could be a variable step), providing a straightforward and effective method for integrating motion over time. Here we implemented a simple forward Euler. 
```Python
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
```

# Next steps
In this tutorial, we explored the basics of simulating multibody systems in Drake, focusing on both continuous and discrete dynamics. We also examined the concept of forward dynamics and how to compute the resulting motion based on forces applied to the system.

In the next tutorial, we will delve deeper into contact models and how to select the appropriate model for different scenarios... Proceed to [03b_contact.md](./03b_contact.md).


<!-- - **TAMSI Solver**, which formulates non-linear compliant contact with regularized friction using a Newton-Raphson solver with a custom "transition aware" line search, see [Castro et al., 2019].
- **Semi-Analytic Primal (“SAP”) contact solver**, a convex formulation originally developed in [[Castro et al., 2022]](https://arxiv.org/abs/2110.10107) as an extension to the work in [[Anitescu, 2006]](https://link.springer.com/article/10.1007/s10107-005-0590-7) and [[Todorov, 2014]](https://ieeexplore.ieee.org/document/6907751) to resolve physical compliance, providing a performant implementation in primal coordinates. -->

<!-- SAP's convex formulation makes it the preferred solver for frictional contact in many scenarios. It also supports more advanced models like Hunt-Crossley for compliant contact, making it suitable for accurately capturing physical interactions. -->

<!-- 
## Simulation of Hybrid Models
A Drake system is a continuous/discrete/hybrid dynamic system where the continuous part is a DAE, that is, it is expected to consist of a set of differential equations and bilateral algebraic constraints. The set of active constraints may change as a result of particular events, such as contact.

The continuous parts of the trajectory are advanced using a numerical integrator. Different integrators have different properties; you can choose the one that is most appropriate for your application or use the default which is adequate for most systems.

## Numerical approximations of dry friction
Drake uses Coulomb's law of friction along with the maximum dissipation principle (MDP) to model friction in both continuous and discrete systems. These physical laws govern how friction forces are applied in response to contact, whether modeled as forces or stresses.

Coulomb's law of friction can be stated in terms of forces for point contact as:

 $$  ‖f_t‖ \le \mu_s f_n $$
where $\mu_s$ is the "static" coefficient of friction, and $f_t$ and $f_n$ are the tangential (friction) and normal components of the contact force, respectively.

For hydroelastic contact the law is defined in terms of stresses:

$$ ‖T_t‖ \le \mu_s p_n, $$
where $T_t$ and $p_n$ are the tangential (friction) and normal stresses, respectively.

The friction force fₜ (or stress Tₜ) is perpendicular to the contact normal. It's direction within a plane perpendicular to the contact normal, is determined by MDP. The MDP states that the direction of the friction force is such that it maximizes the amount of dissipation due to friction. Therefore, for a sliding contact with a non-zero slip velocity vector vₜ, the friction force directly opposes vₜ and its magnitude is μₖfₙ (or μₖpₙ), where μₖ is the "dynamic" (or kinetic) coefficient of friction, with μₖ ≤ μₛ. This can be written as:

 
 $$ fₜ = -μₖ vₜ/‖vₜ‖ fₙ. $$ 
 
 Drake can model multibody systems as being continuous or discrete. Each of these represent different numerical approximations of the same underlying physics.


 
 -->
 ---



