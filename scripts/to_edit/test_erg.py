import numpy as np
import time
import numpy as np
import matplotlib.pyplot as plt
import csv  
from pydrake.all import *
import pydot
from IPython.display import SVG, display
from trajectoryERG import ExplicitReferenceGovernor

def create_system_model(plant, scene_graph):
    """
    Add the Panda arm model to the plant and configure contact properties.
    
    Args:
        plant: The MultibodyPlant object to which the Panda arm model will be added.
        scene_graph: The SceneGraph object for visualization.
    
    Returns:
        Tuple containing the updated plant and scene_graph.
    """
    urdf = "file:///home/cory/Documents/drake_ddp-main/models/panda_fr3/urdf/panda_fr3.urdf"
    arm = Parser(plant).AddModelsFromUrl(urdf)
    plant.set_contact_surface_representation(mesh_type)
    plant.set_contact_model(contact_model)
    plant.set_discrete_contact_approximation(discrete_solver)
    plant.Finalize()
    return plant, scene_graph
####################################
# Configurations parameters
####################################
contact_model = ContactModel.kHydroelasticWithFallback  # Options: Hydroelastic, Point, or HydroelasticWithFallback
mesh_type = HydroelasticContactRepresentation.kTriangle  # Options: Triangle or Polygon
discrete_solver = DiscreteContactApproximation.kTamsi # Options:kTamsi, kSap, kLagged, kSimilar
realtime_factor = 1  # Real-time factor for simulation speed
time_step = 0.001

meshcat_visualisation = True
simulate = True

# Create system diagram
builder = DiagramBuilder()
plant, scene_graph = AddMultibodyPlantSceneGraph(builder, time_step)
plant, scene_graph = create_system_model(plant, scene_graph)
plant_context = plant.CreateDefaultContext()
num_positions = plant.num_positions()
num_velocities = plant.num_velocities()

######################################################################################################
#              ##################Planner Trapezoidal motion profile ################
######################################################################################################
trajDuration_ = 5.0
accDuration_ = 0.1
trajInit_ = np.array([0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785, 0.0, 0.0]) # np.array([0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785, 0.0, 0.0])

# trajEnd_ = np.array([3.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785, 0.0, 0.0]) # np.array([0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785, 0.0, 0.0])
trajEnd_ = np.array([3.0, -0.985, 1.0, -2.356, 0.0, 1.571, 0.785, 0.0, 0.0]) # np.array([0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785, 0.0, 0.0])
class TrajectoryPoint:
    def __init__(self):
        self.pos = np.zeros(9)
        self.vel = np.zeros(9)
        self.acc = np.zeros(9)

class motion_profile(LeafSystem):
    def __init__(self):
        super().__init__()  # Don't forget to initialize the base class.
        self._trajInit_port = self.DeclareVectorInputPort(name="trajInit_", size=9)
        self._trajEnd__port = self.DeclareVectorInputPort(name="trajEnd_", size=9)
        # self.DeclareVectorOutputPort(name="q_r", size=9, calc=self.compute_trajectory) 

        state_index = self.DeclareDiscreteState(9)  # One state variable.
        self.DeclareStateOutputPort("q_r", state_index)  # One output: y=x.
        self.DeclarePeriodicDiscreteUpdateEvent(
            period_sec=0.01,  # time step.
            offset_sec=0.0,  # The first event is at time zero.
            update=self.compute_trajectory) # Call the Update method defined below.
        
        self.traj = TrajectoryPoint()

    def compute_trajectory(self, context, discrete_state):
        # Evaluate the input ports
        self.ttime = context.get_time()
        self.trajInit_ =self._trajInit_port.Eval(context)
        self.trajEnd_ =self._trajEnd__port.Eval(context)

        ddot_traj_c = -1.0 / (accDuration_**2 - trajDuration_ * accDuration_) * (self.trajEnd_ - self.trajInit_)

        if self.ttime <= accDuration_:
            self.traj.pos = self.trajInit_ + 0.5 * ddot_traj_c * self.ttime**2
            self.traj.vel = ddot_traj_c * self.ttime
            self.traj.acc = ddot_traj_c
        elif self.ttime <= trajDuration_ - accDuration_:
            self.traj.pos = self.trajInit_ + ddot_traj_c * accDuration_ * (self.ttime - accDuration_ / 2)
            self.traj.vel = ddot_traj_c * accDuration_
            self.traj.acc = np.zeros(3)
        elif self.ttime <= trajDuration_:
            self.traj.pos = self.trajEnd_ - 0.5 * ddot_traj_c * (trajDuration_ - self.ttime)**2
            self.traj.vel = ddot_traj_c * (trajDuration_ - self.ttime)
            self.traj.acc = -ddot_traj_c
        else:
            # After trajDuration_, hold the final position
            self.traj.pos = trajEnd_
            self.traj.vel = np.zeros(9)
            self.traj.acc = np.zeros(9)
        
        q_r = self.traj.pos
        # q_r = trajEnd_
        # print(f"refrence = \n {q_r}")

        # Write into the output vector.
        discrete_state.get_mutable_vector().SetFromVector(q_r)


######################################################################################################
#                       ################## Trajectory-Based ERG ################
######################################################################################################
class ERG(LeafSystem):
    def __init__(self):
        super().__init__()  # Don't forget to initialize the base class.
        self._state_port = self.DeclareVectorInputPort(name="state", size=18)
        self._tau_port = self.DeclareVectorInputPort(name="tau", size=9)
        self._qr_port = self.DeclareVectorInputPort(name="q_r", size=9)

        state_index = self.DeclareDiscreteState(9)  # One state variable.
        self.DeclareStateOutputPort("q_v_filtered", state_index)  # One output: y=x.
        self.DeclarePeriodicDiscreteUpdateEvent(
            period_sec=0.01,  # time step.
            offset_sec=0.0,  # The first event is at time zero.
            update=self.refrence) # Call the Update method defined below.
        self.erg = ExplicitReferenceGovernor(
            robust_delta_tau_=0.1, kappa_tau_=1.0,
            robust_delta_q_=0.1, kappa_q_=15.0, robust_delta_dq_=0.1, kappa_dq_=7.0,
            robust_delta_dp_EE_=0.01, kappa_dp_EE_=7.0, kappa_terminal_energy_=7.5)

        # Initialize a flag to check if it's the first update
        self.first_update = True

    def refrence(self, context, discrete_state):
        # Evaluate the input ports
        state = self._state_port.Eval(context)
        q = state[:num_positions]
        dq = state[num_positions:]
        tau = self._tau_port.Eval(context)
        q_r = self._qr_port.Eval(context)

        # print(f"tau: \n {tau}")
        self.q_v = context.get_discrete_state_vector().CopyToVector()              
        # Initialize q_v_ only at the first callback
        if self.first_update:
            self.q_v_ = trajInit_  # or an appropriate initial value    
            self.first_update = False
        self.q_v_ = self.erg.get_qv(q, dq, tau, q_r, self.q_v_)

        # Write into the output vector.
        # print(f"filtered ref = \n {self.q_v_}")
        discrete_state.get_mutable_vector().SetFromVector(self.q_v_)


######################################################################################################
#                                  ########PD+G controller#######   Check input output
######################################################################################################
class PD_gravity(LeafSystem):
    def __init__(self):
        super().__init__()

        self._desired_state_port = self.DeclareVectorInputPort(name="Desired_state", size=9)
        self._current_state_port = self.DeclareVectorInputPort(name="Current_state", size=18)
        self.Kp_ = [120.0, 120.0, 120.0, 100.0, 50.0, 45.0, 15.0, 120, 120]
        self.Kd_ = [8.0, 8.0, 8.0, 5.0, 2.0, 2.0, 1.0, 5, 5]

        state_index = self.DeclareDiscreteState(9)  # One state variable.
        self.DeclareStateOutputPort("tau_u", state_index)  # One output: y=x.
        self.DeclarePeriodicDiscreteUpdateEvent(
            period_sec=1/1000,  # One second time step.
            offset_sec=0.0,  # The first event is at time zero.
            update=self.compute_tau_u) # Call the Update method defined below.
        
    def compute_tau_u(self, context, discrete_state):
        # Evaluate the input ports
        self.q_d =self._desired_state_port.Eval(context)
        self.q = self._current_state_port.Eval(context)
        # x = context.get_discrete_state_vector().GetAtIndex(0)
        gravity = -plant.CalcGravityGeneralizedForces(plant_context) # Compute gravity_pred for the current state        
        tau = self.Kp_ * (self.q_d - self.q[:num_positions]) - self.Kd_ * self.q[num_positions:] + gravity
        
        # print(f"applied torque = \n {tau}")
        discrete_state.get_mutable_vector().SetFromVector(tau)

######################################################################################################
#                                  ##################################
######################################################################################################
init_pos = builder.AddNamedSystem("Initial position", ConstantVectorSource(trajInit_))
end_pos = builder.AddNamedSystem("End position", ConstantVectorSource(trajEnd_))
trajectory = builder.AddNamedSystem("Trajectory generator", motion_profile())
erg_system = builder.AddNamedSystem("Trajectory-based ERG", ERG())
pid_controller = builder.AddNamedSystem("PD+G controller", PD_gravity())

builder.Connect(init_pos.get_output_port(), trajectory.GetInputPort("trajInit_"))
builder.Connect(end_pos.get_output_port(), trajectory.GetInputPort("trajEnd_"))
builder.Connect(trajectory.GetOutputPort("q_r"), erg_system.GetInputPort("q_r"))
builder.Connect(plant.get_state_output_port(), pid_controller.GetInputPort("Current_state"))


# builder.Connect(end_pos.get_output_port(), erg_system.GetInputPort("q_r"))
# builder.Connect(trajectory.GetOutputPort("q_r"), pid_controller.GetInputPort("Desired_state"))
builder.Connect(erg_system.GetOutputPort("q_v_filtered"), pid_controller.GetInputPort("Desired_state"))

#### THis is the problem: https://stackoverflow.com/questions/77405027/how-can-i-get-a-feedthrough-from-the-plant-state-port-to-my-leaf-system-port-for
builder.Connect(pid_controller.GetOutputPort("tau_u"), plant.GetInputPort("applied_generalized_force"))
builder.Connect(plant.get_state_output_port(), erg_system.GetInputPort("state"))

# builder.Connect(pid_controller.GetOutputPort("tau_u"), erg_system.GetInputPort("tau"))
builder.Connect(plant.GetOutputPort("panda_net_actuation"), erg_system.GetInputPort("tau"))

# Connect to visualizer
if meshcat_visualisation:
    meshcat = StartMeshcat()
    visualizer = MeshcatVisualizer.AddToBuilder( 
        builder, scene_graph, meshcat,
        MeshcatVisualizerParams(role=Role.kPerception, prefix="visual"))

logger_x = LogVectorOutput(plant.get_state_output_port(), builder) #state
logger_tau = LogVectorOutput(pid_controller.GetOutputPort("tau_u"), builder) #tau_u
logger_qv = LogVectorOutput(erg_system.GetOutputPort("q_v_filtered"), builder) #q_v
logger_qr = LogVectorOutput(trajectory.GetOutputPort("q_r"), builder) #q_r

# Finalize the diagram
diagram = builder.Build()
diagram.set_name("diagram")
diagram_context = diagram.CreateDefaultContext()

####################################
# Run Simple Simulation
####################################
if simulate:
    simulator = Simulator(diagram, diagram_context)
    plant.SetPositions(plant_context, trajInit_)
    simulator.set_target_realtime_rate(realtime_factor)
    simulator.set_publish_every_time_step(True)
    simulator.Initialize()

    # Define the step size and simulation time
    kStep = 0.001
    sim_time = trajDuration_  # or whatever your total simulation time is
    simulator_context = simulator.get_mutable_context()

    # Run simulation
    while simulator_context.get_time() < sim_time:
        next_time = min(sim_time, simulator_context.get_time() + kStep)
        simulator.AdvanceTo(next_time)

    # simulator.AdvanceTo(next_time)


# Assuming you have the following limits defined somewhere in your script
limit_q_min = np.array([-2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973])
limit_q_max = np.array([2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973])
limit_dq = np.array([2.1750, 2.1750, 2.1750, 2.1750, 2.6100, 2.6100, 2.6100])
limit_tau = np.array([87.0, 87.0, 87.0, 87.0, 12.0, 12.0, 12.0])

log_x = logger_x.FindLog(diagram_context)
log_tau = logger_tau.FindLog(diagram_context)
log_qv = logger_qv.FindLog(diagram_context)
log_qr = logger_qr.FindLog(diagram_context)

t_time = log_x.sample_times()

data_q = log_x.data().transpose()[:, 0:9]  # Selecting only the first 7 columns (joints)
data_qdot = log_x.data().transpose()[:, 9:18]  # Selecting only the first 7 columns (joints)
data_tau = log_tau.data().transpose()  # Selecting only the first 7 columns (joints)
data_qv = log_qv.data().transpose()  # Selecting only the first 7 columns (joints)
data_qr = log_qr.data().transpose()  # Selecting only the first 7 columns (joints)

# print(t_time)
# Plotting
joint_names = ['Panda joint 1', 'Panda joint 2', 'Panda joint 3', 'Panda joint 4', 'Panda joint 5', 'Panda joint 6', 'Panda joint 7', 'panda_figer1', 'panda_figer2']
# Plot joint positions
num_positions = 7
# Plot for each joint position

# Create a figure for position plots
fig_pos, axs_pos = plt.subplots(num_positions, 1, figsize=(12, 3*num_positions))
fig_pos.suptitle('Joint Positions')
for i in range(num_positions):
    axs_pos[i].plot(t_time, data_q[:, i], label='Actual Position', linestyle='-')
    axs_pos[i].plot(t_time, data_qv[:, i], label='Filtered Reference Position', linestyle='--')
    axs_pos[i].plot(t_time, data_qr[:, i], label='Reference Position', linestyle=':')
    # axs_pos[i].plot(t_time, limit_q_min[i] * np.ones_like(t_time), label='Minimum Position', linestyle='-.', color='red')
    # axs_pos[i].plot(t_time, limit_q_max[i] * np.ones_like(t_time), label='Maximum Position', linestyle='-.', color='green')
    axs_pos[i].set_xlabel('Time [s]')
    axs_pos[i].set_ylabel('Position [rad]')
    axs_pos[i].set_title(f'{joint_names[i]}')
    axs_pos[i].legend(loc='upper right')
    axs_pos[i].grid(True)
plt.tight_layout(rect=[0, 0.03, 1, 0.95])
plt.show()

# Create a figure for velocity plots
fig_vel, axs_vel = plt.subplots(num_positions, 1, figsize=(12, 3*num_positions))
fig_vel.suptitle('Joint Velocities')
for i in range(num_positions):
    axs_vel[i].plot(t_time, data_qdot[:, i], label='Actual Velocity', linestyle='-')
    axs_vel[i].plot(t_time, limit_dq[i] * np.ones_like(t_time), 'r--', label='Velocity Limit')
    axs_vel[i].plot(t_time, -limit_dq[i] * np.ones_like(t_time), 'r--')
    axs_vel[i].set_xlabel('Time [s]')
    axs_vel[i].set_ylabel('Velocity [rad/s]')
    axs_vel[i].set_title(f'{joint_names[i]}')
    axs_vel[i].legend(loc='upper right')
    axs_vel[i].grid(True)
plt.tight_layout(rect=[0, 0.03, 1, 0.95])
plt.show()

# Create a figure for torque plots
fig_tau, axs_tau = plt.subplots(num_positions, 1, figsize=(12, 3*num_positions))
fig_tau.suptitle('Joint Torques')
for i in range(num_positions):
    axs_tau[i].plot(t_time, data_tau[:, i], label='Actual Torque', linestyle='-')
    axs_tau[i].plot(t_time, limit_tau[i] * np.ones_like(t_time), 'r--', label='Torque Limit')
    axs_tau[i].plot(t_time, -limit_tau[i] * np.ones_like(t_time), 'r--')
    axs_tau[i].set_xlabel('Time [s]')
    axs_tau[i].set_ylabel('Torque [Nm]')
    axs_tau[i].set_title(f'{joint_names[i]}')
    axs_tau[i].legend(loc='upper right')
    axs_tau[i].grid(True)
plt.tight_layout(rect=[0, 0.03, 1, 0.95])
plt.show()

svg_data = diagram.GetGraphvizString(max_depth=2)
graph = pydot.graph_from_dot_data(svg_data)[0]
image_path = "block_diagramERG.png"  # Change this path as needed
graph.write_png(image_path)
print(f"Block diagram saved as: {image_path}")
