import numpy as np
import time
import numpy as np
import matplotlib.pyplot as plt
import csv  
from pydrake.all import *

# Configuration parameters
contact_model = ContactModel.kHydroelasticWithFallback  # Options: Hydroelastic, Point, or HydroelasticWithFallback
mesh_type = HydroelasticContactRepresentation.kTriangle  # Options: Triangle or Polygon
T = 0.2  # Prediction horizon
dt = 1e-3  # Time step for control loop

meshcat_visualisation = True

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
    plant.Finalize()
    return plant, scene_graph

####################################
# Create system diagram
####################################
builder = DiagramBuilder()
plant, scene_graph = AddMultibodyPlantSceneGraph(builder, 1e-3)
plant, scene_graph = create_system_model(plant, scene_graph)
num_positions = plant.num_positions()
num_velocities = plant.num_velocities()

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

where:
  - v ∈ ℝⁿᵛ: Vector of generalized velocities
  - M(q) ∈ ℝⁿᵛˣⁿᵛ: Mass matrix
  - Jₙ(q) ∈ ℝⁿᶜˣⁿᵛ : Jacobian of normal separation velocities
  - Jₜ(q) ∈ ℝ²ⁿᶜˣⁿᵛ: Jacobian of tangent velocities
  - fₙ ∈ ℝⁿᶜ: Vector of normal contact forces
  - fₜ ∈ ℝ²ⁿᶜ: Vector of tangent friction forces
  - τ ∈ ℝⁿᵛ: Vector of generalized forces containing all other applied forces (e.g., Coriolis, gyroscopic terms, actuator forces, etc.) but contact forces.

This solver assumes a compliant law for the normal forces fₙ(q, v) and therefore the functional dependence of fₙ(q, v) with q and v is stated explicitly.

Since TamsiSolver uses regularized friction, we explicitly emphasize the functional dependence of fₜ(q, v) with the generalized velocities. 
The functional dependence of fₜ(q, v) with the generalized positions stems from its direct dependence with the normal forces fₙ(q, v).
'''
assert diagram.IsDifferenceEquationSystem()[0], "must be a discrete-time system"
def calc_dynamics(x, u):
    """
    Calculate the next state given the current state x and control input u.
    
    Args:
        x: Current state vector.
        u: Control input vector.
    
    Returns:
        The next state vector.
    """
    plant_context.SetDiscreteState(x)
    plant.get_actuation_input_port().FixValue(plant_context, u)
    state = diagram_context.get_discrete_state()
    diagram.CalcForcedDiscreteVariableUpdate(diagram_context, state)
    x_next = state.get_vector().value().flatten()
    return x_next

####################################
#   explicit_reference_governor    #
####################################
class ExplicitReferenceGovernor:
    def __init__(self, robust_delta_tau_, kappa_tau_, 
                 robust_delta_q_, kappa_q_, robust_delta_dq_, kappa_dq_, 
                 robust_delta_dp_EE_, kappa_dp_EE_, kappa_terminal_energy_):
        """
        Initialize the Explicit Reference Governor (ERG) with given parameters.
        
        Args:
            robust_delta_tau_ (float): Robustness parameter for joint torques.
            kappa_tau_ (float): Scaling parameter for joint torques.
            robust_delta_q_ (float): Robustness parameter for joint positions.
            kappa_q_ (float): Scaling parameter for joint positions.
            robust_delta_dq_ (float): Robustness parameter for joint velocities.
            kappa_dq_ (float): Scaling parameter for joint velocities.
            robust_delta_dp_EE_ (float): Robustness parameter for end-effector velocities.
            kappa_dp_EE_ (float): Scaling parameter for end-effector velocities.
            kappa_terminal_energy_ (float): Scaling parameter for terminal energy.
        """
        self.eta_ = 0.05 
        self.zeta_q_ = 0.15
        self.delta_q_ = 0.1 
        self.dt_ = 0.01  # Sampling time for the refrence governor

        # Controller gains
        self.Kp_ = [120.0, 120.0, 120.0, 100.0, 50.0, 45.0, 15.0, 120, 120]
        self.Kd_ = [8.0, 8.0, 8.0, 5.0, 2.0, 2.0, 1.0, 5, 5]

        # Prediction parameters
        prediction_dt_ = dt  # Time step for predictions
        prediction_horizon_ = T  # Total prediction horizon
        self.num_pred_samples_ = int(prediction_horizon_ / prediction_dt_)

        # Robustness and scaling parameters
        self.robust_delta_tau_ = robust_delta_tau_
        self.kappa_tau_ = kappa_tau_
        self.robust_delta_q_ = robust_delta_q_
        self.kappa_q_ = kappa_q_
        self.robust_delta_dq_ = robust_delta_dq_
        self.kappa_dq_ = kappa_dq_
        self.robust_delta_dp_EE_ = robust_delta_dp_EE_
        self.kappa_dp_EE_ = kappa_dp_EE_
        self.kappa_terminal_energy_ = kappa_terminal_energy_

        # Prediction lists for joint positions, velocities, and torques
        self.q_pred_list_ = np.zeros((num_positions, self.num_pred_samples_ + 1))
        self.dq_pred_list_ = np.zeros((num_velocities, self.num_pred_samples_ + 1))
        self.tau_pred_list_ = np.zeros((plant.get_actuation_input_port().size(), self.num_pred_samples_ + 1))
        
        # Limits for joint angles, velocities, and torques
        self.limit_q_min_ = np.array([-2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973])
        self.limit_q_max_ = np.array([2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973])
        self.limit_tau_ = np.array([87.0, 87.0, 87.0, 87.0, 12.0, 12.0, 12.0])
        self.limit_dq_ = np.array([2.1750, 2.1750, 2.1750, 2.1750, 2.6100, 2.6100, 2.6100])
        self.limit_dp_EE_ = [1.7, 2.5]  # Translation and rotation limits for the end effector


    def get_qv(self, q, dq, tau, q_r, q_v):
        """
        Compute the new reference joint positions using the navigation field and DSM.
        
        Args:
            q (np.array): Current joint positions.
            dq (np.array): Current joint velocities.
            tau (np.array): Current joint torques.
            q_r (np.array): Desired reference joint positions.
            q_v (np.array): Current applied reference joint positions.
        
        Returns:
            np.array: Updated reference joint positions.
        """
        rho_ = self.navigationField(q_r, q_v)
        DSM_ = self.trajectoryBasedDSM(q, dq, tau, q_v)

        if DSM_ > 0:
          q_v_new = q_v + DSM_ * rho_ * self.dt_ 
        else:
          q_v_new = q + np.min([np.linalg.norm(DSM_ * rho_ * self.dt_), np.linalg.norm(q_r - q_v)]) * DSM_ * rho_ / max(np.linalg.norm(DSM_ * rho_), self.eta_)
        
        return q_v_new

    def navigationField(self, q_r, q_v):
        """
        Compute the navigation field based on attraction and repulsion forces.
        """
        rho_att = np.zeros(num_positions)
        rho_rep_q = np.zeros(num_positions)
        rho = np.zeros(num_positions)

        # Attraction field
        rho_att = (q_r - q_v) / max(np.linalg.norm(q_r - q_v), self.eta_)
        print(f"norm rho_att = {np.linalg.norm(rho_att)}")
        print(f"rho_att = \n {rho_att}")

        # Joint angle repulsion field (q)
        for i in range(7):
            rho_rep_q[i] = max((self.zeta_q_ - abs(q_v[i] - self.limit_q_min_[i])) / (self.zeta_q_ - self.delta_q_), 0.0) - \
                           max((self.zeta_q_ - abs(q_v[i] - self.limit_q_max_[i])) / (self.zeta_q_ - self.delta_q_), 0.0)
        print(f"norm rho_rep_q = {np.linalg.norm(rho_rep_q)}")
        print(f"rho_rep_q = \n {rho_rep_q}")

        # Total navigation field
        rho = rho_att + rho_rep_q
        return rho

    def trajectoryBasedDSM(self, q, dq, tau, q_v):
      """
      Compute the Dynamic Safety Margin (DSM) based on trajectory predictions.
      """
      # Get trajectory predictions and save predicted q, dq, and tau in lists
      self.trajectoryPredictions(np.concatenate((q, dq)), tau, q_v)

      # Compute DSMs
      DSM_tau_ = self.dsmTau()
      DSM_q_ = self.dsmQ()
      DSM_dq_ = self.dsmDq()
      DSM_dp_EE_ = self.dsmDpEE()
      # DSM_terminal_energy_ = self.dsmTerminalEnergy(q_v)

      # Print DSMs
      print(f"DSM_tau_: {DSM_tau_}")
      print(f"DSM_q_: {DSM_q_}")
      print(f"DSM_dq_: {DSM_dq_}")
      print(f"DSM_dp_EE_: {DSM_dp_EE_}")
      # print(f"DSM_terminal_energy_: {DSM_terminal_energy_}")

      # Find the minimum among the DSMs
      DSM = min(DSM_tau_,DSM_dq_)
      DSM = min(DSM,DSM_q_)
      DSM = min(DSM,DSM_dp_EE_)
      # DSM = min(DSM,DSM_terminal_energy_)

      DSM = max(DSM, 0.0)
      print(f"DSM_final: {DSM}")
      
      return DSM

    def trajectoryPredictions(self, state, tau, q_v):
      """
      Predict joint positions, velocities, and torques over the prediction horizon.
      """
      q_pred, dq_pred = state[:num_positions], state[num_positions:]
      tau_pred = tau
      # gravity_pred = np.zeros(plant.get_actuation_input_port().size())  # Placeholder for gravity prediction

      # Initialize lists to store predicted states
      q_pred_traj = [q_pred.copy()]
      dq_pred_traj = [dq_pred.copy()]
      tau_pred_traj = [tau_pred.copy()]

      self.q_pred_list_[:, 0] = q_pred
      self.dq_pred_list_[:, 0] = dq_pred
      self.tau_pred_list_[:, 0] = tau_pred

      # self.distance_tau_ = self.distanceTau(tau_pred)
      # self.distance_q_ = self.distanceQ(q_pred)
      # self.distance_dq_ = self.distanceDq(dq_pred)
      # self.distance_dp_EE_ = self.distanceDpEE(q_pred, dq_pred)

      for k in range(self.num_pred_samples_):
          # Compute tau_pred
          gravity_pred = - plant.CalcGravityGeneralizedForces(plant_context) # Compute gravity_pred for the current state
          tau_pred = self.Kp_ * (q_v - q_pred) - self.Kd_ * dq_pred + gravity_pred 

          # Solve for x[k+1] using the computed tau_pred
          state_pred = calc_dynamics(np.concatenate((q_pred, dq_pred)), tau_pred)  # Adjust this based on your calculation method
          q_pred = state_pred[:num_positions]
          dq_pred = state_pred[num_positions:]

          # Store predicted states
          q_pred_traj.append(q_pred.copy())
          dq_pred_traj.append(dq_pred.copy())
          tau_pred_traj.append(tau_pred.copy())

          # Add q, dq, and tau to prediction list
          self.q_pred_list_[:, k + 1] = q_pred
          self.dq_pred_list_[:, k + 1] = dq_pred
          self.tau_pred_list_[:, k + 1] = tau_pred
      
      # Convert lists to arrays for plotting
      q_pred_traj = np.array(q_pred_traj)
      dq_pred_traj = np.array(dq_pred_traj)

      # Define total prediction time T and time step size dt
      dt = T / self.num_pred_samples_  # Time step size
      time_steps = np.linspace(0, T, self.num_pred_samples_ + 1)  # Time steps array


      # Convert lists to arrays for plotting
      q_pred_traj = np.array(q_pred_traj)
      dq_pred_traj = np.array(dq_pred_traj)
      tau_pred_traj = np.array(tau_pred_traj)
  
      # Define total prediction time T and time step size dt
      dt = 0.2 / self.num_pred_samples_  # Time step size
      time_steps = np.linspace(0, 0.2, self.num_pred_samples_ + 1)  # Time steps array
  
      # Assuming time_steps, q_pred_traj, dq_pred_traj, tau_pred_traj, q_desired_traj, dq_desired_traj, tau_desired_traj are defined
  
    #   plt.figure()
  
    #   # Plot predicted states
    #   plt.plot(time_steps, q_pred_traj, label='Predicted q', linestyle='-', color='b')
    #   plt.plot(time_steps, dq_pred_traj, label='Predicted dq', linestyle='--', color='b')
    #   plt.plot(time_steps, tau_pred_traj, label='Predicted tau', linestyle=':', color='b')
  
    #   plt.xlabel('Time')
    #   plt.ylabel('Angle Position / Velocity / Torque')
    #   plt.legend()
    #   plt.title('Predicted States')
    #   plt.show()
    # Plot predicted states vs desired states for each joint
      num_joints = 9
      fig, axs = plt.subplots(num_joints, 1, figsize=(10, 2 * num_joints))
  
      for i in range(num_joints):
          axs[i].plot(time_steps, q_pred_traj[:, i], label='Predicted q', linestyle='-', color='b')
          axs[i].plot(time_steps, q_v[i] * np.ones_like(time_steps), label='Desired q', linestyle='--', color='r')
          axs[i].set_xlabel('Time')
          axs[i].set_ylabel(f'Joint {i+1} Position')
          axs[i].legend()
          axs[i].set_title(f'Predicted vs Desired Position for Joint {i+1}')
  
      plt.tight_layout()
      plt.show()


    def dsmTau(self):
        """
        Compute the DSM for joint torques.
        """
        for k in range(self.tau_pred_list_.shape[1]):  # number of prediction samples + 1
            tau_pred = self.tau_pred_list_[:, k]
            DSM_tau_temp = self.distanceTau(tau_pred) - self.robust_delta_tau_
            if k == 0:
                DSM_tau = DSM_tau_temp
            else:
                DSM_tau = min(DSM_tau, DSM_tau_temp)

        DSM_tau = self.kappa_tau_ * DSM_tau
        return DSM_tau

    def dsmQ(self):
        """
        Compute the DSM for joint positions.
        """
        for k in range(self.q_pred_list_.shape[1]):  # number of prediction samples + 1
            q_pred = self.q_pred_list_[:, k]
            DSM_q_temp = self.distanceQ(q_pred) - self.robust_delta_q_
            if k == 0:
                DSM_q = DSM_q_temp
            else:
                DSM_q = min(DSM_q, DSM_q_temp)

        DSM_q = self.kappa_q_ * DSM_q
        return DSM_q

    def dsmDq(self):
        """
        Compute the DSM for joint velocities.
        """
        for k in range(self.dq_pred_list_.shape[1]):  # number of prediction samples + 1
            dotq_pred = self.dq_pred_list_[:, k]
            DSM_dotq_temp = self.distanceDq(dotq_pred) - self.robust_delta_dq_
            if k == 0:
                DSM_dotq = DSM_dotq_temp
            else:
                DSM_dotq = min(DSM_dotq, DSM_dotq_temp)

        DSM_dotq = self.kappa_dq_ * DSM_dotq
        return DSM_dotq

    def dsmDpEE(self):
        """
        Compute the DSM for end-effector velocities.
        """
        for k in range(self.q_pred_list_.shape[1]):  # number of prediction samples + 1
            q_pred = self.q_pred_list_[:, k]
            dotq_pred = self.dq_pred_list_[:, k]
            DSM_dotp_EE_temp = self.distanceDpEE(q_pred, dotq_pred) - self.robust_delta_dp_EE_            
            if k == 0:
                DSM_dotp_EE = DSM_dotp_EE_temp
            else:
                DSM_dotp_EE = min(DSM_dotp_EE, DSM_dotp_EE_temp)

        DSM_dotp_EE = self.kappa_dp_EE_ * DSM_dotp_EE
        return DSM_dotp_EE

    # def dsmTerminalEnergy(self, q_v):
    #     k = self.q_pred_list_.shape[1] - 1  # final prediction sample
    #     q_pred = self.q_pred_list_[:, k]
    #     dotq_pred = self.dq_pred_list_[:, k]
    #     distance_terminal_energy = self.distanceTerminalEnergy(q_pred, dotq_pred, q_v)
    #     DSM_terminal_energy = self.kappa_terminal_energy_ * distance_terminal_energy
    #     return DSM_terminal_energy


    def distanceTau(self, tau_pred):
        for i in range(7):
            tau_lowerlimit = tau_pred[i] - (-self.limit_tau_[i])
            tau_upperlimit = self.limit_tau_[i] - tau_pred[i]
            tau_distance_temp = min(tau_lowerlimit, tau_upperlimit)
            if i == 0:
                tau_distance = tau_distance_temp
            else:
                tau_distance = min(tau_distance, tau_distance_temp)
        return tau_distance

    def distanceQ(self, q_pred):
        for i in range(7):  # include all joints
            q_lowerlimit = q_pred[i] - self.limit_q_min_[i]
            q_upperlimit = self.limit_q_max_[i] - q_pred[i]
            q_distance_temp = min(q_lowerlimit, q_upperlimit)
            if i == 0:
                q_distance = q_distance_temp
            else:
                q_distance = min(q_distance, q_distance_temp)
        # print(f"q_distance: {q_distance}")
        return q_distance

    def distanceDq(self, dotq_pred):
        for i in range(7):
            distance_dotq_lowerlimit = dotq_pred[i] - (-self.limit_dq_[i])
            distance_dotq_upperlimit = self.limit_dq_[i] - dotq_pred[i]
            distance_dotq_temp = min(distance_dotq_lowerlimit, distance_dotq_upperlimit)
            if i == 0:
                distance_dotq = distance_dotq_temp
            else:
                distance_dotq = min(distance_dotq, distance_dotq_temp)
        return distance_dotq
      
    def distanceDpEE(self, q_pred, dotq_pred):
        # Placeholder: Adjust this function based on your actual model
          
        # Replace this with your actual computation of the end effector Jacobian
        endeffector_jacobian = np.zeros((6, num_positions))  # Example initialization
        dotp_EE = endeffector_jacobian @ dotq_pred

        for i in range(6):
            if i < 3:  # Translation
                distance_dotp_EE_lowerlimit = dotp_EE[i] - (-self.limit_dp_EE_[0])
                distance_dotp_EE_upperlimit = self.limit_dp_EE_[0] - dotp_EE[i]
            else:  # Rotation
                distance_dotp_EE_lowerlimit = dotp_EE[i] - (-self.limit_dp_EE_[1])
                distance_dotp_EE_upperlimit = self.limit_dp_EE_[1] - dotp_EE[i]
            distance_dotp_EE_temp = min(distance_dotp_EE_lowerlimit, distance_dotp_EE_upperlimit)
            if i == 0:
                distance_dotp_EE = distance_dotp_EE_temp
            else:
                distance_dotp_EE = min(distance_dotp_EE, distance_dotp_EE_temp)
        return distance_dotp_EE
    
    # def distanceTerminalEnergy(self, q_pred, dotq_pred, q_v):
    #     # Placeholder: Adjust this function based on your actual model
    #     m_total = 1.0  # Example mass total
    #     I_total = np.eye(3)  # Example inertia total
    #     F_x_Ctotal = np.zeros(3)  # Example center of mass force

    #     # Replace this with your actual computation of the mass matrix
    #     mass_matrix = np.eye(num_positions)  # Example initialization
    #     Kp_matrix = np.diag([1.0] * num_positions)  # Example Kp matrix

    #     terminal_energy = 0.5 * dotq_pred.T @ mass_matrix @ dotq_pred
    #     terminal_energy += 0.5 * (q_v - q_pred).T @ Kp_matrix @ (q_v - q_pred)

    #     terminal_energy_limit_ = 1.0  # Example limit
    #     distance_terminal_energy = terminal_energy_limit_ - terminal_energy

    #     return distance_terminal_energy
    
####################################
#                                  #
####################################

class ERG(LeafSystem):
    def __init__(self):
        super().__init__()  # Don't forget to initialize the base class.
        self._q_port = self.DeclareVectorInputPort(name="q", size=9)
        self._dq_port = self.DeclareVectorInputPort(name="dq", size=9)
        self._tau_port = self.DeclareVectorInputPort(name="tau", size=9)
        self._qr_port = self.DeclareVectorInputPort(name="q_r", size=9)
        self._qv_port = self.DeclareVectorInputPort(name="q_v", size=9)
        self.DeclareVectorOutputPort(name="q_v_filtered", size=9, calc=self.refrence)

        self.erg = ExplicitReferenceGovernor(
          robust_delta_tau_=0.1, kappa_tau_=1.0,
          robust_delta_q_=0.1, kappa_q_=15.0, robust_delta_dq_=0.1, kappa_dq_=7.0,
          robust_delta_dp_EE_=0.01, kappa_dp_EE_=7.0, kappa_terminal_energy_=7.5)

    def refrence(self, context, output):
        # Evaluate the input ports
        q = self._q_port.Eval(context)
        dq = self._dq_port.Eval(context)
        tau = self._tau_port.Eval(context)
        q_r = self._qr_port.Eval(context)
        q_v = self._qv_port.Eval(context)
        
        q_v_new = self.erg.get_qv(q, dq, tau, q_r, q_v)

        # Write into the output vector.
        output.SetFromVector(q_v_new)


####################################
#                                  #
####################################
# Construct an instance of this system and a context.
erg_system = ERG()
context = erg_system.CreateDefaultContext()


q_r = np.array([2.7, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785, 0.0, 0.0]) # Desired refrence defined by the user or planner
q_v = np.array([2.5 , -0.785, 0.0, -2.356, 0.0, 1.571, 0.785, 0.0, 0.0]) # The currently applied refrence
# Robot state
q = np.array([2.5, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785, 0.0, 0.0]) # np.array([0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785, 0.0, 0.0])
dq = np.array([0.0] * num_velocities)
tau = np.array([0.0] * plant.get_actuation_input_port().size())

start_time = time.time()

# Fix the input ports to some constant values.
erg_system.GetInputPort("q").FixValue(context, q)
erg_system.GetInputPort("dq").FixValue(context, dq)
erg_system.GetInputPort("tau").FixValue(context, tau)
erg_system.GetInputPort("q_r").FixValue(context, q_r)
erg_system.GetInputPort("q_v").FixValue(context, q_v)

# Evaluate the output ports.
print(f"FIltered: {erg_system.GetOutputPort('q_v_filtered').Eval(context)}")

end_time = time.time()
execution_time_ms = (end_time - start_time) * 1000
print(f"Execution Time: {execution_time_ms:.3f} ms")

# print(f"q_v: {q_v_new} ")


##### 1- q_r ---> take from trajecotry source
##### 2- q_v ---> initialize with q first then feedback from erg
##### 3- q ---> initial state take from simulator at each prediction  

