import numpy as np
import time
import numpy as np
import matplotlib.pyplot as plt
import csv  
from pydrake.all import *
import pydot
from IPython.display import SVG, display
from trajectoryERG import *


trajInit_ = np.array([0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785, 0.0, 0.0]) # np.array([0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785, 0.0, 0.0])
######################################################################################################
#                       ################## Trajectory-Based ERG ################
######################################################################################################
# class ERG(LeafSystem):
#     def __init__(self):
#         super().__init__()  # Don't forget to initialize the base class.
#         self._state_port = self.DeclareVectorInputPort(name="state", size=18)
#         self._tau_port = self.DeclareVectorInputPort(name="tau", size=9)
#         self._qr_port = self.DeclareVectorInputPort(name="q_r", size=9)

#         state_index = self.DeclareDiscreteState(9)  # One state variable.
#         self.DeclareStateOutputPort("q_v_filtered", state_index)  # One output: y=x.
#         self.DeclarePeriodicDiscreteUpdateEvent(
#             period_sec=0.001,  # time step.
#             offset_sec=0.0,  # The first event is at time zero.
#             update=self.refrence) # Call the Update method defined below.
#         self.erg = ExplicitReferenceGovernor(
#             robust_delta_tau_=0.1, kappa_tau_=1.0,
#             robust_delta_q_=0.1, kappa_q_=15.0, robust_delta_dq_=0.1, kappa_dq_=7.0,
#             robust_delta_dp_EE_=0.01, kappa_dp_EE_=7.0, kappa_terminal_energy_=7.5)

#         # Initialize a flag to check if it's the first update
#         self.first_update = True

#     def refrence(self, context, discrete_state):
#         # Evaluate the input ports
#         state = self._state_port.Eval(context)
#         q = state[:num_positions]
#         dq = state[num_positions:]
#         tau = self._tau_port.Eval(context)
#         q_r = self._qr_port.Eval(context)

#         # print(f"tau: \n {tau}")
#         self.q_v = context.get_discrete_state_vector().CopyToVector()              
#         # Initialize q_v_ only at the first callback
#         if self.first_update:
#             q = trajInit_
#             dq = np.array([0.0] * num_velocities)
#             tau = np.array([0.0] * plant.get_actuation_input_port().size())
#             self.q_v_ = trajInit_  # or any appropriate initial value  # q_v_ = trajInit_      
#             self.first_update = False

#         q_v_n = self.erg.get_qv(q, dq, tau, q_r, self.q_v_)

#         # Write into the output vector.
#         # print(q_v_n)
#         # print(f"filtered ref = \n {q_v_n}")
#         discrete_state.get_mutable_vector().SetFromVector(q_v_n)

class ERG(LeafSystem):
    def __init__(self):
        super().__init__()  # Don't forget to initialize the base class.
        self._state_port = self.DeclareVectorInputPort(name="state", size=18)
        self._tau_port = self.DeclareVectorInputPort(name="tau", size=9)
        self._qr_port = self.DeclareVectorInputPort(name="q_r", size=9)
        self._qv_port = self.DeclareVectorInputPort(name="q_v", size=9)
        
        self.DeclareVectorOutputPort(name="q_v_filtered", size=9, calc=self.refrence)
        self.q_v_ = trajInit_

        self.erg = ExplicitReferenceGovernor(
          robust_delta_tau_=0.1, kappa_tau_=1.0,
          robust_delta_q_=0.1, kappa_q_=15.0, robust_delta_dq_=0.1, kappa_dq_=7.0,
          robust_delta_dp_EE_=0.01, kappa_dp_EE_=7.0, kappa_terminal_energy_=7.5)
        
    def refrence(self, context, output):
        # Evaluate the input ports
        state = self._state_port.Eval(context)
        q = state[:9]
        dq = state[9:]
        tau = self._tau_port.Eval(context)
        q_r = self._qr_port.Eval(context)
        q_v = self._qv_port.Eval(context)
        print(q_r)
                     
        q_v_n = self.erg.get_qv(q, dq, tau, q_r, q_v)
        print(q_v_n)
        # Write into the output vector.
        output.SetFromVector(q_v_n)


######################################################################################################
#                                  ##################################
######################################################################################################
erg_system = ERG()
context = erg_system.CreateDefaultContext()

q_r = np.array([2.6, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785, 0.0, 0.0]) # Desired refrence defined by the user or planner
q_v = np.array([2.5 , -0.785, 0.0, -2.356, 0.0, 1.571, 0.785, 0.0, 0.0]) # The currently applied refrence
# Robot state
q = np.array([2.5, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785, 0.0, 0.0]) # np.array([0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785, 0.0, 0.0])
dq = np.array([0.0] * 9)
tau = np.array([0.0] * 9)

start_time = time.time()

# Fix the input ports to some constant values.
erg_system.GetInputPort("state").FixValue(context, np.concatenate((q, dq)))
erg_system.GetInputPort("tau").FixValue(context, tau)
erg_system.GetInputPort("q_r").FixValue(context, q_r)
erg_system.GetInputPort("q_v").FixValue(context, q_v)

# Evaluate the output ports.
print(f"FIltered: {erg_system.GetOutputPort('q_v_filtered').Eval(context)}")

end_time = time.time()
execution_time_ms = (end_time - start_time) * 1000
print(f"Execution Time: {execution_time_ms:.3f} ms")



