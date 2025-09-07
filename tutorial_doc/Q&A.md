If problem during build, then sources of the bash file might be the
reason (automated the sourcing of ros and drake at system startup). 

If colcon build crashes the pc, b) Running colcon build
\--symlink-install \--parallel-workers 2 would solve it
[since](https://get-help.theconstruct.ai/t/colcon-build-crashes-ubuntu-22-04/19558). 

Should I use python or C++?

Casadi vs drake?

CasADi offers some overlap with Drake in terms of solvers such as IPOPT,
SNOPT, OSQP, Ipopt, and NLopt. However, CasADi does not have equivalents
to all of the specific solvers in Drake such as Clarabel, CLP, CSDP,
SCS, or Moby LCP. But casadi is...............................

If you specifically need the solvers unique to Drake like Clarabel, you
would need to use Drake. Otherwise, CasADi provides a broader range of
optimization tools but may not have all the specialized solvers listed
for Drake.

Clarifications: 

Q1: Plant and system definition: How can I define multiple plants and
what is the difference when I define a system instead? Can I just add
multiple systems inside a single plant, or do I need to do it in
separate plants? Does it couple the dynamics of the bodies? What is the
difference between a scene graph and a diagram? 

Answer: You can define multiple plants freely, however, the
plants/systems would need to be registered in the same scene-graph to
solve the contact during collision events. Basically a diagram is our
system with its context (state, input, parameters and time), the diagram
can be simulated by the physics engine to solve the dynamic system, and
a scene-graph is the geometry solver that solves the kinematics. 

 

Q2: How does the simulator simulate the system? What are the specific
steps? How to properly model the dynamic system in time? 

Answer: In Drake a diagram is the main graph of Drake. diagram is
composed of systems like MultibodyPlant, controllers and other useful
blocks. Like Simulink, the diagram determines how the system is
constructed, what each block is, and how they are connected. Drake has a
DiagramBuilder class to help glue the system blocks together. It adds
system blocks to the diagram and connects the input and output ports of
blocks. The Simulator takes the whole system diagram and runs the
simulation. Using the robot dynamics equation of motion and environment
forces, the Simulator computes the state change. It then runs numerical
integration for continuous system or state update for discrete system,
to calculate the next system state, and write the states back to the
diagram\'s corresponding context. It keeps updating the states until the
specified simulation time is reached. Furthermore, SceneGraph is the
visualization and collision checking tool.  

-   Collision Checking: Before simulation, SceneGraph is initiated and
    connected to MultibodyPlant. During simulation, SceneGraph would
    give the information of whether two objects collide and what is the
    distance between two objects, given the state input from
    MultibodyPlant. Then the MultibodyPlant decides whether the
    collision is a soft contact or a fierce crash, how much force is
    generated in between objects given the collision information. 

MultibodyPlant offers two different modalities to model mechanical
systems in time. These are: 1. As a discrete system with periodic
updates, (time\_step \> zero). 2. As a continuous system, (time\_step =
zero). A System is a continuous/discrete/hybrid dynamic system where the
continuous part is a Diffrential algebraic equation (DAE), that is, it
is expected to consist of a set of differential equations and bilateral
algebraic constraints. The set of active constraints may change as a
result of particular events, such as contact. Given a current Context,
we expect a System to provide us with:  

 - derivatives for the continuous differential equations that already
satisfy the differentiated form of the constraints (typically,
acceleration constraints),  

\- a projection method for least-squares correction of violated
higher-level constraints (position and velocity level),  

\- a time-of-next-update method that can be used to adjust the
integrator step size in preparation for a discrete update,  

\- methods that can update discrete variables when their update time is
reached,  

\- witness (guard) functions for event isolation,  

\- event handlers (reset functions) for making appropriate changes to
state and mode variables when an event has been isolated. 

 

The continuous parts in the diagram are advanced using a numerical
integrator. Different integrators have different properties; If there
are no continuous-time dynamics being simulated, then the choice of
integrator does not matter. Only the update period(s) of the discrete
systems will matter. 

 

What is the solver used in drake for discrete and continu sys? 

Answer: Drake provides several solver options, including implicit Euler,
Runge-Kutta methods, and more advanced techniques like the semi-implicit
Euler integrator. For discrete-time systems, Drake typically uses
techniques such as discrete-time integration methods, including explicit
and implicit methods like Euler\'s method or backward Euler\'s method. 

 

Q3: How the simulation is stepped: simulation mechanics for authors of
discrete and hybrid systems? 

Answer:
https://drake.mit.edu/doxygen\_cxx/group\_\_discrete\_\_systems.html 

 

Q4: What are the types of constraints that are supported? 

Answer: Drake\'s constraint system helps solve computational dynamics
problems with algebraic constraints, like differential algebraic
equations: 

ẋ = f(x, t, λ) 

0 = g(x, t, λ) 

where g() corresponds to one or more constraint equations and λ is
typically interpretable as \"forces\" that enforce the constraints.
Drake\'s constraint system permits solving initial value problems
subject to such constraints and does so in a manner very different from
\"smoothing methods\" (AKA \"penalty methods\"). Smoothing methods have
traditionally required significant computation to maintain g = 0 to high
accuracy (and typically yielding what is known in ODE and DAE literature
as a \"computationally stiff system\")
\[[REF](https://drake.mit.edu/doxygen_cxx/group__constraint__overview.html)\].
Drake also provide the core components of an efficient first-order
integration scheme for a system with both compliant and rigid unilateral
constraints. Such a system arises in many contact problems, where the
correct modeling parameters would yield a computationally stiff system.
The integration scheme is low-accuracy but very stable for mechanical
systems, even when the algebraic variables (i.e., the constraint forces)
are computed to low accuracy. 

-   Constraint types:
    <https://drake.mit.edu/doxygen_cxx/group__constraint__types.html>  
-   Constraint stabilization:
    <https://drake.mit.edu/doxygen_cxx/group__constraint__stabilization.html>  
-   Constraint Jacobian matrices:
    <https://drake.mit.edu/doxygen_cxx/group__constraint___jacobians.html>  
-   Contact surface constraints:
    https://drake.mit.edu/doxygen\_cxx/group\_\_contact\_\_surface\_\_constraints.html 

 

Q5: What are the types of contact approximation that is available? And
what are the diffrences? 

Answer: Tamsi, Similar and Lagged are all approximations of the same
contact model -- Compliant contact with regularized friction. The key
difference however, is that the Similar and Lagged approximations are
convex and therefore the contact solver has both theoretical and
practical convergence guarantees --- the solver will always succeed.
Conversely, being non-convex, Tamsi can fail to find a solution. Sap is
also a convex model of compliant contact with regularized friction.
There are a couple of key differences, however: 

-   Dissipation is modelled using a linear Kelvin--Voigt model,
    parameterized by a relaxation time constant. See contact
    parameters. 
-   Unlike Tamsi, Similar and Lagged where regularization of friction is
    parameterized by a stiction tolerance (see
    set\_stiction\_tolerance()), SAP determines regularization
    automatically solely based on numerics. Users have no control on the
    amount of regularization. 

 

Q6: How to choose an approximation? 

Answer: The Hunt & Crossley model is based on physics, it is continuous
and has been experimentally validated. Therefore it is the preferred
model to capture the physics of contact. However, being approximations,
Sap and Similar introduce a spurious effect of \"gliding\" in sliding
contact \[Castro et al., 2023\]. This artifact is 𝒪(δt) but can be
significant at large time steps and/or large slip velocities. Lagged
does not suffer from this, but introduces a \"weak\" coupling of
friction that can introduce non-negligible effects in the dynamics
during impacts or strong transients. Summarizing, Lagged is the
preferred approximation when strong transients are not expected or
don\'t need to be accurately resolved. If strong transients do need to
be accurately resolved (unusual for robotics applications), Similar is
the preferred approximation.  

Note:: Lagged is an approximation in which the normal force is lagged n
Coulomb's law:: 

References: 

-   \[Castro et al., 2019\] Castro A., Qu A., Kuppuswamy N., Alspach A.,
    Sherman M, 2019. A Transition-Aware Method for the Simulation of
    Compliant Contact with Regularized Friction. Available online at
    <https://arxiv.org/abs/1909.05700>.  
-   \[Castro et al., 2022\] Castro A., Permenter F. and Han X., 2022. An
    Unconstrained Convex Formulation of Compliant Contact. Available
    online at <https://arxiv.org/abs/2110.10107>.  
-   \[Castro et al., 2023\] Castro A., Han X., and Masterjohn J., 2023.
    A Theory of Irrotational Contact Fields. Available online at
    <https://arxiv.org/abs/2312.03908> 

::::::::::::::::: 

Q6:  WHAT IS A STIFF PROBLEM/SOLVER? WHAT IS A COMPLIANT
PROBLEM/SOLVER? 

Answer: Stiffness refers to a characteristic of an ordinary differential
equation (ODE) system where some components evolve much more rapidly
than others. Traditional explicit solvers may become inefficient or
unstable when dealing with stiff ODEs because they require very small
step sizes to accurately capture the rapid changes in the stiff
components. Stiff solvers are designed to efficiently handle stiff ODEs
by automatically adjusting step sizes and using implicit methods that
can better capture rapid changes without requiring excessively small
steps. Examples of stiff solvers include implicit Runge-Kutta methods
and backward differentiation formulas. 

Compliance refers to the opposite of stiffness; it implies that all
components of the ODE system evolve at similar rates. For compliant
problems, explicit solvers may work well as they can take larger steps
without losing accuracy. Compliant solvers are designed to efficiently
handle compliant ODEs by taking advantage of their smooth behavior.
Explicit methods like explicit Euler or Runge-Kutta methods are often
used for solving compliant problems efficiently. 

::::::::::::::::



WHy are my actruator limits not satisfied? 
https://stackoverflow.com/questions/78462754/debugging-interpenetration-in-drake-simulation



------------------
3. LCM Warning About Buffer Size
Warning:

vbnet
Copy code
LCM detected that large packets are being received, but the kernel UDP receive buffer is very small. The possibility of dropping packets due to insufficient buffer space is very high.
Cause: This warning is related to the use of LCM (Lightweight Communications and Marshalling), which Drake uses for communication, particularly with visualization tools like Meshcat. It suggests that the system is receiving large packets over UDP, and the kernel’s buffer size is too small, leading to a risk of dropped packets.

Solution:

You can increase the size of the UDP receive buffer in your system. To do this, you can modify the kernel parameters with the following commands:
Copy code
sudo sysctl -w net.core.rmem_max=26214400
sudo sysctl -w net.core.rmem_default=26214400
This should help prevent packet drops and improve the stability of communication, especially during large visualizations.

--------------------

Infeasible Simulation During Line Search
Error:

makefile
Copy code
Warning: encountered infeasible simulation in linesearch
Cause: This suggests that during the simulation, some constraints or the dynamics update became infeasible during an optimization or line search process. This often happens with optimization-based solvers (e.g., SAP solver in Drake for discrete systems) when forces or state transitions violate constraints or when the system becomes numerically unstable.

Solution:

Check if the time step is appropriate for the dynamics of the system. If the time step is too large, this might cause instability, especially in discrete-time systems.
The gravity compensation calculation (u_gravitycomp) and the PID control terms might also be contributing to the infeasibility. Consider adding some form of damping to the control or ensuring that control inputs don't become too aggressive (e.g., limiting u).
You might also want to enable debugging information or reduce the complexity of the problem to better isolate where the infeasibility arises.
--------------------
1. Mass Matrix Non Positive-Definite Error
Error:

csharp
Copy code
An internal mass matrix associated with the joint that connects body panda_link4 to body panda_link5 is not positive-definite. Since the joint allows rotation, ensure body panda_link5 (combined with other outboard bodies) has reasonable non-zero moments of inertia about the joint rotation axes.
Cause: This error suggests that the inertia properties of the panda_link5 (and potentially other connected outboard bodies) are not realistic. Specifically, the mass or moment of inertia of the body might be too small or even zero, causing the joint's mass matrix to be singular or non-positive definite.

Solution: You should verify the URDF model to ensure that all bodies have reasonable mass and inertia properties. Look at the inertia tensor (<inertia>) and mass properties (<mass>) for the panda_link5 and any bodies outboard of it.

If using the URDF from an external source, ensure that these values aren't set to zero or unusually small.
You might also try adding a small damping element to the joints involved (e.g., panda_link4 to panda_link5) to stabilize the system during simulation.