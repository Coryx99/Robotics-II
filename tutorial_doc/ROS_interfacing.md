### How to model the rope?

using a series rigid links.

The bushing element will be a bit more expensive to simulate, and one
always needs to understand that very large stiffness or damping
coefficients will require smaller simulation time steps (again slower
simulation).

![](Pictures/10000000000002B90000028B0CA0523BADECDE6D.png){width="6.6929in"
height="6.2508in"}

Check: 22-23 -25 appendix A of the paper of kelly....

1- Drone: Model all drones

2- Check trajectory optimization and ex. Can I use garubi? How to use
it?

3- Do BrickieBot work

### Hybrid System Representation:

-   **Robot Dynamics:** Mr(qr)q¨r+Cr(qr,q˙r)q˙r+gr(qr)=ur+Jr\^T F\_c
-   ****Husky p**latform Dynamics:** Mh(qh)q¨h+Ch(qh,q˙h)q˙h+gh(qh)=uh

#### 1. **Modes:**

Define the discrete modes of the system. In this case:

-   M1: Robot moving without the block (No Contact)
-   M2: Robot moving with the block (Contact)

Each mode will have different dynamic equations (and possibly different
control strategies).

#### 2. **State Space:**

Define the continuous state of the system: z=(q\_r,q˙r,qh,q˙h,pb) where:

-   qr,q˙r: Joint positions and velocities of the Panda robot.
-   qh,q˙h: State variables of the Husky platform.
-   pb: Position of the block.

#### 3. **Guard/ Witness Functions:**

Guard functions determine the conditions under which transitions between
modes occur. Basically a mathematical expressions that represent
conditions for switching modes.

Example guard functions:

-   **Transition from No Contact to Contact:** g1,2(z)=∥xr−pb∥−ϵ≤0 This
    function becomes zero or negative when the robot's end-effector is
    close enough to the block, triggering the contact mode.
-   **Transition from Contact to No Contact:** g2,1(z)=∥xr−ptarget∥−ϵ≤0
    This function becomes zero or negative when the block is placed at
    the target position, triggering a switch back to no contact mode.

#### 4. **Reset Functions:**

Reset functions define how the state changes during mode transitions.

Example reset functions:

-   **Contact Mode Entry:** z+=Δ1,2(z−) Here, we may update the state to
    reflect that the robot is now carrying the block: pb+=xr− This
    ensures the block's position is now tied to the end-effector's
    position.
-   **Contact Mode Exit:** z+=Δ2,1(z−) This may simply involve
    decoupling the block from the end-effector.

### Trajectory Optimization:

#### 1. **Objective Function:**

The goal is to minimize a cost function that typically includes terms
for tracking error, control effort, and any additional task-specific
costs.

ex. cost function: J=∫0T​\[∥x(t)−xd(t)∥2+∥u(t)∥2+λ\_c C(z(t))\] dt
where:

-   x(t): Current trajectory of the end-effector.
-   xd(t): Desired trajectory.
-   u(t): Control inputs.
-   C(z(t)): Additional cost associated with contact, such as force
    constraints.
-   λc: Weighting factor for the contact cost.

#### 2. **Dynamic Constraints:**

Ensure the system dynamics are respected:

-   **Robot Dynamics:** Mr(qr)q¨r+Cr(qr,q˙r)q˙r+gr(qr)=ur+Jr\^T F\_c
-   **Platform Dynamics:** Mh(qh)q¨h+Ch(qh,q˙h)q˙h+gh(qh)=uh

#### 3. **Initial and Final Conditions:**

Specify the start and end states: z(0)=z0,z(T)=zf

#### 4. **State and Control Constraints:**

Incorporate physical and operational limits:
zmin≤z(t)≤zmax,umin≤u(t)≤umax

### Optimization Techniques:

#### Sequential Quadratic Programming (SQP)?

-   **Objective:** Iteratively solve a series of quadratic programming
    (QP) subproblems.

-   **Algorithm:**

    1.  Linearize the constraints and approximate the cost function
        quadratically.
    2.  Solve the QP subproblem to find a search direction.
    3.  Perform a line search along the search direction.
    4.  Update the solution and iterate.

#### Differential Dynamic Programming (DDP)?

-   **Objective:** Use dynamic programming principles to solve the
    trajectory optimization problem.

-   **Algorithm:**

    1.  Forward pass to simulate the system dynamics.
    2.  Backward pass to compute the value function and policy
        improvements.
    3.  Update the control inputs and iterate.

### 1. **Hybrid System Representation:**

#### Modes:

-   **No Contact (Mode 1):** M1 Dynamics without the block.
-   **Contact (Mode 2):** M2 Dynamics with the block.

#### State Space:

-   State vector: z=(qr,q˙r,qh,q˙h,pb)

#### Guard Functions:

-   **From No Contact to Contact:** g1,2(z)=∥xr−pb∥−ϵ≤0
-   **From Contact to No Contact:** g2,1(z)=∥xr−ptarget∥−ϵ≤0

#### Reset Functions:

-   **Contact Mode Entry:** z+=Δ1,2(z−) pb+=xr−
-   **Contact Mode Exit:** z+=Δ2,1(z−)

### 2. **Trajectory Optimization:**

#### Objective Function:

Define the cost function J: J=∫0T\[∥x(t)−xd(t)∥2+∥u(t)∥2+λcC(z(t))\]dt
where:

-   x(t): Current trajectory.
-   xd(t): Desired trajectory.
-   u(t): Control inputs.
-   C(z(t)): Additional cost for contact.
-   λc: Weighting factor.

#### Dynamic Constraints:

-   **Robot Dynamics:** Mr(qr)q¨r+Cr(qr,q˙r)q˙r+gr(qr)=ur+JrTFc
-   **Platform Dynamics:** Mh(qh)q¨h+Ch(qh,q˙h)q˙h+gh(qh)=uh

#### Guard Conditions:

Ensure transitions: gi,j(z)=0

#### Reset Conditions:

Update states during transitions: z+=Δi,j(z−)

#### State and Control Constraints:

zmin≤z(t)≤zmax,umin≤u(t)≤umax

### 3. **Optimization Algorithm:**

#### Sequential Quadratic Programming (SQP):

SQP is well-suited for nonlinear optimization problems. Ex....

1.  **Initialization:**

    -   Initialize z0 and u0.
    -   Set parameters for convergence criteria.

2.  **Iterative Optimization:**

    -   **Linearize constraints and approximate the cost function:**
        minΔz,Δu21 ΔzTHΔz+gTΔz subject to: AΔz=b
    -   **Solve the QP subproblem** to find search direction (Δz,Δu).
    -   **Line Search**: Determine the optimal step size α: zk+1=zk+αΔz
        uk+1=uk+αΔu
    -   **Update solution** and iterate until convergence.

#### Differential Dynamic Programming (DDP):

DDP leverages the Bellman principle for dynamic programming. A
simplified implementation:

1.  **Initialization:**

    -   Initialize state and control trajectories.

2.  **Forward Pass:**

    -   Simulate system dynamics using the current control trajectory to
        obtain state trajectory.

3.  **Backward Pass:**

    -   Compute the value function and policy improvements:
        Q(zk,uk)=l(zk,uk)+V(zk+1) where l(zk,uk) is the stage cost, and
        V(zk+1) is the value function.
    -   Calculate the gradients: Qz=∂z∂Q,Qu=∂u∂Q,Qzz=∂z2∂2Q,Quu=∂u2∂2Q
    -   Update control policy: Δuk=−Quu−1Qu

4.  **Update Trajectories:**

    -   Update state and control trajectories based on policy
        improvements.

5.  **Iterate until convergence:**

### 

#### Pseudocode for Hybrid Trajectory Optimization using SQP:

python

Copy code

*import numpy as np*

*from scipy.optimize import minimize*

*\# Define system dynamics and constraints*

*def robot\_dynamics(z, u):*

* \# Implement robot dynamics equations*

* pass*

*def platform\_dynamics(z, u):*

* \# Implement platform dynamics equations*

* pass*

*def combined\_dynamics(z, u):*

* \# Combine robot and platform dynamics*

* dz = np.zeros\_like(z)*

* dz\[:nr\] = robot\_dynamics(z\[:nr\], u\[:nr\])*

* dz\[nr:\] = platform\_dynamics(z\[nr:\], u\[nr:\])*

* return dz*

*def cost\_function(z, u, z\_desired):*

* \# Define the cost function J*

* tracking\_error = np.linalg.norm(z - z\_desired)\*\*2*

* control\_effort = np.linalg.norm(u)\*\*2*

* contact\_cost = contact\_penalty(z) if in\_contact\_mode(z) else 0*

* return tracking\_error + control\_effort + lambda\_c \* contact\_cost*

*def guard\_condition\_contact(z):*

* \# Check guard condition for contact mode*

* return np.linalg.norm(z\[:3\] - z\[3:6\]) \<= epsilon*

*def guard\_condition\_no\_contact(z):*

* \# Check guard condition for no contact mode*

* return np.linalg.norm(z\[:3\] - target\_position) \<= epsilon*

*def reset\_contact(z):*

* \# Reset function for contact mode entry*

* z\[3:6\] = z\[:3\] \# Position of the block matches end-effector*

* return z*

*def reset\_no\_contact(z):*

* \# Reset function for contact mode exit*

* return z*

*\# Define optimization variables*

*z = np.random.rand(nz)*

*u = np.random.rand(nu)*

*z\_desired = np.zeros(nz) \# Define the desired state trajectory*

*\# Define constraints and bounds*

*bounds = \[(zmin, zmax) for zmin, zmax in zip(z\_min, z\_max)\] +
\[(umin, umax) for umin, umax in zip(u\_min, u\_max)\]*

*constraints = \[{\'type\': \'eq\', \'fun\': lambda z:
combined\_dynamics(z, u)}\]*

*\# Define the optimization problem*

*def objective(zu):*

* z, u = np.split(zu, \[nz\])*

* return cost\_function(z, u, z\_desired)*

*\# Solve the optimization problem using SQP*

*result = minimize(objective, np.hstack((z, u)),
constraints=constraints, bounds=bounds, method=\'SLSQP\')*

*\# Extract optimized state and control*

*z\_opt = result.x\[:nz\]*

*u\_opt = result.x\[nz:\]*

*\# Apply the optimized control inputs*

*apply\_control(z\_opt, u\_opt)*
