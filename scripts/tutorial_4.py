from pydrake.solvers import MathematicalProgram, Solve, SnoptSolver
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from IPython.display import display, Markdown

# Arm segment lengths
L1 = 1.0
L2 = 1.0

# Target position (x, y)
x_target = 1.0
y_target = -1.0

# Setup the optimization problem
prog = MathematicalProgram()
auto = True  # Toggle automatic solver selection

# Decision variables: joint angles theta1 and theta2
theta = prog.NewContinuousVariables(2, "theta")

# Cost: minimize sum of squared joint angles (proxy for energy)
prog.AddCost(theta[0]**2 + theta[1]**2)

# Constraints: arm must reach the target position (x_target, y_target)
x_end = L1 * np.cos(theta[0]) + L2 * np.cos(theta[0] + theta[1])
y_end = L1 * np.sin(theta[0]) + L2 * np.sin(theta[0] + theta[1])

x_constraint = prog.AddConstraint(x_end == x_target)
x_constraint.evaluator().set_description("x_constraint")
y_constraint = prog.AddConstraint(y_end == y_target)
y_constraint.evaluator().set_description("y_constraint")

# Render the program in LaTeX for display
display(prog.ToLatex())

# Storage for intermediate solutions during optimization
trajectory = []

# Custom callback function to store joint angles without stopping execution
def store_arm_position(theta_vals):
    trajectory.append(np.copy(theta_vals))

prog.AddVisualizationCallback(store_arm_position, theta)

##### Solve the optimization problem ####
x_init = np.array([0.5, 0.1])  # Initial guess for joint angles

if auto:  # Automatic solver selection
    result = Solve(prog, x_init)
else:     # Manual solver selection (SNOPT in this case)
    solver = SnoptSolver()
    result = solver.Solve(prog, x_init, None)  # Optionally configure solver settings here

# Check if the optimization found a feasible solution
if result.is_success():
    theta_opt = result.GetSolution(theta)
    print(f"Optimal solution found: theta1 = {theta_opt[0]:.4f}, theta2 = {theta_opt[1]:.4f}")
    
    # Print the optimal cost and solver info
    print(f"Optimal cost: {result.get_optimal_cost():.4f}")
    print(f"Solver used: {result.get_solver_id().name()}")
    solver_details = result.get_solver_details()
    print(f"Solver details: {solver_details}")

    
    # Create a smoother, more visually appealing animation of the results
    fig, ax = plt.subplots()
    ax.set_xlim(-2, 2)
    ax.set_ylim(-2, 2)
    ax.set_aspect('equal', adjustable='box')
    ax.set_title("2-DOF Arm Reaching Target")
    ax.set_xlabel("X Position")
    ax.set_ylabel("Y Position")

    # Initial plot elements
    arm_line, = ax.plot([], [], 'o-', lw=3, color='b', label='Arm')  # Blue arm
    target_plot, = ax.plot(x_target, y_target, 'rx', markersize=10, label='Target')  # Red target point

    def update(frame):
        theta_vals = trajectory[frame]
        # Arm joint positions
        x_arm = [0, L1 * np.cos(theta_vals[0]), L1 * np.cos(theta_vals[0]) + L2 * np.cos(theta_vals[0] + theta_vals[1])]
        y_arm = [0, L1 * np.sin(theta_vals[0]), L1 * np.sin(theta_vals[0]) + L2 * np.sin(theta_vals[0] + theta_vals[1])]

        # Update arm position in plot
        arm_line.set_data(x_arm, y_arm)
        return arm_line, target_plot

    ani = FuncAnimation(fig, update, frames=len(trajectory), interval=200, blit=True, repeat=False)

    plt.legend()
    plt.show()
else:
    print("No solution found")