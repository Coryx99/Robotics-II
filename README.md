# Robotics-II
Tutorial and materials for the Robotics II practical sessions. 

### Objectives
This course focuses on the design and implementation of a complete robotics application. Students will learn to integrate the core components of robotics, modeling, dynamics, planning, and control, into a coherent system. By the end of the project, students will be able to:

1. Model robotic systems and their environment.
2. Derive and simulate robot dynamics.
3. Design and implement control strategies for motion or interaction.
4. Incorporate sensing, planning, or decision-making elements where relevant.
5. Validate their solution in simulation.

***Results demonstrated/communicated through:*** Clear report and group presentation.

## Project scope
Each team will design a robotics application that demonstrates:

- **System Modeling**: kinematic/dynamic modeling of the chosen robot (manipulator, mobile robot, aerial system, multi-robot system, etc.).

- **Robot control**: feedback or trajectory-based control to achieve a task.

- **Simulation & Verfication**: testing, validation, and performance analysis.


## Possible application examples:
* Pick-and-place with a manipulator.
* Mobile robot trajectory tracking and obstacle avoidance.
* Cooperative transport of an object by multiple robots.
* Contact-rich interaction such as pushing or grasping.

## Tutorial content
| File                                                                                                                | Summary                         |
| ---                                                                                                                 | ---                             |
| [00_Startup.md](./tutorial_doc/00_Startup.md)         | Installation guide of the project material and its dependancies, along with a guide on setting up Ubuntu for your machine. |
| [01_Introduction.md](./tutorial_doc/01_Introduction.md)         | Introduction to the simulation framework (Drake), its basic components, and what you can achieve with it. |
| [02_Modelling.md](./tutorial_doc/02_Modelling.md)                 | Modeling and simulating robots and environments in Drake using the `MultibodyPlant`. Includes visualization with MeshCat. |
| [03_Dynamics.md](./tutorial_doc/03_Dynamics.md)                 | 	Running simulations with dynamics. Introduction to Drake’s `LeafSystem` for designing system blocks, and building model-based controllers. |
| [05_TrajectoryPlanning.md](./tutorial_doc/05_TrajectoryPlanning.md)                 | 	Short introduction to trajectory planning: defining a reference trajectory, sending it to the controller, and visualizing results. |
| [Q&A.md](./tutorial_doc/Q&A.md)   | 	Practical tips and debugging guide: common issues, error messages, and how to fix them.|
---

### How to Navigate the tutorials?

1. Follow the [`00_Startup.md`](./tutorial_doc/00_Startup.md) installation section for a complete installation of the repository and its dependancies.

2. Start with the [tutorials](./tutorial_doc/) starting from [`00_Introduction.md`](./tutorial_doc/01_Introduction.md) for an overview of the simulation framework and its functionalities.

3. Use the accompanying [python scripts](./tutorial_scripts/) to run the examples from the tutorial. For example, the tutorial [`02_Modelling.md`](./tutorial_doc/02_Modelling.md) is accompayned by [`tutorial_02.py`](../tutorial_scripts/tutorial_02.py). 

# Troubleshooting & Support 
If you find a bug in the repository, require assistance, or have any other questions, please open an issue in the repository **(recommended)** or contact one or more of the following via email:
* xx xx `xxxx.xxx@vub.be`
* Mohayad Omer `Mohayad.omer99@gmail`
* xx xx `xxxx.xxx@vub.be`

We will try to help you as soon as possible.

<!-- --------------------
## 📘 What Needs Strengthening (since students lack software background @Mohayad)

Every tutorial should include step-by-step terminal commands.

Provide template Python scripts with plenty of comments (not blank exercises?).

Explain Python basics as you go (e.g., “this function defines a controller”).

Concept Bridges: Before each Drake concept, give a short plain-language intro in control terms. Example: “A MultibodyPlant is Drake’s way of representing the equations of motion of a robot (like your free-body diagrams, but in code).”

Debugging Support: A dedicated Q&A.md with common errors, like:

Module not found → check PYTHONPATH.
MeshCat not showing → restart the webpage.
Simulation exploding → timestep too large. -->