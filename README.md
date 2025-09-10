# Robotics-II
Tutorial and materials for the Robotics II practical sessions. The tutorial provides a guide to using Drake for robotics simulation

### Objectives
This course focuses on the design and implementation of a complete robotics application. Students will learn to integrate the core components of robotics, modeling, dynamics, planning, and control, into a coherent system. By the end of the project, students will be able to:

1. Model robotic systems and their environment.
2. Derive and simulate robot dynamics.
3. Design and implement control strategies for motion or interaction.
4. Incorporate sensing, planning, or decision-making elements where relevant.
5. Validate their solution in simulation.
6. Communicate results through a clear report and presentation.

## Project scope
Each team will design a robotics application that demonstrates:

- System Modeling – kinematic/dynamic modeling of the chosen robot (manipulator, mobile robot, aerial system, multi-robot system, etc.).

- Control – feedback or trajectory-based control to achieve a task.

- Simulation & vERFICATION – testing, validation, and performance analysis.


## Possible application examples:
* Pick-and-place with a manipulator.
* Mobile robot trajectory tracking and obstacle avoidance.
* Cooperative transport of an object by multiple robots.
* Contact-rich interaction such as pushing or grasping.

## Contents
| File                                                                                                                | Summary                         |
| ---                                                                                                                 | ---                             |
| [00_Startup.md](./tutorial_doc/00_Startup.md)         | Instaallation guide of the repo and its dependancies. |
| [01_Introduction.md](./tutorial_doc/01_Introduction.md)         | Introduction to the simulation framework (Drake), its basic components, and what you can achieve with it. |
| [02_Modelling.md](./tutorial_doc/02_Modelling.md)                 | Modeling robots and environments in Drake using the `MultibodyPlant`: joints, links, and URDF files. Includes visualization with MeshCat. |
| [03_Dynamics.md](./tutorial_doc/03_dynamics.md)                 | 	Running simulations with dynamics. Introduction to Drake’s `LeafSystem` for designing system blocks, and building model-based controllers. |
| [05_TrajectoryPlanning.md](./tutorial_doc/05_TrajectoryPlanning.md)                 | 	A simple introduction to trajectory planning: defining a reference trajectory, sending it to the controller, and visualizing results. |
| [Q&A.md](./tutorial_doc/Q&A.md)   | 	Practical tips and debugging guide: common issues, error messages, and how to fix them.|
---

### How to Navigate the tutorials?

1. Follow the [`00_Startup.md`](./tutorial_doc/00_Startup.md) installation section for a complete installation of the repository and its dependancies.

2. Start with the [drake tutorials](./tutorial_doc/) starting from [`00_Introduction.md`](./tutorial_doc/01_Introduction.md) for an overview of the simulation framework and its functionalities.

3. Use the accompanying [python scripts](./python_tutorials/) to run the examples from the tutorial. For example, the tutorial [`02_Modelling.md`](./02a_Modelling.md) is accompayned by [`tutorial_2a.py`](../python_tutorials/tutorial_2a.py). 

# Troubleshooting & Support 
If you find a bug in the repository, require assistance, or have any other questions, please open an issue in the repository **(recommended)** or contact one or more of the following via email:
* xx xx `xxxx.xxx@vub.be`
* Mohayad Omer `Mohayad.omer99@gmail`
* xx xx `xxxx.xxx@vub.be`

We will try to help you as soon as possible.


## Additional open source tutorials and examples
- [Getting Started with Drake](https://drake.guzhaoyuan.com/to-get-started) 
- [Kinova Drake Examples](https://github.com/vincekurtz/kinova_drake)
