# RoboticsII
Tutorial and materials for the Robotics I course


# Drake tutorial
This directory contains the documentation for the Drake tutorial. The tutorial provides a guide to using Drake for robotics simulation, suitable for both master's students and researchers.

## Usage
Follow the [installation](../README.md/#installation) section for a complete installation of the repository or to ensure you have Drake installed and to set your `PYTHONPATH`.

### Navigating the tutorial
- Start with the [drake tutorials](./drake_tutorial/) starting from [`01_Introduction.md`](./drake_tutorial/01_Introduction.md) for an overview of Drake and its functionalities.
- Use the accompanying [python scripts](./python_tutorials/) to run the examples from the tutorial. For example, the tutorial [`02a_Modelling.md`](./02a_Modelling.md) is accompayned by [`tutorial_2a.py`](../python_tutorials/tutorial_2a.py). Follow these steps to run it:

   1. Ensure you have Python 3 installed: You can download it from [python.org](python.org).

   3. Navigate to the tutorial directory: Open a terminal and change to the directory where [`tutorial_2a.py`](../python_tutorials/tutorial_2a.py) is located. For example:
   ```sh
   cd path/to/python_tutorials
   ```
   4. Run the tutorial script: Execute the script in your terminal using Python:
   ```sh
   python3 ./tutorial_2a.py
   ```
By following these steps, you can run the tutorial examples and explore the capabilities alongside the documentation. For further clarifications or frequently asked questions, refer to the [Q&A.md](./drake_tutorial/Q&A.md) file.


## Contents
<span style="color:red;">Currently, all the tutorials are ready for review except `03b_contact.md`, `05_resultsviztools.md` and `06_hardware.md`.</span>
| File                                                                                                                | Summary                         |
| ---                                                                                                                 | ---                             |
| [01_Introduction.md](./drake_tutorial/01_Introduction.md)         | Overview of Drake, its basic concepts, capabilities, and the scope of the tutorial. |
| [02a_Modelling.md](./drake_tutorial/02a_Modelling.md)                 | Introduction to the MultibodyPlant class in Drake for modeling interconnected physical systems such as robots. |
| [02b_dynamics.md](./drake_tutorial/02b_dynamics.md)                 | 	Advanced modeling techniques in Drake, including system blocks, custom controllers, and evaluating system dynamics. |
| [03a_solvers.md](./drake_tutorial/03a_solvers.md)           | 	Introduction to integration and solvers for simulation of multibody systems with frictional contact in Drake. |
| [03b_contact.md](./drake_tutorial/03b_contact.md)           | 	Contact modelling and mechanics in drake. |
| [04_Mathmaticalprogram.md](./drake_tutorial/04_Mathmaticalprogram.md)   | 	how to formulate and solve a mathematical optimization problem using Drake's MathematicalProgram class. |
| [05_resultsviztools.md](./drake_tutorial/05_resultsviztools.md)                   | 	Tools and methods for visualizing results, including plotting and interpreting simulation data. |
| [06_hardware.md](./drake_tutorial/06_hardware.md)                   | Guide to interfacing with hardware, including setting up and controlling physical robots. |
| [Q&A.md](./drake_tutorial/Q&A.md)   | 	Techniques and tips for debugging in Drake, including common issues and solutions.|
---

## C++ examples
Drake is a C++ library that provides an interface in Python using Pybind11 to enable rapid prototyping of new algorithms, and also aims to provide solid open-source implementations for many state-of-the-art algorithms. In this tutorial, we will be using the Python bindings of Drake. For equivalent examples in C++, you can refer to the examples in [drake_ws](https://github.com/Coryx99/drake_brubotics/tree/main/ros2_ws/src/drake_ws). 

Additional open source tutorials:
- [Getting Started with Drake](https://drake.guzhaoyuan.com/to-get-started) 
- [Kinova Drake Examples](https://github.com/vincekurtz/kinova_drake)

## Contributing
Contributions to the tutorial documentation are welcome. To contribute:

1. Fork the repository.
2. Create a new branch for your feature or fix.
3. Make your changes and commit them with clear messages.
4. Submit a pull request for review.

## Support
For any questions or issues realted to drake, please refer to the [official website](https://drake.mit.edu/getting_help.html). You can also reach out to the tutorial maintainers for specific inquiries related to this tutorial.

## License
This tutorial documentation is licensed under the MIT License. See the [LICENSE](../LICENSE) file for more details.


-----------
-----------
-----------
-----------
## To-do and add later: Mohayad
https://github.com/RobotLocomotion/drake/tree/master/tutorials
* Appendix 1: parsing rules (URDF, SDF, MJ) and .STL to .OBJ: https://imagetostl.com/convert/file/dae/to/obj#convert
- https://automaticaddison.com/how-to-convert-a-xacro-file-to-urdf-and-then-to-sdf/

https://github.com/RobotLocomotion/drake/blob/master/tutorials/README.md


<!-- - If you are using ROS and Gazebo and have a URDF robot file that has Xacro in it (i.e. XML macros) like this one, you can convert this file to a pure URDF file in Ubuntu Linux using the following command (everything below goes on the same line inside the Linux terminal window):

- xacro two_wheeled_robot.xacro > two_wheeled_robot.urdf
You can then convert the URDF file without Xacro into an SDF file if you wish. Here is the command to do that:
gz sdf -p two_wheeled_robot.urdf > two_wheeled_robot.sdf
That’s it! -->
