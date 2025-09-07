import sys
sys.path.append('/opt/drake/lib/python3.8/site-packages')  # Adjust the path as needed

import numpy as np
import pydot
import matplotlib.pyplot as plt
from pydrake.all import (DiagramBuilder, MultibodyPlant, Parser,
                         SceneGraph, Simulator, ConstantVectorSource,
                         InverseDynamicsController, WeldJoint, Isometry3,
                         FindResourceOrThrow, StartMeshcat, MeshcatVisualizer, PiecewisePolynomial, TrajectorySource, LogVectorOutput, RigidTransform)

meshcat = StartMeshcat()
set_point = False

def run_panda_idc_demo():
    simulation_time = 30.0
    max_time_step = 0.01
    target_realtime_rate = 1.0
    add_gravity = True
    Kp = 8.0
    Ki = 0.1
    Kd = 4.0

    builder = DiagramBuilder()
    scene_graph = builder.AddSystem(SceneGraph())
    plant = MultibodyPlant(max_time_step)
    plant.RegisterAsSourceForSceneGraph(scene_graph)
    plant_index = Parser(plant).AddModelsFromUrl(
        "file:///home/cory/Documents/drake_ddp-main/models/panda_fr3/urdf/panda_fr3.urdf")

    if not add_gravity:
        plant.mutable_gravity_field().set_gravity_vector(np.zeros(3))

    # Weld the robot to world
    # root_link = plant.GetBodyByName("panda_link0")
    # plant.AddJoint(WeldJoint("weld_arm", plant.world_frame(), root_link.body_frame(),
    #                                 RigidTransform.Identity()))

    # Parser(plant).AddModels(url="file:///home/cory/Documents/object_models/table.sdf")
    plant.Finalize()

    # Desired state includes positions and velocities for each joint
    num_positions = plant.num_positions()
    num_velocities = plant.num_velocities()

    desired_state = np.zeros(num_positions + num_velocities)  # positions + velocities
    desired_state[0:num_positions] = [0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785, 0.0, 0.0]  # Positions + zero velocities
    
    # Define desired positions and times for the trajectory
    times = np.array([0.0, 10.0, 12.0, 14.0, 16.0, 18.0, 20.0, 22])  # Time points
    positions = np.array([
        [0.0, 0.0, 0.0, 0.0, -0.5, -0.5, 0.8, 0.8],
        [-0.785, -0.785, 1.0, 1.0, -0.785, -0.785, -0.785, -0.785],
        [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        [-2.356, -2.356, -1.256, -1.256, -2.356, -2.356, -2.356, -2.356],
        [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        [1.571, 1.571, 2.271, 2.271, 1.571, 1.571, 1.571, 1.571],
        [0.785, 0.785, 0.785, 0.785, 0.785, 0.785, 0.785, 0.785],
        [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    ])  # Corresponding desired positions
    desired_position_trajectory = PiecewisePolynomial.CubicShapePreserving(times, positions)
    desired_state_s = TrajectorySource(desired_position_trajectory,
                                            output_derivative_order=1)
    
    if set_point:
        desired_state_source = builder.AddSystem(
            ConstantVectorSource(desired_state))
    else:
        desired_state_source = builder.AddSystem(desired_state_s)

    IDC = builder.AddSystem(
        InverseDynamicsController(plant, np.ones(num_positions) * Kp,
                                  np.ones(num_positions) * Ki,
                                  np.ones(num_positions) * Kd, False))

    builder.AddSystem(plant)  # Add the plant system to the builder

    builder.Connect(IDC.get_output_port_control(),
                    plant.get_applied_generalized_force_input_port())

    builder.Connect(plant.get_state_output_port(),
                    IDC.get_input_port_estimated_state())

    builder.Connect(desired_state_source.get_output_port(),
                    IDC.get_input_port_desired_state())

    constant_zero_torque = builder.AddSystem(
        ConstantVectorSource(np.zeros(plant.num_actuators())))

    builder.Connect(constant_zero_torque.get_output_port(),
                    plant.get_actuation_input_port())

    source_id = plant.get_source_id()
    builder.Connect(plant.get_geometry_poses_output_port(),
                    scene_graph.get_source_pose_port(source_id))

    builder.Connect(scene_graph.get_query_output_port(),
                    plant.get_geometry_query_input_port())

    # Loggers
    logger = LogVectorOutput(plant.get_output_port(5), builder) #21
    MeshcatVisualizer.AddToBuilder(builder, scene_graph, meshcat)

    diagram = builder.Build()
    diagram_context = diagram.CreateDefaultContext()
    diagram.SetDefaultContext(diagram_context)

    plant_context = diagram.GetMutableSubsystemContext(plant, diagram_context)
    q = plant.GetPositions(plant_context)
    q[0] = 0.1
    q[1] = 0.5
    plant.SetPositions(plant_context, q)

    simulator = Simulator(diagram, diagram_context)
    simulator.set_publish_every_time_step(True)
    simulator.set_target_realtime_rate(target_realtime_rate)
    simulator.Initialize()

    svg_data = diagram.GetGraphvizString(max_depth=2)
    graph = pydot.graph_from_dot_data(svg_data)[0]
    image_path = "block_diagramID.png"  # Change this path as needed
    graph.write_png(image_path)
    print(f"Block diagram saved as: {image_path}")

    dt = 0.1  # Time step for each step
    step_number = 0

    C_m = np.array([7,1])
    try:
        while True:
            if set_point:
                # Prompt user for desired joint positions
                print("Enter desired joint positions (comma-separated):")
                print("Example: 0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785, 0, 0")
                user_input = input(">> ")
                user_positions = [float(val.strip()) for val in user_input.split(',')]

                if len(user_positions) != num_positions:
                    print(f"Error: Expected {num_positions} values.")
                    continue

                # Update desired state with user input (positions only)
                desired_state[0:num_positions] = user_positions
                desired_state_source.get_mutable_source_value().set_value(desired_state)
            # Advance   the simulation
            simulator.AdvanceTo(step_number * dt)

            # Regular update event that can modify state when needed
            # Here you can add your own logic to modify state at regular intervals
            # For example, modifying desired positions, etc.
            if step_number % 100 == 0:
                pass  # Add your update logic here
                # print(logger.FindLog(diagram_context).data().transpose())
                # print(plant.CalcBiasTerm(plant_context))
                print("STOPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPP")
                print(plant.CalcMassMatrix(plant_context))
            step_number += 1

    except KeyboardInterrupt:
        # Save the block diagram as an image file
        print("Simulation interrupted.")
        pass

        # Plot the results for the first 7 joints only.
    log = logger.FindLog(diagram_context)
    time = log.sample_times()
    data_q = log.data().transpose()[:, 0:9]  # Selecting only the first 7 columns (joints)
    data_qdot = log.data().transpose()[:, 9:18]  # Selecting only the first 7 columns (joints)

    # Plotting
    joint_names = ['Panda joint 1', 'Panda joint 2', 'Panda joint 3', 'Panda joint 4', 'Panda joint 5', 'Panda joint 6', 'Panda joint 7', 'panda_figer1', 'panda_figer2']
    for i in range(7):
        plt.plot(time, data_q[:, i], label=joint_names[i])

    plt.xlabel('Time [s]')
    plt.ylabel('Angle [rad]')
    plt.legend()
    plt.show()

    for i in range(7):
        plt.plot(time, data_qdot[:, i], label=joint_names[i])
    
    plt.xlabel('Time [s]')
    plt.ylabel('Velocity [rad/s]')
    plt.legend()
    plt.show()

if __name__ == '__main__':
    run_panda_idc_demo()


