#!/usr/bin/env python3

from re import T
import threading
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
from std_srvs.srv import SetBool
from linkattacher_msgs.srv import AttachLink, DetachLink
from gazebo_msgs.srv import SpawnEntity
from geometry_msgs.msg import Pose
import sympy as sp
from time import sleep
import os
import sys
import select
import tty
import termios
from ament_index_python.packages import get_package_share_directory
import xacro
import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d import Axes3D 
import math  

# Define key codes
LIN_VEL_STEP_SIZE = 10
ANG_VEL_STEP_SIZE = 5
MAX_VEL = 30

class RobotControlNode(Node):
    def __init__(self):
        super().__init__('robot_control_node')
        
        # Publisher and Service Clients
        self.velocity_controller_pub = self.create_publisher(Float64MultiArray, '/velocity_controller/commands', 10)
        self.joint_position_pub = self.create_publisher(Float64MultiArray, '/position_controller/commands', 10)
        self.joint_state_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.gripper = self.create_client(SetBool, 'bot/vaccum/vaccum_switch')
        self.gripper2 = self.create_client(SetBool, 'bot/vaccum/vaccum_switch2')
        self.gripper3 = self.create_client(SetBool, 'bot/vaccum/vaccum_switch3')
        self.gripper4 = self.create_client(SetBool, 'bot/vaccum/vaccum_switch4')
        self.gripper5 = self.create_client(SetBool, 'bot/vaccum/vaccum_switch5')
        self.attach_link_client = self.create_client(AttachLink, '/ATTACHLINK')
        self.detach_link_client = self.create_client(DetachLink, '/DETACHLINK')
        self.spawn_client = self.create_client(SpawnEntity, '/spawn_entity')
        
        # Settings for keyboard input
        self.settings = termios.tcgetattr(sys.stdin)
        self.wait_for_services()
        self.req = SetBool.Request()
        self.latest_box_name = ''
        self.attached_box_name = ''
        self.in_teleop_mode = False
        
    def wait_for_services(self):
        services = [self.gripper, self.attach_link_client, self.spawn_client, self.gripper2, self.gripper3, self.gripper4, self.gripper5, self.detach_link_client]
        for service in services:
            while not service.wait_for_service(timeout_sec=1.0):
                self.get_logger().info(f'Waiting for service {service.srv_name} to start...')

    def spawn_object(self, name, xml, position):
        if not self.spawn_client.service_is_ready():
            self.spawn_client.wait_for_service()

        request = SpawnEntity.Request()
        request.name = name
        request.xml = xml
        request.initial_pose.position.x = position[0]
        request.initial_pose.position.y = position[1]
        request.initial_pose.position.z = position[2]

        future = self.spawn_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            # self.get_logger().info(f'Spawned object: {name}')
            self.latest_box_name = name
        # else:
        #     self.get_logger().error(f'Failed to spawn: {name}')

    def attach_latest_box(self, attach):
        # Method to attach the latest box to the robot arm
        if attach is True:
            self.get_logger().info(f'Attaching box: {self.latest_box_name}')
            if self.latest_box_name and self.attach_link_client.service_is_ready():
                attach_link_request = AttachLink.Request()
                attach_link_request.model1_name = 'bot'
                attach_link_request.link1_name = 'vaccum'
                attach_link_request.model2_name = self.latest_box_name
                attach_link_request.link2_name = 'box'

                future = self.attach_link_client.call_async(attach_link_request)
                rclpy.spin_until_future_complete(self, future)
                # if future.result() is not None:
                #     self.get_logger().info('AttachLink service call succeeded')
                # else:
                #     self.get_logger().error('AttachLink service call failed')
        elif attach is False:
            self.get_logger().info(f'Detaching box: {self.attached_box_name}')
            if self.attached_box_name and self.detach_link_client.service_is_ready():
                detach_link_request = DetachLink.Request()
                detach_link_request.model1_name = 'bot'
                detach_link_request.link1_name = 'vaccum'
                detach_link_request.model2_name = self.attached_box_name
                detach_link_request.link2_name = 'box'

                future = self.deteach_link_client.call_async(detach_link_request)
                rclpy.spin_until_future_complete(self, future)
                # if future.result() is not None:
                #     self.get_logger().info('DetachLink service call succeeded')
                # else:
                #     self.get_logger().error('DetachLink service call failed')

    def gripper_control(self, state):
            self.req.data = state
            gripper_clients = [self.gripper, self.gripper2, self.gripper3, self.gripper4, self.gripper5]
            for gripper in gripper_clients:
                gripper.call_async(self.req)

    def dh_transform_matrix(self, a, alpha, d, theta):
        """Compute the DH transformation matrix for given parameters."""
        return sp.Matrix([
            [sp.cos(theta), -sp.sin(theta)*sp.cos(alpha),  sp.sin(theta)*sp.sin(alpha), a*sp.cos(theta)],
            [sp.sin(theta),  sp.cos(theta)*sp.cos(alpha), -sp.cos(theta)*sp.sin(alpha), a*sp.sin(theta)],
            [0, sp.sin(alpha), sp.cos(alpha), d],
            [0, 0, 0, 1]
        ])

    def kinematics(self, joint_positions):
        theta_symbols, a_symbols, d_symbols = sp.symbols('theta1:7'), sp.symbols('a1:7'), sp.symbols('d1:7')
        
        # DH parameters and offsets
        a_values, alpha_values, d_values = [0, 0.612, 0.5723, 0, 0, 0], [-sp.pi/2, 0, 0, -sp.pi/2, sp.pi/2, 0], [0.1273, 0, 0, 0.163941, 0.1157, 0.0922]
        theta_offsets = [0, -sp.pi/2, 0, -sp.pi/2, 0, 0]
        
        a_dict = {a: value for a, value in zip(a_symbols, a_values)}
        d_dict = {d: value for d, value in zip(d_symbols, d_values)}
        subs_lengths = {**a_dict, **d_dict}
        
        # Generate transformation matrices
        T_matrices = [self.dh_transform_matrix(a, alpha, d, theta) for a, alpha, d, theta in zip(a_symbols, alpha_values, d_symbols, theta_symbols)]
        
        # Compute final transformation matrix
        T_final = T_matrices[0]
        for matrix in T_matrices[1:]:
            T_final *= matrix     

        # Calculate and print Jacobian and its components
        Xp = T_final[:-1, -1].subs(subs_lengths)
        dif_q = [sp.diff(Xp, theta_symbol) for theta_symbol in theta_symbols]   # List of differentials of Xp wrt each theta
        T_0_n = [sp.prod(T_matrices[:i+1]) for i in range(len(T_matrices))]   # List of T_0_i matrices for each i
        z_n = [T[:-1, 2] for T in T_0_n]    # List of z vectors for each frame i
        Jacobian = sp.Matrix.vstack(sp.Matrix.hstack(*dif_q), sp.Matrix.hstack(*z_n)) # Jacobian matrix
        
        total_time = 20 # Total time for which the robot moves
        t = 0
        dt = total_time/200 # Time step
        omega = 2*sp.pi/total_time  # Angular velocity
        initial_angles = [0.0001, -0.0002, 0.0004, -0.0004, 0.0001, -0.0002]    # Initial joint angles with small offsets
        initial_angles = [x + y for x, y in zip(initial_angles, theta_offsets)] # Add offsets to initial angles
        q = sp.Matrix(initial_angles).evalf()   # Joint variables
        subs_theta = dict(zip(theta_symbols, initial_angles)) # Substitution dict for joint variables
        joint_positions.data = [0.0001, -0.0002, 0.0004, -0.0004, 0.0001, -0.0002]
        self.joint_position_pub.publish(joint_positions)
        sleep(1)
        T_final = T_final.subs(subs_lengths)
        return T_final, theta_offsets, subs_theta, Jacobian, q, t, dt, omega, theta_symbols, total_time    

    def control(self, validation=False):
        self.get_logger().info("Entering Control Mode\n")
        joint_positions = Float64MultiArray()
        velocity = Float64MultiArray()
        
        velocity.data = [0.0,0.0,0.0,0.0]
        self.velocity_controller_pub.publish(velocity)
        
        x_data, y_data, z_data = [], [], []
        direction = 1
        reverse_index = -1
        print("Moving Manipulator\n")
        joint_positions_history = []
        fig = plt.figure(figsize=(12, 6))
        ax = plt.axes(projection='3d')
        T_final, theta_offsets, subs_theta, Jacobian, q, t, dt, omega, theta_symbols, total_time  = self.kinematics(joint_positions)
        while t <= total_time:
            if direction == 1:
                # Update joint angles
                publish_angles = []
                J_thetas = sp.Matrix(Jacobian.subs(subs_theta)).evalf()
                
                vx = 0.5*omega*sp.sin(omega*t+sp.pi/2) # Fixed end-effector velocity in x-direction
                vz = 0.5*omega*sp.cos(omega*t+sp.pi/2)  # Fixed end-effector velocity in z-direction
                x_dot = sp.Matrix([vx, 0, vz, 0.0, 0.0, 0.0]).evalf()  # Fixed end-effector velocity

                J_inv = J_thetas.pinv()  # Compute pseudo-inverse
                q_dot = J_inv * x_dot   # Compute joint velocities
                q = q + q_dot * dt  # Update joint angles
                subs_theta = dict(zip(theta_symbols, [q[i, 0] for i in range(6)]))  # Update substitution dict for joint variables 
                t += dt

                for i in range(6):
                    if i==1:
                        publish_angles.append(float(-q[i, 0] + theta_offsets[i])) # For joint 2, Offset is added to make it move in the correct direction
                    elif i==4:
                        publish_angles.append(float(-q[i, 0] + theta_offsets[i])) # For joint 5, Offset is added to make it move in the correct direction
                    else:
                        publish_angles.append(float(q[i, 0] - theta_offsets[i]))
                        
                if validation:         
                    A = T_final.subs(subs_theta).evalf()    # Compute end-effector pose        
                    # Store new points in the lists
                    x_data.append(A[:3, -1][0])
                    y_data.append(A[:3, -1][1])
                    z_data.append(A[:3, -1][2])
                    
                joint_positions.data = publish_angles
                joint_positions_history.append(publish_angles.copy())
                self.joint_position_pub.publish(Float64MultiArray(data=publish_angles))
                if t > 3.3:
                    if not validation:
                        # Turn on the gripper
                        self.gripper_control(True)
                        # Call the AttachLink service
                        self.attached_box_name = self.latest_box_name
                        self.attach_latest_box(attach=True)
                        sleep(10)
                    direction = -1
                    reverse_index = len(joint_positions_history) - 1
                    t = 0
            # Reverse motion logic
            elif direction == -1:
                if reverse_index >= 0:
                    publish_angles = joint_positions_history[reverse_index]
                    self.joint_position_pub.publish(Float64MultiArray(data=publish_angles))
                    reverse_index -= 1

                else:
                    print("Reverse Motion Completed!")
                    break 
            # Increment or reset time based on direction
            if direction == 1:
                t += dt
            else:
                t -= dt  # Decrement time during reverse motion

            sleep(dt) 
            
        if validation:
            ax.set_xlabel("X Coordinate (m)")
            ax.set_ylabel("Y Coordinate (m)")
            ax.set_zlabel("Z Coordinate (m)")
            if len(x_data) == len(y_data) == len(z_data):
                ax.scatter(x_data, y_data, z_data, color='blue')
            # Setting equal aspect ratio for 3D plot
            max_range = np.array([float(max(x_data)-min(x_data)), float(max(y_data)-min(y_data)), float(max(z_data)-min(z_data))]).max() / 2.0
            mid_x = float((max(x_data) + min(x_data)) * 0.5)
            mid_y = float((max(y_data) + min(y_data)) * 0.5)
            mid_z = float((max(z_data) + min(z_data)) * 0.5)
            buffer = 0.01   # Buffer for the plot 
            ax.set_xlim(mid_x - max_range - buffer, mid_x + max_range + buffer)
            ax.set_ylim(mid_y - max_range - buffer, mid_y + max_range + buffer)
            ax.set_zlim(mid_z - max_range - buffer, mid_z + max_range + buffer)
            plt.show()   
        print("\nPicked up the box!\nControl Mode Complete. Returning to main menu...\n")  

    def forward_validation(self):
        joint_positions = Float64MultiArray()
        self.get_logger().info("Forward Kinematics Validation\n")
        T_final, theta_offsets, subs_theta, Jacobian, q, t, dt, omega, theta_symbols, total_time  = self.kinematics(joint_positions)
        user_input = input("Enter joint angles separated by spaces (e.g., 0 0 0 0 0 0): ")
        try:
            joint_values_degrees = [float(angle) for angle in user_input.split()]
            if len(joint_values_degrees) != 6:
                raise ValueError("Exactly 6 joint angles are required.")

            # Convert each joint angle from degrees to radians
            joint_values_radians = [math.radians(angle) for angle in joint_values_degrees]

            print("Calculating Forward Kinematics...\n")
            theta_subs = {theta: value for theta, value in zip(theta_symbols, joint_values_radians)}
            T_final_substituted = T_final.subs(theta_subs)

            # Print the final transformation matrix
            print(f"Final Transformation Matrix with given joint angles:\n{sp.pretty(T_final_substituted)}\n")
            
            joint_positions.data = joint_values_radians
            self.joint_position_pub.publish(joint_positions)
        except ValueError as e:
            print(f"Invalid input: {e}")
        print("Forward Kinematics Validation Complete. Returning to main menu...\n\n")


    def periodic_spawn(self):
        package_name = 'bot'
        urdf_file_name = 'box.urdf'
        position = (3.8, 5.45, 2.0)
        box_counter = 0

        urdf_file_path = os.path.join(get_package_share_directory(package_name), 'urdf', urdf_file_name)
        xacro_file = xacro.process_file(urdf_file_path)
        object_xml = xacro_file.toxml()

        while rclpy.ok():
            object_name = f'box_{box_counter}'
            self.spawn_object(object_name, object_xml, position)
            box_counter += 1
            sleep(7)  # Spawn every 7 seconds
            
    def getKey(self):
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def run_teleop(self):
        self.get_logger().info("Entering Teleoperation Mode...\n")
        # For Differential Drive Mechanism without plugin
        self.msg = """
        Control The Robot!
        ---------------------------
        Moving around:
             w
        a    s    d
             x
        s : force stop
        c: Run Control Script
        Esc to quit
        """

        self.get_logger().info(self.msg)
        wheel_velocities = Float64MultiArray()
        left_vel=0.0
        right_vel=0.0

        while self.in_teleop_mode:
            key = self.getKey()
            if key is not None:
                if key == 's':  # Quit
                    left_vel=0.0
                    right_vel=0.0
                elif key == 'w' and left_vel <MAX_VEL and right_vel<MAX_VEL:  # Forward
                    left_vel += LIN_VEL_STEP_SIZE 
                    right_vel += LIN_VEL_STEP_SIZE
                elif key == 'x'and left_vel >-MAX_VEL and right_vel>-MAX_VEL:  # Reverse
                    left_vel -= LIN_VEL_STEP_SIZE 
                    right_vel -= LIN_VEL_STEP_SIZE 
                elif key == 'd' and left_vel in range(-MAX_VEL,MAX_VEL) and right_vel in range(-MAX_VEL,MAX_VEL):  # Right
                    right_vel -= ANG_VEL_STEP_SIZE
                    left_vel += ANG_VEL_STEP_SIZE
                elif key == 'a' and left_vel in range(-MAX_VEL,MAX_VEL) and right_vel in range(-MAX_VEL,MAX_VEL):  # Left
                    left_vel -= ANG_VEL_STEP_SIZE
                    right_vel += ANG_VEL_STEP_SIZE
                elif key == '\x1b':  # Escape key to exit
                    rclpy.shutdown()
                    return       
                elif key == 'c':
                    self.in_teleop_mode = False
                print(f"Left Wheel Velocity: {left_vel}, Right Wheel Velocity: {right_vel}",end="\r")

                wheel_velocities.data = [left_vel, right_vel, -left_vel, right_vel]
                self.velocity_controller_pub.publish(wheel_velocities)
    
    def run(self):
        self.get_logger().info("Robot Control Node Running...\n")
        self.msg = """
        Press t to start teleop mode
        ---------------------------
        Press c to start control script
        ---------------------------
        Press f to run forward kinematics validation
        ---------------------------
        Press i to run inverse kinematics validation
        ---------------------------
        Esc to quit
        """   
        print_menu = True  # Flag to control when to print the menu

        while rclpy.ok():
            if print_menu:
                self.get_logger().info(self.msg)  # Print the message only if flag is set
                print_menu = False  # Reset the flag after printing

            key = self.getKey()
            if key == 't' and not self.in_teleop_mode:
                self.in_teleop_mode = True
                self.run_teleop()
                print_menu = True  # Set the flag to print the menu again
            elif key == 'c' and not self.in_teleop_mode:
                self.control()  # Start control script
                print_menu = True  # Set the flag to print the menu again
            elif key == 'f':
                self.forward_validation()
                print_menu = True  # Set the flag to print the menu again
            elif key == 'i':
                self.control(validation=True)
                print_menu = True  # Set the flag to print the menu again
            elif key == '\x1b':  # Escape key to exit
                break

        self.get_logger().info("Exiting Robot Control Node...\n")
        
def main(args=None):
    rclpy.init(args=args)
    node = RobotControlNode()

    spawn_thread = threading.Thread(target=node.periodic_spawn)
    spawn_thread.start()

    node.run()

    spawn_thread.join()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()