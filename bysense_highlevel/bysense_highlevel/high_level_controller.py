import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
import numpy as np
import torch

import io

from imu_handler import ImuHandler
from utils import map_array_to_target_order, generate_spline_interpolation


class HighLevelController(Node):

    def __init__(self):
        super().__init__('high_level_controller')

        #Initialize the IMU handler
        self.imu_handler = ImuHandler(self)

        # turn this on for debug logs
        self.verbose = False

        # Publisher for sending setpoints
        self.setpoint_pub = self.create_publisher(Float64MultiArray, 'high_level/setpoints', 10)

        # Subscriber to receive current joint states from the robot simulation
        self.joint_state_subscriber = self.create_subscription(JointState, '/joint_states', self.joint_state_callback, 10)

        self.current_positions = None  # Will store current joint positions
        self.current_velocities = None  # Will store current joint velocities
        self.setpoints = []  # Interpolated setpoints will go here
        self.current_index = 0

        # Timer to publish setpoints at a fixed rate
        self.timer = self.create_timer(0.02, self.publish_setpoints)

        # Flag to track whether the joint states have been received
        self.joint_positions_received = False

        self.current_joint_names = None

        self.lin_vel_scale = 2.0
        self.ang_vel_scale = 0.25
        self.action_scale = 0.25
        self.dof_vel_scale = 0.05
        self.dof_pos_scale = 1.0
        self.device = 'cpu'
        self.policy_mapping = [
            '1-FL-HAA', '2-FL-HFE', '3-FL-KFE',
            '4-FR-HAA', '5-FR-HFE', '6-FR-KFE',
            '10-HL-HAA', '11-HL-HFE', '12-HL-KFE',
            '7-HR-HAA', '8-HR-HFE', '9-HR-KFE'
        ]
        
        self.default_joint_angles =    [0.0, 1.0, -1.5, 
                                        0.0, 1.0, -1.5, 
                                        0.0, 1.0, -1.5, 
                                        0.0, 1.0, -1.5]


    def joint_state_callback(self, msg):
        # Check if names, positions, and velocities are all present and match in length
        if not msg.name or not msg.position or not msg.velocity:
            self.get_logger().warn('Received empty or mismatched joint names, positions, or velocities')
            return
        
        if len(msg.name) != len(msg.position) or len(msg.name) != len(msg.velocity):
            self.get_logger().warn('Mismatch in lengths of joint names, positions, or velocities')
            return

        # Log each joint's name, position, and velocity if verbose is True
        if self.verbose:
            for name, position, velocity in zip(msg.name, msg.position, msg.velocity):
                self.get_logger().info(f"Joint '{name}': Position = {position}, Velocity = {velocity}")


        # Store the joint names, positions, and velocities
        self.current_joint_names = msg.name  # Store joint names here
        self.current_positions = msg.position
        self.current_velocities = msg.velocity
        self.joint_positions_received = True


    def publish_setpoints(self):
        if self.current_index < len(self.setpoints):
            msg = Float64MultiArray()
            msg.data = self.setpoints[self.current_index]
            self.setpoint_pub.publish(msg)
            self.current_index += 1
        else:
            self.timer.cancel()


    def interpolate_and_move(self, target_positions, duration=2.0, frequency=50):
        # Wait for the initial joint states to be received
        while not self.joint_positions_received:
            self.get_logger().info('Waiting for current joint positions...')
            rclpy.spin_once(self, timeout_sec=1.0)  # Allow processing of messages for 1 second

        # Once joint positions are received, generate interpolated setpoints
        self.setpoints = generate_spline_interpolation(self.current_positions, target_positions, duration, frequency)
        self.current_index = 0

        # Start publishing the setpoints at regular intervals
        while self.current_index < len(self.setpoints):
            self.publish_setpoints()
            rclpy.spin_once(self, timeout_sec=0.02)  # Wait for the next cycle


    def walk(self, walk_direction, duration_sec):
        # Wait until the robot's joint positions and IMU data are available
        while self.current_positions is None or self.imu_handler.orientation is None:
            self.get_logger().info("Waiting for joint positions and IMU data...")
            rclpy.spin_once(self, timeout_sec=0.5)  # Wait for 0.5 seconds for data to arrive

        # Calculate the end time
        end_time = self.get_clock().now().seconds_nanoseconds()[0] + duration_sec
        obs = torch.zeros(1, 48, dtype=torch.float, device=self.device, requires_grad=False)
        actions = torch.zeros(1, 12, dtype=torch.float, device=self.device, requires_grad=False)

        #load policy
        policy = torch.jit.load('../policy/m1000_policy.pt', map_location=self.device)
        policy.eval()

        while self.get_clock().now().seconds_nanoseconds()[0] < end_time:

            # Step 1: Get the latest IMU and joint data
            imu_data = self.imu_handler.get_imu_data()
            linear_velocity = self.imu_handler.get_linear_velocity()

            obs[0][:3] = torch.FloatTensor([
                linear_velocity['x'] * self.lin_vel_scale,
                linear_velocity['y'] * self.lin_vel_scale,
                linear_velocity['z'] * self.lin_vel_scale
            ])

            obs[0][3:6] = torch.FloatTensor([
                imu_data['angular_velocity']['x'] * self.ang_vel_scale,
                imu_data['angular_velocity']['y'] * self.ang_vel_scale,
                imu_data['angular_velocity']['z'] * self.ang_vel_scale
            ])

            obs[0][6:9] = torch.FloatTensor([
                            imu_data['projected_gravity_vector']['x'], 
                            imu_data['projected_gravity_vector']['z'],
                            -imu_data['projected_gravity_vector']['y']
                            ])

            # Step 2: Get the walk direction into the obs
            obs[0][9:12] = torch.FloatTensor(walk_direction)

            # Step 3: Ensure `current_positions` is a numpy array (if it's not already)
            current_positions = np.array(self.current_positions)  # Convert it to a numpy array, if it's not one
            ordered_positions = map_array_to_target_order(self.policy_mapping, self.current_joint_names, current_positions)
            default_joint_angles_tensor = torch.FloatTensor(self.default_joint_angles)
            ordered_positions_tensor = torch.FloatTensor(ordered_positions)
            obs[0][12:24] = torch.FloatTensor((ordered_positions_tensor - default_joint_angles_tensor) * self.dof_pos_scale)

            # Step 4: Get Velocities
            current_velocities = np.array(self.current_velocities)  # Convert to numpy array if not already
            ordered_velocities = map_array_to_target_order(self.policy_mapping, self.current_joint_names, current_velocities)
            ordered_velocities_tensor = torch.FloatTensor(ordered_velocities)
            obs[0][24:36] = torch.FloatTensor(np.array([ordered_velocities_tensor[i] * self.dof_vel_scale for i in range(12)]))

            obs[0][36:] = torch.FloatTensor(actions)
            

            obsclipped = torch.clip(obs, -3, 3).to(self.device)

            with torch.no_grad():
                actions = policy(obsclipped.detach())


            actions = torch.clip(actions, -3, 3).to(self.device)
            position_output = actions.detach().cpu().numpy()
            
            position_output_scaled = position_output * self.action_scale
            policy_out_positions = position_output_scaled[0][:] + self.default_joint_angles

            policy_out_positions_ordered = map_array_to_target_order(self.current_joint_names, self.policy_mapping, policy_out_positions)

            for _ in range(4):  # Loop runs 4 times
                policy_out_positions_ordered = [float(x) for x in policy_out_positions_ordered]
                msg = Float64MultiArray()
                msg.data = policy_out_positions_ordered  # Set target positions as message data
                self.setpoint_pub.publish(msg)
            
            
            rclpy.spin_once(self, timeout_sec=0.02)
        

        self.get_logger().info("Walking finished.")
  




