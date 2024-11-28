import numpy as np
from sensor_msgs.msg import Imu
from gazebo_msgs.msg import ModelStates
from scipy.spatial.transform import Rotation as R
import torch

class ImuHandler:

    def __init__(self, node):
        # Initialize attributes for IMU data
        self.node = node
        self.orientation = None
        self.angular_velocity = None
        self.velocity = np.array([0.0, 0.0, 0.0])  # Initialize velocity
        self.robot_name = "bysense"  # The robot name
        self.last_time = None
        self.verbose = False
        self.device = 'cpu'
        self.gravity_vec = np.array([0, 0, -1])

        # Subscriber to receive IMU data from the correct topic
        self.imu_subscriber = self.node.create_subscription(Imu, '/imu_plugin/out', self.imu_callback, 10)
        self.subscription = self.node.create_subscription(ModelStates, '/ros2_grasp/model_states', self.model_states_callback, 10)

    def imu_callback(self, msg):
        # Capture and store IMU data
        self.orientation = msg.orientation
        self.angular_velocity = msg.angular_velocity

        q = [self.orientation.x, self.orientation.y, self.orientation.z, self.orientation.w]
        zyx = self.quat_to_zyx(q)
        rotation_matrix = self.get_rot_matrix_from_zyx_euler_angles(zyx)
        inverse_rot = np.linalg.inv(rotation_matrix)
        gravity_vector = np.array([0, 0, -1])  # Gravity vector in world frame
        projected_gravity_vector = inverse_rot @ gravity_vector  # Matrix-vector multiplication

        # Calculate the gravity vector from the IMU orientation data
        # projected_gravity_vector = self.quat_rotate_inverse(self.orientation, self.gravity_vec)

        #following line for debug purposes
        if self.verbose:
            self.node.get_logger().info(f"Calculated Gravity Vector: {projected_gravity_vector}")

    def get_imu_data(self):
        if self.orientation:

            q = [self.orientation.x, self.orientation.y, self.orientation.z, self.orientation.w]
            zyx = self.quat_to_zyx(q)
            rotation_matrix = self.get_rot_matrix_from_zyx_euler_angles(zyx)
            inverse_rot = np.linalg.inv(rotation_matrix)
            gravity_vector = np.array([0, 0, -1])  # Gravity vector in world frame
            projected_gravity_vector = inverse_rot @ gravity_vector  # Matrix-vector multiplication
            
            pj_g_x = projected_gravity_vector[0]
            pj_g_y = projected_gravity_vector[1]
            pj_g_z = projected_gravity_vector[2]

        else:
            pj_g_x, pj_g_y, pj_g_z = None, None, None

        return {
            "projected_gravity_vector": {
                "x": pj_g_x,
                "y": pj_g_y,
                "z": pj_g_z
            },
            "angular_velocity": {
                "x": self.angular_velocity.x if self.angular_velocity else None,
                "y": self.angular_velocity.y if self.angular_velocity else None,
                "z": self.angular_velocity.z if self.angular_velocity else None
            }
        }


    def quat_to_zyx(self, q):
        # Normalize quaternion if needed
        q = q / np.linalg.norm(q)
        
        # Extract quaternion components
        w, x, y, z = q

        # Compute the ZYX Euler angles
        as_ = min(-2. * (x * z - w * y), 0.99999)
        zyx = np.zeros(3)

        zyx[0] = np.arctan2(2 * (x * y + w * z), w**2 + x**2 - y**2 - z**2)
        zyx[1] = np.arcsin(as_)
        zyx[2] = np.arctan2(2 * (y * z + w * x), w**2 - x**2 - y**2 + z**2)

        return zyx


    def get_rot_matrix_from_zyx_euler_angles(self, euler_angles):
        z, y, x = euler_angles

        c1 = np.cos(z)
        c2 = np.cos(y)
        c3 = np.cos(x)
        s1 = np.sin(z)
        s2 = np.sin(y)
        s3 = np.sin(x)

        s2s3 = s2 * s3
        s2c3 = s2 * c3

        # Construct rotation matrix
        rotation_matrix = np.array([
            [c1 * c2, c1 * s2s3 - s1 * c3, c1 * s2c3 + s1 * s3],
            [s1 * c2, s1 * s2s3 + c1 * c3, s1 * s2c3 - c1 * s3],
            [-s2, c2 * s3, c2 * c3]
        ])

        return rotation_matrix



    def quat_rotate_inverse(self, q, v):
        v = torch.tensor([v] ,dtype=torch.float, device=self.device, requires_grad=False)
        q = torch.tensor([[q.x, q.y, q.z, q.w]] ,dtype=torch.float, device=self.device, requires_grad=False)
        #q = torch.tensor([[0.0021, 0.0111, -0.0487, 0.9987]] ,dtype=torch.float, device=self.device, requires_grad=False)
        print(q)
        shape = q.shape
        q_w = q[:, -1]
        q_vec = q[:, :3]
        a = v * (2.0 * q_w ** 2 - 1.0).unsqueeze(-1)
        b = torch.cross(q_vec, v, dim=-1) * q_w.unsqueeze(-1) * 2.0
        c = q_vec * \
            torch.bmm(q_vec.view(shape[0], 1, 3), v.view(
                shape[0], 3, 1)).squeeze(-1) * 2.0
        return a - b + c


    def model_states_callback(self, msg):
        # Loop through all model names in the msg to find the 'bysense' robot
        for i, name in enumerate(msg.name):
            if name.startswith(self.robot_name):
                # Extract the linear velocity for the correct robot
                self.velocity = np.array([
                    msg.twist[i].linear.x,
                    msg.twist[i].linear.y,
                    msg.twist[i].linear.z
                ])
                if self.verbose:
                    self.node.get_logger().info(f"Updated velocity for {self.robot_name}: {self.velocity}")
                break  # Exit once the correct robot is found


    def get_linear_velocity(self):
        return {
            "x": self.velocity[0],
            "y": self.velocity[1],
            "z": self.velocity[2]
        }