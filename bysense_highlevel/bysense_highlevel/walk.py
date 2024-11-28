import gaitplanner
from high_level_controller import HighLevelController
import rclpy
from inverse_kin import ik_solver
import time

def generate_trajectory(direction, step_length, step_height, num_points, initial_positions):
    """
    Generate trajectories for all legs based on initial positions.
    """
    return {
        leg: gaitplanner.generate_step_trajectory(direction, step_length, step_height, num_points, pos)
        for leg, pos in initial_positions.items()
    }

def compute_joint_angles(target_positions, link_lengths):
    """
    Compute joint angles for all target positions.
    """
    joint_angles = []
    for target_position in target_positions:
        HAA, HFE, KFE = ik_solver(target_position, link_lengths)
        joint_angles.extend([HAA, HFE, KFE])
    return joint_angles

def execute_step(controller, trajectory, moving_leg, initial_positions, link_lengths):
    """
    Execute a step for a single leg while keeping others stationary.
    """
    for pos in trajectory:
        # Set target positions for all legs
        target_positions = [
            pos if leg == moving_leg else initial_positions[leg]
            for leg in ["FL", "FR", "HL", "HR"]
        ]

        # Compute joint angles
        joint_angles = compute_joint_angles(target_positions, link_lengths)

        # Map joint angles to the desired order
        sitdown_setpoints = [
            joint_angles[1], joint_angles[2], joint_angles[3], joint_angles[0],
            joint_angles[10], joint_angles[11], joint_angles[5], joint_angles[9],
            joint_angles[6], joint_angles[4], joint_angles[7], joint_angles[8]
        ]

        # Command the robot to move
        controller.interpolate_and_move(sitdown_setpoints, duration=0.05, frequency=50)

def main():
    rclpy.init()
    controller = HighLevelController()

    # Define common parameters
    direction = [1.0, 0.0, 0.0]  # Direction of the step
    step_length = 0.1  # Step length (meters)
    step_height = 0.15  # Maximum step height (meters)
    num_points = 20  # Number of points in the trajectory
    link_lengths = [0.25, 0.28]

    # Initial positions of legs
    initial_positions = {
        "FL": [-0.02, 0.01, -0.30],
        "FR": [-0.02, -0.01, -0.30],
        "HL": [-0.05, 0.01, -0.30],
        "HR": [-0.05, -0.01, -0.30],
    }

    start_time = time.time()
    while time.time() - start_time < 10:

        # Generate trajectories for all legs
        trajectories = generate_trajectory(direction, step_length, step_height, num_points, initial_positions)

        # Execute steps for each leg
        for leg in ["FL", "HR", "FR", "HL"]:
            execute_step(controller, trajectories[leg], leg, initial_positions, link_lengths)

    rclpy.shutdown()

if __name__ == '__main__':
    main()
