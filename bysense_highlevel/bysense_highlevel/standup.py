from high_level_controller import HighLevelController
import rclpy

def main():
    rclpy.init()

    # Define standup setpoints
    # Axis ID              2,    3,   4,   1,   8,    9,    6,   7,  10,   5,  11,   12
    standup_setpoints = [0.9, -1.7, 0.0, 0.0, 1.0, -1.7, -1.7, 0.0, 0.0, 0.9, 1.0, -1.7]

    controller = HighLevelController()
    controller.interpolate_and_move(standup_setpoints, duration=10.0, frequency=50)

    rclpy.shutdown()

if __name__ == '__main__':
    main()
