from high_level_controller import HighLevelController
import rclpy

def main():
    rclpy.init()

    # Define sitdown setpoints
    # Axis ID              2,    3,   4,   1,   8,    9,    6,   7,  10,   5,  11,   12
    sitdown_setpoints = [1.5, -2.9, 0.0, 0.0, 1.5, -2.9, -2.9, 0.0, 0.0, 1.5, 1.5, -2.9]

    controller = HighLevelController()
    controller.interpolate_and_move(sitdown_setpoints, duration=5.0, frequency=50)

    rclpy.shutdown()

if __name__ == '__main__':
    main()
