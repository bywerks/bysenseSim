from high_level_controller import HighLevelController
import rclpy

def main():
    rclpy.init()
    controller = HighLevelController()

    # standup
    # Axis ID              2,    3,   4,   1,   8,    9,    6,   7,  10,   5,  11,   12
    standup_setpoints = [0.9, -1.7, 0.0, 0.0, 1.0, -1.7, -1.7, 0.0, 0.0, 0.9, 1.0, -1.7]
    controller.interpolate_and_move(standup_setpoints, duration=5.0, frequency=50)

    # walk
    direction = (0.0, 0.0, 0.0)
    duration_sec = 5.0
    controller.walk(direction,duration_sec)

    # sitdown
    sitdown_setpoints = [1.5, -2.9, 0.0, 0.0, 1.5, -2.9, -2.9, 0.0, 0.0, 1.5, 1.5, -2.9]
    controller.interpolate_and_move(sitdown_setpoints, duration=5.0, frequency=50)

    rclpy.shutdown()

if __name__ == '__main__':
    main()
