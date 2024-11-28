from high_level_controller import HighLevelController
import rclpy
from inverse_kin import ik_solver

def main():
    rclpy.init()
    target_positions = [
        [-0.02, 0.01, -0.30],
        [-0.02, -0.01, -0.30],
        [-0.05, 0.01, -0.30],
        [-0.05, -0.01, -0.30]
    ]

    link_lengths = [ 0.25, 0.28]
    Joint_angles = []

    # Iterate through each target position
    for target_position in target_positions:
        HAA, HFE, KFE = ik_solver(target_position, link_lengths)
        Joint_angles.append(HAA)
        Joint_angles.append(HFE)
        Joint_angles.append(KFE)


    # Define sitdown setpoints
    #ID order 
    #                    2,    3,   4,   1,   8,    9,    6,   7,   10,  5,   11,  12
    #                   FL-HFE,FL-KFE,FR-HAA,FL-HAA,HR-HFE,HR-KFE,FR-KFE,HR-HAA,HL-HAA,FR-HFE,HL-HFE,HL-KFE
    #         Joint_angles : FL-HAA,FL-HFE,FL-KFE, FR-HAA,FR-HFE,FR-KFE, HL-HAA,HL-HFE,HL-KFE, HR-HAA,HR-HFE,HR-KFE
    sitdown_setpoints = [Joint_angles[1],Joint_angles[2],Joint_angles[3],Joint_angles[0],Joint_angles[10],Joint_angles[11],Joint_angles[5],Joint_angles[9],Joint_angles[6],Joint_angles[4],Joint_angles[7],Joint_angles[8]]

    controller = HighLevelController()
    controller.interpolate_and_move(sitdown_setpoints, duration=5.0, frequency=50)

    rclpy.shutdown()

if __name__ == '__main__':
    main()