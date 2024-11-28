import numpy as np

def ik_solver(target_position, link_lengths):
    """
    Compute the inverse kinematics for a single leg.

    Parameters:
        target_position (list or np.array): [x, y, z] coordinates of the foot in the leg's local frame.
        link_lengths (list): [l1, l2] where l1 is the length of the upper leg (hip-to-knee),
                             and l2 is the length of the lower leg (knee-to-foot).
                             
    Returns:
        tuple: (hip_angle, thigh_angle, knee_angle) in radians.
    """
    x, y, z = target_position
    l1, l2 = link_lengths
    x = -x
    # Calculate the distance from the hip joint to the target position in the xz-plane (ignoring y for now)
    planar_distance = np.sqrt(x**2 + z**2)

    # Check reachability: if the target is out of reach (greater than the sum of link lengths)
    if planar_distance > (l1 + l2):
        raise ValueError("Target position is out of reach.")

    # Calculate the hip angle around the x-axis (HAA)
    HAA = np.arctan2(y, -1*z)  # This gives us the rotation needed to align with the xz-plane

    # Now, calculate the knee angle (KFE) using the law of cosines on the xz-plane
    cos_knee_angle = (planar_distance**2 + x**2 - l1**2 - l2**2) / (2 * l1 * l2)
    KFE = -1 * np.arccos(np.clip(cos_knee_angle, -1.0, 1.0))  # Clip to avoid domain errors in arccos

    # Solve for the thigh angle (HFE)
    angle_a = np.arctan2(x, planar_distance)  # Angle from hip to knee in the xz-plane
    angle_b = np.arccos(np.clip((l1**2 + planar_distance**2 + x**2 - l2**2) / (2 * l1 * np.sqrt(planar_distance**2 + x**2)), -1.0, 1.0))
    HFE = angle_a + angle_b  # Total angle of the thigh

    return HAA, HFE, KFE