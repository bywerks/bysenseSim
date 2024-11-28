import numpy as np
from scipy.interpolate import CubicSpline

def map_array_to_target_order(target_map, source_map, array_2_map):
    """
    Maps values from array_2_map (current_positions or current_velocities) based on the order defined
    in source_map to the order defined in target_map.

    :param target_map: List of target order (policy_mapping).
    :param source_map: List of current joint names (current_joint_names).
    :param array_2_map: The array (positions or velocities) that needs to be reordered.
    :return: A new array ordered according to target_map.
    """
    # Create a dictionary mapping each source joint name to its index in the target order
    name_to_target_index = {name: idx for idx, name in enumerate(target_map)}

    # Create a new array with the same length as target_map
    ordered_array = [0] * len(target_map)

    # Loop through source_map and map values from array_2_map to target order
    for idx, name in enumerate(source_map):
        target_index = name_to_target_index.get(name)
        if target_index is not None:
            ordered_array[target_index] = array_2_map[idx]

    return ordered_array


def generate_spline_interpolation(current_positions, target_positions, duration=1.0, frequency=50):
    """
    Generates a spline interpolation between current joint positions and target positions.

    :param current_positions: List of current joint positions
    :param target_positions: List of target joint positions
    :param duration: Duration of the interpolation (default 1.0)
    :param frequency: Frequency of interpolation (default 50 Hz)
    :return: Interpolated setpoints as a 2D list
    """
    if current_positions is None:
        raise ValueError("Current joint positions are not provided.")

    # Generate time array from 0 to duration with the given frequency
    t_points = np.linspace(0, duration, int(duration * frequency))

    # Interpolate each joint's position using cubic splines
    interpolated_setpoints = []
    for i in range(len(current_positions)):
        spline = CubicSpline([0, duration], [current_positions[i], target_positions[i]])
        interpolated_values = spline(t_points)
        interpolated_setpoints.append(interpolated_values)

    return np.array(interpolated_setpoints).T.tolist()

