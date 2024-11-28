import numpy as np

def bezier_closed_curve(t, p0, p1, p2, p3, p4):
    """
    Quadratic Bézier curve for a closed loop.
    Parameters:
        t: Float (0 <= t <= 1), normalized time in the step cycle.
        p0, p1, p2, p3, p4: Control points for the closed Bézier curve.
    Returns:
        Interpolated position [x, y, z].
    """
    if t <= 0.5:
        # First segment (forward trajectory)
        t_segment = t / 0.5  # Map t to [0, 1] for the first segment
        return (1 - t_segment)**2 * np.array(p0) + 2 * (1 - t_segment) * t_segment * np.array(p1) + t_segment**2 * np.array(p2)
    else:
        # Second segment (return trajectory)
        t_segment = (t - 0.5) / 0.5  # Map t to [0, 1] for the second segment
        return (1 - t_segment)**2 * np.array(p2) + 2 * (1 - t_segment) * t_segment * np.array(p3) + t_segment**2 * np.array(p4)

def generate_step_trajectory(direction, step_length, step_height, num_points, initial_position):
    """
    Generate a closed Bézier curve for a single step trajectory.
    
    Parameters:
        direction: [dx, dy, dz], direction vector (normalized).
        step_length: Total step length (scalar).
        step_height: Maximum height of the foot during the swing phase.
        num_points: Number of trajectory points.
        initial_position: [x, y, z], starting position of the foot.
    
    Returns:
        trajectory: List of positions [x, y, z] along the step.
    """
    # Normalize the direction vector
    direction = np.array(direction)
    norm = np.linalg.norm(direction)
    if norm == 0:
        raise ValueError("Direction vector cannot be zero.")
    direction /= norm
    
    # Scale the direction vector by the step length
    step_vector = direction * step_length
    
    # Define control points for the closed curve
    p0 = np.array(initial_position)  # Start position
    p1 = p0 + np.array([step_vector[0] / 2, step_vector[1] / 2, step_height])  # Apex of forward trajectory
    p2 = p0 + step_vector  # End position (new ground contact point)
    p3 = p0 + np.array([step_vector[0] / 2, step_vector[1] / 2, -0.02 * step_height])  # Apex of return trajectory
    p4 = p0  # Return to start position
    
    # Generate closed trajectory
    trajectory = []
    for i in range(num_points):
        t = i / (num_points - 1)  # Normalize time step (0 to 1)
        position = bezier_closed_curve(t, p0, p1, p2, p3, p4)
        trajectory.append(position.tolist())
    
    return trajectory
