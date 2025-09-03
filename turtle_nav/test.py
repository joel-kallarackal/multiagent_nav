import numpy as np
import matplotlib.pyplot as plt

def generate_sinusoidal_between_points(p1, p2, num_points=100, amplitude=1, frequency=1):
    """
    Generates sinusoidal waypoints along the line between two points.

    Parameters:
        p1 (tuple): (x1, y1) start point
        p2 (tuple): (x2, y2) end point
        num_points (int): Number of waypoints
        amplitude (float): Amplitude of the sine wave
        frequency (float): Number of sine cycles along the line

    Returns:
        waypoints (list of tuples): List of (x, y) waypoints
    """
    x1, y1 = p1
    x2, y2 = p2

    # Line vector and length
    line_vec = np.array([x2 - x1, y2 - y1])
    line_length = np.linalg.norm(line_vec)
    line_unit = line_vec / line_length

    # Perpendicular unit vector
    perp_unit = np.array([-line_unit[1], line_unit[0]])

    # Sample distances along the line
    t = np.linspace(0, 1, num_points)
    base_points = np.array([x1, y1]) + np.outer(t, line_vec)

    # Apply sinusoidal offset along perpendicular direction
    offsets = amplitude * np.sin(2 * np.pi * frequency * t)
    waypoints = base_points + np.outer(offsets, perp_unit)

    return waypoints.tolist()

# Example usage

p1 = (-2.1270616951528085, -2.7723084523012593)
# p2 = (0.8108174780823566, -2.232505364315845)
p2 = (2.745143914068928, -2.0507380772115447)

waypoints = generate_sinusoidal_between_points(p1, p2, num_points=25, amplitude=0.5, frequency=2)


# Visualization
waypoints = np.array(waypoints)
plt.figure(figsize=(8, 5))
plt.plot(waypoints[:, 0], waypoints[:, 1], 'b-', label="Sinusoidal Path")
plt.scatter([p1[0], p2[0]], [p1[1], p2[1]], color="red", label="Endpoints")
plt.xlabel("X")
plt.ylabel("Y")
plt.title("Sinusoidal Waypoints Between Two Points")
plt.legend()
plt.axis("equal")
plt.grid()
plt.show()
