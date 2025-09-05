from scipy.io import loadmat
import numpy as np

# Load your .mat file
data = loadmat("/home/joel/biorobotics_lab/turtlebots_ws/src/turtle_nav/data/traj.mat")

# Check what variables are inside the file
print(data.keys())

trajectory = data["currTraj"]

scale_x = 0.2
scale_y = 0.2

center = trajectory.mean(axis=0)
shift_from_center = [14.167286794795572, 27.445176474273982]

scale_shift_trajectory = (trajectory - center) * np.array([scale_x, scale_y]) + shift_from_center

sampled_trajectory = scale_shift_trajectory[::10]
print(len(sampled_trajectory))

import matplotlib.pyplot as plt

# If 2D (x, y)
plt.figure(figsize=(6,6))
plt.plot(sampled_trajectory[:,0], sampled_trajectory[:,1], marker='o')
plt.title("Trajectory")
plt.xlabel("X")
plt.ylabel("Y")
plt.axis("equal")
plt.show()
