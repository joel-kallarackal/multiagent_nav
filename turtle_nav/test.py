import random
import numpy as np

b = 1.01
x_list1 = np.array([random.uniform((b-1)/(b*np.pi-2), b/(b*np.pi-2)) for i in range(5)])
x_list2 = np.array([random.uniform((b-1)/(b*np.pi-2), b/(b*np.pi-2)) for i in range(5)])

theta1 = np.arccos(b-(b*np.pi-2)*x_list1)
theta2 = -np.arccos(b-(b*np.pi-2)*x_list2)
theta = np.concatenate((theta1, theta2))

chaf_pos_vec = np.array([0, 0])
ev_pos_vec = np.array([0, 1])

a_hat = (ev_pos_vec-chaf_pos_vec)/np.linalg.norm(ev_pos_vec-chaf_pos_vec)
b1 = np.sqrt(1/(1+a_hat[0]**2/a_hat[1]**2))
b2 = -b1*a_hat[0]/a_hat[1]
b_hat = np.array([b1, b2])

r = 1
example_waypoints = ev_pos_vec + r*np.cos(np.pi/2 - theta)[:, None]*b_hat + r*np.sin(np.pi/2 - theta)[:, None]*a_hat

x = np.random.uniform(-5, 5, 10)
y = np.random.uniform(0, 5, 10)

free_cells = np.column_stack((x, y))

scores = []
for waypoint in free_cells:
    for example_waypoint in example_waypoints:
        score = 1*np.linalg.norm(waypoint-ev_pos_vec) + 1/(np.dot(example_waypoint/np.linalg.norm(example_waypoint), (waypoint-ev_pos_vec)/np.linalg.norm(waypoint-ev_pos_vec)))
        scores.append((waypoint,score))

scores.sort(key=lambda s: s[1], reverse=True)
best_waypoint = scores[0][0]
print(best_waypoint)
import matplotlib.pyplot as plt

x = [p[0] for p in example_waypoints]
y = [p[1] for p in example_waypoints]

# Plot points
plt.figure(figsize=(6, 6))
plt.scatter(x, y, color="blue", label="Points")
plt.scatter(best_waypoint[0], best_waypoint[1], color="red", label="Points")

# Labels and grid
plt.xlabel("X")
plt.ylabel("Y")
plt.title("Scatter Plot of Points")
plt.legend()
plt.grid(True)
plt.axis("equal")  # keep aspect ratio equal

plt.show()
