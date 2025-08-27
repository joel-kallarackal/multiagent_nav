WAYPOINTS = [
            (-1.1888759073286665,1.6263000837780282),
            (-1.7573231267877698, 1.1763000837780282),
            (-2.0487147682992837, 0.6371434216059553),
            (-2.523020336341442, 1.5389058964368052),
            (-2.761993990854728, 0.7684627066245723),
            
            (-3.0333638445221593, 1.6243871917065487),
            (-3.575691666712014, 0.8540813600863703)
        ]
import numpy as np
from scipy.interpolate import CubicSpline

waypoints = np.array(WAYPOINTS[::-1])
k=0
x = []
y = []

for i in range(len(waypoints)-1):
    # Generate cubic spline
    spline = CubicSpline(waypoints[i:i+3, 0], waypoints[i:i+3, 1])

    # Get interpolated values
    xs = np.linspace(min(waypoints[i:i+3, 0]), max(waypoints[i:i+3, 0]), 20)
    ys = spline(xs)

    x.extend(xs)
    y.extend(ys)




import matplotlib.pyplot
import matplotlib.pyplot as plt
plt.figure(figsize=(8, 6))
plt.plot(x, y, label='Cubic Spline', color='b')  # Plot spline
plt.scatter(waypoints[:, 0], waypoints[:, 1], color='r', zorder=5, label='Waypoints')  # Plot waypoints
plt.title('Cubic Spline Interpolation')
plt.xlabel('X')
plt.ylabel('Y')
plt.legend()
plt.grid(True)
plt.show()
