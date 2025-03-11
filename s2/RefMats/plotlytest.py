import numpy as np
import matplotlib.pyplot as plt

# Define vectors
origin = np.zeros((3, 3))  # One origin for each vector
vectors = np.array([[2, 3, 0], [-1, 2, 2], [3, -2, 3]])

# Create figure and 3D axis
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# Plot vectors
ax.quiver(origin[:, 0], origin[:, 1], origin[:, 2],  # Origin points
          vectors[:, 0], vectors[:, 1], vectors[:, 2],  # Vector components
          color=['r', 'g', 'b'])

# Set axis labels
ax.set_xlim([-3, 4])
ax.set_ylim([-3, 4])
ax.set_zlim([-3, 4])
ax.set_xlabel('X-axis')
ax.set_ylabel('Y-axis')
ax.set_zlabel('Z-axis')
ax.set_title('3D Vector Plot')

plt.show()
