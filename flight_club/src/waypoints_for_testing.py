import numpy as np

# Define three waypoints (x, y, z)
waypoints = np.array([
    [1, 0.1, 1.1],  # Start point
    [3.0, -0.1, 1],  # Mid point
    [6.0, 0.1, 1.2],  
    [8.0, 0.0, 1.0],  # End point
])

# Save to .npy file
np.save('waypoints.npy', waypoints)
