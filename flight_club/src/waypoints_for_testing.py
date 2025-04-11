import numpy as np

# Define three waypoints (x, y, z)
waypoints = np.array([
    [0.0, 0.0, 1.0],  # Start point
    [1.0, 1.0, 1.5],  # Mid point
    [2.0, -2.0, 1.0],  
    [3.0, 0.0, 1.0],  # End point
])

# Save to .npy file
np.save('waypoints.npy', waypoints)