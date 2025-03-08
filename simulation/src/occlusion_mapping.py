import numpy as np

# Create a voxel grid (resolution 0.1m)
grid_size = (100, 100, 50)  #  world size
voxel_grid = np.zeros(grid_size)  # 0 = unknown, 1 = visible, -1 = occluded

seeker_position = np.array([1, 2, 3])  # Seeker location
hit_points = np.load("hit_points.npy")  # Load raycast results

def mark_occlusion(ray_hit):
    """Extend occlusion from hit point along the ray."""
    direction = ray_hit - seeker_position
    direction /= np.linalg.norm(direction)

    for i in range(100):  # 10m extension at 0.1m steps
        occluded_voxel = np.round(ray_hit + direction * (i * 0.1)).astype(int)
        if (0 <= occluded_voxel[0] < grid_size[0] and
            0 <= occluded_voxel[1] < grid_size[1] and
            0 <= occluded_voxel[2] < grid_size[2]):
            voxel_grid[tuple(occluded_voxel)] = -1  # Mark as occluded

for hit in hit_points:
    mark_occlusion(hit)

np.save("occluded_voxels.npy", voxel_grid)  # Save occlusion data
