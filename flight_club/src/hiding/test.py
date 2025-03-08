#!/usr/bin/env python3
import argparse
import xml.etree.ElementTree as ET
import numpy as np
import trimesh
import trimesh.transformations as tra

def parse_sdf_tree(root):
    """
    Parse the SDF XML tree and extract cylinder models and the soldier pose.
    """
    # Look inside the <world> element if present
    world = root.find('world')
    if world is None:
        world = root

    models = []
    soldier_pose = None

    for model in world.findall('model'):
        name = model.get('name')
        pose_elem = model.find('pose')
        if pose_elem is not None:
            pose_vals = list(map(float, pose_elem.text.strip().split()))
            if len(pose_vals) < 6:
                pose_vals += [0.0] * (6 - len(pose_vals))
            pose = np.array(pose_vals)
        else:
            pose = np.zeros(6)

        if name == 'soldier':
            soldier_pose = pose
            continue

        # Check for cylinder geometry in collision
        cyl_elem = model.find('.//cylinder')
        if cyl_elem is None:
            continue  # Skip models with no cylinder

        radius = float(cyl_elem.find('radius').text)
        length = float(cyl_elem.find('length').text)
        models.append({
            'name': name,
            'type': 'cylinder',
            'pose': pose,
            'radius': radius,
            'length': length
        })
    return models, soldier_pose

def create_cylinder_mesh(radius, length):
    """
    Create a cylinder mesh along the z-axis.
    trimesh.creation.cylinder creates a mesh centered on the origin.
    """
    mesh = trimesh.creation.cylinder(radius=radius, height=length, sections=32)
    mesh.visual.face_colors = [0, 0, 255, 255]  # blue
    return mesh

def get_transform_from_pose(pose):
    """
    Build a transformation matrix from a pose.
    Pose is given as [x, y, z, roll, pitch, yaw].
    """
    x, y, z, roll, pitch, yaw = pose
    transform = tra.euler_matrix(roll, pitch, yaw, 'sxyz')
    transform[0:3, 3] = [x, y, z]
    return transform

def compute_occlusions(camera_pos, sample_points, combined_mesh):
    """
    Vectorize ray casting to determine which sample points are occluded.
    """
    # Compute ray directions and distances from camera
    directions = sample_points - camera_pos
    distances = np.linalg.norm(directions, axis=1)
    # Avoid division by zero for rays originating at the camera
    nonzero = distances > 1e-6
    directions_norm = np.zeros_like(directions)
    directions_norm[nonzero] = directions[nonzero] / distances[nonzero][:, None]
    directions_norm[~nonzero] = np.array([1, 0, 0])  # dummy direction

    # Repeat camera position for each ray
    origins = np.tile(camera_pos, (len(sample_points), 1))
    # Use trimesh's batch ray intersection: returns the distance to the first intersection (or NaN if none)
    hit_distances = combined_mesh.ray.intersects_first(
        ray_origins=origins, ray_directions=directions_norm)

    # A point is visible if there's no hit (NaN) or if the hit is farther than the sample point distance
    visible_mask = np.isnan(hit_distances) | (hit_distances >= distances)
    return visible_mask

def main():
    parser = argparse.ArgumentParser(
        description='Compute occluded regions and visualize the scene from the soldier/camera view using an SDF file.')
    parser.add_argument('sdf_file', type=str, help='Path to the SDF file')
    args = parser.parse_args()

    # Parse the SDF file once.
    tree = ET.parse(args.sdf_file)
    root = tree.getroot()
    models, soldier_pose = parse_sdf_tree(root)

    if not models:
        print("No cylinder obstacles found in the SDF file.")
        return

    print("Parsed cylinder obstacles:")
    for m in models:
        print(f"  {m['name']}: radius={m['radius']}, length={m['length']}, pose={m['pose']}")

    # Determine the camera (soldier) position.
    if soldier_pose is not None:
        camera_pos = soldier_pose[0:3]
        camera_pos[2] += 1.5  # Adjust camera height
    else:
        camera_pos = np.array([0, 0, 5])
    print(f"Camera (soldier) position: {camera_pos}")

    # Create cylinder meshes with transforms.
    meshes = []
    obstacle_centers = []
    for model in models:
        if model['type'] == 'cylinder':
            mesh = create_cylinder_mesh(model['radius'], model['length'])
            transform = get_transform_from_pose(model['pose'])
            mesh.apply_transform(transform)
            meshes.append(mesh)
            obstacle_centers.append(model['pose'][0:3])

    # Compute the average obstacle center.
    obstacle_centers = np.array(obstacle_centers)
    avg_center = np.mean(obstacle_centers, axis=0)
    print(f"Average obstacle center: {avg_center}")

    # Compute the camera's view direction toward the obstacle center.
    view_dir = avg_center - camera_pos
    norm = np.linalg.norm(view_dir)
    view_dir = view_dir / norm if norm > 1e-6 else np.array([1, 0, 0])
    print(f"Camera view direction: {view_dir}")

    # Combine all obstacle meshes into a single mesh for efficient ray queries.
    combined_mesh = trimesh.util.concatenate(meshes)

    # Define the region of interest.
    grid_x = np.linspace(-40, 40, 80)
    grid_y = np.linspace(-40, 40, 80)
    grid_z = np.linspace(0, 15, 30)
    sample_points = np.array(np.meshgrid(grid_x, grid_y, grid_z)).T.reshape(-1, 3)

    # Determine occluded and visible sample points using vectorized ray casting.
    visible_mask = compute_occlusions(camera_pos, sample_points, combined_mesh)
    visible_points = sample_points[visible_mask]
    occluded_points = sample_points[~visible_mask]

    print(f"Total sample points: {len(sample_points)}")
    print(f"Visible points: {len(visible_points)}")
    print(f"Occluded points: {len(occluded_points)}")

    # --- Visualization using trimesh Scene ---
    # Create point clouds (green for visible, red for occluded).
    visible_pc = trimesh.points.PointCloud(
        visible_points,
        colors=np.tile([0, 255, 0, 255], (len(visible_points), 1))
    )
    occluded_pc = trimesh.points.PointCloud(
        occluded_points,
        colors=np.tile([255, 0, 0, 255], (len(occluded_points), 1))
    )

    scene = trimesh.Scene()
    # Add obstacle meshes.
    scene.add_geometry(combined_mesh)
    # Add point clouds.
    scene.add_geometry(visible_pc)
    scene.add_geometry(occluded_pc)

    # Mark the camera position with a small sphere.
    camera_sphere = trimesh.creation.icosphere(radius=0.5)
    camera_sphere.apply_translation(camera_pos)
    camera_sphere.visual.face_colors = [255, 255, 0, 255]  # yellow
    scene.add_geometry(camera_sphere)

    # Create an arrow indicating the view direction.
    arrow_length = 5.0
    arrow_points = np.vstack([camera_pos, camera_pos + view_dir * arrow_length])
    arrow_path = trimesh.load_path(arrow_points)
    arrow_path.colors = np.tile([255, 255, 0, 255], (len(arrow_path.entities), 1))
    scene.add_geometry(arrow_path)

    print("Displaying scene. Close the window to exit.")
    scene.show()

if __name__ == '__main__':
    main()
