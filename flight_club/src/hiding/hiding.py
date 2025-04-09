#!/usr/bin/env python3
from __future__ import annotations

from pathlib import Path
from dataclasses import dataclass
import numpy as np
import xml.etree.ElementTree as ET
import logging
import trimesh
import trimesh.transformations as tra
import matplotlib.pyplot as plt

from queue import PriorityQueue
from dataclasses import dataclass

from scipy.spatial import KDTree
import multiprocessing as mp


GLOBAL_MESH = None

INITIAL_POSITION = np.array([10, -15.0, 1])
SEEKER_OFFSET = np.array([0.0, 0.0, 1.5, 0.0, 0.0, 0.0])
OBSTACLE_SAFETY_MARGIN = 2.0
MIN_EDGE_DISTANCE = 0.5 # play with this to encourage bigger jumps

MAP_X_MIN = -40
MAP_X_MAX = 40
MAP_Y_MIN = -40
MAP_Y_MAX = 40
MAP_Z_MIN = 0
MAP_Z_MAX = 15

MAX_VELOCITY = 1.0
MAX_TURN_DURATION = 7.0

DOWNSAMPLING_FACTOR = 2.0

NUM_SAMPLES_HORIZONTAL = 60
NUM_SAMPLES_VERTICAL = 20

# NUM_SAMPLES_HORIZONTAL = 50
# NUM_SAMPLES_VERTICAL = 10


logging.basicConfig(
    format='%(asctime)s - %(levelname)s - %(message)s',
    level=logging.DEBUG
)

@dataclass
class CylinderModel:
    name: str
    type: str
    pose: np.ndarray
    radius: float
    length: float

@dataclass
class World:
    models: list[CylinderModel]
    seeker_pose: np.ndarray


def parse_sdf_map(map_file: Path) -> World:
    """
    Parse the SDF XML tree and extract cylinder models and the soldier pose.
    """
    tree = ET.parse(map_file)
    root = tree.getroot()

    world = root.find('world')
    if world is None:
        world = root

    models = []
    seeker_pose = None

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

        if name == 'soldier': # TODO: change to seeker in map file
            seeker_pose = pose + SEEKER_OFFSET
            continue

        # Check for cylinder geometry in collision
        cyl_elem = model.find('.//cylinder')
        if cyl_elem is None:
            continue  # Skip models with no cylinder

        radius = float(cyl_elem.find('radius').text) 
        length = float(cyl_elem.find('length').text) 
        models.append(CylinderModel(name, 'cylinder', pose, radius, length))

    return World(models, seeker_pose)
      
def create_cylinder_mesh(radius: float, length: float) -> trimesh.Trimesh:
    """
    Create a cylinder mesh along the z-axis.
    trimesh.creation.cylinder creates a mesh centered on the origin.
    """
    mesh = trimesh.creation.cylinder(radius=radius, height=length, sections=32)
    mesh.visual.face_colors = [0, 0, 255, 255]  # blue
    return mesh


def get_transform_from_pose(pose: np.ndarray) -> np.ndarray:
    """
    Build a transformation matrix from a pose.
    Pose is given as [x, y, z, roll, pitch, yaw].
    """
    x, y, z, roll, pitch, yaw = pose
    transform = tra.euler_matrix(roll, pitch, yaw, 'sxyz')
    transform[0:3, 3] = [x, y, z]
    return transform


def compute_occlusion_map(
        seeker_pose: np.ndarray, 
        sample_points: np.ndarray,
        combined_mesh: trimesh.Trimesh,
        ) -> tuple[np.ndarray, np.ndarray]:
    # Compute ray origins and directions from the seeker to each sample point.
    seeker_coords = seeker_pose[:3]
    directions = sample_points - seeker_coords
    distances = np.linalg.norm(directions, axis=1)
    nonzero = distances > 1e-6
    directions_norm = np.zeros_like(directions)
    directions_norm[nonzero] = directions[nonzero] / distances[nonzero][:, None]
    directions_norm[~nonzero] = np.array([1, 0, 0])  # dummy direction

    # Prepare origins for each ray.
    origins = np.tile(seeker_coords, (len(sample_points), 1))
    
    # Use trimesh's ray intersections that return all hit locations.
    locations, index_ray, index_tri = combined_mesh.ray.intersects_location(
        ray_origins=origins, ray_directions=directions_norm, multiple_hits=True)
    
    # Initialize an array for the closest valid (front-face) hit for each ray.
    hit_distances = np.full(len(sample_points), np.nan)
    
    if len(locations) > 0:
        # Compute distances for each intersection.
        all_dists = np.linalg.norm(locations - origins[index_ray], axis=1)
        # Get the corresponding face normals.
        face_normals = combined_mesh.face_normals[index_tri]
        # For each intersection, compute the dot product between the ray direction and the face normal.
        ray_dirs = directions_norm[index_ray]
        # A valid front-face hit should have the ray hitting the front (i.e. dot < 0).
        valid = np.einsum('ij,ij->i', ray_dirs, face_normals) < 0
        
        # For each ray, choose the smallest distance among valid hits.
        for i in range(len(sample_points)):
            mask = (index_ray == i) & valid
            if np.any(mask):
                hit_distances[i] = np.min(all_dists[mask])
    
    # A sample point is visible if either no valid hit exists or the valid hit is further than the sample point.
    visible_mask = np.isnan(hit_distances) | ((hit_distances + 1e-6) >= distances)
    visible_points = sample_points[visible_mask]
    occluded_points = sample_points[~visible_mask]
    
    return visible_points, occluded_points

def visualize_map(
        visible_points: np.ndarray,
        occluded_points: np.ndarray,
        goal_points: np.ndarray,
        inside_points: np.ndarray,
        show_visible: bool = False,
        waypoints: np.ndarray | None = None,
):
  visible_pc = trimesh.points.PointCloud(
      visible_points,
      colors=np.tile([255, 0, 0, 255], (len(visible_points), 1))
  )
  occluded_pc = trimesh.points.PointCloud(
      occluded_points,
      colors=np.tile([0, 255, 0, 255], (len(occluded_points), 1))
  )
  goal_pc = trimesh.points.PointCloud(
      goal_points,
      colors=np.tile([255, 20, 147, 255], (len(goal_points), 1))
  )
  inside_pc = trimesh.points.PointCloud(
        inside_points,
        colors=np.tile([0, 0, 255, 255], (len(inside_points), 1))  # blue
  )

  scene = trimesh.Scene()
  if show_visible:
    scene.add_geometry(visible_pc)
  scene.add_geometry(occluded_pc)
  scene.add_geometry(goal_pc)
  scene.add_geometry(inside_pc)


  # Mark the camera position with a small sphere.
  camera_sphere = trimesh.creation.icosphere(radius=0.5)
  camera_sphere.apply_translation(world.seeker_pose[:3])
  camera_sphere.visual.face_colors = [255, 255, 0, 255]  # yellow
  scene.add_geometry(camera_sphere)

  # If waypoints are provided, add black markers and connecting lines.
  if waypoints is not None and len(waypoints) > 0:
      # Add waypoint markers (black points).
      waypoint_cloud = trimesh.points.PointCloud(
          waypoints,
          colors=np.tile([0, 0, 0, 255], (len(waypoints), 1))
      )
      scene.add_geometry(waypoint_cloud)
      
      # For each consecutive pair of waypoints, create a thin cylinder to represent the connecting line.
      for i in range(len(waypoints) - 1):
          p0 = waypoints[i]
          p1 = waypoints[i + 1]
          direction = p1 - p0
          length = np.linalg.norm(direction)
          if length < 1e-6:
              continue  # Skip if points are too close.
          # Create a thin cylinder with a small radius.
          line_cylinder = trimesh.creation.cylinder(radius=0.1, height=length, sections=8)
          # Align the cylinder's z-axis with the direction from p0 to p1.
          z_axis = np.array([0, 0, 1])
          direction_norm = direction / length
          rotation = trimesh.geometry.align_vectors(z_axis, direction_norm)
          line_cylinder.apply_transform(rotation)
          # Translate the cylinder so that its center lies at the midpoint between p0 and p1.
          midpoint = (p0 + p1) / 2
          line_cylinder.apply_translation(midpoint)
          line_cylinder.visual.face_colors = [0, 0, 0, 255]  # black
          scene.add_geometry(line_cylinder)

  print("Displaying scene. Close the window to exit.")
  scene.show()


def save_map_ply(obstacle_points: np.ndarray, file_path: str | Path) -> None:
    """
    Save a point cloud of obstacle points to a .ply file.

    Parameters:
        obstacle_points (np.ndarray): A numpy array of shape (N, 3) containing the obstacle points.
        file_path (str | Path): The file path where the .ply file will be saved.
    """
    colors = np.tile(np.array([255, 0, 0, 255]), (obstacle_points.shape[0], 1))
    point_cloud = trimesh.points.PointCloud(obstacle_points, colors=colors)
    ply_data = point_cloud.export(file_type='ply')
    with open(file_path, 'wb') as f:
        f.write(ply_data)


############################
# PARALLEL ADJACENCY BUILD
############################
def init_worker(mesh: trimesh.Trimesh):
    """
    Called once in each worker to set a global reference to the mesh.
    """
    global GLOBAL_MESH
    GLOBAL_MESH = mesh


def process_edge_chunk(
    origins_chunk: np.ndarray,
    directions_chunk: np.ndarray,
    distances_chunk: np.ndarray,
    edges_chunk: list[tuple[int, int]],
    obstacle_safety_margin: float
) -> list[tuple[int, int, float]]:
    """
    For each ray in this chunk, compute the distance to the first intersection.
    Return a list of (i, j, dist) for edges that are FREE (not blocked).
    """
    global GLOBAL_MESH
    locations, index_ray, _ = GLOBAL_MESH.ray.intersects_location( # TODO: find some faster way to do this
        ray_origins=origins_chunk, 
        ray_directions=directions_chunk, 
        multiple_hits=False
    )

    # Initialize intersection distances
    intersection_distances = np.full(len(origins_chunk), np.inf)
    for loc, ray_idx in zip(locations, index_ray):
        dist = np.linalg.norm(loc - origins_chunk[ray_idx])
        intersection_distances[ray_idx] = dist

    # Figure out which edges are NOT blocked
    free_edges = []
    for idx, (i, j) in enumerate(edges_chunk):
        actual_dist = distances_chunk[idx]
        # If intersection is closer than the endpoint minus a safety margin => blocked
        if intersection_distances[idx] < actual_dist - obstacle_safety_margin:
            continue
        # If we get here => free edge
        free_edges.append((i, j, actual_dist))

    return free_edges


############################
# PATH PLANNING
############################

def compute_waypoints(
    occluded_points: np.ndarray,
    obstacle_points: np.ndarray,
    combined_mesh: trimesh.Trimesh,
    initial_position: np.ndarray,
    goal_points: np.ndarray,
    iteration_limit: int,
    max_velocity: float,
    max_turn_duration: float,
) -> np.ndarray:
    """
    Compute a path of waypoints using A*, ensuring we remain within occluded space
    (or at least away from obstacles) as we navigate from initial_position to any
    point in goal_points.
    """

    # We can load the mesh globally (main process)
    global GLOBAL_MESH
    GLOBAL_MESH = combined_mesh

    # Build the set of "safe" points
    all_points_with_obstacles = np.vstack([initial_position, occluded_points, goal_points])
    obstacle_tree = KDTree(obstacle_points)
    safe_all_points = []
    for pt in all_points_with_obstacles:
        dist, _ = obstacle_tree.query(pt)
        if dist > OBSTACLE_SAFETY_MARGIN:
            safe_all_points.append(pt)
    all_points = np.array(safe_all_points)
    tree = KDTree(all_points)

    def cost_to_go(point: np.ndarray, goal_pts: np.ndarray) -> float:
        return np.linalg.norm(point - goal_pts, axis=1).min()

    max_inter_node_distance = max_velocity * max_turn_duration
    logging.debug(f"Building adjacency with max inter-node distance {max_inter_node_distance} for {len(all_points)} pts")

    # Collect edges
    edges = []
    origins_list = []
    directions_list = []
    distances_list = []

    for i, pt_i in enumerate(all_points):
        neighbor_indices = tree.query_ball_point(pt_i, r=max_inter_node_distance)
        for j in neighbor_indices:
            if i == j:
                continue
            direction = all_points[j] - pt_i
            dist = np.linalg.norm(direction)
            if dist < MIN_EDGE_DISTANCE:
                continue
            direction_norm = direction / dist
            edges.append((i, j))
            origins_list.append(pt_i)
            directions_list.append(direction_norm)
            distances_list.append(dist)

    origins_arr = np.array(origins_list)
    directions_arr = np.array(directions_list)
    distances_arr = np.array(distances_list)

    #########################################
    # PARALLEL RAY INTERSECTIONS
    #########################################
    num_cores = 4  # or mp.cpu_count() for all
    chunk_size = len(edges) // num_cores + 1

    # Split into chunks
    chunks = []
    for idx in range(0, len(edges), chunk_size):
        origins_chunk = origins_arr[idx : idx + chunk_size]
        directions_chunk = directions_arr[idx : idx + chunk_size]
        distances_chunk = distances_arr[idx : idx + chunk_size]
        edges_chunk = edges[idx : idx + chunk_size]
        chunks.append((origins_chunk, directions_chunk, distances_chunk, edges_chunk, OBSTACLE_SAFETY_MARGIN))

    with mp.Pool(processes=num_cores, initializer=init_worker, initargs=(combined_mesh,)) as pool:
        results = pool.starmap(process_edge_chunk, chunks)

    free_edges = [item for sublist in results for item in sublist]

    adjacency = np.zeros((len(all_points), len(all_points)), dtype=float)
    for (i, j, dist) in free_edges:
        adjacency[i, j] = dist
        adjacency[j, i] = dist

    goal_mat = np.zeros(len(all_points), dtype=bool)
    for idx, pt in enumerate(all_points):
        if np.any(np.all(pt == goal_points, axis=1)):
            goal_mat[idx] = True

    visited = np.zeros((len(all_points), 1), dtype=bool)
    cost_to_come = np.full((len(all_points), 1), np.inf)
    parents = np.full((len(all_points), 1), -1)

    queue = PriorityQueue()
    cost_to_come[0] = 0
    queue.put((cost_to_go(initial_position, goal_points), 0))

    logging.debug(f"Starting A* with {len(all_points)} points")

    iteration = 0
    while not queue.empty() and iteration < iteration_limit:
        if iteration % 100 == 0:
            logging.debug(f"Iteration {iteration}, queue size: {queue.qsize()}")

        _, current_idx = queue.get()
        if visited[current_idx]:
            iteration += 1
            continue
        visited[current_idx] = True

        # Check if this is a goal
        if goal_mat[current_idx]:
            # Reconstruct path
            path = []
            while current_idx != -1:
                path.append(all_points[current_idx])
                current_idx = int(parents[current_idx])
            return np.array(path[::-1])  # Reverse
        # Expand neighbors
        for neighbor in np.where(adjacency[current_idx] > 0)[0]:
            if visited[neighbor]:
                continue
            new_cost = cost_to_come[current_idx] + adjacency[current_idx, neighbor]
            if new_cost < cost_to_come[neighbor]:
                cost_to_come[neighbor] = new_cost
                parents[neighbor] = current_idx
                fval = cost_to_come[neighbor] + cost_to_go(all_points[neighbor], goal_points)
                queue.put((fval, neighbor))

        iteration += 1

    # If we exit, no path found (or iteration limit reached)
    return np.array([initial_position])


if __name__ == "__main__":
  # TODO: put this stuff in an argparse
  map_file = Path("simulation/worlds/dust2.sdf")
  logging.info(f"Reading map file: {map_file}")
  output_dir = Path("output")
  headless = False

  world = parse_sdf_map(map_file)
  logging.info(f"Extracted {len(world.models)} obstacles")
  for model in world.models:
      logging.debug(f"{model}")
  logging.info(f"Seeker pose: {world.seeker_pose}")

  # Combine all obstacle meshes into a single mesh for efficient ray queries.
  meshes = []
  for model in world.models:
      if model.type == 'cylinder':
          mesh = create_cylinder_mesh(model.radius, model.length)
          transform = get_transform_from_pose(model.pose)
          mesh.apply_transform(transform)
          meshes.append(mesh)
  combined_mesh = trimesh.util.concatenate(meshes)
  logging.debug(f"Combined mesh: {combined_mesh}")

  for i, mesh in enumerate(meshes):
    print(f"Mesh {i} bounding box: {mesh.bounds}")


  # Instead of checking the combined mesh, check each individual obstacle.
  if any(mesh.contains([world.seeker_pose[:3]])[0] for mesh in meshes):
      logging.error("Camera position is inside an obstacle. Aborting.")
      exit()

  # Map bounds
  grid_x = np.linspace(MAP_X_MIN, MAP_X_MAX, NUM_SAMPLES_HORIZONTAL)
  grid_y = np.linspace(MAP_Y_MIN, MAP_Y_MAX, NUM_SAMPLES_HORIZONTAL)
  grid_z = np.linspace(MAP_Z_MIN, MAP_Z_MAX, NUM_SAMPLES_VERTICAL)
  sample_points = np.array(np.meshgrid(grid_x, grid_y, grid_z)).T.reshape(-1, 3)

  visible_points, occluded_points = compute_occlusion_map(
      seeker_pose=world.seeker_pose,
      sample_points=sample_points, 
      combined_mesh=combined_mesh,
  )
  logging.info(f"Computed {len(occluded_points)} occluded points ({len(occluded_points) / len(sample_points) * 100:.2f}% of total)")
  logging.debug(f"Occluded points: {occluded_points}")

  # Sample randomly occluded poiints with downsampling factor
  num_occluded_points = int(len(occluded_points) / DOWNSAMPLING_FACTOR)
  occluded_indices = np.random.choice(len(occluded_points), num_occluded_points, replace=False)
  occluded_points = occluded_points[occluded_indices]
  logging.info(f"Downsampled occluded points to {num_occluded_points} points")

  inside_mask = combined_mesh.contains(sample_points)
  inside_points = sample_points[inside_mask]
  logging.info(f"Found {len(inside_points)} points in obstacles ({len(inside_points) / len(sample_points) * 100:.2f}% of total)")

  save_map_ply(
      obstacle_points=inside_points,
      file_path=output_dir / "obstacle_points.ply",
  )

  

  border_y = np.linspace(MAP_Y_MIN, MAP_Y_MAX, NUM_SAMPLES_HORIZONTAL)
  border_z = np.linspace(MAP_Z_MIN, MAP_Z_MAX, NUM_SAMPLES_VERTICAL)
  B_y, B_z = np.meshgrid(border_y, border_z)
  border_x = np.full(B_y.shape, world.seeker_pose[0])
  goal_points = np.column_stack((border_x.ravel(), B_y.ravel(), B_z.ravel()))


  logging.debug(f"Generated {len(goal_points)} goal points")

  waypoints = compute_waypoints(
      occluded_points=occluded_points,
      obstacle_points=inside_points,
      combined_mesh=combined_mesh,
      initial_position=INITIAL_POSITION,
      iteration_limit=10000, 
      goal_points=goal_points,
      max_velocity=MAX_VELOCITY,
      max_turn_duration=MAX_TURN_DURATION,
  )

  logging.info(f"Computed {len(waypoints)} waypoints")
  logging.debug(f"Waypoints: {waypoints}")

  np.save(output_dir / "waypoints.npy", waypoints)

  if not headless:
      visualize_map(
          visible_points=visible_points, 
          occluded_points=occluded_points, 
          inside_points=inside_points,
          goal_points=goal_points,
          waypoints=waypoints,)


  

