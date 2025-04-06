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



SEEKER_OFFSET = np.array([0.0, 0.0, 1.5, 0.0, 0.0, 0.0])
OBSTACLE_SAFETY_MARGIN = 1.0


NUM_SAMPLES_HORIZONTAL = 80
NUM_SAMPLES_VERTICAL = 30

# NUM_SAMPLES_HORIZONTAL = 50
# NUM_SAMPLES_VERTICAL = 10

MOVE_PENALTY = 5.0 # penalise trajectory with too many waypoints


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
  inside_pc = trimesh.points.PointCloud(
        inside_points,
        colors=np.tile([0, 0, 255, 255], (len(inside_points), 1))  # blue
  )

  scene = trimesh.Scene()
  if show_visible:
    scene.add_geometry(visible_pc)
  scene.add_geometry(occluded_pc)
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

@dataclass
class Node:
    position: np.ndarray
    parent: Node = None
    g: float = 0.0

    def __eq__(self, other: object) -> bool:
        if not isinstance(other, Node):
            return NotImplemented
        return np.array_equal(self.position, other.position)

    def __lt__(self, other: Node) -> bool:
        return self.g < other.g
    
    def __hash__(self) -> int:
        return hash(tuple(self.position))
    

def compute_waypoints(
        occluded_points: np.ndarray,
        obstacle_points: np.ndarray,
        initial_position: np.ndarray,
        seek_position: np.ndarray,
        winning_radius: float,
        iteration_limit: int,
        max_velocity: float,
        max_turn_duration: float,
) -> np.ndarray:
  """Compute waypoints to navigate to the seek position while remaining hidden.
  """

  # Use A* to compute the shortest path to the seek position. 

  def position_key(pos, precision=2):
    return tuple(np.round(pos, decimals=precision))

  queue = PriorityQueue()
  start_node = Node(position=initial_position)
  queue.put((0, start_node))

  # create all_points, indicating points ok to visit
  # this starts with all occluded points
  # add a ball of size winning radius around the seek position
  num_goal_samples = 20
  goal_angles = np.linspace(0, 2 * np.pi, num_goal_samples, endpoint=False)
  goal_points = np.array([
        seek_position + winning_radius * np.array([np.cos(angle), np.sin(angle), 0])
        for angle in goal_angles
    ])
  all_points_with_obstacles = np.vstack([occluded_points, goal_points])
  
  obstacle_tree = KDTree(obstacle_points)
  safe_all_points = []
  for point in all_points_with_obstacles:
      distance, _ = obstacle_tree.query(point)
      if distance > OBSTACLE_SAFETY_MARGIN:
          safe_all_points.append(point)

  all_points = np.array(safe_all_points)
  tree = KDTree(all_points) 

  max_inter_node_distance = max_velocity * max_turn_duration
  logging.debug(f"Starting waypoint computation at {initial_position}")
  logging.debug(f"Max inter-node distance: {max_inter_node_distance}")

  visited = set()
  while not queue.empty():
      _, current_node = queue.get()
      visited.add(position_key(current_node.position))

      # Check if we reached the seek position.
      if np.linalg.norm(current_node.position - seek_position) < winning_radius: # Actually need to let it run longer for A*
          # Add final node, at exact seek position 
          final_node = Node(position=seek_position, parent=current_node, g=current_node.g)
          current_node = final_node
          path = []
          while current_node is not None:
              path.append(current_node.position)
              current_node = current_node.parent
          return np.array(path[::-1])  # Reverse the path

      indices = tree.query_ball_point(current_node.position, r=max_inter_node_distance)
      for i in indices:
          point = all_points[i]
          if position_key(point) in visited:
            continue
          distance = np.linalg.norm(current_node.position - point)
          new_g = current_node.g + distance + MOVE_PENALTY
          new_node = Node(position=point, parent=current_node, g=new_g)
          h = np.linalg.norm(point - seek_position) #TODO try penalizing h more than g to encourage going for further points first
          f = new_g + h
          queue.put((f, new_node))

  return np.array([])  # No path found


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
  grid_x = np.linspace(-40, 40, NUM_SAMPLES_HORIZONTAL)
  grid_y = np.linspace(-40, 40, NUM_SAMPLES_HORIZONTAL)
  grid_z = np.linspace(0, 15, NUM_SAMPLES_VERTICAL)
  sample_points = np.array(np.meshgrid(grid_x, grid_y, grid_z)).T.reshape(-1, 3)

  visible_points, occluded_points = compute_occlusion_map(
      seeker_pose=world.seeker_pose,
      sample_points=sample_points, 
      combined_mesh=combined_mesh,
  )
  logging.info(f"Computed {len(occluded_points)} occluded points ({len(occluded_points) / len(sample_points) * 100:.2f}% of total)")
  logging.debug(f"Occluded points: {occluded_points}")

  # TODO: save occlued points to file

  inside_mask = combined_mesh.contains(sample_points)
  inside_points = sample_points[inside_mask]
  logging.info(f"Found {len(inside_points)} points in obstacles ({len(inside_points) / len(sample_points) * 100:.2f}% of total)")

  save_map_ply(
      obstacle_points=inside_points,
      file_path=output_dir / "obstacle_points.ply",
  )
      
  waypoints = compute_waypoints(
      occluded_points=occluded_points,
      obstacle_points=inside_points,
      initial_position=np.array([0, 0, 0]),
      seek_position=world.seeker_pose[:3],
      winning_radius=1.0,
      iteration_limit=100,
      max_velocity=1.0,
      max_turn_duration=10.0,
  )

  logging.info(f"Computed {len(waypoints)} waypoints")
  logging.debug(f"Waypoints: {waypoints}")

  if not headless:
      visualize_map(
          visible_points=visible_points, 
          occluded_points=occluded_points, 
          inside_points=inside_points,
          waypoints=waypoints,)


  

