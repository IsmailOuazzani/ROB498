#!/usr/bin/env python3

from pathlib import Path
from dataclasses import dataclass
import numpy as np
import xml.etree.ElementTree as ET
import logging
import trimesh
import trimesh.transformations as tra
import matplotlib.pyplot as plt


SEEKER_OFFSET = np.array([0.0, 0.0, 1.5, 0.0, 0.0, 0.0])
OBSTACLE_SAFETY_MARGIN = 0.0 # TODO: this should be considered for visualisation and path planning, but not for occlusion
NUM_SAMPLES_HORIZONTAL = 50
NUM_SAMPLES_VERTICAL = 10


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

        radius = float(cyl_elem.find('radius').text) + OBSTACLE_SAFETY_MARGIN
        length = float(cyl_elem.find('length').text) + OBSTACLE_SAFETY_MARGIN
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
    # TODO: take into account orientation of the camera
    seeker_coords = seeker_pose[:3]
    # Compute ray directions and distances from camera
    directions = sample_points - seeker_coords
    distances = np.linalg.norm(directions, axis=1)
    # Avoid division by zero for rays originating at the camera
    nonzero = distances > 1e-6
    directions_norm = np.zeros_like(directions)
    directions_norm[nonzero] = directions[nonzero] / distances[nonzero][:, None]
    directions_norm[~nonzero] = np.array([1, 0, 0])  # dummy direction

    # Repeat camera position for each ray
    origins = np.tile(seeker_coords, (len(sample_points), 1))
    # Use trimesh's batch ray intersection: returns the distance to the first intersection (or NaN if none)
    hit_distances = combined_mesh.ray.intersects_first(
        ray_origins=origins, ray_directions=directions_norm)

    # A point is visible if there's no hit (NaN) or if the hit is farther than the sample point distance
    visible_mask = np.isnan(hit_distances) | (hit_distances >= distances)
    visible_points = sample_points[visible_mask]
    occluded_points = sample_points[~visible_mask]

    return occluded_points, visible_points



def compute_waypoints(
        occlusion_map: np.ndarray,
        initial_position: np.ndarray,
        seek_position: np.ndarray,
        winning_radius: float,
        iteration_limit: int,
        max_velocity: float,
        max_game_duration: float,
) -> np.ndarray:
    ...

if __name__ == "__main__":
  # TODO: put this stuff in an argparse
  map_file = Path("simulation/worlds/easy.sdf")
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


  # TODO: move this in a separate function, with headless flag
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
  # scene.add_geometry(visible_pc)
  scene.add_geometry(occluded_pc)
  scene.add_geometry(inside_pc)


  # Mark the camera position with a small sphere.
  camera_sphere = trimesh.creation.icosphere(radius=0.5)
  camera_sphere.apply_translation(world.seeker_pose[:3])
  camera_sphere.visual.face_colors = [255, 255, 0, 255]  # yellow
  scene.add_geometry(camera_sphere)

  print("Displaying scene. Close the window to exit.")
  scene.show()



  

