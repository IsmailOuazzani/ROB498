from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="simulation",
            executable="raycaster",
            name="raycaster"
        ),
        Node(
            package="simulation",
            executable="occlusion_mapping",
            name="occlusion_mapping"
        ),
        Node(
            package="simulation",
            executable="occlusion_visualizer",
            name="occlusion_visualizer"
        ),
    ])
