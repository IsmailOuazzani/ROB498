import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    waypoint_arg = DeclareLaunchArgument(
        'waypoints',
        default_value='/src/ros_ws/src/drone_packages/output/baldwin_waypoints.npy',  # Default folder for waypoints
        description='Folder where waypoints are specified'
    )
    # declare the in collsion points
    occluded_arg = DeclareLaunchArgument(
        'occluded_region',
        default_value='/src/ros_ws/src/drone_packages/output/baldwin_occluded.npy',  # Default folder for waypoints
        description='Folder where out of collision points are specified'
    )

    obstacle_arg = DeclareLaunchArgument(
        'obstacles',
        default_value='/src/ros_ws/src/drone_packages/output/baldwin_obstacles.npy',  # Default folder for waypoints
        description='Folder where out of collision points are specified'
    )
    # Get the share directories for the packages
    px4_autonomy_share = get_package_share_directory('px4_autonomy_modules')
    flight_club_share = get_package_share_directory('flight_club')

    # Declare launch argument for the map name
    map_arg = DeclareLaunchArgument(
        'map_name',
        default_value='baldwin',
        description='Map name to be used in the game loop node.'
    )

    game_loop_node = Node(
        package='flight_club',
        executable='game_loop.py',  # Make sure this matches the entry point or filename
        name='game_loop_node',
        namespace='flight_club',
        output='screen',
        emulate_tty=True,
        parameters=[{'map_name': LaunchConfiguration('map_name')}]
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', "/src/ros_ws/src/drone_packages/rviz/game.rviz"]
    )
    sequence_node = Node(
        package='flight_club',
        executable='sequence_node.py',
        name='sequence_node',
        output='screen',
        parameters=[{'waypoints': LaunchConfiguration('waypoints'),
                     'occluded_region': LaunchConfiguration('occluded_region'),
                     'obstacles': LaunchConfiguration('obstacles')}],
        )

    return LaunchDescription([
        waypoint_arg,
        occluded_arg,
        obstacle_arg,
        sequence_node,
        map_arg,
        game_loop_node,
        rviz_node
    ])
