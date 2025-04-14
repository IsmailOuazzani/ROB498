import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # folder where plan trajectory is contained
    waypoint_arg = DeclareLaunchArgument(
        'waypoints',
        default_value='/src/ros_ws/src/drone_packages/output/arena_waypoints.npy',  # Default folder for waypoints
        description='Folder where waypoints are specified'
    )
    # declare the in collsion points
    occluded_arg = DeclareLaunchArgument(
        'occluded_region',
        default_value='/src/ros_ws/src/drone_packages/output/arena_occluded.npy',  # Default folder for waypoints
        description='Folder where out of collision points are specified'
    )

    obstacle_arg = DeclareLaunchArgument(
        'obstacles',
        default_value='/src/ros_ws/src/drone_packages/output/arena_obstacles.npy',  # Default folder for waypoints
        description='Folder where out of collision points are specified'
    )

    # Get the share directories for the packages
    px4_autonomy_share = get_package_share_directory('px4_autonomy_modules')
    flight_club_share = get_package_share_directory('flight_club')

    # Path to the MAVROS launch file in the px4_autonomy_modules package
    mavros_launch_path = os.path.join(px4_autonomy_share, 'launch', 'mavros.launch.py')

    # Include the MAVROS launch file with the fcu_url argument
    mavros_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(mavros_launch_path),
        launch_arguments={'fcu_url': 'udp://:14540@127.0.0.1:14557'}.items()
    )

    # Declare launch argument for the map name
    map_arg = DeclareLaunchArgument(
        'map_name',
        default_value='arena',
        description='Map name to be used in the game loop node.'
    )

    game_loop_node = Node(
        package='flight_club',
        executable='game_loop.py',  # Make sure this matches the entry point or filename
        name='game_loop_node',
        namespace='flight_club',
        output='screen',
        parameters=[{'map_name': LaunchConfiguration('map_name')}],
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

        # Node to launch exercise2 from flight_club
    executer_node = Node(
        package='flight_club',
        executable='velocity_control.py',
        name='drone_node',
        output='screen'
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', "/src/ros_ws/src/drone_packages/rviz/game.rviz"]
    )

    return LaunchDescription([
        waypoint_arg,
        occluded_arg,
        obstacle_arg,
        map_arg,
        game_loop_node,
        rviz_node,
        mavros_launch,
        sequence_node, 
        executer_node
    ])
