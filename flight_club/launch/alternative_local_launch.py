import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

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



    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', "/src/ros_ws/src/drone_packages/rviz/game.rviz"]
    )

    return LaunchDescription([
        map_arg,
        game_loop_node,
        rviz_node,
    ])
