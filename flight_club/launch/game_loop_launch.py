import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Get the share directories for the packages
    px4_autonomy_share = get_package_share_directory('px4_autonomy_modules')
    flight_club_share = get_package_share_directory('flight_club')

    # Path to the MAVROS launch file in the px4_autonomy_modules package
    mavros_launch_path = os.path.join(px4_autonomy_share, 'launch', 'mavros.launch.py')

    game_loop_node = Node(
        package='flight_club',
        executable='game_loop.py',
        name='game_loop_node',
        output='screen')

    return LaunchDescription([
        game_loop_node
    ])
