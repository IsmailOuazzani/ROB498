from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    visualizer_node = Node(
        package='flight_club',
        executable='visualizer.py',
        name='visualizer_node',
        output='screen'
    )
    flight_club_share = get_package_share_directory('flight_club')

    return LaunchDescription([
        visualizer_node
    ])