import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
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

    # waypoint publisher for task 3 testing
    wp_pub_node = Node(
        package='flight_club',
        executable='waypoint_publisher.py',
        name='waypoint_pub_node',
        output='screen'
    )

    # Node to launch exercise2 from flight_club
    exercise2_node = Node(
        package='flight_club',
        executable='exercise2.py',
        name='exercise2_node',
        output='screen'
    )

    # Node to launch exercise2 from flight_club
    exercise3_node = Node(
        package='flight_club',
        executable='velocity_controll.py',
        name='vel_control_node',
        output='screen'
    )

    planner_node = Node(
        package='flight_club',
        executable='planner.py',
        name='planner_node',
        output='screen')

    return LaunchDescription([
        mavros_launch,
        exercise3_node,
        planner_node,
        wp_pub_node
        ])
