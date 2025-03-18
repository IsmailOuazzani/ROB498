import os

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
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

    wp_pub_node = Node(
        package='flight_club',
        executable='waypoint_publisher.py',
        name='waypoint_pub_node',
        output='screen'
    )

    velocity_control_node = Node(
        package='flight_club',
        executable='velocity_control.py',
        name='vel_control_node',
        output='screen'
    )

    planner_node = Node(
        package='flight_club',
        executable='planner.py',
        name='planner_node',
        output='screen')

    vicon_node = Node(
        package='flight_club',
        executable='vicon.py',
        name='estimator_node',
        output='screen',
        parameters=[{
            'sim': LaunchConfiguration('sim')
        }]
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'sim', 
            default_value='True', 
            description='simulation? (True/False)'),
        mavros_launch,
        velocity_control_node,
        planner_node,
        wp_pub_node,
        vicon_node
        ])
