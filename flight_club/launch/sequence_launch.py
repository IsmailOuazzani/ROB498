import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # folder where plan trajectory is contained
    waypoint_output_folder_arg = DeclareLaunchArgument(
        'waypoint_output_folder',
        default_value='/src/ros_ws/src/drone_packages/output',  # Default folder for waypoints
        description='Folder where waypoints are specified'
    )
    # declare the in collsion points
    occluded_folder_arg = DeclareLaunchArgument(
        'occluded_folder',
        default_value='/src/ros_ws/src/drone_packages/output',  # Default folder for waypoints
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


    sequence_node = Node(
        package='flight_club',
        executable='sequence_node.py',
        name='sequence_node',
        output='screen',
        parameters=[{'output_folder': LaunchConfiguration('waypoint_output_folder')}, {'occluded_folder': LaunchConfiguration('occluded_folder')}]
        )

        # Node to launch exercise2 from flight_club
    executer_node = Node(
        package='flight_club',
        executable='velocity_control.py',
        name='drone_node',
        output='screen'
    )

    return LaunchDescription([
        waypoint_output_folder_arg,
        occluded_folder_arg,
        mavros_launch,
        sequence_node, 
        executer_node
    ])
