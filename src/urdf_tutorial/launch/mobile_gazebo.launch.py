import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    urdf_file = LaunchConfiguration('urdf_file')
    use_sim_time = LaunchConfiguration('use_sim_time', default=True)

    declare_urdf_file_cmd = DeclareLaunchArgument(
        'urdf_file',
        default_value='box_robot.urdf',
        description='URDF file to load'
    )

    pkg_path = get_package_share_directory('urdf_tutorial')
    urdf_path = PathJoinSubstitution([pkg_path, 'urdf', urdf_file])

    robot_description_content = Command(['xacro ', urdf_path])

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_content}, {'use_sim_time': use_sim_time}]
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', os.path.join(pkg_path, 'rviz', 'default.rviz')]
    )

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={'verbose': 'true', 'pause': 'false'}.items()
    )

    spawn_entity_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'with_robot'],
        output='screen'
    )

    return LaunchDescription([
        declare_urdf_file_cmd,
        robot_state_publisher_node,
        rviz_node,
        gazebo_launch,
        spawn_entity_node
    ])
