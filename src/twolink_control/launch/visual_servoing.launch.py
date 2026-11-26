from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    port_arg = DeclareLaunchArgument(
        'port',
        default_value='/dev/ttyACM0',
        description='Serial port for the twolink robot controller',
    )
    target_distance_arg = DeclareLaunchArgument(
        'target_distance',
        default_value='0.35',
        description='Desired distance to ArUco marker in meters',
    )
    camera_yaw_offset_arg = DeclareLaunchArgument(
        'camera_yaw_offset',
        default_value='0.0',
        description='Camera mounting yaw offset relative to link2 in radians',
    )
    use_camera_arg = DeclareLaunchArgument(
        'use_camera',
        default_value='true',
        description='Launch RealSense camera and ArUco marker detector',
    )
    use_gazebo_arg = DeclareLaunchArgument(
        'use_gazebo',
        default_value='false',
        description='Launch Gazebo/RViz and sync robot_state via joint_publisher',
    )

    port = LaunchConfiguration('port')
    target_distance = LaunchConfiguration('target_distance')
    camera_yaw_offset = LaunchConfiguration('camera_yaw_offset')
    use_camera = LaunchConfiguration('use_camera')
    use_gazebo = LaunchConfiguration('use_gazebo')

    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('realsense2_camera'),
                'launch',
                'rs_launch.py',
            ])
        ]),
        launch_arguments={
            'enable_color': 'true',
            'enable_depth': 'false',
        }.items(),
        condition=IfCondition(use_camera),
    )

    aruco_detector_node = Node(
        package='marker_detection',
        executable='detection_realsense',
        name='aruco_detector_node_realsense',
        output='log',
        condition=IfCondition(use_camera),
    )

    serial_node = Node(
        package='twolink_control',
        executable='serial_node',
        name='serial_interface',
        parameters=[{'port': port}],
        output='screen',
    )

    visual_servoing_node = Node(
        package='twolink_control',
        executable='visual_servoing_node',
        name='visual_servoing_node',
        parameters=[{
            'target_distance': target_distance,
            'camera_yaw_offset': camera_yaw_offset,
        }],
        output='screen',
    )

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('twolink_gazebo'),
                'launch',
                'gazebo.launch.py',
            ])
        ]),
        condition=IfCondition(use_gazebo),
    )

    joint_publisher_node = Node(
        package='twolink_gazebo',
        executable='joint_publisher.py',
        name='joint_publisher',
        output='log',
        condition=IfCondition(use_gazebo),
    )

    return LaunchDescription([
        port_arg,
        target_distance_arg,
        camera_yaw_offset_arg,
        use_camera_arg,
        use_gazebo_arg,
        realsense_launch,
        aruco_detector_node,
        serial_node,
        visual_servoing_node,
        gazebo_launch,
        joint_publisher_node,
    ])
