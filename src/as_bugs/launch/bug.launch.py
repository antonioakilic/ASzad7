import os
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit

def generate_launch_description():


    pkg_name='as_bugs'
    dir_name='models'
    xacro_file='arena.urdf.xacro'
    xacro_location = os.path.join(get_package_share_directory(pkg_name), dir_name, xacro_file)  ## NE RADI U GAZEBO
    polygon_description_xacro = xacro.process_file(xacro_location).toxml()                        ## NE RADI U GAZEBO

    world = LaunchConfiguration('world')
    use_sim_time = LaunchConfiguration('use_sim_time')
    x_pose = LaunchConfiguration('x_pose', default='0.0')
    y_pose = LaunchConfiguration('y_pose', default='0.0')

    world_arg = DeclareLaunchArgument(
        'world',
        default_value=os.path.join(get_package_share_directory('as_bugs'), 'worlds', 'Arena_1.world'),
        description='Full path to the world file to load.'
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Define if you want to use simulation time'
    )

    start_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
        launch_arguments={'world': world}.items()
    )

    turtlebot3_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(get_package_share_directory('turtlebot3_gazebo'), 'launch', 'robot_state_publisher.launch.py')]),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    spawn_turtlebot3 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(get_package_share_directory('turtlebot3_gazebo'), 'launch', 'spawn_turtlebot3.launch.py')]),
        launch_arguments={
            'x_pose': x_pose,
            'y_pose': y_pose
        }.items()
    )

    rviz2_base = os.path.join(get_package_share_directory('as_bugs'), 'config')
    rviz2_full_config = os.path.join(rviz2_base, 'bug.rviz')

    rviz2_node = Node(
        executable='rviz2',
        name='rviz2',
        output='log',
        arguments=['-d', rviz2_full_config],
    )

    parser_node = Node(
        package="as_bugs",
        executable="parse_urdf2marker",
        name="parser",
        output="screen"
    )
    
    tracker_node = Node(
        package="as_bugs",
        executable="tracker",
        name="tracker",
        output="screen"
    )

    fl_sensor_link_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments = ['--x', '0', '--y', '0', '--z', '0', '--yaw', '0.523598776', '--pitch', '0', '--roll', '0', '--frame-id', 'base_scan', '--child-frame-id', 'fl_sensor_link']
    )

    fr_sensor_link_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments = ['--x', '0', '--y', '0', '--z', '0', '--yaw', '-0.523598776', '--pitch', '0', '--roll', '0', '--frame-id', 'base_scan', '--child-frame-id', 'fr_sensor_link']
    )

    scan_to_range_node = Node(
        package="as_bugs",
        executable="scan_to_range",
        name="scan_to_range"
    )

    return LaunchDescription([
        world_arg,
        use_sim_time_arg,
        start_gazebo,
        turtlebot3_publisher,
        spawn_turtlebot3,
        rviz2_node,
        parser_node,
        tracker_node,
        fl_sensor_link_node,
        fr_sensor_link_node,
        scan_to_range_node
    ])
