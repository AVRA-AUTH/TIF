import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    pkg_asv_exhibition = get_package_share_directory('asv_exhibition')
    world_file = os.path.join(pkg_asv_exhibition, 'worlds', 'exhibition_water.sdf')

    headless_arg = DeclareLaunchArgument(
        'headless',
        default_value='true',
        description='Run Gazebo in headless server mode'
    )

    headless = LaunchConfiguration('headless')

    # 1a. Gazebo Headless Server
    gazebo_server = ExecuteProcess(
        cmd=['ign', 'gazebo', '-s', '-r', world_file],
        output='screen',
        condition=IfCondition(headless)
    )

    # 1b. Gazebo GUI
    gazebo_gui = ExecuteProcess(
        cmd=['ign', 'gazebo', '-r', world_file],
        output='screen',
        condition=UnlessCondition(headless)
    )

    # 2. Bridge
    bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist',
            '/odom@nav_msgs/msg/Odometry@ignition.msgs.Odometry',
            '/scan@sensor_msgs/msg/LaserScan@ignition.msgs.LaserScan'
        ],
        output='screen'
    )

    # 3. ROSBridge
    rosbridge_launch = IncludeLaunchDescription(
        XMLLaunchDescriptionSource(
            os.path.join(get_package_share_directory('rosbridge_server'), 'launch', 'rosbridge_websocket_launch.xml')
        )
    )

    # 4. Spawner & Game Manager & Autonomous Navigator
    spawner_node = Node(
        package='asv_exhibition',
        executable='obstacle_spawner.py',
        name='obstacle_spawner',
        output='screen'
    )

    game_manager_node = Node(
        package='asv_exhibition',
        executable='game_manager.py',
        name='game_manager',
        output='screen'
    )

    navigator_node = Node(
        package='asv_exhibition',
        executable='asv_navigator.py',
        name='asv_navigator',
        output='screen'
    )

    return LaunchDescription([
        headless_arg,
        gazebo_server,
        gazebo_gui,
        bridge_node,
        rosbridge_launch,
        spawner_node,
        game_manager_node,
        navigator_node
    ])
