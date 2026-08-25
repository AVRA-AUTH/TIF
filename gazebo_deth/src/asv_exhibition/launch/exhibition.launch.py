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
        cmd=['gz', 'sim', '-s', '-r', world_file],
        output='screen',
        condition=IfCondition(headless)
    )

    # 1b. Gazebo GUI
    gazebo_gui = ExecuteProcess(
        cmd=['gz', 'sim', '-r', world_file],
        output='screen',
        condition=UnlessCondition(headless)
    )

    # 2. Bridge
    # Thruster topics are /asv_boat/thrusters/{left,right}/thrust, not
    # /thrusters/{left,right}/thrust — the Thruster plugin in
    # exhibition_water.sdf always prefixes its <topic> with the model name
    # (see the comment there for how this was found empirically).
    bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
            '/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry',
            '/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
            '/asv_boat/thrusters/left/thrust@std_msgs/msg/Float64@gz.msgs.Double',
            '/asv_boat/thrusters/right/thrust@std_msgs/msg/Float64@gz.msgs.Double'
        ],
        output='screen'
    )

    # 3. ROSBridge
    rosbridge_launch = IncludeLaunchDescription(
        XMLLaunchDescriptionSource(
            os.path.join(get_package_share_directory('rosbridge_server'), 'launch', 'rosbridge_websocket_launch.xml')
        )
    )

    # 4. Spawner & Game Manager & Autonomous Navigator & Thrust Mixer
    spawner_node = Node(
        package='asv_exhibition',
        executable='obstacle_spawner.py',
        name='obstacle_spawner',
        output='screen'
    )

    # Translates /cmd_vel (published unchanged by web_ui/app.js in all 3
    # modes) into per-thruster force commands for the real physics below —
    # keeps app.js and the ROS graph's public interface untouched.
    thrust_mixer_node = Node(
        package='asv_exhibition',
        executable='cmd_vel_thrust_mixer.py',
        name='cmd_vel_thrust_mixer',
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
        navigator_node,
        thrust_mixer_node
    ])
