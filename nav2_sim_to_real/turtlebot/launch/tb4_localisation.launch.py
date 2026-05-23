from launch import LaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
from launch.actions import RegisterEventHandler, ExecuteProcess

def generate_launch_description():

    
    pkg = FindPackageShare('turtlebot')

    localisation_params_yaml = PathJoinSubstitution([
        pkg, 'config', 'localisation_params.yaml'
    ])

    map_yaml = PathJoinSubstitution([
        pkg, 'maps', 'tb4_map.yaml'
    ])

    # relay node for scan
    scan_relay = Node(
        package='turtlebot',   
        executable='scan_fresh_relay_node',
        name='scan_fresh_relay_node',
        output='screen',
        parameters=[{'use_sim_time': False}]
    )

    # ==========================================================
    # LOCALIZATION STACK
    # ==========================================================

    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{
            'yaml_filename': map_yaml,
            'use_sim_time': False
        }]
    )

    amcl = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[localisation_params_yaml]
    )

    localization_lifecycle = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'autostart': True,
            'node_names': ['map_server', 'amcl']
        }]
    )

    return LaunchDescription([
        map_server,
        scan_relay,
        amcl,
        localization_lifecycle,
    
    ])
