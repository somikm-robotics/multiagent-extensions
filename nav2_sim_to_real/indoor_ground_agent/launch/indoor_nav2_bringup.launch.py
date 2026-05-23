from launch import LaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
from launch.actions import RegisterEventHandler, ExecuteProcess

def generate_launch_description():

    pkg = FindPackageShare('indoor_ground_agent')

    nav2_params = PathJoinSubstitution([
        pkg, 'config', 'nav2_params.yaml'
    ])

    map_yaml = PathJoinSubstitution([
        pkg, 'maps', 'indoor_map.yaml'
    ])

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
            'use_sim_time': True
        }]
    )

    amcl = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[nav2_params]
    )

    localization_lifecycle = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'autostart': True,
            'node_names': ['map_server', 'amcl']
        }]
    )

    # ----------------------------
    # Gate: wait for localization ACTIVE
    # ----------------------------

    wait_for_localisation = ExecuteProcess(
        cmd=[
            'bash', '-c',
            'until ros2 lifecycle get /amcl | grep -q "active"; do sleep 0.5; done'
        ],
        output='screen'
    )


    # ==========================================================
    # NAV2 NAVIGATION STACK (EXPLICIT)
    # ==========================================================

    controller_server = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[nav2_params]
    )

    planner_server = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[nav2_params]
    )

    smoother_server = Node(
        package='nav2_smoother',
        executable='smoother_server',
        name='smoother_server',
        output='screen',
        parameters=[nav2_params]
    )

    behavior_server = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        parameters=[nav2_params]
    )

    bt_navigator = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[nav2_params]
    )

    waypoint_follower = Node(
        package='nav2_waypoint_follower',
        executable='waypoint_follower',
        name='waypoint_follower',
        output='screen',
        parameters=[nav2_params]
    )

    velocity_smoother = Node(
        package='nav2_velocity_smoother',
        executable='velocity_smoother',
        name='velocity_smoother',
        output='screen',
        parameters=[nav2_params]
    )

    navigation_lifecycle = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'autostart': True,
            'node_names': [
                'controller_server',
                'planner_server',
                'smoother_server',
                'behavior_server',
                'bt_navigator',
                'waypoint_follower',
                'velocity_smoother'
            ]
        }]
    )

    # ==========================================================
    # START NAVIGATION ONLY AFTER LOCALIZATION
    # ==========================================================

    start_navigation = RegisterEventHandler(
        OnProcessExit(
            target_action=wait_for_localisation,
            on_exit=[
                controller_server,
                planner_server,
                smoother_server,
                behavior_server,
                waypoint_follower,
                velocity_smoother,
                bt_navigator,
                navigation_lifecycle
            ]
        )
    )

    twist_relay_node = Node(
        package='indoor_ground_agent',  
        executable='twist_relay_node',  # Make sure this script is installed
        name='twist_relay_node',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    return LaunchDescription([
        twist_relay_node,
        map_server,
        amcl,
        localization_lifecycle,
        wait_for_localisation,
        start_navigation
    ])
