from launch import LaunchDescription
from launch.actions import RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
from launch.actions import RegisterEventHandler, ExecuteProcess

def generate_launch_description():

    
    pkg = FindPackageShare('turtlebot')

    nav2_params_yaml = PathJoinSubstitution([
        pkg, 'config', 'nav2_params.yaml'
    ])

    

    # ==========================================================
    # NAV2 NAVIGATION STACK (EXPLICIT)
    # ==========================================================

    controller_server = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[nav2_params_yaml]
    )

    planner_server = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[nav2_params_yaml]
    )

    # smoother_server = Node(
    #     package='nav2_smoother',
    #     executable='smoother_server',
    #     name='smoother_server',
    #     output='screen',
    #     parameters=[nav2_params_yaml]
    # )

    behavior_server = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        parameters=[nav2_params_yaml]
    )

    bt_navigator = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[nav2_params_yaml]
    )

    # waypoint_follower = Node(
    #     package='nav2_waypoint_follower',
    #     executable='waypoint_follower',
    #     name='waypoint_follower',
    #     output='screen',
    #     parameters=[nav2_params_yaml]
    # )

    # velocity_smoother = Node(
    #     package='nav2_velocity_smoother',
    #     executable='velocity_smoother',
    #     name='velocity_smoother',
    #     output='screen',
    #     parameters=[nav2_params_yaml]
    # )

    navigation_lifecycle = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'autostart': True,
            'node_names': [
                'controller_server',
                'planner_server',
                'behavior_server',
                'bt_navigator',
            ]
        }]
    )

    delayed_lifecycle = TimerAction(
        period=5.0,
        actions=[navigation_lifecycle]
    )
    
    # ==========================================================
    # START NAVIGATION ONLY AFTER LOCALIZATION
    # ==========================================================


    return LaunchDescription([
        controller_server,
        planner_server,
        behavior_server,
        bt_navigator,
        delayed_lifecycle
    ])
