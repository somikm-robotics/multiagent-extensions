#/bin/python3

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node

from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

use_sim_time = LaunchConfiguration('use_sim_time', default=True)
log_level = LaunchConfiguration('log_level', default='info')

def generate_launch_description():
    
    # make sure the arg exists (harmless if parent already declared it)
    declare_sim_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (/clock) time'
    )
    
    declare_log_level_arg = DeclareLaunchArgument(
        'log_level',
        default_value='debug',
        description='log level'
    )

    pkg_share = get_package_share_directory('multi_agent_simulation')


    params_file_name = "nav2_params.yaml"

    # Update map file path in params file
    params_file = os.path.join(pkg_share, 'config', params_file_name)

    map_file = os.path.join(pkg_share, 'maps', 'mining_map.yaml')
    
    
     # Start map server
    map_server_node = Node(
                package='nav2_map_server',
                executable='map_server',
                name='map_server',
                output='screen',
                # parameters=[configured_params],
                parameters = [
                    {
                        'yaml_filename': str(map_file),
                        'topic_name':    '/map',        # default
                        'frame_id':      'map',
                        'use_sim_time':  True
                    }
                ],
                arguments=['--ros-args', '--log-level', 'info'])

    map_server_lifecycle_node = Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_map',
                output='screen',
                arguments=['--ros-args', '--log-level', 'info'],
                parameters=[{'use_sim_time': True},
                            {'autostart': True},
                            {'node_names': ['map_server']}
                            ])


    
    
    twist_relay_node = Node(
        package='multi_agent_simulation',  
        executable='twist_relay_node_exec',  
        name='twist_relay_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    pkg_nav2_bringup = get_package_share_directory('nav2_bringup')


    # Start navigation
    nav2_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2_bringup, "launch", "navigation_launch.py")
        ),
        launch_arguments=[
            ('map', ''),
            ("slam", "False"), 
            ('use_sim_time', use_sim_time),
            {'autostart', "True" },
            ('params_file',  str(params_file)),
            ('wait_for_transform',   'True'),   # <— Nav2 waits until map→base_link exists
            ('transform_timeout',    '3.0'),
            #('log_level', log_level),

        ]
    )

    return LaunchDescription(
        [
            declare_sim_arg,
            SetEnvironmentVariable(
                name='GZ_SIM_SYSTEM_PLUGIN_PATH',
                value='/root/ros2_ws/install/lib'
            ),
            SetEnvironmentVariable(
                name='IGN_GAZEBO_SYSTEM_PLUGIN_PATH',
                value='/root/ros2_ws/install/lib'
            ),
            
            map_server_node,
            map_server_lifecycle_node,
            twist_relay_node,
            nav2_bringup_launch
        ]
    )