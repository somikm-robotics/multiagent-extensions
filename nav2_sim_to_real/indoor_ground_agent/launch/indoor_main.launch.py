from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command, PathJoinSubstitution, FindExecutable, EnvironmentVariable
from launch_ros.substitutions import FindPackageShare
from launch.actions import SetEnvironmentVariable
from launch.event_handlers import OnProcessExit
from launch.actions import RegisterEventHandler
from launch_ros.parameter_descriptions import ParameterValue
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():

    ground_pkg = FindPackageShare('indoor_ground_agent')

    models_path = PathJoinSubstitution([ground_pkg, 'models'])
    
    set_gz_env = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=[models_path, ':',
            EnvironmentVariable('GZ_SIM_RESOURCE_PATH',
                                default_value='')]
    )

    rviz_config = PathJoinSubstitution([ground_pkg, 'visualisations', 'indoor_ground_agent.rviz']
    )

    # --- Agilex 4 robot description (URDF via xacro) ---
    urdf_name = "agilex.urdf"
    
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name='xacro')]),
            ' ',
            PathJoinSubstitution(
                [ground_pkg, 'urdfs', urdf_name]             
            ),
        ]
    )

    robot_description = {
    'robot_description': ParameterValue(robot_description_content, value_type=str)
    }

    
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[robot_description,
                    {'frame_prefix': 'agilex_diff_drive/'},
                    {'use_sim_time': True},
                    {'tf_buffer_duration': 30.0} 
                    ],
        output='screen'
    )

    # ---  custom world ---
    world_file = PathJoinSubstitution([
        FindPackageShare('indoor_ground_agent'),
        'worlds',
        'indoor_world.sdf'
    ])

    spawn_agilex = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', 'robot_description',
                '-name', 'agilex_diff_drive',
                '-x', '-1',
                '-y', '0',
                '-z', '0.05',
                '-Y', '0.0',          # 👈 yaw in radians
                '-allow_renaming', 'true'],
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    load_joint_state_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
            'joint_state_broadcaster'],
        output='screen'
    )

    load_joint_trajectory_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
            'diff_drive_base_controller'],
        output='screen'
    )

    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    agilex_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
                '/agilex/imu/data@sensor_msgs/msg/Imu[ignition.msgs.IMU',
                '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
                
                # Camera image
                '/world/indoor_world/model/agilex_diff_drive/link/base_link/sensor/rgb_cam/image@sensor_msgs/msg/Image[gz.msgs.Image',

                # Camera info
                '/world/indoor_world/model/agilex_diff_drive/link/base_link/sensor/rgb_cam/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
                '/world/indoor_world/create@ros_gz_interfaces/srv/SpawnEntity',
                ],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    # static tf
    static_tf_prefixed_lidar = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_prefixed_lidar',
        arguments=['0','0','0','0','0','0',
                'agilex_diff_drive/base_link','agilex_diff_drive/base_link/gpu_lidar'],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare('nav2_bringup'), 'launch', 'navigation_launch.py']
            )
        ),
        launch_arguments={
            'use_sim_time': 'true',
            'map': '/root/ros2_ws/src/indoor_ground_agent/maps/indoor_map.yaml',
            'params_file': '/root/ros2_ws/src/indoor_ground_agent/config/nav2_params.yaml'
        }.items()
    )


    return LaunchDescription([

         #  Make Ignition aware of agilex meshes
         set_gz_env,
        
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [PathJoinSubstitution([FindPackageShare('ros_gz_sim'),
                                       'launch', 'gz_sim.launch.py']
                                       )]
            ),
            launch_arguments={
                'gz_args': [
                    '-r -v 1 ',
                    world_file
                ]}.items()
        ),

        clock_bridge,

        # 2. Publish robot_description and TF
        robot_state_publisher,

        # 3. Spawn Agilex into Ignition
        spawn_agilex,

        agilex_bridge,

        # 4. Spawn JSB & DDC (for robot movement)
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_agilex,
                on_exit=[load_joint_state_controller],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_joint_state_controller,
                on_exit=[load_joint_trajectory_controller],
            )
        ),

        # static_tf_prefixed_base,
        static_tf_prefixed_lidar,

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config,
                    '--ros-args', '--log-level', 'warn'],
            parameters=[{'use_sim_time': True}],
            output='screen',
    )
    ])
