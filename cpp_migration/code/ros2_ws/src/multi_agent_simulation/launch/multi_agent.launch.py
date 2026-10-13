from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.actions import RegisterEventHandler
from launch.actions import SetEnvironmentVariable, ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution, EnvironmentVariable, TextSubstitution
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.actions import Node
from launch.event_handlers import OnProcessExit
from multi_agent_simulation.utils import get_agilex_base_pose

def generate_launch_description():
    ground_agent_pkg = FindPackageShare('ground_agent_nodes')
    curr_pkg = FindPackageShare('multi_agent_simulation')

    set_env = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=[
            # multi_agent simulation (shared) models
            PathJoinSubstitution([curr_pkg, 'models']),
            TextSubstitution(text=':'),
            # multi_agent (hazard overlays) models
            PathJoinSubstitution([curr_pkg, 'models', 'hazard_overlays']),
            TextSubstitution(text=':'),
            # multi_agent (obstacles) models
            PathJoinSubstitution([curr_pkg, 'models', 'obstacles']),
            TextSubstitution(text=':'),
            # ← don’t crash if the variable isn’t set yet
            EnvironmentVariable('GZ_SIM_RESOURCE_PATH', default_value=''),
        ],
    )
    gz_sim_include = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [PathJoinSubstitution([FindPackageShare('ros_gz_sim'),
                                    'launch', 'gz_sim.launch.py'])]
            ),
            launch_arguments=[('gz_args', [' -r -v 1 ',
            PathJoinSubstitution([curr_pkg,
                        'worlds', 'mining_world.sdf'])
                        ],
    )])

    # --- Agilex 4 robot description (URDF via xacro) ---
    urdf_name = "agilex.urdf"
    
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name='xacro')]),
            ' ',
            PathJoinSubstitution(
                [curr_pkg, 'urdf', urdf_name]             
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

    xyzY = get_agilex_base_pose()
    spawn_agilex = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', 'robot_description',
                '-name', 'agilex_diff_drive',
                '-x', str(xyzY[0]),
                '-y', str(xyzY[1]),
                '-z', str(xyzY[2]),
                '-Y', str(xyzY[3]),          # 👈 yaw in radians
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

    gnss_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
                '/agilex/imu/data@sensor_msgs/msg/Imu[ignition.msgs.IMU',
                '/gps/fix@sensor_msgs/msg/NavSatFix[gz.msgs.NavSat',
                ],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    camera_bridge = Node(
    package='ros_gz_bridge',
    executable='parameter_bridge',
    arguments=[
                # Camera image
                '/world/mining_world/model/agilex_diff_drive/link/base_link/sensor/rgb_cam/image@sensor_msgs/msg/Image[gz.msgs.Image',

                # Camera info
                '/world/mining_world/model/agilex_diff_drive/link/base_link/sensor/rgb_cam/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',

        ],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    scan_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan'],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    

    rviz_config = PathJoinSubstitution([curr_pkg, 'visualisation', 'multi_agent.rviz']
    )
    rviz2_node = Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config,
                    '--ros-args', '--log-level', 'warn'],
            parameters=[{'use_sim_time': True}],
            output='screen',
    )


    return LaunchDescription(
        [
            # core stuff 
            set_env,
            gz_sim_include,

           clock_bridge,

        # 2. Publish robot_description and TF
        robot_state_publisher,

        # 3. Spawn Agilex into Ignition
        spawn_agilex,

        gnss_bridge,

        camera_bridge,

        scan_bridge,


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

        # Launch agents
    
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([ground_agent_pkg, 'launch', 'ground_agents_nodes.launch.py'])
            )
        ),

        ]
    )

    