from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="ground_agent_nodes",
                executable="mission_handler_node_exec",
                name="mission_handler_node",
                output="screen"
            ),

            Node(
                package="ground_agent_nodes",
                executable="arrival_task_node_exec",
                name="arrival_task_node",
                output="screen"
            ),

            Node(
                package="ground_agent_nodes",
                executable="dust_plume_actions_node_exec",
                name="dust_plume_actions_node",
                output="screen"
            ),

            Node(
                package="ai_python",
                executable="density_regressor_node",
                name="density_regressor_node",
                output="screen"
            ),
        ]
    )