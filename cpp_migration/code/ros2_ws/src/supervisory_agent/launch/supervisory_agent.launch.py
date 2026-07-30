from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="supervisory_agent",
                executable="alert_receiver_node_exec",
                name="alert_receiver_node",
                output="screen"
            ),
            Node(
                package="supervisory_agent",
                executable="mission_overrrider_node_exec",
                name="mission_overrrider_node",
                output="screen"
            ),
            Node(
                package="supervisory_agent",
                executable="test_alerts_node_exec",
                name="test_alerts_node",
                output="screen"
            ),
            Node(
                package="supervisory_agent",
                executable="supervisory_agent_web_server_exec",
                name="supervisory_agent_web_server",
                output="screen"
            ),
        ]
    )