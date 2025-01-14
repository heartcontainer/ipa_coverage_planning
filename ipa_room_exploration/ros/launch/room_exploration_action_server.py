from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    return LaunchDescription(
        [
            Node(
                package="ipa_room_exploration",
                executable="room_exploration_server",
                namespace="room_exploration",
                name="room_exploration_server",
                output="screen",
                respawn=True,
                respawn_delay=2.0,
            ),
        ]
    )
