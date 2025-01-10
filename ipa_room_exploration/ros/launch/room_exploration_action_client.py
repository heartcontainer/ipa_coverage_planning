from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    return LaunchDescription(
        [
            Node(
                package="ipa_room_exploration",
                executable="room_exploration_client",
                namespace="room_exploration",
                name="room_exploration_client",
                output="screen",
            ),
        ]
    )
