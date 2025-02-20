from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "image_path",
                default_value="",
            ),
            DeclareLaunchArgument(
                "save_exploration_map",
                default_value="false",
            ),
            DeclareLaunchArgument(
                "room_exploration_algorithm",
                default_value="8",
                choices=["1", "2", "3", "4", "5", "6", "7", "8"],
                description="1.grid_point 2.boustrophedon 3.neural_network 4.convexSPP 5.flowNetwork 6. energyFunctional 7. voronoi 8. boustrophedon_variant",
            ),
            Node(
                package="ipa_room_exploration",
                executable="room_exploration_client",
                namespace="room_exploration",
                name="room_exploration_client",
                output="screen",
                parameters=[
                    {
                        "image_path": LaunchConfiguration("image_path"),
                        "save_exploration_map": LaunchConfiguration(
                            "save_exploration_map"
                        ),
                        "room_exploration_algorithm": LaunchConfiguration(
                            "room_exploration_algorithm"
                        ),
                    },
                ],
            ),
        ]
    )
