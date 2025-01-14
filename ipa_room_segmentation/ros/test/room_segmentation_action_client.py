from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # Get the package share directory
    package_dir = get_package_share_directory("ipa_room_segmentation")
    default_map = os.path.join(package_dir, "common/files/test_maps/lab_ipa.png")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "room_segmentation_algorithm",
                default_value="3",
                choices=["1", "2", "3", "4", "5"],
                description='1.morphological 2.distance 3.Voronoi 4.semantic/feature-based 5.voronoi-random-field',
            ),
            Node(
                package="ipa_room_segmentation",
                executable="room_segmentation_client",
                name="room_segmentation_client",
                namespace="room_segmentation",
                parameters=[
                    {
                        "image_path": default_map,
                        "room_segmentation_algorithm": LaunchConfiguration(
                            "room_segmentation_algorithm"
                        ),
                    },
                ],
                output="screen",
            ),
        ]
    )
