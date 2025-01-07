from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Get the package share directory
    package_dir = get_package_share_directory("ipa_room_segmentation")
    default_map = os.path.join(package_dir, "common/files/test_maps/lab_ipa.png")

    return LaunchDescription(
        [
            Node(
                package="ipa_room_segmentation",
                executable="room_segmentation_client",
                name="room_segmentation_client",
                namespace="room_segmentation",
                parameters=[
                    {
                        "image_path": default_map,
                    },
                ],
                output="screen",
            ),
        ]
    )
