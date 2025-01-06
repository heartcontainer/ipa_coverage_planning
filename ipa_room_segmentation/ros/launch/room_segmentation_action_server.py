from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get the package share directory
    package_dir = get_package_share_directory('ipa_room_segmentation')

    # Parameters
    room_segmentation_params = os.path.join(package_dir, 'ros/launch/room_segmentation_action_server_params.yaml')

    semantic_training_maps_room_file_list = [
        os.path.join(package_dir, 'common/files/training_maps/lab_ipa_room_training_map.png'),
        os.path.join(package_dir, 'common/files/training_maps/lab_d_room_training_map.png'),
        os.path.join(package_dir, 'common/files/training_maps/Freiburg52_scan_room_training.png'),
        os.path.join(package_dir, 'common/files/training_maps/Freiburg52_scan_furnitures_room_training.png'),
        os.path.join(package_dir, 'common/files/training_maps/lab_intel_furnitures_room_training_map.png')
    ]

    semantic_training_maps_hallway_file_list = [
        os.path.join(package_dir, 'common/files/training_maps/lab_ipa_hallway_training_map.png'),
        os.path.join(package_dir, 'common/files/training_maps/lab_a_hallway_training_map.png'),
        os.path.join(package_dir, 'common/files/training_maps/Freiburg52_scan_hallway_training.png'),
        os.path.join(package_dir, 'common/files/training_maps/Freiburg52_scan_furnitures_hallway_training.png'),
        os.path.join(package_dir, 'common/files/training_maps/lab_intel_hallway_training_map.png')
    ]

    vrf_original_maps_file_list = [
        os.path.join(package_dir, 'common/files/training_maps/voronoi_random_field_training/original_maps/Fr52_original.png'),
        os.path.join(package_dir, 'common/files/training_maps/voronoi_random_field_training/original_maps/Fr101_original.png'),
        os.path.join(package_dir, 'common/files/training_maps/voronoi_random_field_training/original_maps/lab_intel_original.png'),
        os.path.join(package_dir, 'common/files/training_maps/voronoi_random_field_training/original_maps/lab_d_furnitures_original.png'),
        os.path.join(package_dir, 'common/files/training_maps/voronoi_random_field_training/original_maps/lab_ipa_original.png'),
        os.path.join(package_dir, 'common/files/training_maps/voronoi_random_field_training/original_maps/NLB_original.png'),
        os.path.join(package_dir, 'common/files/training_maps/voronoi_random_field_training/original_maps/office_e_original.png'),
        os.path.join(package_dir, 'common/files/training_maps/voronoi_random_field_training/original_maps/office_h_original.png'),
        os.path.join(package_dir, 'common/files/training_maps/voronoi_random_field_training/original_maps/lab_c_furnitures_original.png')
    ]

    vrf_training_maps_file_list = [
        os.path.join(package_dir, 'common/files/training_maps/voronoi_random_field_training/training_maps/training_Fr52.png'),
        os.path.join(package_dir, 'common/files/training_maps/voronoi_random_field_training/training_maps/training_Fr101.png'),
        os.path.join(package_dir, 'common/files/training_maps/voronoi_random_field_training/training_maps/training_intel.png'),
        os.path.join(package_dir, 'common/files/training_maps/voronoi_random_field_training/training_maps/training_lab_d_furniture.png'),
        os.path.join(package_dir, 'common/files/training_maps/voronoi_random_field_training/training_maps/training_lab_ipa.png'),
        os.path.join(package_dir, 'common/files/training_maps/voronoi_random_field_training/training_maps/training_NLB_furniture.png'),
        os.path.join(package_dir, 'common/files/training_maps/voronoi_random_field_training/training_maps/training_office_e.png'),
        os.path.join(package_dir, 'common/files/training_maps/voronoi_random_field_training/training_maps/training_office_h.png'),
        os.path.join(package_dir, 'common/files/training_maps/voronoi_random_field_training/training_maps/training_lab_c_furnitures.png')
    ]

    vrf_voronoi_maps_file_list = [
        os.path.join(package_dir, 'common/files/training_maps/voronoi_random_field_training/voronoi_maps/Fr52_voronoi.png'),
        os.path.join(package_dir, 'common/files/training_maps/voronoi_random_field_training/voronoi_maps/Fr101_voronoi.png'),
        os.path.join(package_dir, 'common/files/training_maps/voronoi_random_field_training/voronoi_maps/lab_intel_voronoi.png'),
        os.path.join(package_dir, 'common/files/training_maps/voronoi_random_field_training/voronoi_maps/lab_d_furnitures_voronoi.png'),
        os.path.join(package_dir, 'common/files/training_maps/voronoi_random_field_training/voronoi_maps/lab_ipa_voronoi.png'),
        os.path.join(package_dir, 'common/files/training_maps/voronoi_random_field_training/voronoi_maps/NLB_voronoi.png'),
        os.path.join(package_dir, 'common/files/training_maps/voronoi_random_field_training/voronoi_maps/office_e_voronoi.png'),
        os.path.join(package_dir, 'common/files/training_maps/voronoi_random_field_training/voronoi_maps/office_h_voronoi.png'),
        os.path.join(package_dir, 'common/files/training_maps/voronoi_random_field_training/voronoi_maps/lab_c_furnitures_voronoi.png')
    ]

    vrf_voronoi_node_maps_file_list = [
        os.path.join(package_dir, 'common/files/training_maps/voronoi_random_field_training/voronoi_node_maps/Fr52_voronoi_nodes.png'),
        os.path.join(package_dir, 'common/files/training_maps/voronoi_random_field_training/voronoi_node_maps/Fr101_voronoi_nodes.png'),
        os.path.join(package_dir, 'common/files/training_maps/voronoi_random_field_training/voronoi_node_maps/lab_intel_voronoi_nodes.png'),
        os.path.join(package_dir, 'common/files/training_maps/voronoi_random_field_training/voronoi_node_maps/lab_d_furnitures_voronoi_nodes.png'),
        os.path.join(package_dir, 'common/files/training_maps/voronoi_random_field_training/voronoi_node_maps/lab_ipa_voronoi_nodes.png'),
        os.path.join(package_dir, 'common/files/training_maps/voronoi_random_field_training/voronoi_node_maps/NLB_voronoi_nodes.png'),
        os.path.join(package_dir, 'common/files/training_maps/voronoi_random_field_training/voronoi_node_maps/office_e_voronoi_nodes.png'),
        os.path.join(package_dir, 'common/files/training_maps/voronoi_random_field_training/voronoi_node_maps/office_h_voronoi_nodes.png'),
        os.path.join(package_dir, 'common/files/training_maps/voronoi_random_field_training/voronoi_node_maps/lab_c_furnitures_voronoi_nodes.png')
    ]

    return LaunchDescription(
        [
            Node(
                package="ipa_room_segmentation",
                executable="room_segmentation_server",
                name="room_segmentation_server",
                namespace="room_segmentation",
                output="screen",
                parameters=[
                    room_segmentation_params,
                    {
                        "semantic_training_maps_room_file_list": semantic_training_maps_room_file_list
                    },
                    {
                        "semantic_training_maps_hallway_file_list": semantic_training_maps_hallway_file_list
                    },
                    {"vrf_original_maps_file_list": vrf_original_maps_file_list},
                    {"vrf_training_maps_file_list": vrf_training_maps_file_list},
                    {"vrf_voronoi_maps_file_list": vrf_voronoi_maps_file_list},
                    {
                        "vrf_voronoi_node_maps_file_list": vrf_voronoi_node_maps_file_list
                    },
                ],
                # emulate_tty=True,
                respawn=True,
                respawn_delay=2.0,
            ),
        ]
    )
