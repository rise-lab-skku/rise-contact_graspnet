from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
# from ament_index_python.packages import get_package_share_directory
import pathlib


def generate_launch_description():
    # parameters_file_path = str(pathlib.Path(__file__).parents[1]) # get current path and go one level up
    # parameters_file_path += '/launch/params.cfg'
    # print(parameters_file_path)
    return LaunchDescription([
        Node(
            package="rise_contact_graspnet2",
            executable="ros2_node",
            name="contact_graspnet_planner",
            output="screen",
            # parameters=[
            #     parameters_file_path
            # ]
        )
    ])



# def generate_launch_description():
#     ckpt_dir_name = "ckpt_dir"
#     z_min_name = "z_min"
#     z_max_name = "z_max"
#     local_regions_name = "local_regions"
#     filter_grasps_name = "filter_grasps"
#     skip_border_objects_name = "skip_border_objects"
#     forward_passes_name = "forward_passes"
#     segmap_id_name = "segmap_id"

#     ckpt_dir_param = LaunchConfiguration(ckpt_dir_name)
#     z_min_param = LaunchConfiguration(z_min_name)
#     z_max_param = LaunchConfiguration(z_max_name)
#     local_regions_param = LaunchConfiguration(local_regions_name)
#     filter_grasps_param = LaunchConfiguration(filter_grasps_name)
#     skip_border_objects_param = LaunchConfiguration(skip_border_objects_name)
#     forward_passes_param = LaunchConfiguration(forward_passes_name)
#     segmap_id_param = LaunchConfiguration(segmap_id_name)

#     ckpt_dir_arg = DeclareLaunchArgument(
#         ckpt_dir_name,
#         default_value="$(find contact_graspnet_planner)/checkpoints/scene_test_2048_bs3_hor_sigma_001",
#         description="Log dir [default: checkpoints/scene_test_2048_bs3_hor_sigma_001]",
#     )
#     z_min_arg = DeclareLaunchArgument(
#         z_min_name,
#         default_value="0.2",
#         description="Z min value threshold to crop the input point cloud",
#     )
#     z_max_arg = DeclareLaunchArgument(
#         z_max_name,
#         default_value="1.8",
#         description="Z max value threshold to crop the input point cloud",
#     )
#     local_regions_arg = DeclareLaunchArgument(
#         local_regions_name,
#         default_value="False",
#         description="Crop 3D local regions around given segments.",
#     )
#     filter_grasps_arg = DeclareLaunchArgument(
#         filter_grasps_name,
#         default_value="False",
#         description="Filter grasp contacts according to segmap.",
#     )
#     skip_border_objects_arg = DeclareLaunchArgument(
#         skip_border_objects_name,
#         default_value="False",
#         description="When extracting local_regions, ignore segments at depth map boundary.",
#     )
#     forward_passes_arg = DeclareLaunchArgument(
#         forward_passes_name,
#         default_value="1",
#         description="Run multiple parallel forward passes to mesh_utils more potential contact points.",
#     )
#     segmap_id_arg = DeclareLaunchArgument(
#         segmap_id_name,
#         default_value="0",
#         description="Only return grasps of the given object id",
#     )
#     return LaunchDescription(
#         [
#             ckpt_dir_arg,
#             z_min_arg,
#             z_max_arg,
#             local_regions_arg,
#             filter_grasps_arg,
#             skip_border_objects_arg,
#             forward_passes_arg,
#             segmap_id_arg,
#             Node(
#                 package="rise_contact_graspnet2",
#                 executable="ros2_node",
#                 name="contact_graspnet_planner",
#                 output="screen",
#                 parameters=[
#                     {'ckpt_dir': ckpt_dir_param},
#                     {'z_min': z_min_param},
#                     {'z_max': z_max_param},
#                     {'local_regions': local_regions_param},
#                     {'filter_grasps': filter_grasps_param},
#                     {'skip_border_objects': skip_border_objects_param},
#                     {'forward_passes': forward_passes_param},
#                     {'segmap_id': segmap_id_param},
#                 ]
#                 # remappings=[
#                 #     ("/input/pose", "/turtlesim1/turtle1/pose"),
#                 #     ("/output/cmd_vel", "/turtlesim2/turtle1/cmd_vel"),
#                 # ],
#             ),
#         ]
#     )
