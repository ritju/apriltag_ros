import os
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.substitutions import TextSubstitution
from nav2_common.launch import RewrittenYaml

def generate_launch_description():
    
    launch_description = LaunchDescription()

    # get pkg path
    camera_pkg_path = get_package_share_directory('astra_camera')
    apriltag_pkg_path = get_package_share_directory('apriltag_ros')

    apriltag_node_params_file = os.path.join(apriltag_pkg_path, 'cfg', 'tags_36h11.yaml')

    # camera(orbbec dabai_dcw) launch file
    camera_launch_file = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(camera_pkg_path, 'launch', 'multi_dabai_dcw.launch.py'))
    )

    # manual dock node
    apriltag_node = Node(
        executable='apriltag_node',
        package='apriltag_ros',
        name='apriltag_node',
        namespace='',
        output='screen',
        parameters=[apriltag_node_params_file],
        remappings=[('/image_rect', '/camera3/color/image_raw'),
                    ('/camera_info', '/camera3/color/camera_info')]
    )

#     launch_description.add_action(camera_launch_file)
    launch_description.add_action(apriltag_node)



    return launch_description
