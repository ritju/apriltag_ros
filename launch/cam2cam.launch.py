import os
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.substitutions import TextSubstitution
from nav2_common.launch import RewrittenYaml

def launch_setup(context, *args, **kwargs):   
    
        
    apriltag_ros_extra_params = {
        'size': LaunchConfiguration("size"),
        'image_ref_topic': LaunchConfiguration('image_ref_topic'),
        'camera_info_ref_topic': LaunchConfiguration('camera_info_ref_topic'),
        'image_source_topic': LaunchConfiguration('image_source_topic'),
        'camera_info_source_topic': LaunchConfiguration('camera_info_source_topic'),
        'calibration_id': LaunchConfiguration('calibration_id'),
        'base_frame_id': LaunchConfiguration('base_frame_id'),
        "sync_size": LaunchConfiguration('sync_size'), 
        'target_camera_type': LaunchConfiguration('target_camera_type'),
    }    

    # get pkg path
    apriltag_pkg_path = get_package_share_directory('apriltag_ros')

    # get params file
    apriltag_node_params_file = os.path.join(apriltag_pkg_path, 'cfg', 'tags_36h11.yaml')

    # cam2cam node
    cam2cam_node = Node(
        executable='cam2cam_node',
        package='apriltag_ros',
        name='cam2cam_node',
        namespace='',
        output='screen',
        parameters=[apriltag_node_params_file, apriltag_ros_extra_params]
    )

    return [cam2cam_node]

def generate_launch_description():
    
    ld = LaunchDescription()

    size_arg = DeclareLaunchArgument("size", default_value="0.24", description="calibration marker size")
    image_ref_topic_arg = DeclareLaunchArgument("image_ref_topic", default_value="/rgb_camera_front/image_raw", description="image ref topic")
    camera_info_ref_topic_arg = DeclareLaunchArgument("camera_info_ref_topic", default_value="/rgb_camera_front/camera_info", description="camera_info_ref topic")
    image_source_topic_arg = DeclareLaunchArgument("image_source_topic", default_value="/camera2/color/image_raw", description="image source topic")
    camera_info_source_topic_arg = DeclareLaunchArgument("camera_info_source_topic", default_value="/camera2/color/camera_info", description="camera_info_source topic")
    base_frame_id_arg = DeclareLaunchArgument("base_frame_id", default_value="base_link", description="base frame id")
    calibration_id_arg = DeclareLaunchArgument("calibration_id", default_value="0", description="april tag id")
    sync_size_arg = DeclareLaunchArgument("sync_size", default_value="2", description="sync size")
    target_camera_type_arg =  DeclareLaunchArgument("target_camera_type", default_value="rgb", description="type of target camera")

    ld.add_action(size_arg)
    ld.add_action(image_ref_topic_arg)
    ld.add_action(camera_info_ref_topic_arg)
    ld.add_action(image_source_topic_arg)
    ld.add_action(camera_info_source_topic_arg)
    ld.add_action(base_frame_id_arg)
    ld.add_action(calibration_id_arg) 
    ld.add_action(sync_size_arg) 
    ld.add_action(target_camera_type_arg) 

    ld.add_action(OpaqueFunction(function=launch_setup))

    return ld
