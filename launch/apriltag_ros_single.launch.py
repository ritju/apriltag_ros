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
        'image_topic': LaunchConfiguration('image_topic'),
        'camera_info_topic': LaunchConfiguration('camera_info_topic'),
        'calibration_id': LaunchConfiguration('calibration_id'),
        'base_frame_id': LaunchConfiguration('base_frame_id'),
        'boader_height': LaunchConfiguration('boader_height'),
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
        parameters=[apriltag_node_params_file, apriltag_ros_extra_params],
        arguments=['--ros-args', '--log-level', ['cam2cam_node:=', LaunchConfiguration("log_level")]],
    )

    return [cam2cam_node]

def generate_launch_description():
    
    ld = LaunchDescription()

    size_arg = DeclareLaunchArgument("size", default_value="0.24", description="calibration marker size")
    image_topic_arg = DeclareLaunchArgument("image_topic", default_value="/rgb_camera_front/image_raw", description="image ref topic")
    camera_info_topic_arg = DeclareLaunchArgument("camera_info_topic", default_value="/rgb_camera_front/camera_info", description="camera_info_ref topic")
    calibration_id_arg = DeclareLaunchArgument("calibration_id", default_value="0", description="april tag id")
    boader_height_arg = DeclareLaunchArgument('boader_height', default_value="0.01", description="height of boader") 
    log_level_arg =  DeclareLaunchArgument("log_level", default_value="info", description="log level of this node")

    ld.add_action(size_arg)
    ld.add_action(image_topic_arg)
    ld.add_action(camera_info_topic_arg)
    ld.add_action(base_frame_id_arg)
    ld.add_action(calibration_id_arg) 
    ld.add_action(boader_height_arg) 
    ld.add_action(log_level_arg) 

    ld.add_action(OpaqueFunction(function=launch_setup))

    return ld
