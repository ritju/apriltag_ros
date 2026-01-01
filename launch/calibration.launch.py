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
        'calibration_id': LaunchConfiguration('calibration_id'),
        'image_topic': LaunchConfiguration('image_topic'),
        'camera_info_topic': LaunchConfiguration('camera_info_topic'),
        'marker_translation_x': LaunchConfiguration("marker_translation_x"),
        'marker_translation_y': LaunchConfiguration("marker_translation_y"),
        'marker_translation_z': LaunchConfiguration("marker_translation_z"),
        'marker_roll': LaunchConfiguration("marker_roll"),
        'marker_pitch': LaunchConfiguration("marker_pitch"),
        'marker_yaw': LaunchConfiguration("marker_yaw"),
    }    

    # get pkg path
    apriltag_pkg_path = get_package_share_directory('apriltag_ros')

    # get params file
    apriltag_node_params_file = os.path.join(apriltag_pkg_path, 'cfg', 'tags_36h11.yaml')

    # calibration node
    calibration_node = Node(
        executable='calibration_node',
        package='apriltag_ros',
        name='calibration_node',
        namespace='',
        output='screen',
        parameters=[apriltag_node_params_file, apriltag_ros_extra_params]
    )

    return [calibration_node]


def generate_launch_description():
    
    ld = LaunchDescription()

    size_arg = DeclareLaunchArgument("size", default_value="0.2", description="calibration marker size")
    image_topic_arg = DeclareLaunchArgument("image_topic", default_value="/camera/color/image_raw", description="image topic")
    camera_info_topic_arg = DeclareLaunchArgument("camera_info_topic", default_value="/camera/color/camera_info", description="camera info topic")
    calibration_id_arg = DeclareLaunchArgument("calibration_id", default_value="0", description="calibration marker id")

    marker_translation_x_arg = DeclareLaunchArgument("marker_translation_x", default_value="2.0", description="x of Translation")
    marker_translation_y_arg = DeclareLaunchArgument("marker_translation_y", default_value="0.0", description="y of Translation")
    marker_translation_z_arg = DeclareLaunchArgument("marker_translation_z", default_value="0.0", description="z of Translation")

    marker_roll_arg = DeclareLaunchArgument("marker_roll", default_value="0.0", description="roll of Rotation")
    marker_pitch_arg = DeclareLaunchArgument("marker_pitch", default_value="0.0", description="roll of Rotation")
    marker_yaw_arg = DeclareLaunchArgument("marker_yaw", default_value="0.0", description="roll of Rotation")

    ld.add_action(size_arg)
    ld.add_action(image_topic_arg)
    ld.add_action(camera_info_topic_arg)
    ld.add_action(calibration_id_arg)    

    ld.add_action(marker_translation_x_arg)
    ld.add_action(marker_translation_y_arg)
    ld.add_action(marker_translation_z_arg)
    ld.add_action(marker_roll_arg)
    ld.add_action(marker_pitch_arg)
    ld.add_action(marker_yaw_arg)
    ld.add_action(OpaqueFunction(function=launch_setup))

    return ld
