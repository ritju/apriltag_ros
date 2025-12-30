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
    
    size = 0.20
    try:
        if 'APRILTAG_CALIBRATION_SIZE' in os.environ:
            size = float(os.environ.get('APRILTAG_CALIBRATION_SIZE'))
            print(f'get apriltag_calibration size {size} from environment')
        else:
            size = 0.20
            print("Using default apriltag_calibration size 0.20.")
    except Exception as e:
        print(f'exception: {str(e)}')
        print("Please input APRILTAG_CALIBRATION_SIZE in environment")
    
    image_topic = '/camera2/color/image_raw'    
    try :
        if 'APRILTAG_CALIBRATION_IMAGE_TOPIC' in os.environ:
            image_topic = os.environ.get('APRILTAG_CALIBRATION_IMAGE_TOPIC')
            print(f'Get image_calibration_topic {image_topic} from environment')
        else:
            image_topic = "/camera2/color/image_raw"
            print(f'Using default calibration_image_topic /camera2/color/image_raw')
    except Exception as e:
        print(f'exception: {str(e)}')
        print("Please input APRILTAG_CALIBRATION_IMAGE_TOPIC in environment")

    camera_info_topic = '/camera2/color/camera_info'
    try :
        if 'APRILTAG_CALIBRATION_CAMERA_INFO_TOPIC' in os.environ:
            camera_info_topic = os.environ.get('APRILTAG_CALIBRATION_CAMERA_INFO_TOPIC')
            print(f'Get calibration_camera_info_topic {camera_info_topic} from environment')
        else:
            camera_info_topic = "/camera2/color/camera_info"
            print(f'Using default calibration_camera_info_topic /camera2/color/camera_info')
    except Exception as e:
        print(f'exception: {str(e)}')
        print("Please input APRILTAG_CALIBRATION_CAMERA_INFO_TOPIC in environment")

    
    apriltag_ros_extra_params = {
        'size': size,
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
        parameters=[apriltag_node_params_file, apriltag_ros_extra_params],
        remappings=[('/image_rect', image_topic),
                    ('/camera_info', camera_info_topic)]
    )

    return [calibration_node]


def generate_launch_description():
    
    ld = LaunchDescription()

    marker_translation_x_arg = DeclareLaunchArgument("marker_translation_x", default_value="2.0", description="x of Translation")
    marker_translation_y_arg = DeclareLaunchArgument("marker_translation_y", default_value="0.0", description="y of Translation")
    marker_translation_z_arg = DeclareLaunchArgument("marker_translation_z", default_value="0.0", description="z of Translation")

    marker_roll_arg = DeclareLaunchArgument("marker_roll", default_value="0.0", description="roll of Rotation")
    marker_pitch_arg = DeclareLaunchArgument("marker_pitch", default_value="0.0", description="roll of Rotation")
    marker_yaw_arg = DeclareLaunchArgument("marker_yaw", default_value="0.0", description="roll of Rotation")
    
    ld.add_action(marker_translation_x_arg)
    ld.add_action(marker_translation_y_arg)
    ld.add_action(marker_translation_z_arg)
    ld.add_action(marker_roll_arg)
    ld.add_action(marker_pitch_arg)
    ld.add_action(marker_yaw_arg)
    ld.add_action(OpaqueFunction(function=launch_setup))

    return ld
