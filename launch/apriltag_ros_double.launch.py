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
    
    marker_id_and_bluetooth_mac_vec = ['']
    try:
        if 'marker_id_and_bluetooth_mac' in os.environ:
            print('get marker_id_and_bluetooth_mac from env.')
            marker_id_and_bluetooth_mac_vec = os.environ.get('marker_id_and_bluetooth_mac').split(',')
            if marker_id_and_bluetooth_mac_vec == ['']:
                raise
        else:
            print('using default marker_id_and_bluetooth_mac')
            marker_id_and_bluetooth_mac_vec = ['0:1/94:C9:60:43:BE:01', '2:3/94:C9:60:43:BE:02']
    except:
        print("Please input aruco marker_id and bluetooth_mac !")
    
    size = 0.10
    try:
        if 'APRILTAG_SIZE' in os.environ:
            size = float(os.environ.get('APRILTAG_SIZE'))
            print(f'Get apriltag size {size} from docker-compose.yaml file')
        else:
            size = 0.10
            print("Using default apriltag size 0.10.")
    except Exception as e:
        print(f'exception: {str(e)}')
        print("Please input APRILTAG_SIZE in docker-compose.yaml")
    
    marker_frame_translation = -0.04
    try :
        if 'MARKER_FRAME_TRANSLATION' in os.environ:
            marker_frame_translation = float(os.environ.get('MARKER_FRAME_TRANSLATION'))
            print(f'Get marker_frame_translation from docker-compose.yml')
        else:
            marker_frame_translation = -0.04
            print(f'Using default marker_frame_translation -0.04')
    except Exception as e:
        print(f'exception: {str(e)}')
        print("Please input MARKER_FRAME_TRANSLATION in docker-compose.yml")
    
    similarity_threshold = 0.96
    try :
        if 'SIMILARITY_THRESHOLD' in os.environ:
            similarity_threshold = float(os.environ.get('SIMILARITY_THRESHOLD'))
            print(f'Get similarity_threshold from docker-compose.yml')
        else:
            similarity_threshold = 0.96
            print(f'Using default similarity_threshold 0.96')
    except Exception as e:
        print(f'exception: {str(e)}')
        print("Please input SIMILARITY_THRESHOLD in docker-compose.yml")
    
    
    apriltag_ros_extra_params = {
        'marker_id_and_bluetooth_mac_vec': marker_id_and_bluetooth_mac_vec,
        'size': size,
        'marker_frame_translation': marker_frame_translation,
        'similarity_threshold': similarity_threshold,
    }    

    # get pkg path
    apriltag_pkg_path = get_package_share_directory('apriltag_ros')

    # get params file
    apriltag_node_params_file = os.path.join(apriltag_pkg_path, 'cfg', 'tags_36h11.yaml')

    # apriltag_ros node
    apriltag_ros_node = Node(
        executable='apriltag_double_node',
        package='apriltag_ros',
        name='apriltag_double_node',
        namespace='',
        output='screen',
        parameters=[apriltag_node_params_file, apriltag_ros_extra_params],
        remappings=[('/image_rect', '/camera3/color/image_raw'),
                    ('/camera_info', '/camera3/color/camera_info')]
    )

    return [apriltag_ros_node]


def generate_launch_description():
    
    ld = LaunchDescription()

    ld.add_action(OpaqueFunction(function=launch_setup))

    return ld
