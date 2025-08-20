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
    log_level_arg = DeclareLaunchArgument('log_level', default_value='info', description='define apriltag_double node log level')

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

    radius_threshold = 0.013
    try :
        if 'MARKER_FRAME_RADIUS_THRESHOLD' in os.environ:
            radius_threshold = float(os.environ.get('MARKER_FRAME_RADIUS_THRESHOLD'))
            print(f'Get radius_threshold from docker-compose.yml')
        else:
            radius_threshold = 0.013
            print(f'Using default radius_threshold 0.013')
    except Exception as e:
        print(f'exception: {str(e)}')
        print("Please input MARKER_FRAME_RADIUS_THRESHOLD in docker-compose.yml")
        
    base_link_dummy_transform_x = -0.374
    try :
        if 'BASE_LINK_DUMMY_TRANSFORM_X' in os.environ:
            base_link_dummy_transform_x = float(os.environ.get('BASE_LINK_DUMMY_TRANSFORM_X'))
            print(f'Get base_link_dummy_transform_x {base_link_dummy_transform_x} from docker-compose.yml')
        else:
            base_link_dummy_transform_x = -0.374
            print(f'Using default base_link_dummy_transform_x -0.374')
    except Exception as e:
        print(f'exception: {str(e)}')
        print("Please input BASE_LINK_DUMMY_TRANSFORM_X in docker-compose.yml")
    
    base_link_dummy_transform_y = 0.0
    try :
        if 'BASE_LINK_DUMMY_TRANSFORM_Y' in os.environ:
            base_link_dummy_transform_y = float(os.environ.get('BASE_LINK_DUMMY_TRANSFORM_Y'))
            print(f'Get base_link_dummy_transform_y {base_link_dummy_transform_y} from docker-compose.yml')
        else:
            base_link_dummy_transform_y = 0.0
            print(f'Using default base_link_dummy_transform_y 0.0')
    except Exception as e:
        print(f'exception: {str(e)}')
        print("Please input BASE_LINK_DUMMY_TRANSFORM_Y in docker-compose.yml")

    base_link_dummy_transform_z = 0.50
    try :
        if 'BASE_LINK_DUMMY_TRANSFORM_Z' in os.environ:
            base_link_dummy_transform_z = float(os.environ.get('BASE_LINK_DUMMY_TRANSFORM_Z'))
            print(f'Get base_link_dummy_transform_z {base_link_dummy_transform_z} from docker-compose.yml')
        else:
            base_link_dummy_transform_z = 0.50
            print(f'Using default base_link_dummy_transform_Z 0.50')
    except Exception as e:
        print(f'exception: {str(e)}')
        print("Please input BASE_LINK_DUMMY_TRANSFORM_Z in docker-compose.yml")
    
    
    apriltag_ros_extra_params = {
        'marker_id_and_bluetooth_mac_vec': marker_id_and_bluetooth_mac_vec,
        'size': size,
        'marker_frame_translation': marker_frame_translation,
        'similarity_threshold': similarity_threshold,
        'radius_threshold': radius_threshold,
        'base_link_dummy_transform_x': base_link_dummy_transform_x,
        'base_link_dummy_transform_y': base_link_dummy_transform_y,
    }    

    # get pkg path
    apriltag_pkg_path = get_package_share_directory('apriltag_ros')

    # get params file
    apriltag_node_params_file = os.path.join(apriltag_pkg_path, 'cfg', 'tags_36h11.yaml')

    image_topic = '/camera3/color/image_raw'
    image_info_topic = '/camera3/color/camera_info'
    robot_version = 'real_robot_mk'
    try:
        if 'ROBOT_VERSION' in os.environ:
            robot_version = os.environ.get('ROBOT_VERSION')
            print(f'get ROBOT_VERSION {robot_version} from docker-compose.yaml file')
        else:
            robot_version = 'real_robot_mk.yaml'
            print("Using default robot_version real_robot_mk.")
    except Exception as e:
        print(f'exception: {str(e)}')
        print("Please input ROBOT_VERSION in docker-compose.yaml")
        robot_version = 'real_robot_mk.yaml'
    
    if robot_version == 'outdoor_cleaner_1':
        image_topic =      '/rgb_camera_back/image_raw'
        image_info_topic = '/rgb_camera_back/camera_info'

    # apriltag_ros node
    apriltag_ros_node = Node(
        executable='apriltag_double_node',
        package='apriltag_ros',
        name='apriltag_double_node',
        namespace='',
        output='screen',
        parameters=[apriltag_node_params_file, apriltag_ros_extra_params],
        remappings=[('/image_rect', image_topic),
                    ('/camera_info', image_info_topic)],
        arguments=['--ros-args', '--log-level', ['apriltag_double:=', LaunchConfiguration('log_level')]],
    )

    return [apriltag_ros_node]


def generate_launch_description():
    
    ld = LaunchDescription()

    ld.add_action(OpaqueFunction(function=launch_setup))
    # ld.add_action(log_level_arg)

    return ld
