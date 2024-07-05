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
            marker_id_and_bluetooth_mac_vec = os.environ.get('marker_id_and_bluetooth_mac').split(',')
            if marker_id_and_bluetooth_mac_vec == ['']:
                raise
        else:
            marker_id_and_bluetooth_mac_vec = ['0/94:C9:60:43:BE:01', '1/94:C9:60:43:BE:06']
    except:
        print("Please input aruco marker_id and bluetooth_mac !")
    
    size = 0.10
    try:
        if 'APRILTAG_SIZE' in os.environ:
            size = int(os.environ.get('marker_id_and_bluetooth_mac'))
        else:
            size = 0.10
            print("Using default apriltag size 0.10.")
    except:
        print("Please input APRILTAG_SIZE in docker-compose.yaml")
    
    
    apriltag_ros_extra_params = {
        'marker_id_and_bluetooth_mac_vec': marker_id_and_bluetooth_mac_vec,
        'size': size
    }    

    # get pkg path
    apriltag_pkg_path = get_package_share_directory('apriltag_ros')

    # get params file
    apriltag_node_params_file = os.path.join(apriltag_pkg_path, 'cfg', 'tags_36h11.yaml')

    # apriltag_ros node
    apriltag_ros_node = Node(
        executable='apriltag_node',
        package='apriltag_ros',
        name='apriltag_node',
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
