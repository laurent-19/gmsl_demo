import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    # Your camera namespace
    camera_names = ['camera5004', 'camera5005', 'camera5006', 'camera5007']
    
    # Location of configuration directory
    config_dir = os.path.join(get_package_share_directory('gscam2'), 'cfg')

    # Parameters file
    params_file = os.path.join(config_dir, 'params.yaml')

    # Camera calibration file
    camera_config = 'file://' + os.path.join(config_dir, 'my_camera.ini')

    nodes = []
    for index, camera_name in enumerate(camera_names):
        if index == 0 : # just for webcamera cause it's only one
            node = Node(
            package='gscam2',
            executable='gscam_main',
            output='screen',
            name='gscam_publisher_' + str(index),
            namespace=camera_name,
            parameters=[
                {
                    'camera_name': camera_name,  # Camera Name
                    'camera_info_url': camera_config,  # Camera calibration information
                    'preroll' : False,
                    'use_gst_timestamps' : False,
                    #'gscam_config': f'udpsrc caps=application/x-rtp, sampling=YCbCr-4:2:2, depth=8, width=1920, height=1080 port={5004 + index} ! rtpvrawdepay ! videoconvert ! fpsdisplaysink video-sink=xvimagesink text-overlay=true sync=false'
                    'gscam_config' : 'v4l2src device=/dev/video0 ! videoconvert', # just for webcam, use above line for gmsl cams
                    'frame_id' : 'camera' + str(5004 + index) + '_frame'
                },
            ],
            # Remap outputs to the correct namespace
            remappings=[
                ('/image_raw', '/' + camera_name + '/image_raw'),
                ('/camera_info', '/' + camera_name + '/camera_info'),
            ],
            )
            nodes.append(node)

    return LaunchDescription(nodes)
