from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='v4l2_camera',
            executable='v4l2_camera_node',
            name='v4l2_camera_node',
            parameters=[{
                'image_size': [640, 480],
            }],
            remappings=[
                ('/image_raw', '/camera/image_raw')  # Raw image topic
            ]
        ),
        
        # Start an image transport node for compression
        Node(
            package='image_transport',
            executable='republish',
            name='image_compressor',
            arguments=['raw', 'compressed'],
            remappings=[
                ('in', '/camera/image_raw'),        # Input raw image
                ('out/compressed', '/camera/image_compressed')  # Output compressed image
            ]
        ),
    ])