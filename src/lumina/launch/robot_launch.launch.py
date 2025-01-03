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
        
        # Start the Lidar
        Node(
            package='rplidar_ros',
            executable='rplidar_composition',
            output='screen',
            parameters=[{
                'serial_port': '/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0',
                'frame_id': 'laser_frame',
                'angle_compensate': True,
                'scan_mode': 'Standard',
                'serial_baudrate': 115200
            }]
        )
    ])