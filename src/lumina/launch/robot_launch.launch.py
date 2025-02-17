import os
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import xacro

def generate_launch_description():
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "gui",
            default_value="false",
            description="Start Rviz2 automatically with this launch file"
        )
    )
    
    declared_arguments.append(
        DeclareLaunchArgument(
            "sim_mode",
            default_value="false",
            description="Start robot with mock hardware mirroring command to its states.",
        )
    )
    
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Use sim time. (Important when using mock hardware)",
        )
    )
    
    gui = LaunchConfiguration("gui")
    sim_mode = LaunchConfiguration("sim_mode")
    use_sim_time = LaunchConfiguration("use_sim_time")

        
    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("lumina"),
            "config",
            "my_controllers.yaml"
        ]
    )
    
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_controllers],
        output="both",
        remappings=[
            ("~/robot_description", "/robot_description"),
            ("/diffbot_base_controller/cmd_vel", "/cmd_vel"),
        ],
    )
    
    pkg_path = os.path.join(get_package_share_directory('lumina'))
    xacro_file = os.path.join(pkg_path, 'description', 'robot.urdf.xacro')

    # Process the xacro file into a string
    robot_description_config = xacro.process_file(
        xacro_file, 
        mappings={'use_ros2_control': 'true', 'sim_mode': 'false'}  # You can dynamically set these if needed
    ).toxml()
    params = {'robot_description': robot_description_config, 'use_sim_time': use_sim_time}

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='both',
        parameters=[params]
    )
    
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        # arguments=["-d, rviz_config_file"]
        condition=IfCondition(gui),
    )
    
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"]
    )
    
    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diffbot_base_controller", "--controller-manager", "/controller_manager"],
    )
    
    # Delay rviz start until after `joint_state_broadcaster`
    delay_rviz_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit = [rviz_node]
        )
    )
    
    # Delay start of joint_state_Broadcaster after `robot_controller`
    delay_joint_state_broadcaster_after_robot_controller_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=robot_controller_spawner,
            on_exit=[joint_state_broadcaster_spawner]
        )
    )
    
    camera = Node(
            package='v4l2_camera',
            executable='v4l2_camera_node',
            name='v4l2_camera_node',
            parameters=[{
                'image_size': [640, 480],
                "camera_frame_id": "camera_link_optical"
            }],
    )
    
    
    lidar = Node(
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
    
    
    imu = Node(
        package="lumina",
        executable="imu",
        output="screen"
    )
    
    pressure_temperature = Node(
        package="lumina",
        executable="pressure_temperature_altitude",
        output="screen"
    )
    
    # Process the URDF file
    ekf_yaml_path = os.path.join(pkg_path, 'config', 'ekf.yaml')
    
    imu_filtered = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_yaml_path]
    )
    

    
    nodes = [
        control_node,
        robot_state_publisher,
        robot_controller_spawner,
        delay_rviz_after_joint_state_broadcaster_spawner,
        delay_joint_state_broadcaster_after_robot_controller_spawner,
        camera,
        lidar,
        imu,
        pressure_temperature,
        imu_filtered
    ]
    
    return LaunchDescription(declared_arguments + nodes)