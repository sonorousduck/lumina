import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

    # Set the package name
    package_name = 'lumina'

    # Include the robot_state_publisher launch file
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory(package_name), 'launch', 'rsp.launch.py'
        )]), 
        launch_arguments={'use_sim_time': 'true'}.items()
    )

    # Gazebo parameters file
    default_world = os.path.join(
        get_package_share_directory(package_name),
        'worlds',
        'empty.world'
        ) 

    world = LaunchConfiguration('world')

    world_arg = DeclareLaunchArgument(
        'world',
        default_value=default_world,
        description='World to load'
        )

    # Include the Gazebo launch file and pass the world file argument
    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('ros_ign_gazebo'), 'launch', 'ign_gazebo.launch.py')]),
                    launch_arguments={'gz_args': ['-r -v4 ', world], 'on_exit_shutdown': 'true'}.items()
             )

    # Run the spawner node from the gazebo_ros package to spawn the entity
    spawn_entity = Node(package='ros_ign_gazebo', executable='create',
                        arguments=['-topic', 'robot_description',
                                   '-name', 'lumina',
                                   '-z', '0.1'],
                        output='screen')
    
    bridge_params = os.path.join(get_package_share_directory(package_name),'config','ign_bridge.yaml')
    ros_gz_bridge = Node(
        package="ros_ign_bridge",
        executable="parameter_bridge",
        arguments=[
            '--ros-args',
            '-p',
            f'config_file:={bridge_params}',
        ]
    )
    
    # ros_gz_image_bridge = Node(
    #     package="ros_ign_image",
    #     executable="image_bridge",
    #     arguments=["/camera/image_raw"]
    # )


    # Launch all components together
    return LaunchDescription([
        rsp,
        # joystick,
        # twist_mux,
        world_arg,
        gazebo,
        spawn_entity,
        # diff_drive_spawner,
        # joint_broad_spawner,
        ros_gz_bridge,
        # ros_gz_image_bridge
    ])