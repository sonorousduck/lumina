# Copyright 2020 ros2_control Development Team
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import xacro
from launch import LaunchContext

def generate_launch_description():
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "gui",
            default_value="true",
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
    launch_context = LaunchContext()

    # # Get URDF via xacro
    # robot_description_content = Command(
    #     [
    #         PathJoinSubstitution([FindExecutable(name="xacro")]),
    #         " ",
    #         PathJoinSubstitution(
    #             [FindPackageShare("lumina"), "description", "robot.urdf.xacro"]
    #         ),
    #         # " ",
    #         # "sim_mode:=",
    #         # sim_mode,
    #     ]
    # )
    # robot_description = {"robot_description": robot_description_content}

        
    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("lumina"),
            "config",
            "my_controllers.yaml"
        ]
    )
    print(robot_controllers.perform(launch_context))
    
    # rviz_config_file = PathJoinSubstitution(
    #     [FindPackageShare(lumina"), "rviz", "diffbot.rviz"]
    # )
    
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
        arguments = [
            "diffbot_base_controller",
            "--controller_manager",
            "/controller_manager"
        ],
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
    
    nodes = [
        control_node,
        robot_state_publisher,
        robot_controller_spawner,
        delay_rviz_after_joint_state_broadcaster_spawner,
        delay_joint_state_broadcaster_after_robot_controller_spawner
    ]
    
    return LaunchDescription(declared_arguments + nodes)