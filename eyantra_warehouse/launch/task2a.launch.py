#!/usr/bin/python3
# -*- coding: utf-8 -*-

''' 
*****************************************************************************************
*
*        =============================================
*                  LB Theme (eYRC 2024-25)
*        =============================================
*
*
*  Filename:			task1a.launch.py
*  Description:         Use this file to spawn ebot inside e-yantra warehouse world in the gazebo simulator and publish robot states.
*  Created:				08/08/2024
*  Last Modified:	    13/08/2024
*  Modified by:         Siddharth
*  Author:				e-Yantra Team
*  
*****************************************************************************************
'''

import launch
import launch_ros
import os
import xacro
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def get_package_file(package, file_path):
    """Get the location of a file installed in an ament package"""
    package_path = get_package_share_directory(package)
    absolute_file_path = os.path.join(package_path, file_path)
    return absolute_file_path

def generate_launch_description():
    pkg_share = launch_ros.substitutions.FindPackageShare(package='ebot_description').find('ebot_description')

    xacro_file_ebot = os.path.join(pkg_share, 'models/','ebot/', 'ebot_description_2a.xacro')
    assert os.path.exists(xacro_file_ebot), "The box_bot.xacro doesnt exist in "+str(xacro_file_ebot)
    robot_description_config_ebot = xacro.process_file(xacro_file_ebot)
    robot_description_ebot = robot_description_config_ebot.toxml()
    start_world = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ebot_description'), 'launch', 'start_world_task2a_launch.py'),
        )
    )

    static_transform = launch_ros.actions.Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher',
        arguments = ["1.6", "-2.4", "-0.8", "3.14", "0", "0", "world", "odom"],
        output='screen')
    robot_state_publisher_node_ebot = launch_ros.actions.Node(
        package='robot_state_publisher',
        name='ebot_RD',
        executable='robot_state_publisher',
        parameters=[{"robot_description": robot_description_ebot}],
        remappings=[('robot_description', 'robot_description_ebot')]

    )

    spawn_ebot = launch_ros.actions.Node(
    	package='gazebo_ros', 
        name='ebot_spawner',
    	executable='spawn_entity.py',
        # arguments=['-entity', 'ebot', '-topic', 'robot_description_ebot', '-x', '1.1', '-y', '4.35', '-z', '0.1', '-Y', '3.14'],
        # arguments=['-entity', 'ebot', '-topic', 'robot_description_ebot', '-x', '0.0', '-y', '0.0', '-z', '0.1', '-Y', '0.0'],
        arguments=['-entity', 'ebot', '-topic', 'robot_description_ebot', '-x', '1.07', '-y', '-2.4', '-z', '0.05', '-Y', '3.14'],
        output='screen'
    )
    spwanner = launch_ros.actions.Node(package='ur_description',
                       executable='spawner_box_2a.py')
                                 
    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(name='gui', default_value='True',
                                            description='Flag to enable joint_state_publisher_gui'),
        launch.actions.DeclareLaunchArgument(name='use_sim_time', default_value='True',
                                            description='Flag to enable use_sim_time'),
        start_world,
        robot_state_publisher_node_ebot,
        spawn_ebot,
        static_transform,
        spwanner
    ])
