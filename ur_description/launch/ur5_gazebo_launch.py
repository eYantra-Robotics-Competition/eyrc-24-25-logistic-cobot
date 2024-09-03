#!/usr/bin/python3
# -*- coding: utf-8 -*-

''' 
*****************************************************************************************
*
*        =============================================
*                  TBD Theme (eYRC 2023-24)
*        =============================================
*
*
*  Filename:			ebot_display_launch.py
*  Description:         Use this file to spawn UR5 inside e-yantra warehouse world in the gazebo simulator and publish robot states.
*  Created:				16/07/2023
*  Modified by:         Archit, Jaison
*  Author:				e-Yantra Team
*  Copyright (c) 2023 e-Yantra IITB
*****************************************************************************************
'''

import launch
import launch_ros
import os
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def get_package_file(package, file_path):
    """Get the location of a file installed in an ament package"""
    package_path = get_package_share_directory(package)
    absolute_file_path = os.path.join(package_path, file_path)
    return absolute_file_path

def generate_launch_description():

    start_world = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ur_description'), 'launch', 'start_ur_world_launch.py'),
        )
    )    

    spawn_arm = launch_ros.actions.Node(
    	package='gazebo_ros', 
        name='ur5_spawner',
    	executable='spawn_entity.py',
        arguments=['-entity', 'ur5', '-topic', 'robot_description_ur5', '-x', '1.6', '-y', '-2.4', '-z', '0.58', '-Y', '3.14'],
        output='screen')

                                                 
    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(name='gui', default_value='True',
                                            description='Flag to enable joint_state_publisher_gui'),
        launch.actions.DeclareLaunchArgument(name='use_sim_time', default_value='True',
                                            description='Flag to enable use_sim_time'),
        start_world,
        # spawn_arm
    ])


