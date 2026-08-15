#!/usr/bin/env python3
# Software License Agreement (BSD License)
#
# Copyright (c) 2021, UFACTORY, Inc.
# All rights reserved.
#
# Author: Vinman <vinman.wen@ufactory.cc> <vinman.cub@gmail.com>

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

from launch_ros.actions import Node


def generate_launch_description():
    hw_ns = LaunchConfiguration('hw_ns', default='xarm')
    
    
    # robot moveit gazebo launch
    # xarm_moveit_config/launch/_robot_moveit_gazebo.launch.py
    robot_moveit_gazebo_launch = IncludeLaunchDescription(
    
    	#changed to valm package (was xarm_moveit_config) 
        PythonLaunchDescriptionSource(PathJoinSubstitution([FindPackageShare('valm'), 'launch', '_robot_moveit_gazebo.launch.py'])),
        
        launch_arguments={
            'dof': '5',
            'robot_type': 'xarm',
            'hw_ns': hw_ns,
            'no_gui_ctrl': 'false',
            
            'add_gripper': 'True',
            'add_realsense_d435i': 'True',
            
            #chooses the world
            'world': PathJoinSubstitution([FindPackageShare('valm'), 'worlds', 'vision_test.world']),
            
        }.items(),
    )
    
    #defines the controller node
    controller_node = Node(
    package='valm',
    executable='controller',
    name='controller',
    output='screen'
    )
    
    #defines the vision node
    vision_node = Node(
    package='valm',
    executable='vision',
    name='vision',
    output='screen'
    )
    
    #displays the image node
    image_view = Node(
    package='rqt_image_view',
    executable='rqt_image_view',
    name='camera_view',
    #arguments=['/color/image_raw'],
    arguments=['/vision/annotated_image'],
    output='screen'
    )
    
    return LaunchDescription([
        robot_moveit_gazebo_launch,
        controller_node,
        vision_node,
        image_view
    ])
