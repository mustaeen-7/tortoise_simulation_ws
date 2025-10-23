#!/usr/bin/env python3
#
# Copyright 2019 ROBOTIS CO., LTD.
# Licensed under the Apache License, Version 2.0
# Authors: Joep Tool
#

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    launch_file_dir = os.path.join(get_package_share_directory('tortoise_bot'), 'launch')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    # Pose arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    x_pose = LaunchConfiguration('x_pose', default='8.087546')
    y_pose = LaunchConfiguration('y_pose', default='2.548081')
    #z_pose = LaunchConfiguration('z_pose', default='0.008509')
    roll = LaunchConfiguration('roll', default='0.000490')
    pitch = LaunchConfiguration('pitch', default='0.006094')
    yaw = LaunchConfiguration('yaw', default='-1.524284')

    # Declare launch arguments
    declare_x_cmd = DeclareLaunchArgument('x_pose', default_value='8.087546', description='X position')
    declare_y_cmd = DeclareLaunchArgument('y_pose', default_value='2.548081', description='Y position')
    #declare_z_cmd = DeclareLaunchArgument('z_pose', default_value='0.008509', description='Z position')
    declare_roll_cmd = DeclareLaunchArgument('roll', default_value='0.000490', description='Roll')
    declare_pitch_cmd = DeclareLaunchArgument('pitch', default_value='0.006094', description='Pitch')
    declare_yaw_cmd = DeclareLaunchArgument('yaw', default_value='-1.524284', description='Yaw')

    # Set TurtleBot3 model environment variable
    env1 = SetEnvironmentVariable(
        name='TURTLEBOT3_MODEL',
        value='burger_cam'
    )

    # Load Gazebo world
    world = os.path.join(
        get_package_share_directory('tortoise_bot'),
        'worlds',
        'Nxtwave_full_world.world'
    )

    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={'world': world}.items()
    )

    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
        )
    )

    robot_state_publisher_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, 'robot_state_publisher.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    spawn_turtlebot_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, 'spawn_turtlebot3.launch.py')
        ),
        launch_arguments={
            'x_pose': x_pose,
            'y_pose': y_pose,
            #'z_pose': z_pose,
            'roll': roll,
            'pitch': pitch,
            'yaw': yaw
        }.items()
    )

    # LaunchDescription
    ld = LaunchDescription()

    # Declare arguments first
    ld.add_action(declare_x_cmd)
    ld.add_action(declare_y_cmd)
    #ld.add_action(declare_z_cmd)
    ld.add_action(declare_roll_cmd)
    ld.add_action(declare_pitch_cmd)
    ld.add_action(declare_yaw_cmd)

    # Add environment and nodes
    ld.add_action(env1)
    ld.add_action(gzserver_cmd)
    ld.add_action(gzclient_cmd)
    ld.add_action(robot_state_publisher_cmd)
    ld.add_action(spawn_turtlebot_cmd)

    return ld

