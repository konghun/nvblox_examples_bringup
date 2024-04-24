# SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
# Copyright (c) 2023 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# SPDX-License-Identifier: Apache-2.0

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    bringup_dir = get_package_share_directory('nvblox_examples_bringup')

    # Launch Arguments
    run_rviz_arg = DeclareLaunchArgument(
        'run_rviz', default_value='True',
        description='Whether to start RVIZ')
    run_nav2_arg = DeclareLaunchArgument(
        'run_nav2', default_value='True',
        description='Whether to run nav2')
    from_bag_arg = DeclareLaunchArgument(
        'from_bag', default_value='False',
        description='Whether to run from a bag or live zed data')
    bag_path_arg = DeclareLaunchArgument(
        'bag_path', default_value='rosbag2*',
        description='Path of the bag (only used if from_bag == True)')
    global_frame = LaunchConfiguration('global_frame',
                                       default='map')

    # Create a shared container to hold composable nodes 
    # for speed ups through intra process communication.
    shared_container_name = "shared_nvblox_container"
    shared_container = Node(
        name=shared_container_name,
        package='rclcpp_components',
        executable='component_container_mt',
        output='screen')
    
    # Nav2
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            bringup_dir, 'launch', 'nav2', 'nav2_aimy.launch.py')),
        launch_arguments={'global_frame': global_frame}.items(),
        condition=IfCondition(LaunchConfiguration('run_nav2')))



    # ZED
    # Note(remos): This was only tested with a ZED2 camera so far.
    zed_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            bringup_dir, 'launch', 'sensors', 'zed2.launch.py')]),
        launch_arguments={
            'attach_to_shared_component_container': 'True',
            'component_container_name': shared_container_name}.items(),
        condition=UnlessCondition(LaunchConfiguration('from_bag')))

    # Ros2 bag
    bag_play = ExecuteProcess(
        cmd=['ros2', 'bag', 'play', LaunchConfiguration('bag_path')],
        shell=True, output='screen',
        condition=IfCondition(LaunchConfiguration('from_bag')))

    # Rviz
    rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            bringup_dir, 'launch', 'rviz', 'rviz.launch.py')]),
        launch_arguments={'config_name': 'zed_example.rviz',
                          'global_frame': global_frame}.items(),
        condition=IfCondition(LaunchConfiguration('run_rviz')))

    # Map server
    map_server_launch = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{'yaml_filename': '/workspaces/isaac_ros-dev/src/isaac_ros_nvblox/nvblox_examples/nvblox_examples_bringup/maps/map_1713336985.yaml'}],
    )


    return LaunchDescription([
        run_rviz_arg,
        from_bag_arg,
        bag_path_arg,
        shared_container,
        zed_launch,
        bag_play,
        run_nav2_arg,
        rviz_launch,
        nav2_launch,
        map_server_launch])
