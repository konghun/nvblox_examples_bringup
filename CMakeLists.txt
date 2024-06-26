# SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
# Copyright (c) 2021-2023 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

cmake_minimum_required(VERSION 3.5)
project(nvblox_examples_bringup)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)       
find_package(tf2_geometry_msgs REQUIRED)

add_executable(base src/base.cpp)
ament_target_dependencies(base
  rclcpp
  tf2
  tf2_ros
  geometry_msgs
)

add_executable(map_odom_base src/map_odom_base.cpp)
ament_target_dependencies(map_odom_base
  rclcpp
  tf2_ros
  tf2
  tf2_geometry_msgs
  geometry_msgs
  nav_msgs
)

install(TARGETS
  base
  map_odom_base
  DESTINATION lib/${PROJECT_NAME}
)


install(DIRECTORY config launch src maps
  DESTINATION share/${PROJECT_NAME})

ament_package()
