#!/usr/bin/env python3
# Copyright (c) 2026 TOYOTA MOTOR CORPORATION
# All rights reserved.
# Redistribution and use in source and binary forms, with or without
# modification, are permitted (subject to the limitations in the disclaimer
# below) provided that the following conditions are met:
# * Redistributions of source code must retain the above copyright notice, this
#   list of conditions and the following disclaimer.
# * Redistributions in binary form must reproduce the above copyright notice,
#   this list of conditions and the following disclaimer in the documentation
#   and/or other materials provided with the distribution.
# * Neither the name of the copyright holder nor the names of its contributors may be used
#   to endorse or promote products derived from this software without specific
#   prior written permission.
# NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
# LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
# THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
# GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
# HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
# OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
# DAMAGE.
import os

from launch import LaunchDescription

from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
)

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def declare_arguments():
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument('runtime_config_package',
                              default_value='hsrb_bringup',
                              description='Package with the diagnostics configuration in "config" folder.'))
    declared_arguments.append(
        DeclareLaunchArgument('aggregator_config_file',
                              default_value='diagnostic_aggregator.yaml',
                              description='YAML file with the diagnostics configuration.'))

    return declared_arguments


def generate_launch_description():
    runtime_config_package = LaunchConfiguration('runtime_config_package')
    aggregator_config_file = LaunchConfiguration('aggregator_config_file')
    diagnostics_parameter = PathJoinSubstitution(
        [FindPackageShare(runtime_config_package), 'config', aggregator_config_file])

    aggregator_node = Node(package='diagnostic_aggregator',
                           executable='aggregator_node',
                           name='diagnostic_aggregator_node',
                           parameters=[diagnostics_parameter])

    robot_version = os.environ.get("ROBOT_VERSION")
    robot_name = robot_version.replace('"', '').split('-')[0].lower()

    if robot_name in ['hsrb']:
        additional_aggregator_config_file = 'diagnostic_aggregator_hsrb.yaml'
    elif robot_name in ['hsrc', 'hsrd']:
        additional_aggregator_config_file = 'diagnostic_aggregator_hsrc.yaml'
    additional_aggregator_config = PathJoinSubstitution(
        [FindPackageShare(runtime_config_package), 'config', additional_aggregator_config_file])

    add_analyzer = Node(package='diagnostic_aggregator',
                        executable='add_analyzer',
                        parameters=[additional_aggregator_config],
                        remappings=[('/analyzers/set_parameters_atomically',
                                     '/diagnostic_aggregator_node/set_parameters_atomically')])

    auto_diag_launch_dir = PathJoinSubstitution(
        [FindPackageShare('hsrb_auto_diagnostics'), 'launch'])
    auto_diag = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([auto_diag_launch_dir, '/hsrb_device_check_all.launch.py']))

    return LaunchDescription(declare_arguments() + [aggregator_node, add_analyzer, auto_diag])
