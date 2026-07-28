#!/usr/bin/env python
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
# -*- coding: utf-8 -*-
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

from launch_ros.substitutions import FindPackageShare


def declare_arguments():
    declared_arguments = []
    declared_arguments.append(DeclareLaunchArgument('frame_id', default_value='head_l_stereo_camera_frame'))
    declared_arguments.append(DeclareLaunchArgument('camera_setting_file_path',
                              default_value='/etc/opt/tmc/robot/conf.d/stereo_pgr_camera.yml'))
    declared_arguments.append(DeclareLaunchArgument('image_topic_names',
                              default_value='["/head_l_stereo_camera/image_raw", "/head_r_stereo_camera/image_raw"]'))
    declared_arguments.append(DeclareLaunchArgument('use_blackfly', default_value='True'))

    return declared_arguments


def generate_launch_description():
    pgr_launch_dir = PathJoinSubstitution([FindPackageShare('tmc_pgr_camera'), 'launch'])
    stereo_camera_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([pgr_launch_dir, '/stereo.launch.py']),
        launch_arguments={
            'node_name': 'stereo_camera',
            'frame_id': LaunchConfiguration('frame_id'),
            'camera_setting_file_path': LaunchConfiguration('camera_setting_file_path'),
            'image_topic_names': LaunchConfiguration('image_topic_names'),
            'use_blackfly': LaunchConfiguration('use_blackfly'),
        }.items())

    ld = LaunchDescription(declare_arguments() + [stereo_camera_node])
    return ld
