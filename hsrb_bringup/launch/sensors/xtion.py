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

from launch.actions import DeclareLaunchArgument
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
)

from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def declare_arguments():
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument('camera_name',
                              default_value='head_rgbd_sensor',
                              description="Camera's name used for namespace."))
    declared_arguments.append(
        DeclareLaunchArgument('calibration_file_directory',
                              default_value='/etc/opt/tmc/robot/conf.d/calib_results',
                              description='Directory containing calibration files.'))
    return declared_arguments


def generate_launch_description():
    calib_dir = LaunchConfiguration('calibration_file_directory')
    rgb_camera_info_file = PathJoinSubstitution([calib_dir, 'rgbd_sensor_rgb_camera_params.yaml'])
    depth_camera_info_file = PathJoinSubstitution([calib_dir, 'rgbd_sensor_depth_camera_params.yaml'])

    rgb_camera_info_url = ['file://', rgb_camera_info_file]
    depth_camera_info_url = ['file://', depth_camera_info_file]
    # TODO(Takeshita) キャリブレーション済みのフレームを利用するようにする
    driver_parameters = {'depth_registration': True,
                         'use_device_time': False,
                         'rgb_frame_id': 'head_rgbd_sensor_rgb_frame',
                         'depth_frame_id': 'head_rgbd_sensor_depth_frame',
                         'rgb_camera_info_url': rgb_camera_info_url,
                         'depth_camera_info_url': depth_camera_info_url}

    camera_name = LaunchConfiguration('camera_name')
    driver_node = ComposableNode(package='openni2_camera',
                                 plugin='openni2_wrapper::OpenNI2Driver',
                                 name='openni2_driver',
                                 namespace=camera_name,
                                 parameters=[driver_parameters],
                                 remappings=[('depth/camera_info', 'depth_registered/camera_info'),
                                             ('depth/image', 'depth_registered/image'),
                                             ('depth/image/compressed', 'depth_registered/image/compressed'),
                                             ('depth/image/compressedDepth', 'depth_registered/image/compressedDepth'),  # noqa
                                             ('depth/image/theora', 'depth_registered/image/theora'),
                                             ('depth_raw/image', 'depth_registered/image_raw'),
                                             ('depth_raw/image/compressed', 'depth_registered/image_raw/compressed'),  # noqa
                                             ('depth_raw/image/compressedDepth', 'depth_registered/image_raw/compressedDepth'),  # noqa
                                             ('depth_raw/image/theora', 'depth_registered/image_raw/theora'),
                                             ('rgb/image_raw', 'rgb/image_rect_color'),
                                             ('rgb/image_raw/compressed', 'rgb/image_rect_color/compressed'),
                                             ('rgb/image_raw/compressedDepth', 'rgb/image_rect_color/compressedDepth'),
                                             ('rgb/image_raw/theora', 'rgb/image_rect_color/theora')])
    depth_rectifier_node = ComposableNode(
        package='tmc_depth_calibration',
        plugin='tmc_depth_calibration::DepthRectifierNodelet',
        name='depth_rectifier',
        namespace=camera_name,
        parameters=[{
            'depth_rectify_info_url':
                PathJoinSubstitution([calib_dir, 'rgbd_sensor_depth_rectification_params.yaml'])}],
        remappings=[
            ('image_rect', 'depth_registered/image_raw'),
            ('image', 'depth_registered/image_rect_raw'),
            ('image/compressed', 'depth_registered/image_rect_raw/compressed'),
            ('image/compressedDepth', 'depth_registered/image_rect_raw/compressedDepth'),
            ('image/theora', 'depth_registered/image_rect_raw/theora'),
        ],
    )
    pointcloud_node = ComposableNode(package='depth_image_proc',
                                     plugin='depth_image_proc::PointCloudXyzrgbNode',
                                     name='points_xyzrgb',
                                     namespace=camera_name,
                                     remappings=[('rgb/image_rect_color', 'rgb/image_rect_color'),
                                                 ('rgb/camera_info', 'rgb/camera_info'),
                                                 ('depth_registered/image_rect', 'depth_registered/image_rect_raw'),
                                                 ('points', 'depth_registered/rectified_points')])
    container = ComposableNodeContainer(name='container',
                                        namespace=camera_name,
                                        package='rclcpp_components',
                                        executable='component_container',
                                        composable_node_descriptions=[
                                            driver_node, depth_rectifier_node, pointcloud_node])

    return LaunchDescription(declare_arguments() + [container])
