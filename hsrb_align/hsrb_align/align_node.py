#! /usr/bin/env python
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

import sys
import time

from controller_manager_msgs.srv import ListControllers
from hsrb_angle_sensors_alignment_msgs.action import AlignAngleSensors

import rclpy
from rclpy.action import ActionClient
from rclpy.duration import Duration

from tmc_manipulation_msgs.srv import SafeJointChange

global align_angle_sensors_response

# Called when receiving the response of align_angle_sensors


def action_align_angle_sensors_response(future):
    goal_handle = future.result()
    if not goal_handle.accepted:
        return
    # Retrieve the result
    result_future = goal_handle.get_result_async()
    result_future.add_done_callback(action_align_angle_sensors_result)
    return

# Called when receiving the result of align_angle_sensors


def action_align_angle_sensors_result(future):
    global align_angle_sensors_response
    align_angle_sensors_response = future.result().result
    return


def align_angle_sensors(node):
    global align_angle_sensors_response
    client = ActionClient(node,
                          AlignAngleSensors,
                          '/align_angle_sensors')

    if not client.wait_for_server(timeout_sec=60.0):
        node.get_logger().error('alignment server is not active.')
        return False

    goal = AlignAngleSensors.Goal()
    future = client.send_goal_async(goal)
    future.add_done_callback(action_align_angle_sensors_response)
    align_angle_sensors_response = None

    node.get_logger().info('alignment action sent')

    start = node.get_clock().now()
    timeout = 10.0
    while node.get_clock().now() - start < Duration(seconds=timeout) and rclpy.ok():
        rclpy.spin_once(node)
        if align_angle_sensors_response is not None:
            break
    if align_angle_sensors_response is None:
        node.get_logger().error('align_angle_sensors response is none.')
        return False
    if align_angle_sensors_response.success:
        node.get_logger().info('alignment is finished successfully')
    else:
        node.get_logger().error('alignment is not finished for some reason.')
        return False
    return True


def move_to_initial_pose(node):
    change_joint = node.create_client(SafeJointChange,
                                      '/change_joint')

    start = node.get_clock().now()
    timeout = 60.0
    is_joint_state_publish = False
    while node.get_clock().now() - start < Duration(seconds=timeout) and rclpy.ok():
        count = node.count_publishers('/joint_states')
        if count > 0:
            is_joint_state_publish = True
            break

    if not is_joint_state_publish:
        node.get_logger().error('joint_states is not published')
        return False
    if change_joint.wait_for_service(timeout_sec=60.0) is False:
        node.get_logger().error('safe_pose_changer service is not found')
        return False
    ref = SafeJointChange.Request()

    ref.ref_joint_state.name = ['arm_lift_joint',
                                'arm_flex_joint',
                                'arm_roll_joint',
                                'wrist_flex_joint',
                                'wrist_roll_joint',
                                'head_pan_joint',
                                'head_tilt_joint',
                                'hand_motor_joint']

    ref.ref_joint_state.position = [float(0.05), float(
        0), float(-1.57), float(-1.57), float(0), float(0), float(0), float(0.6)]
    future = change_joint.call_async(ref)
    rclpy.spin_until_future_complete(node, future)
    ret = future.result()

    return ret.success


def wait_for_controllers(node, timeout=10.0):
    """Wait for these controllers

    - arm_trajectory_controller
    - head_trajectory_controller
    - gripper_controller
    - drive_mode_controller
    - joint_state_controller
    """
    controller_client = node.create_client(ListControllers,
                                           '/controller_manager/list_controllers')

    if controller_client.wait_for_service(timeout_sec=60.0) is False:
        node.get_logger().error('safe_pose_changer service is not found')
        return False

    using_controllers = {'arm_trajectory_controller',
                         'head_trajectory_controller',
                         'gripper_controller',
                         'joint_state_broadcaster'}

    start = node.get_clock().now()
    while node.get_clock().now() - start < Duration(seconds=timeout) and rclpy.ok():
        req = ListControllers.Request()
        future = controller_client.call_async(req)
        rclpy.spin_until_future_complete(node, future, timeout_sec=1.0)
        ret = future.result()
        if ret is not None:
            running = {
                controller.name
                for controller in ret.controller
                if controller.state == 'active'
            }
            if using_controllers.issubset(running):
                return True
        time.sleep(0.1)
    return False


def main():
    rclpy.init()
    node = rclpy.create_node('align_angles')
    node.get_logger().info("Checking the controller")

    try:
        if not wait_for_controllers(node):
            node.get_logger().error('controllers are not running.')
            return 1
        node.get_logger().info("Transitioning to the initial state")
        if not move_to_initial_pose(node):
            node.get_logger().error('Fail to change to intial pose')
            return 1
        node.get_logger().info("align_angle_sensors")
        if not align_angle_sensors(node):
            return 1

    finally:
        node.destroy_node()
        rclpy.shutdown()

    return


if __name__ == '__main__':
    sys.exit(main())
