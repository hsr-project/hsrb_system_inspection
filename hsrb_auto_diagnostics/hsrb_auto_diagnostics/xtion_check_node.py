#! /usr/bin/env python
# Copyright (c) 2025 TOYOTA MOTOR CORPORATION
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
from hsrb_auto_diagnostics.check_action import CheckAction
from hsrb_auto_diagnostics.xtion_check import DepthImageCheck
from hsrb_auto_diagnostics.xtion_check import PointCloudCheck
from hsrb_auto_diagnostics.xtion_check import RGBImageCheck
import rclpy
from rclpy.executors import MultiThreadedExecutor

ACTION_NAME_ = 'xtion'


def main():
    rclpy.init()
    node = rclpy.create_node('hsrb_xtion_check',
                             allow_undeclared_parameters=True,
                             automatically_declare_parameters_from_overrides=True)
    node.declare_parameter('action_name', ACTION_NAME_)
    action_name = node.get_parameter('action_name').get_parameter_value().string_value
    xtion_rgb_check_action = CheckAction(node, action_name + '_rgb', RGBImageCheck)  # noqa: F841
    xtion_depth_check_action = CheckAction(  # noqa: F841
        node, action_name + '_depth', DepthImageCheck)
    xtion_point_check_action = CheckAction(  # noqa: F841
        node, action_name + '_point', PointCloudCheck)

    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().fatal("Keyboard interrupt!")

    executor.shutdown()
    rclpy.try_shutdown()
    node.destroy_node()


if __name__ == '__main__':
    main()
