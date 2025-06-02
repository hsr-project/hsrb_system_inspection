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
import hsrb_auto_diagnostics.check_tools as check_tools
from hsrb_auto_diagnostics.device_check import DeviceCheck
from rclpy.duration import Duration
from std_msgs.msg import (
    Bool,
    ColorRGBA
)
from std_srvs.srv import Empty
from tmc_msgs.srv import SetColor


"""
StatusLED Test Class
"""


class StatusLEDCheck(DeviceCheck):
    COLOR = [[0.0, 0.0, 0.0], [0.0, 0.0, 1.0], [0.0, 1.0, 0.0], [0.0, 1.0, 1.0],
             [1.0, 0.0, 0.0], [1.0, 0.0, 1.0], [1.0, 1.0, 0.0], [1.0, 1.0, 1.0]]

    def __init__(self, *args):
        super().__init__(*args)

        if not self.node.has_parameter('lighting_time'):
            self.node.declare_parameter('lighting_time', 1.0)
        if not self.node.has_parameter('wait_for_response_timeout'):
            self.node.declare_parameter('wait_for_response_timeout', 10.0)

    def check_interactive(self, _fb, _as):
        self._interactive_check_tools = check_tools.Interactive(
            self.node, '/user_response', Bool)

        fb_msg = ['Please look at status LED']
        question = ['?Push the OK button if status LED shift various colors']
        err_msg = ['LED_Error']

        lighting_time = self.node.get_parameter('lighting_time').get_parameter_value().double_value
        wait_for_response_timeout = self.node.get_parameter(
            'wait_for_response_timeout').get_parameter_value().double_value

        set_led_client = self.node.create_client(
            SetColor, '/hsrb/status_led_node/set_color')
        auto_led_client = self.node.create_client(
            Empty, '/hsrb/status_led_node/activate_auto_mode')

        set_led_client.wait_for_service(timeout_sec=5.0)
        auto_led_client.wait_for_service(timeout_sec=5.0)

        try:
            self._interactive_check_tools.publish_feedback(fb_msg[0],
                                                           _fb, _as)
            req = SetColor.Request()
            data = ColorRGBA()
            for color in self.COLOR:
                data.b = color[0]
                data.g = color[1]
                data.r = color[2]
                req.color = data
                self.node.get_logger().info("set_color...")
                future = set_led_client.call_async(req)
                init_time = self.node.get_clock().now()
                now = init_time
                while not future.done():
                    now = self.node.get_clock().now()
                    if (now - init_time) > Duration(seconds=3.0):
                        self.node.get_logger().error("set_color timeout.")
                        future.cancel()
                        break
                    self.node.get_clock().sleep_for(Duration(seconds=0.1))
                self.node.get_logger().info("set_color done.")

                self.node.get_clock().sleep_for(Duration(seconds=lighting_time))

            # reset the led color's control mode
            self.node.get_logger().info("activate_auto_mode...")
            future = auto_led_client.call_async(Empty.Request())
            init_time = self.node.get_clock().now()
            now = init_time
            while not future.done():
                now = self.node.get_clock().now()
                if (now - init_time) > Duration(seconds=3.0):
                    self.node.get_logger().error("activate_auto_mode timeout.")
                    future.cancel()
                    break
                self.node.get_clock().sleep_for(Duration(seconds=0.1))
            self.node.get_logger().info("activate_auto_mode done.")

            self.error_msg = (
                self._interactive_check_tools.wait_for_user_response(
                    _fb, _as, question[0], err_msg[0],
                    wait_for_response_timeout))
            self.node.destroy_subscription(self._interactive_check_tools.sub)

            return self.error_msg

        except Exception as e:
            self.node.get_logger().error(str(e))
            return [str(e)]

        finally:
            self.node.destroy_client(set_led_client)
            self.node.destroy_client(auto_led_client)
