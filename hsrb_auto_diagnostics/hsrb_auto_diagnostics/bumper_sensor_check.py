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
from std_msgs.msg import Bool
from std_msgs.msg import ColorRGBA
from std_srvs.srv import Empty

"""
Bumper Test Class
"""


class BumperSensorCheck(DeviceCheck):
    GREEN = ColorRGBA(g=1.0)
    BLUE = ColorRGBA(b=1.0)
    CYAN = ColorRGBA(g=1.0, b=1.0)

    PARAM_CHECK_TIME = 'check_time'

    def __init__(self, *args):
        super().__init__(*args)
        self._f_bumper_pushed = False
        self._f_bumper_flag = False
        self._f_bumper_received = False
        self._b_bumper_pushed = False
        self._b_bumper_flag = False
        self._b_bumper_received = False

        if not self.node.has_parameter(self.PARAM_CHECK_TIME):
            self.node.declare_parameter(self.PARAM_CHECK_TIME, 60.0)

    def f_bumper_callback(self, data):
        self._f_bumper_received = True
        if data.data:
            self._f_bumper_pushed = True
        self._f_bumper_flag = data.data

    def b_bumper_callback(self, data):
        self._b_bumper_received = True
        if data.data:
            self._b_bumper_pushed = True
        self._b_bumper_flag = data.data

    def user_res_callback(self, data):
        self._user_res_flag = data.data

    def bumper_reset_client(self):
        client = self.node.create_client(Empty, '/bumper/reset')
        self.node.get_logger().info("wait for bumper reset service...")
        client.wait_for_service(timeout_sec=5.0)
        self.node.get_logger().info("bumper reset service ready.")
        try:
            self.node.get_logger().info("reset bumper...")
            future = client.call_async(Empty.Request())
            init_time = self.node.get_clock().now()
            now = init_time
            while not future.done():
                now = self.node.get_clock().now()
                if (now - init_time) > Duration(seconds=3.0):
                    self.node.get_logger().error("reset bumper timeout.")
                    future.cancel()
                    break
                self.node.get_clock().sleep_for(Duration(seconds=0.1))
            self.node.get_logger().info("reset bumper done.")
        except Exception as e:
            self.error_msg.append('Service call failed: ' + str(e))
        finally:
            self.node.destroy_client(client)

    def print_and_append_err_msg(self, msg, _fb, _goal_handle):
        self._interactive_check_tools.publish_feedback(msg, _fb, _goal_handle)
        self.error_msg.append(msg)

    def check_interactive(self, _fb, _goal_handle):
        fb_msg = ['Now, checking sensors.Please do not touch bumper',
                  'Restart robot controller']
        err_msg = ['Front Bumper Sensor Error',
                   'Back Bumper Sensor Error',
                   'Bumper Error',
                   'Please push bumpers at least once']
        question = ['?If you push front bumper, LED lights GREEN. If you'
                    ' push back bumper, LED lights BLUE. Please check and push'
                    ' the OK button if it works correctly']
        cancel_msg = ['Diagnostic Canceled']

        check_time = self.node.get_parameter(self.PARAM_CHECK_TIME).get_parameter_value().double_value

        self._f_bumper_flag = False
        self._b_bumper_flag = False
        self._user_res_flag = None

        status_led_pub = self.node.create_publisher(
            ColorRGBA,
            'command_status_led_rgb',
            1)
        base_f_bumper_sub = self.node.create_subscription(
            Bool,
            '/base_f_bumper_sensor',
            self.f_bumper_callback, 10)
        base_b_bumper_sub = self.node.create_subscription(
            Bool,
            '/base_b_bumper_sensor',
            self.b_bumper_callback, 10)
        user_response_sub = self.node.create_subscription(
            Bool,
            '/user_response',
            self.user_res_callback, 10)

        self._interactive_check_tools = check_tools.Interactive(
            self.node, '/user_response', Bool)
        self.error_msg = []

        try:
            self._interactive_check_tools.publish_feedback(fb_msg[0],
                                                           _fb, _goal_handle)
            self.node.get_logger().info("wait for front bumper...")
            init = self.node.get_clock().now()
            now = init
            while (now - init) < Duration(seconds=10.0):
                if self._f_bumper_received:
                    break
            self.node.get_logger().info("wait for rear bumper...")
            init = self.node.get_clock().now()
            now = init
            while (now - init) < Duration(seconds=10.0):
                if self._b_bumper_received:
                    break

            if self._f_bumper_flag:
                self.node.get_logger().info("front bumper pushed.")
                self.print_and_append_err_msg(err_msg[0], _fb, _goal_handle)
                self.node.destroy_subscription(self._interactive_check_tools.sub)
                self.node.destroy_publisher(status_led_pub)
                self.node.destroy_subscription(base_f_bumper_sub)
                self.node.destroy_subscription(base_b_bumper_sub)
                self.node.destroy_subscription(user_response_sub)
                return self.error_msg
            if self._b_bumper_flag:
                self.node.get_logger().info("rear bumper pushed.")
                self.print_and_append_err_msg(err_msg[1], _fb, _goal_handle)
                self.node.destroy_subscription(self._interactive_check_tools.sub)
                self.node.destroy_publisher(status_led_pub)
                self.node.destroy_subscription(base_f_bumper_sub)
                self.node.destroy_subscription(base_b_bumper_sub)
                self.node.destroy_subscription(user_response_sub)
                return self.error_msg

            self._interactive_check_tools.publish_feedback(question[0],
                                                           _fb, _goal_handle)

            self.node.get_logger().info("wait for user response...")
            init_time = self.node.get_clock().now()
            now = init_time
            while (now - init_time) < Duration(seconds=check_time):
                if self._f_bumper_flag:
                    status_led_pub.publish(self.GREEN)
                if self._b_bumper_flag:
                    status_led_pub.publish(self.BLUE)
                if not self._f_bumper_flag and not self._b_bumper_flag:
                    status_led_pub.publish(self.CYAN)
                if self._user_res_flag is not None:
                    self.node.get_logger().info("received user response.")
                    break
                now = self.node.get_clock().now()
                self.node.get_clock().sleep_for(Duration(seconds=0.02))

            if self._user_res_flag is None:
                self.print_and_append_err_msg(cancel_msg[0], _fb, _goal_handle)
            elif not self._user_res_flag:
                self.print_and_append_err_msg(err_msg[2], _fb, _goal_handle)
            elif not self._f_bumper_pushed or not self._b_bumper_pushed:
                self.print_and_append_err_msg(err_msg[3], _fb, _goal_handle)

            self._interactive_check_tools.publish_feedback(fb_msg[1], _fb, _goal_handle)
            self.node.destroy_subscription(self._interactive_check_tools.sub)

            self.node.get_clock().sleep_for(Duration(seconds=5.0))
            self.bumper_reset_client()
            self.node.destroy_publisher(status_led_pub)
            self.node.destroy_subscription(base_f_bumper_sub)
            self.node.destroy_subscription(base_b_bumper_sub)
            self.node.destroy_subscription(user_response_sub)
            return self.error_msg

        except Exception as e:
            return [str(e)]
