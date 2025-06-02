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
# -*- coding: utf-8 -*-
import hsrb_auto_diagnostics.check_tools as check_tools
from hsrb_auto_diagnostics.device_check import DeviceCheck
import rclpy
from rclpy.duration import Duration
from std_msgs.msg import Bool
from tmc_voice_msgs.msg import Voice

"""
Multifunctional Button and LED Test Class
"""


class MultifunctionalButtonCheck(DeviceCheck):
    def __init__(self, *args):
        super().__init__(*args)
        self._user_res_flag = None
        self._multifunctional_button = None

        if not self.node.has_parameter('check_time'):
            self.node.declare_parameter('check_time', 60.0)
        if not self.node.has_parameter('is_check_mf_button_led'):
            self.node.declare_parameter('is_check_mf_button_led', True)
        if not self.node.has_parameter('multifunctional_button_led_topic'):
            self.node.declare_parameter("multifunctional_button_led_topic",
                                        "/hsrb/multifunctional_button_led")
        if not self.node.has_parameter('multifunctional_button_topic'):
            self.node.declare_parameter("multifunctional_button_topic",
                                        "/hsrb/multi_functional_button")

    def multifunctional_button_callback(self, data):
        self._multifunctional_button = data.data

    def user_res_callback(self, data):
        self._user_res_flag = data.data

    def wait_for_user_response_wrapper(self, _fb, _as, fb_msg,
                                       err_msg, latency):
        self.error_msg = self._check_tools.wait_for_user_response(
            _fb, _as, fb_msg, err_msg, latency)

        if len(self.error_msg) == 0:
            return True
        elif len(self.error_msg) > 0:
            return False

    def input_error_msg(self, cancel_msg, err_msg):
        if self._user_res_flag is None:
            self.error_msg = [cancel_msg]
        elif self._user_res_flag:
            self.error_msg = []
        else:
            self.error_msg = [err_msg]

    def check_interactive(self, _fb, _as):
        err_msg = ['Multifunctional button LED Error',
                   'Multifunctional button Error']
        question = ['?Please push the OK button'
                    ' if the multifunctional button LED is lighting.',
                    '?Please push the OK button if the multifunctional'
                    ' button LED turns off the lights.',
                    '?Push the OK button if you can hear'
                    '[Multifunctional button OK]']
        fb_msg = ['?If you push the multifunctional button, the LED is'
                  ' lighting. If you release it, the LED turn off the '
                  'lights. Please check and push the OK button '
                  'if it works correctly.',
                  'Please push the multifunctional button and'
                  ' listen robot voice']
        cancel_msg = ['Diagnostic Canceled']

        check_time = self.node.get_parameter('check_time').get_parameter_value().double_value
        is_check_mf_button_led = self.node.get_parameter(
            'is_check_mf_button_led').get_parameter_value().bool_value
        # loop_rate = self.node.create_rate(10, self.node.get_clock())

        led_pub = self.node.create_publisher(
            Bool,
            self.node.get_parameter(
                "multifunctional_button_led_topic"
            ).get_parameter_value().string_value,
            1)
        talk_request_pub = self.node.create_publisher(
            Voice,
            "/talk_request",
            1)
        self.node.create_subscription(
            Bool,
            self.node.get_parameter(
                "multifunctional_button_topic"
            ).get_parameter_value().string_value,
            self.multifunctional_button_callback,
            10)
        self.node.create_subscription(
            Bool,
            '/user_response',
            self.user_res_callback,
            10)

        self._check_tools = check_tools.Interactive(self.node, '/user_response', Bool)
        self.error_msg = []

        try:
            if is_check_mf_button_led:
                self.node.get_logger().info("led check true.")
                # First, check that the LED lights up and turns off (starting from the default inverse state)
                while (rclpy.ok() and led_pub.get_subscription_count() == 0):
                    self.node.get_clock().sleep_for(Duration(seconds=0.1))
                led_pub.publish(Bool(data=True))
                if self.wait_for_user_response_wrapper(
                        _fb, _as, question[0], err_msg[0], 10):

                    led_pub.publish(Bool(data=False))
                    if self.wait_for_user_response_wrapper(
                            _fb, _as, question[1], err_msg[0], 10):
                        # Sync the multifunction button ON/OFF with the LED
                        self._user_res_flag = None
                        self._check_tools.publish_feedback(fb_msg[0], _fb, _as)
                        init_time = self.node.get_clock().now()
                        while (self.node.get_clock().now() - init_time) < Duration(seconds=check_time):
                            if self._multifunctional_button is not None:
                                led_pub.publish(
                                    Bool(data=self._multifunctional_button))
                            if self._user_res_flag is not None:
                                break
                            self.node.get_clock().sleep_for(Duration(seconds=0.1))
                            # loop_rate.sleep()
                        self.input_error_msg(cancel_msg[0], err_msg[1])
            else:
                self.node.get_logger().info("led check false.")
                while (rclpy.ok() and talk_request_pub.get_subscription_count() == 0):
                    self.node.get_clock().sleep_for(Duration(seconds=0.1))
                # It speaks when the multifunction button is turned ON
                self._check_tools.publish_feedback(fb_msg[1], _fb, _as)
                self._user_res_flag = None
                init_time = self.node.get_clock().now()
                while (self.node.get_clock().now() - init_time) < Duration(seconds=check_time):
                    if self._multifunctional_button:
                        self.node.get_logger().info("button pushed.")
                        voice_msg = Voice()
                        voice_msg.language = 1
                        voice_msg.sentence = 'multifunctional button OK'
                        talk_request_pub.publish(voice_msg)
                        self.error_msg = \
                            self._check_tools.wait_for_user_response(
                                _fb, _as, question[2], err_msg[1], 10.0)
                        break
                    if self._user_res_flag is not None:
                        self.node.get_logger().info("user response.")
                        break
                    self.node.get_clock().sleep_for(Duration(seconds=0.1))
                    # loop_rate.sleep()
                self.node.get_logger().info("return msg.")
                self.input_error_msg(cancel_msg[0], err_msg[1])
            return self.error_msg
        except Exception as e:
            return [str(e)]
