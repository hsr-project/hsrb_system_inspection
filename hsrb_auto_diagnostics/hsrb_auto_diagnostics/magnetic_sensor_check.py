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
from std_msgs.msg import Bool

"""
MagneticSensor Test Class
"""


class MagneticSensorCheck(DeviceCheck):

    _SENSOR_1_TOPIC = '/base_magnetic_sensor_1'
    _SENSOR_2_TOPIC = '/base_magnetic_sensor_2'

    def __init__(self, *args):
        super().__init__(*args)
        self._sensor_1_responded = False
        self._sensor_2_responded = False

        if not self.node.has_parameter('wait_for_placement_response_timeout'):
            self.node.declare_parameter('wait_for_placement_response_timeout', 20.0)
        if not self.node.has_parameter('wait_for_removal_response_timeout'):
            self.node.declare_parameter('wait_for_removal_response_timeout', 60.0)

    def wait_for_user_response(self, _fb, _as, fb_msg, err_msg, latency):
        self._interactive_check_tools.publish_feedback(fb_msg, _fb, _as)
        if not any(self._interactive_check_tools.subscribe_user_response
                   (_as, latency)):
            self.error_msg.append(err_msg)
            return False
        return True

    def judge_sensor_1_msg(self, sub_duration, _as):
        subscribed_topic, _ = self._sensor_1_sub_check_tools.subscribe_topic(
            'data', sub_duration, _as)
        return any(subscribed_topic)

    def judge_sensor_2_msg(self, sub_duration, _as):
        subscribed_topic, _ = self._sensor_2_sub_check_tools.subscribe_topic(
            'data', sub_duration, _as)
        return any(subscribed_topic)

    def sensor_1_callback(self, data):
        if data.data:
            self._sensor_1_responded = True

    def sensor_2_callback(self, data):
        if data.data:
            self._sensor_2_responded = True

    def print_and_append_err_msg(self, msg, _fb, _as):
        self._interactive_check_tools.publish_feedback(msg, _fb, _as)
        self.error_msg.append(msg)

    def check_interactive(self, _fb, _as):
        self._sensor_1_sub_check_tools = check_tools.SubCheck(
            self.node, self._SENSOR_1_TOPIC, Bool)
        self._sensor_2_sub_check_tools = check_tools.SubCheck(
            self.node, self._SENSOR_2_TOPIC, Bool)

        self._interactive_check_tools = check_tools.Interactive(
            self.node, '/user_response', Bool)
        self.error_msg = []

        wait_for_placement_timeout = self.node.get_parameter(
            'wait_for_placement_response_timeout').get_parameter_value().double_value
        wait_for_removal_timeout = self.node.get_parameter(
            'wait_for_removal_response_timeout').get_parameter_value().double_value

        try:
            """
            for front sensor
            """
            if not self.wait_for_user_response(_fb, _as, '?Please remove a '
                                               'magnet around the robot.'
                                               ' Then, push the OK button',
                                               'Not ready to check'
                                               ' magnetic sensor',
                                               20.0):
                self.node.destroy_subscription(self._interactive_check_tools.sub)
                return self.error_msg
            if (self.judge_sensor_1_msg(2.0, _as) or self.judge_sensor_2_msg(2.0, _as)):
                self.print_and_append_err_msg('Before setting a magnet, '
                                              'sensors return wrong data',
                                              _fb, _as)
                self.node.destroy_subscription(self._interactive_check_tools.sub)
                return self.error_msg
            self.node.create_subscription(
                Bool,
                self._SENSOR_1_TOPIC,
                self.sensor_1_callback, 10)
            if not self.wait_for_user_response(_fb, _as, '?Please set a '
                                               'magnet front right of the '
                                               'robot. Then, push the'
                                               ' OK button',
                                               'Not ready to set a magnet',
                                               40.0):
                self.node.destroy_subscription(self._interactive_check_tools.sub)
                return self.error_msg
            if not self.wait_for_user_response(_fb, _as, '?Please push the'
                                               ' button if LED is blinking'
                                               ' on and off',
                                               'LED is not blinking',
                                               wait_for_placement_timeout):
                self.node.destroy_subscription(self._interactive_check_tools.sub)
                return self.error_msg
            if not self._sensor_1_responded:
                self.print_and_append_err_msg('There are no responses of the '
                                              'front right magnetic sensor',
                                              _fb, _as)
                self.node.destroy_subscription(self._interactive_check_tools.sub)
                return self.error_msg
            if not self.wait_for_user_response(_fb, _as, '?Please remove a '
                                               'magnet and push the release '
                                               'button. Then, push the '
                                               'OK button',
                                               'Robot cannot recover',
                                               wait_for_removal_timeout):
                self.node.destroy_subscription(self._interactive_check_tools.sub)
                return self.error_msg
            if not self.wait_for_user_response(_fb, _as, '?Please push the'
                                               ' button if LED is '
                                               'lighting-up',
                                               'LED is not lighting-up',
                                               20.0):
                self.node.destroy_subscription(self._interactive_check_tools.sub)
                return self.error_msg

            """
            for rear sensor
            """
            self.node.create_subscription(
                Bool,
                self._SENSOR_2_TOPIC,
                self.sensor_2_callback, 10)
            if not self.wait_for_user_response(_fb, _as, '?Please set a '
                                               'magnet rear left of the '
                                               'robot. Then, push the'
                                               ' OK button',
                                               'Not ready to set a magnet',
                                               40.0):
                self.node.destroy_subscription(self._interactive_check_tools.sub)
                return self.error_msg
            if not self.wait_for_user_response(_fb, _as, '?Please push the'
                                               ' button if LED is blinking'
                                               ' on and off',
                                               'LED is not blinking',
                                               wait_for_placement_timeout):
                self.node.destroy_subscription(self._interactive_check_tools.sub)
                return self.error_msg
            if not self._sensor_2_responded:
                self.print_and_append_err_msg('There are no responses of the '
                                              'rear left magnetic sensor',
                                              _fb, _as)
                self.node.destroy_subscription(self._interactive_check_tools.sub)
                return self.error_msg
            if not self.wait_for_user_response(_fb, _as, '?Please remove a '
                                               'magnet and push the release '
                                               'button. Then, push the '
                                               'OK button',
                                               'Robot cannot recover',
                                               wait_for_removal_timeout):
                self.node.destroy_subscription(self._interactive_check_tools.sub)
                return self.error_msg
            self.wait_for_user_response(_fb, _as, '?Please push the'
                                        ' button if LED is '
                                        'lighting-up',
                                        'LED is not lighting-up',
                                        20.0)
            self.node.destroy_subscription(self._interactive_check_tools.sub)
            return self.error_msg

        except Exception as e:
            return [str(e)]
