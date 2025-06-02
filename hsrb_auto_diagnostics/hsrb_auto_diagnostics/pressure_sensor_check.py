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
from action_msgs.msg import GoalStatus
import hsrb_auto_diagnostics.check_tools as check_tools
from hsrb_auto_diagnostics.device_check import DeviceCheck
from rclpy.action import ActionClient
from rclpy.duration import Duration
from std_msgs.msg import Bool
from tmc_suction.action import SuctionControl

"""
PressureSensor Test Class
"""


# NOTE: The suction-related parts lack unit tests, and messages are not compatible with ROS 2.
# Need to implement further after waiting for ROS 2 conversion of tmc_drivers/tmc_suction.
class PressureSensorCheck(DeviceCheck):
    def __init__(self, *args):
        super().__init__(*args)

    def suction_control(self, on, action_timeout):
        arg = Bool()
        arg.data = on

        goal = SuctionControl.Goal(timeout=action_timeout,
                                   suction_on=arg)
        # NOTE: Interactive.action_client is only called here.
        if not (self._interactive_check_tools.action_client(self._client,
                                                            goal)):
            raise Exception('Wait for suction server TIMEOUT')

    def wait_for_user_response_wrapper(self, _fb, _as, fb_msg,
                                       err_msg, latency):
        self.error_msg = self._interactive_check_tools.wait_for_user_response(
            _fb, _as, fb_msg, err_msg, latency)

        if len(self.error_msg) == 0:
            return True
        elif len(self.error_msg) > 0:
            return False

    # NOTE: The content of this function is not compatible with ROS 2.
    def judge_action_response(self, timeout, expected_state,
                              unexpected_state, err_msg, timeout_msg):
        if self._client.wait_for_result(timeout):
            if self._client.get_state() == expected_state:
                return True
            elif self._client.get_state() == unexpected_state:
                self.error_msg.append(err_msg)
            else:
                self.node.get_logger().error(
                    'Suction Controller Action return state:%d',
                    self._client.get_state())
                self.error_msg.append('Suction Controller Error')
            return False
        else:
            self.error_msg.append(timeout_msg)
            return False

    def suction_test(self, _fb, _as):
        err_msg = ['Pressure Sensor Error',
                   'Suction Control Action does not work',
                   'Suction Error']
        question = ['?Push the OK button if pump is running',
                    '?Push the OK button if a card is suctioned rightly']

        self._interactive_check_tools = check_tools.Interactive(
            self.node, '/user_response', Bool)
        self._client = ActionClient(
            self, SuctionControl, '/hsrb/suction_control')
        self.error_msg = []

        try:
            u"""
            Abnormal case (Test without suction)
            """
            self.suction_control(True, Duration(2.0))
            if self.judge_action_response(Duration(3.0),
                                          GoalStatus.STATUS_ABORTED,
                                          GoalStatus.STATUS_SUCCEEDED,
                                          err_msg[0], err_msg[1]):

                u"""
                Normal case (Test by suctioning a card)
                """
                self.suction_control(True, Duration(40.0))
                if self.wait_for_user_response_wrapper(
                        _fb, _as, question[0], err_msg[2], 10.0):

                    if self.wait_for_user_response_wrapper(
                            _fb, _as, question[1], err_msg[2], 30.0):

                        self.judge_action_response(Duration(2.0),
                                                   GoalStatus.STATUS_SUCCEEDED,
                                                   GoalStatus.STATUS_ABORTED,
                                                   err_msg[0], err_msg[0])
            return self.error_msg
        except Exception as e:
            self.suction_control(False, Duration(0.0))
            raise Exception(e)

    def check_interactive(self, _fb, _as):
        try:
            self.suction_test(_fb, _as)
            self.suction_control(False, Duration(0.0))
            self.node.destroy_subscription(self._interactive_check_tools.sub)
            return self.error_msg
        except Exception as e:
            return [str(e)]
