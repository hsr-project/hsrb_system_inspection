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
EmergencyButton Test Class
"""


class EmergencyButtonCheck(DeviceCheck):
    def __init__(self, *args):
        super().__init__(*args)

    def wait_for_user_response(self, _fb, _goal_handle, fb_msg, err_msg, latency):
        self.node.get_logger().info("wait for user reponse...")
        self._interactive_check_tools.publish_feedback(fb_msg, _fb, _goal_handle)
        if not any(self._interactive_check_tools.subscribe_user_response
                   (_goal_handle, latency)):
            self.error_msg.append(err_msg)
            self.node.get_logger().error("no user reponse.")
            return False
        self.node.get_logger().info("received user reponse.")
        return True

    def judge_runstop_button_msg(self, sub_duration, _goal_handle):
        subscribed_topic, _ = self._runstop_check_tools.subscribe_topic(
            'data', sub_duration, _goal_handle)
        return any(subscribed_topic)

    def print_and_append_err_msg(self, msg, _fb, _goal_handle):
        self._interactive_check_tools.publish_feedback(msg, _fb, _goal_handle)
        self.error_msg.append(msg)

    def check_interactive(self, _fb, _goal_handle):
        self._runstop_check_tools = check_tools.SubCheck(
            self.node, '/runstop_button', Bool)
        self._interactive_check_tools = check_tools.Interactive(
            self.node, '/user_response', Bool)
        self.error_msg = []

        try:
            if not self.wait_for_user_response(_fb, _goal_handle, '?Lock the '
                                               'emergency switch. Then,'
                                               ' push the OK button',
                                               'User response timeout(lock)',
                                               10.0):
                self.node.destroy_subscription(self._interactive_check_tools.sub)
                return self.error_msg
            if not self.judge_runstop_button_msg(2.0, _goal_handle):
                self.print_and_append_err_msg('Subscribed data is all False',
                                              _fb, _goal_handle)
                self.node.destroy_subscription(self._interactive_check_tools.sub)
                return self.error_msg
            if not self.wait_for_user_response(_fb, _goal_handle, '?Unlock the '
                                               ' emergency switch. Then,'
                                               ' push the OK button',
                                               'User response timeout(unlock)',
                                               10.0):
                self.node.destroy_subscription(self._interactive_check_tools.sub)
                return self.error_msg
            if self.judge_runstop_button_msg(2.0, _goal_handle):
                self.print_and_append_err_msg('Value True is in'
                                              ' subscribed data',
                                              _fb, _goal_handle)
            self.node.destroy_subscription(self._interactive_check_tools.sub)
            return self.error_msg

        except Exception as e:
            return [str(e)]
