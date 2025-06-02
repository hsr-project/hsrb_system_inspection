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
from hsrb_auto_diagnostics_msgs.action import HsrbDeviceCheck
from rclpy.action import ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup


"""
Class for ROS ActionServer
"""


class CheckAction(object):
    _feedback = HsrbDeviceCheck.Feedback()
    _result = HsrbDeviceCheck.Result()

    def __init__(self, node, name, obj):
        self.node = node
        self.device_name = name
        self.obj = obj

        self._action_name = self.device_name + '_check'
        self._action_server = ActionServer(
            self.node,
            HsrbDeviceCheck,
            self._action_name,
            execute_callback=self.execute_cb,
            callback_group=ReentrantCallbackGroup())

    def form_result(self, result_list):
        return str(result_list)[1:][:-1].replace("'", "")

    def execute_cb(self, goal):
        device = self.obj(self.node)
        if device.check_connection():
            self._feedback.feedback_msg = 'Connection: OK'
            goal.publish_feedback(self._feedback)
            check_sub_result = device.check_sub_data(goal)
            if len(check_sub_result) == 0:
                self._feedback.feedback_msg = 'Subscribing Data: OK'
                goal.publish_feedback(self._feedback)
                interactive_result = device.check_interactive(
                    self._feedback, goal)
                if len(interactive_result) == 0:
                    self._result.ok = True
                    self._result.error_msg = ''
                    self._feedback.feedback_msg = self._result.error_msg
                    self.node.get_logger().info('Device [%s]: OK!' % self.device_name)
                else:
                    self._result.ok = False
                    self._result.error_msg = (self.form_result
                                              (interactive_result))
                    self._feedback.feedback_msg = self._result.error_msg
                    self.node.get_logger().warn('%s Interactive Test: NG' %
                                                self.device_name)
            else:
                self._result.ok = False
                self._result.error_msg = self.form_result(check_sub_result)
                self._feedback.feedback_msg = self._result.error_msg
                self.node.get_logger().warn('%s Message Test: NG' % self.device_name)
        else:
            self._result.ok = False
            self._result.error_msg = 'Connection Error'
            self._feedback.feedback_msg = self._result.error_msg
            self.node.get_logger().warn('%s Connection Test: NG' % self.device_name)

        goal.publish_feedback(self._feedback)
        goal.succeed()

        return self._result
