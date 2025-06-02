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
DrivePower Test Class
"""


class DrivePowerCheck(DeviceCheck):
    def __init__(self, *args):
        super().__init__(*args)

    def wait_for_user_response(self, _fb, _as, fb_msg, err_msg, latency):
        self._interactive_check_tools.publish_feedback(fb_msg, _fb, _as)
        if not any(self._interactive_check_tools.subscribe_user_response
                   (_as, latency)):
            self.error_msg.append(err_msg)
            return False
        return True

    def check_interactive(self, _fb, _goal_handle):
        self._interactive_check_tools = check_tools.Interactive(
            self.node, '/user_response', Bool)
        self.drive_power_pub = self.node.create_publisher(
            Bool,
            '/command_drive_power',
            1)
        self.error_msg = []

        try:
            self._interactive_check_tools.publish_feedback('Please look '
                                                           'at status LED',
                                                           _fb, _goal_handle)
            data = Bool()
            data.data = False
            self.drive_power_pub.publish(data)
            if not self.wait_for_user_response(_fb, _goal_handle, '?Push the OK button'
                                               ' if status LED is flashing',
                                               'LED cannot flasing',
                                               10.0):
                self.node.destroy_subscription(self._interactive_check_tools.sub)
                return self.error_msg
            data.data = True
            self.drive_power_pub.publish(data)
            self.wait_for_user_response(_fb, _goal_handle, '?Push the OK button'
                                        ' if status LED is'
                                        ' lighting-up',
                                        'LED cannot lighting-up',
                                        10.0)
            self.node.destroy_subscription(self._interactive_check_tools.sub)
            return self.error_msg

        except Exception as e:
            return [str(e)]
