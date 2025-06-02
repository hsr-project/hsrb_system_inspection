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
import os

import hsrb_auto_diagnostics.check_tools as check_tools
from hsrb_auto_diagnostics.device_check import DeviceCheck
from rclpy.duration import Duration
from std_msgs.msg import Bool

"""
Speaker Test Class
"""


class SpeakerCheck(DeviceCheck):
    PARAM_NAME = 'wav_file_name'
    PLAY_CMD = 'paplay '

    def __init__(self, *args):
        super().__init__(*args)

    def check_interactive(self, _fb, _as):
        fb_msg = ['Please listen HSR voice']
        err_msg = ['Speaker Error']
        question = ['?Push the OK button if you can hear [HSR-start]']

        self._interactive_check_tools = check_tools.Interactive(
            self.node, '/user_response', Bool)

        if self.node.has_parameter(self.PARAM_NAME):
            self.FILE_PATH = self.node.get_parameter(self.PARAM_NAME).get_parameter_value().string_value
        else:
            self.node.get_logger().error('Parameter [%s] is not set' % self.PARAM_NAME)
            self.FILE_PATH = 'invalid_file_path'

        self._interactive_check_tools.publish_feedback(fb_msg[0], _fb, _as)
        os.system(self.PLAY_CMD + self.FILE_PATH)
        self.node.get_clock().sleep_for(Duration(seconds=1.0))
        self.error_msg = self._interactive_check_tools.wait_for_user_response(
            _fb, _as, question[0], err_msg[0], 10.0)
        self.node.destroy_subscription(self._interactive_check_tools.sub)

        return self.error_msg
