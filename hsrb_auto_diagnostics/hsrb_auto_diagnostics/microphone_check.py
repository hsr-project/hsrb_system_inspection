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
import subprocess
import time

import hsrb_auto_diagnostics.check_tools as check_tools
from hsrb_auto_diagnostics.device_check import DeviceCheck
from std_msgs.msg import Bool

"""
Microphone Test Class
"""


class MicrophoneCheck(DeviceCheck):
    VID = 'vendor_id'
    PID = 'product_id'
    WAV_FILE_PATH = '/tmp/mic_check.wav'
    REC_CMD = ['/usr/bin/pacat', '-r']
    PLAY_CMD = ['/usr/bin/pacat', '-p', '--volume=100000', WAV_FILE_PATH]

    def __init__(self, *args):
        super().__init__(*args)

        if not self.node.has_parameter(self.VID):
            self.node.declare_parameter(self.VID, '0x1415')
        if not self.node.has_parameter(self.PID):
            self.node.declare_parameter(self.PID, '0x2000')
        if not self.node.has_parameter('wait_for_response_timeout'):
            self.node.declare_parameter('wait_for_response_timeout', 10.0)

    def check_connection(self):
        vid = self.node.get_parameter(self.VID).get_parameter_value().string_value
        pid = self.node.get_parameter(self.PID).get_parameter_value().string_value
        return check_tools.check_pyusb(vid, pid)

    def check_interactive(self, _fb, _as):
        fb_msg = ['Please speak to microphone for 5 seconds',
                  'Now, playing the data']
        question = ['?Push the OK button if you can hear '
                    'a playback of a recording']
        err_msg = ['Microphone Error']
        wait_for_response_timeout = self.node.get_parameter(
            'wait_for_response_timeout').get_parameter_value().double_value

        self._interactive_check_tools = check_tools.Interactive(
            self.node, '/user_response', Bool)

        self._interactive_check_tools.publish_feedback(fb_msg[0], _fb, _as)
        with open(self.WAV_FILE_PATH, 'wb') as f:
            p = subprocess.Popen(self.REC_CMD, stdout=f)
            time.sleep(5)
            p.kill()
        self._interactive_check_tools.publish_feedback(fb_msg[1], _fb, _as)
        subprocess.call(self.PLAY_CMD)

        self.error_msg = (self._interactive_check_tools.wait_for_user_response(
            _fb, _as, question[0], err_msg[0], wait_for_response_timeout))
        self.node.destroy_subscription(self._interactive_check_tools.sub)

        os.remove(self.WAV_FILE_PATH)

        return self.error_msg
