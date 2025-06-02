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
import os

from ament_index_python.packages import get_package_share_directory
import hsrb_auto_diagnostics.check_tools as check_tools
from hsrb_auto_diagnostics.speaker_check import SpeakerCheck
from rclpy.duration import Duration
from std_msgs.msg import Bool
from std_srvs.srv import SetBool

"""
Speaker with Mute Function Test Class
"""


class SpeakerWithMuteCheck(SpeakerCheck):
    PARAM_NAME = 'wav_file_name'
    PLAY_CMD = 'paplay '

    def __init__(self, *args):
        super().__init__(*args)

        if not self.node.has_parameter("mute_srv"):
            self.node.declare_parameter("mute_srv", "/hsrb/mute")
        if not self.node.has_parameter(self.PARAM_NAME):
            package_dir = get_package_share_directory('hsrb_auto_diagnostics')
            self.node.declare_parameter(self.PARAM_NAME,
                                        package_dir + "/media/hsr-start.wav")

    def check_interactive(self, _fb, _as):
        fb_msg = ['Now, play the [HSR-start] sound']
        err_msg = ['Mute Error']
        question = ['?Push the OK button if you CANNOT hear [HSR-start]']
        mute_srv = self.node.get_parameter("mute_srv").get_parameter_value().string_value
        mute_client = self.node.create_client(SetBool, mute_srv)
        mute_client.wait_for_service(timeout_sec=30.0)

        tool = check_tools.Interactive(self.node, '/user_response', Bool)

        # Mute test added after R501
        self.node.get_logger().info("mute...")
        future = mute_client.call_async(SetBool.Request(data=True))
        init_time = self.node.get_clock().now()
        now = init_time
        while not future.done():
            now = self.node.get_clock().now()
            if (now - init_time) > Duration(seconds=3.0):
                self.node.get_logger().error("mute timeout.")
                future.cancel()
                break
            self.node.get_clock().sleep_for(Duration(seconds=0.1))
        self.node.get_logger().info("mute done.")

        if self.node.has_parameter(self.PARAM_NAME):
            self.FILE_PATH = self.node.get_parameter(self.PARAM_NAME).get_parameter_value().string_value
        else:
            self.node.get_logger().error('Parameter [%s] is not set' % self.PARAM_NAME)
            self.FILE_PATH = 'invalid_file_path'
        tool.publish_feedback(fb_msg[0], _fb, _as)
        os.system(self.PLAY_CMD + self.FILE_PATH)
        self.node.get_clock().sleep_for(Duration(seconds=1.0))
        mute_result = tool.wait_for_user_response(
            _fb, _as, question[0], err_msg[0], 10.0)
        # Unmute
        self.node.get_logger().info("unmute...")
        future = mute_client.call_async(SetBool.Request(data=False))
        init_time = self.node.get_clock().now()
        now = init_time
        while not future.done():
            now = self.node.get_clock().now()
            if (now - init_time) > Duration(seconds=3.0):
                self.node.get_logger().error("unmute timeout.")
                future.cancel()
                break
            self.node.get_clock().sleep_for(Duration(seconds=0.1))
        self.node.get_logger().info("unmute done.")

        self.node.destroy_subscription(tool.sub)
        if len(mute_result) > 0:
            return mute_result
        # Speaker test before R501
        speaker_check_result = super(
            SpeakerWithMuteCheck, self).check_interactive(
                _fb, _as)
        return speaker_check_result
