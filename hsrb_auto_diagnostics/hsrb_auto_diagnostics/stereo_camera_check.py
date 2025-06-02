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
from sensor_msgs.msg import Image


"""
StereoCamera Test Class
"""


class LStereoCameraCheck(DeviceCheck):
    VID = 'vendor_id'
    PID = 'product_id'
    PARAM_HEIGHT_SIZE = 'height_size'
    PARAM_WIDTH_SIZE = 'width_size'
    DEVICE_NUM = 'device_num'
    NORMAL_FREQUENCY = 1.5

    def __init__(self, *args):
        super().__init__(*args)

        if not self.node.has_parameter(self.VID):
            self.node.declare_parameter(self.VID, '0x1e10')
        if not self.node.has_parameter(self.PID):
            self.node.declare_parameter(self.PID, '0x2004')
        if not self.node.has_parameter(self.DEVICE_NUM):
            self.node.declare_parameter(self.DEVICE_NUM, 2)
        if not self.node.has_parameter(self.PARAM_HEIGHT_SIZE):
            self.node.declare_parameter(self.PARAM_HEIGHT_SIZE, 3840)
        if not self.node.has_parameter(self.PARAM_WIDTH_SIZE):
            self.node.declare_parameter(self.PARAM_WIDTH_SIZE, 960)

    def check_connection(self):
        vid = self.node.get_parameter(self.VID).get_parameter_value().string_value
        pid = self.node.get_parameter(self.PID).get_parameter_value().string_value
        device_num = self.node.get_parameter(self.DEVICE_NUM).get_parameter_value().integer_value
        if (check_tools.check_num_usb_devices(vid, pid) == device_num):
            return True
        else:
            False

    def check_sub_data(self, _as):
        height = self.node.get_parameter(self.PARAM_HEIGHT_SIZE).get_parameter_value().integer_value
        width = self.node.get_parameter(self.PARAM_WIDTH_SIZE).get_parameter_value().integer_value
        expected_size = height * width
        try:
            sub_check_tools = check_tools.SubCheck(
                self.node,
                '/head_l_stereo_camera/image_raw',
                Image)
            return sub_check_tools.check_camera_msg([expected_size], _as,
                                                    self.NORMAL_FREQUENCY)
        except Exception as e:
            return [str(e)]


class RStereoCameraCheck(DeviceCheck):
    VID = 'vendor_id'
    PID = 'product_id'
    PARAM_HEIGHT_SIZE = 'height_size'
    PARAM_WIDTH_SIZE = 'width_size'
    DEVICE_NUM = 'device_num'
    NORMAL_FREQUENCY = 1.5

    def __init__(self, *args):
        super().__init__(*args)

        if not self.node.has_parameter(self.VID):
            self.node.declare_parameter(self.VID, '0x1e10')
        if not self.node.has_parameter(self.PID):
            self.node.declare_parameter(self.PID, '0x2004')
        if not self.node.has_parameter(self.DEVICE_NUM):
            self.node.declare_parameter(self.DEVICE_NUM, 2)
        if not self.node.has_parameter(self.PARAM_HEIGHT_SIZE):
            self.node.declare_parameter(self.PARAM_HEIGHT_SIZE, 3840)
        if not self.node.has_parameter(self.PARAM_WIDTH_SIZE):
            self.node.declare_parameter(self.PARAM_WIDTH_SIZE, 960)

    def check_connection(self):
        vid = self.node.get_parameter(self.VID).get_parameter_value().string_value
        pid = self.node.get_parameter(self.PID).get_parameter_value().string_value
        device_num = self.node.get_parameter(self.DEVICE_NUM).get_parameter_value().integer_value
        if (check_tools.check_num_usb_devices(vid, pid) == device_num):
            return True
        else:
            False

    def check_sub_data(self, _as):
        height = self.node.get_parameter(self.PARAM_HEIGHT_SIZE).get_parameter_value().integer_value
        width = self.node.get_parameter(self.PARAM_WIDTH_SIZE).get_parameter_value().integer_value
        expected_size = height * width
        try:
            sub_check_tools = check_tools.SubCheck(
                self.node,
                '/head_r_stereo_camera/image_raw',
                Image)
            return sub_check_tools.check_camera_msg([expected_size], _as,
                                                    self.NORMAL_FREQUENCY)
        except Exception as e:
            return [str(e)]
