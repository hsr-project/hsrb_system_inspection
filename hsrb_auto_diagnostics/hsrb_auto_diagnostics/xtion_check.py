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
import itertools

import hsrb_auto_diagnostics.check_tools as check_tools
from hsrb_auto_diagnostics.device_check import DeviceCheck
from rclpy.qos import (
    QoSProfile,
    QoSReliabilityPolicy,
)
from sensor_msgs.msg import Image
from sensor_msgs.msg import PointCloud2

"""
Xtion Test Class
"""


class RGBImageCheck(DeviceCheck):
    VID = 'vendor_id'
    PID = 'product_id'
    PARAM_HEIGHT_SIZE = 'rgb_height_size'
    PARAM_WIDTH_SIZE = 'rgb_width_size'
    PARAM_CHANNEL_SIZE = 'rgb_channel_size'
    NORMAL_FREQUENCY = 8.0

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

        if not self.node.has_parameter(self.VID):
            self.node.declare_parameter(self.VID, '0x1d27')
        if not self.node.has_parameter(self.PID):
            self.node.declare_parameter(self.PID, '0x601')
        if not self.node.has_parameter(self.PARAM_HEIGHT_SIZE):
            self.node.declare_parameter(self.PARAM_HEIGHT_SIZE, [480])
        if not self.node.has_parameter(self.PARAM_WIDTH_SIZE):
            self.node.declare_parameter(self.PARAM_WIDTH_SIZE, [640])
        if not self.node.has_parameter(self.PARAM_CHANNEL_SIZE):
            self.node.declare_parameter(self.PARAM_CHANNEL_SIZE, [2, 3])

    def check_connection(self):
        vendor_id = self.node.get_parameter(self.VID).get_parameter_value().string_value
        product_id = self.node.get_parameter(self.PID).get_parameter_value().string_value
        return check_tools.check_pyusb(vendor_id, product_id)

    def check_sub_data(self, _as):
        height = self.node.get_parameter(self.PARAM_HEIGHT_SIZE).get_parameter_value().integer_array_value
        width = self.node.get_parameter(self.PARAM_WIDTH_SIZE).get_parameter_value().integer_array_value
        channel = self.node.get_parameter(self.PARAM_CHANNEL_SIZE).get_parameter_value().integer_array_value
        expected_sizes = [a * b * c for (a, b, c)
                          in itertools.product(height, width, channel)]
        try:
            sub_check_tools = check_tools.SubCheck(
                self.node, '/head_rgbd_sensor/rgb/image_rect_color', Image)
            return sub_check_tools.check_camera_msg(expected_sizes, _as,
                                                    self.NORMAL_FREQUENCY)
        except Exception as e:
            return [str(e)]


class DepthImageCheck(DeviceCheck):
    VID = 'vendor_id'
    PID = 'product_id'
    PARAM_HEIGHT_SIZE = 'depth_height_size'
    PARAM_WIDTH_SIZE = 'depth_width_size'
    PARAM_CHANNEL_SIZE = 'depth_channel_size'
    NORMAL_FREQUENCY = 8.0

    def __init__(self, *args):
        super().__init__(*args)

        if not self.node.has_parameter(self.VID):
            self.node.declare_parameter(self.VID, '0x1d27')
        if not self.node.has_parameter(self.PID):
            self.node.declare_parameter(self.PID, '0x601')
        if not self.node.has_parameter(self.PARAM_HEIGHT_SIZE):
            self.node.declare_parameter(self.PARAM_HEIGHT_SIZE, 480)
        if not self.node.has_parameter(self.PARAM_WIDTH_SIZE):
            self.node.declare_parameter(self.PARAM_WIDTH_SIZE, 640)
        if not self.node.has_parameter(self.PARAM_CHANNEL_SIZE):
            self.node.declare_parameter(self.PARAM_CHANNEL_SIZE, 2)

    def check_connection(self):
        vendor_id = self.node.get_parameter(self.VID).get_parameter_value().string_value
        product_id = self.node.get_parameter(self.PID).get_parameter_value().string_value
        return check_tools.check_pyusb(vendor_id, product_id)

    def check_sub_data(self, _as):
        height = self.node.get_parameter(self.PARAM_HEIGHT_SIZE).get_parameter_value().integer_value
        width = self.node.get_parameter(self.PARAM_WIDTH_SIZE).get_parameter_value().integer_value
        channel = self.node.get_parameter(self.PARAM_CHANNEL_SIZE).get_parameter_value().integer_value
        expected_size = height * width * channel
        try:
            sub_check_tools = check_tools.SubCheck(
                self.node,
                '/head_rgbd_sensor/depth_registered/image_raw',
                Image)
            return sub_check_tools.check_camera_msg([expected_size], _as,
                                                    self.NORMAL_FREQUENCY)
        except Exception as e:
            return [str(e)]


class PointCloudCheck(DeviceCheck):
    VID = 'vendor_id'
    PID = 'product_id'
    PARAM_HEIGHT_SIZE = 'point_height_size'
    PARAM_ROWSTEP_SIZE = 'point_rowstep_size'
    NORMAL_FREQUENCY = 0.8

    def __init__(self, *args):
        super().__init__(*args)

        if not self.node.has_parameter(self.VID):
            self.node.declare_parameter(self.VID, '0x1d27')
        if not self.node.has_parameter(self.PID):
            self.node.declare_parameter(self.PID, '0x601')
        if not self.node.has_parameter(self.PARAM_HEIGHT_SIZE):
            self.node.declare_parameter(self.PARAM_HEIGHT_SIZE, 480)
        if not self.node.has_parameter(self.PARAM_ROWSTEP_SIZE):
            self.node.declare_parameter(self.PARAM_ROWSTEP_SIZE, 20480)

    def check_connection(self):
        vendor_id = self.node.get_parameter(self.VID).get_parameter_value().string_value
        product_id = self.node.get_parameter(self.PID).get_parameter_value().string_value
        return check_tools.check_pyusb(vendor_id, product_id)

    def check_sub_data(self, _as):
        height = self.node.get_parameter(self.PARAM_HEIGHT_SIZE).get_parameter_value().integer_value
        rowstep = self.node.get_parameter(self.PARAM_ROWSTEP_SIZE).get_parameter_value().integer_value
        expected_size = height * rowstep
        try:
            sub_check_tools = check_tools.SubCheck(
                self.node,
                '/head_rgbd_sensor/depth_registered/rectified_points',
                PointCloud2)
            return sub_check_tools.check_camera_msg(
                [expected_size], _as, self.NORMAL_FREQUENCY,
                qos_profile=QoSProfile(
                    depth=1,
                    reliability=QoSReliabilityPolicy.BEST_EFFORT
                ))
        except Exception as e:
            return [str(e)]
