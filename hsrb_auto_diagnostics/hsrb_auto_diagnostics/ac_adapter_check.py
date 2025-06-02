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
import hsrb_auto_diagnostics.check_tools as check_tools
from hsrb_auto_diagnostics.device_check import DeviceCheck
import numpy as np
from rclpy.duration import Duration
from std_msgs.msg import Bool, UInt8

"""
AC Adapter Test Class
"""


class ACAdapterCheck(DeviceCheck):
    def __init__(self, *args):
        super().__init__(*args)
        if not self.node.has_parameter("ac_adapter_topic"):
            self.node.declare_parameter("ac_adapter_topic", "/hsrb/ac_adapter")

    def ac_adapter_callback(self, data):
        self._ac_adapter.append(data.data)

    def get_sub_data_with_duration(self, duration):
        self._ac_adapter = []
        self.node.get_clock().sleep_for(Duration(seconds=duration))
        self.node.get_logger().info(self._ac_adapter)
        return self._ac_adapter

    # Whether it is a list increasing within the range of lower_limit to upper_limit
    def check_data_pattern(self, src_list, lower_limit, upper_limit):
        if src_list[0] != lower_limit or src_list[-1] != upper_limit:
            return False
        src_array = np.array(src_list)
        return all((src_array[1:] - src_array[:-1]) >= 0)

    def check_interactive(self, _fb, _as):
        self._ac_adapter = []
        err_msg = ['Diagnostic Canceled', 'AC Adapter Error']
        question = ['?Push the OK button if you plug in the power source '
                    'cable and turn on the charging unit’s switch.',
                    '?Push the OK button if you pull the power source cable',
                    '?Push the OK button if you plug in '
                    'the power source cable']

        ac_adapter_sub = self.node.create_subscription(
            UInt8,
            self.node.get_parameter("ac_adapter_topic").get_parameter_value().string_value,
            self.ac_adapter_callback, 10)

        tool = check_tools.Interactive(self.node, '/user_response', Bool)

        # Prompt to insert cables and turn on the power in advance
        self.error_msg = tool.wait_for_user_response(
            _fb, _as, question[0], err_msg[0], 30.0)
        if len(self.error_msg) == 0:
            # When the power plug is removed, the message for the automatic charging terminal is 6
            self.error_msg = tool.wait_for_user_response(
                _fb, _as, question[1], err_msg[0], 30.0)
        if len(self.error_msg) == 0:
            data = self.get_sub_data_with_duration(1.0)
            if set(data) != {6}:
                self.error_msg.append(err_msg[1])
            else:
                # When the power plug is inserted, the message for the automatic charging terminal increases from 1 to 2
                # Depending on the timing of the user_response returning
                # It is highly likely to miss 1, so it's okay if all elements are 2
                self.error_msg = tool.wait_for_user_response(
                    _fb, _as, question[2], err_msg[0], 30.0)
                if len(self.error_msg) == 0:
                    data = self.get_sub_data_with_duration(5.0)
                    if set(data) != {2} and (
                            not self.check_data_pattern(data, 1, 2)):
                        self.error_msg.append(err_msg[1])

        self.node.destroy_subscription(ac_adapter_sub)
        self.node.destroy_subscription(tool.sub)

        return self.error_msg
