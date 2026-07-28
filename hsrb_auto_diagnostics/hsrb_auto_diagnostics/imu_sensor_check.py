#! /usr/bin/env python
# Copyright (c) 2026 TOYOTA MOTOR CORPORATION
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
from hsrb_auto_diagnostics.util import ParamQosReliability
from sensor_msgs.msg import Imu


"""
ImuSensor Test Class
"""


class ImuSensorCheck(DeviceCheck):
    DEVPATH = "dev_path"
    ROBOT_TYPE = "robot_type"
    CHECK_INFO_PARAM = "imu_check_info_"
    NORMAL_FREQUENCY = 90.0
    PARAM_FREQUENCY = "frequency"
    PARAM_TOPIC = 'topic'

    def __init__(self, *args):
        super().__init__(*args)

        if not self.node.has_parameter(self.DEVPATH):
            self.node.declare_parameter(self.DEVPATH, "/dev/ttyCTI0")
        if not self.node.has_parameter(self.ROBOT_TYPE):
            self.node.declare_parameter(self.ROBOT_TYPE, "hsrb")
        if not self.node.has_parameter(self.PARAM_FREQUENCY):
            self.node.declare_parameter(self.PARAM_FREQUENCY, self.NORMAL_FREQUENCY)
        if not self.node.has_parameter(self.PARAM_TOPIC):
            self.node.declare_parameter(self.PARAM_TOPIC, "/imu/data")

    def check_connection(self):
        dev_path = self.node.get_parameter(self.DEVPATH).get_parameter_value().string_value
        return check_tools.check_dev_path(dev_path)

    def param_to_test_info(self, robot_name):
        # test_key = ["name", do_freeze_check, l_limit, u_limit]
        test_info = []

        # name must always follow this order.
        param_names = [
            "ori_y",
            "ori_z",
            "ori_w",
            "ang_vel_x",
            "ang_vel_y",
            "ang_vel_z",
            "linear_acc_x",
            "linear_acc_y",
            "linear_acc_z"
        ]

        # Stored in param is <name>: {
        #   do_freeze_check: Bool, lower_limit: Double, upper_limit: Double}
        # The dictionary-type param is saved with keys connected by ".".
        for param_name in param_names:
            do_freeze_check = self.node.get_parameter(
                robot_name + "." + param_name + ".do_freeze_check").get_parameter_value().bool_value
            lower_limit = self.node.get_parameter(
                robot_name + "." + param_name + ".lower_limit").get_parameter_value().double_value
            upper_limit = self.node.get_parameter(
                robot_name + "." + param_name + ".upper_limit").get_parameter_value().double_value
            # For formatting the output, ":" was added to name, so we follow that.
            test_info.append([param_name + ":", do_freeze_check, lower_limit, upper_limit])
        return test_info

    def check_sub_data(self, _as):
        try:
            topic = self.node.get_parameter(self.PARAM_TOPIC).get_parameter_value().string_value
            sub_check_tools = check_tools.SubCheck(self.node, topic, Imu)
            robot_type = self.node.get_parameter(self.ROBOT_TYPE).get_parameter_value().string_value
            robot_version = os.environ.get("ROBOT_VERSION")
            if robot_version is not None:
                robot_type = robot_version.replace('"', '').split('-')[0].lower()
            frequency = self.node.get_parameter(self.PARAM_FREQUENCY).get_parameter_value().double_value

            if self.node.has_parameter(self.CHECK_INFO_PARAM + robot_type):
                test_info = self.param_to_test_info(self.CHECK_INFO_PARAM + robot_type)
            else:
                try:
                    robot_type = self.node.get_parameter(
                        self.ROBOT_TYPE).get_parameter_value().string_value
                    test_info = self.param_to_test_info(self.CHECK_INFO_PARAM + robot_type)
                except Exception:
                    self.node.get_logger().error(
                        "Invalid ROBOT_VERSION:{0}".format(robot_version))
            return sub_check_tools.check_imu_msg(
                _as, frequency, test_info, ParamQosReliability(self.node))
        except Exception as e:
            return [str(e)]
