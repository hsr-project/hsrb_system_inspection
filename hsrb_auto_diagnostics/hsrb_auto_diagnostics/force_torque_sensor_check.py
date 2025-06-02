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

from geometry_msgs.msg import WrenchStamped
import hsrb_auto_diagnostics.check_tools as check_tools
from hsrb_auto_diagnostics.device_check import DeviceCheck

"""
ForceTorqueSensor Test Class
"""


class ForceTorqueSensorCheck(DeviceCheck):
    NORMAL_FREQUENCY = 90.0

    def __init__(self, *args):
        super().__init__(*args)

    def get_devpath(self):
        device = "/dev/dynpick"

        # ROBOT_VERSION=HSRB-PHASEX or "HSRB-PHASEX"
        robot_version = os.environ.get("ROBOT_VERSION")
        version = robot_version[:4]
        phase = int(robot_version[10:])

        if (version == "HSRB" and phase < 5):
            device = "/dev/ttyCTI3"

        return device

    def check_connection(self):
        return check_tools.check_dev_path(self.get_devpath())

    def check_sub_data(self, _as):
        try:
            sub_check_tools = check_tools.SubCheck(
                self.node,
                '/wrist_wrench/raw',
                WrenchStamped)
            return sub_check_tools.check_wrench_msg(_as,
                                                    self.NORMAL_FREQUENCY)
        except Exception as e:
            return [str(e)]
