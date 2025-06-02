#!/usr/bin/env python
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
import threading
import unittest

from hsrb_auto_diagnostics.stereo_camera_check import LStereoCameraCheck
from hsrb_auto_diagnostics.stereo_camera_check import RStereoCameraCheck
import launch
import launch_ros.actions
import launch_testing
from nose.tools import eq_, ok_
import pytest
import rclpy
from rclpy.executors import MultiThreadedExecutor
import usb

from . import usb_busses_stub
from .goal_handle_stub import GoalHandleStub
from .utils import update_val_client_impl

PKG = 'hsrb_auto_diagnostics'
NAME = 'test_stereo_camera_check'

CHAMELEON_CONTAIN2 = (('0x1e10', '0x2004'), ('0x1e10', '0x2004'))
CHAMELEON_CONTAIN1 = (('0x1e10', '0x2004'), ('0x3353', '0x5345'))
CHAMELEON_NO_CONTAIN = (('0x3345', '0x3245'), ('0x3353', '0x5345'))
CHAMELEON_VID_CONTAIN = (('0x1e10', '0x8491'), )
CHAMELEON_PID_CONTAIN = (('0x1489', '0x2000'), ('0x3353', '0x2004'),
                         ('0x1d7a', '0x126'))


@pytest.mark.launch_test
def generate_test_description():
    stereo_camera_pub_stub_node = launch_ros.actions.Node(
        executable='test/stereo_camera_pub_stub_srv.py',
        name='stereo_camera_pub_stub_node',
        output='screen'
    )
    return launch.LaunchDescription([
        stereo_camera_pub_stub_node,
        launch_testing.actions.ReadyToTest()
    ])


class TestStereoCameraCheck(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node("test_stereo_camera_check",
                                      allow_undeclared_parameters=True,
                                      automatically_declare_parameters_from_overrides=True)
        self.executor = MultiThreadedExecutor()
        self.thread = threading.Thread(target=rclpy.spin, args=(self.node, self.executor), daemon=True)
        self.thread.start()

        self._l_instance = LStereoCameraCheck(self.node)
        self._r_instance = RStereoCameraCheck(self.node)

    def tearDown(self):
        self.node.destroy_node()

    def update_val_client(self, device, rate, data_type):
        update_val_client_impl(self.node, '/stereo_camera_pub_stub_node/update_val',
                               device=device, rate=rate, data_type=data_type)

    def test_connection_ok(self):
        ubs = usb_busses_stub.UsbBussesStub(CHAMELEON_CONTAIN2)
        usb.busses = ubs
        ok_(self._l_instance.check_connection())
        ok_(self._r_instance.check_connection())

    def test_connection_ng1(self):
        ubs = usb_busses_stub.UsbBussesStub(CHAMELEON_CONTAIN1)
        usb.busses = ubs
        ok_(not self._l_instance.check_connection())
        ok_(not self._r_instance.check_connection())

    def test_connection_ng2(self):
        ubs = usb_busses_stub.UsbBussesStub(CHAMELEON_NO_CONTAIN)
        usb.busses = ubs
        ok_(not self._l_instance.check_connection())
        ok_(not self._r_instance.check_connection())

    def test_connection_ng3(self):
        ubs = usb_busses_stub.UsbBussesStub(CHAMELEON_VID_CONTAIN)
        usb.busses = ubs
        ok_(not self._l_instance.check_connection())
        ok_(not self._r_instance.check_connection())

    def test_connection_ng4(self):
        ubs = usb_busses_stub.UsbBussesStub(CHAMELEON_PID_CONTAIN)
        usb.busses = ubs
        ok_(not self._l_instance.check_connection())
        ok_(not self._r_instance.check_connection())

    def test_sub_msg_unavailable_l(self):
        self.update_val_client('l_stereo_camera', 2.0, 'empty')
        eq_(self._l_instance.check_sub_data(GoalHandleStub()),
            ['Data Size Error', 'Data Freeze Error'])
        self.update_val_client('stop', 1.0, 'empty')

    def test_sub_msg_unavailable_r(self):
        self.update_val_client('r_stereo_camera', 2.0, 'empty')
        eq_(self._r_instance.check_sub_data(GoalHandleStub()),
            ['Data Size Error', 'Data Freeze Error'])
        self.update_val_client('stop', 1.0, 'empty')

    def test_sub_msg_unavailable2_l(self):
        self.update_val_client('l_stereo_camera', 2.0, 'unavailability')
        eq_(self._l_instance.check_sub_data(GoalHandleStub()), ['Data Size Error'])
        self.update_val_client('stop', 1.0, 'empty')

    def test_sub_msg_unavailable2_r(self):
        self.update_val_client('r_stereo_camera', 2.0, 'unavailability')
        eq_(self._r_instance.check_sub_data(GoalHandleStub()), ['Data Size Error'])
        self.update_val_client('stop', 1.0, 'empty')

    def test_sub_msg_freeze_l(self):
        self.update_val_client('l_stereo_camera', 2.0, 'freezed')
        eq_(self._l_instance.check_sub_data(GoalHandleStub()), ['Data Freeze Error'])
        self.update_val_client('stop', 1.0, 'empty')

    def test_sub_msg_freeze_r(self):
        self.update_val_client('r_stereo_camera', 2.0, 'freezed')
        eq_(self._r_instance.check_sub_data(GoalHandleStub()), ['Data Freeze Error'])
        self.update_val_client('stop', 1.0, 'empty')

    def test_sub_msg_ok_l(self):
        self.update_val_client('l_stereo_camera', 2.0, 'ok')
        eq_(self._l_instance.check_sub_data(GoalHandleStub()), [])
        self.update_val_client('stop', 1.0, 'empty')

    def test_sub_msg_ok_r(self):
        self.update_val_client('r_stereo_camera', 2.0, 'ok')
        eq_(self._r_instance.check_sub_data(GoalHandleStub()), [])
        self.update_val_client('stop', 1.0, 'empty')

    def test_sub_msg_slow_l(self):
        self.update_val_client('l_stereo_camera', 1.0, 'sluggish')
        eq_(self._l_instance.check_sub_data(GoalHandleStub()),
            ['Subscribe Frequency Error'])
        self.update_val_client('stop', 1.0, 'empty')

    def test_sub_msg_slow_r(self):
        self.update_val_client('r_stereo_camera', 1.0, 'sluggish')
        eq_(self._r_instance.check_sub_data(GoalHandleStub()),
            ['Subscribe Frequency Error'])
        self.update_val_client('stop', 1.0, 'empty')

    def test_sub_msg_unavailable_slow_l(self):
        self.update_val_client('l_stereo_camera', 1.0,
                               'empty_sluggish')
        eq_(self._l_instance.check_sub_data(GoalHandleStub()),
            ['Data Size Error', 'Subscribe Frequency Error',
            'Data Freeze Error'])
        self.update_val_client('stop', 1.0, 'empty')

    def test_sub_msg_unavailable_slow_r(self):
        self.update_val_client('r_stereo_camera', 1.0,
                               'empty_sluggish')
        eq_(self._r_instance.check_sub_data(GoalHandleStub()),
            ['Data Size Error', 'Subscribe Frequency Error',
            'Data Freeze Error'])
        self.update_val_client('stop', 1.0, 'empty')

    def test_sub_msg_unavailable_slow2_l(self):
        self.update_val_client('l_stereo_camera', 1.0,
                               'unavailability_sluggish')
        eq_(self._l_instance.check_sub_data(GoalHandleStub()),
            ['Data Size Error', 'Subscribe Frequency Error'])
        self.update_val_client('stop', 1.0, 'empty')

    def test_sub_msg_unavailable_slow2_r(self):
        self.update_val_client('r_stereo_camera', 1.0,
                               'unavailability_sluggish')
        eq_(self._r_instance.check_sub_data(GoalHandleStub()),
            ['Data Size Error', 'Subscribe Frequency Error'])
        self.update_val_client('stop', 1.0, 'empty')

    def test_sub_msg_freeze_slow_l(self):
        self.update_val_client('l_stereo_camera', 1.0, 'sluggish_freezed')
        eq_(self._l_instance.check_sub_data(GoalHandleStub()),
            ['Subscribe Frequency Error', 'Data Freeze Error'])
        self.update_val_client('stop', 1.0, 'empty')

    def test_sub_msg_freeze_slow_r(self):
        self.update_val_client('r_stereo_camera', 1.0, 'sluggish_freezed')
        eq_(self._r_instance.check_sub_data(GoalHandleStub()),
            ['Subscribe Frequency Error', 'Data Freeze Error'])
        self.update_val_client('stop', 1.0, 'empty')
