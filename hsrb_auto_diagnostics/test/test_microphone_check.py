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
from unittest.mock import patch

from hsrb_auto_diagnostics.microphone_check import MicrophoneCheck
from hsrb_auto_diagnostics_msgs.action import HsrbDeviceCheck
import launch
import launch_ros.actions
import launch_testing
from nose.tools import assert_less, eq_, ok_
import pytest
import rclpy
from rclpy.executors import MultiThreadedExecutor
import usb

from . import usb_busses_stub
from .goal_handle_stub import GoalHandleStub
from .utils import update_val_client_impl

PKG = 'hsrb_auto_diagnostics'
NAME = 'test_microphone_check'

MIC_CONTAIN = (('0x1415', '0x2000'), ('0x3353', '0x5345'))
MIC_NO_CONTAIN = (('0x3345', '0x3245'), ('0x3353', '0x5345'))
MIC_VID_CONTAIN = (('0x1415', '0x8491'), )
MIC_PID_CONTAIN = (('0x1489', '0x2000'), ('0x3353', '0x5345'),
                   ('0x1d7a', '0x126'))


@pytest.mark.launch_test
def generate_test_description():
    user_response_stub_node = launch_ros.actions.Node(
        executable='test/bool_pub_stub.py',
        name='user_response_stub_node',
        remappings=[('topic_name', '/user_response')],
        output='screen'
    )
    return launch.LaunchDescription([
        user_response_stub_node,
        launch_testing.actions.ReadyToTest()
    ])


class TestMicrophoneCheck(unittest.TestCase):
    _fb = HsrbDeviceCheck.Feedback()

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node("test_microphone_check",
                                      allow_undeclared_parameters=True,
                                      automatically_declare_parameters_from_overrides=True)
        self.executor = MultiThreadedExecutor()
        self.thread = threading.Thread(target=rclpy.spin, args=(self.node, self.executor), daemon=True)
        self.thread.start()

        self._instance = MicrophoneCheck(self.node)

    def tearDown(self):
        self.node.destroy_node()

    def set_user_response_client(self, data_type, rate=10.0):
        update_val_client_impl(self.node, '/user_response_stub_node/set_bool_button_data',
                               device='none', rate=rate, data_type=data_type)

    def test_connection_ok(self):
        ubs = usb_busses_stub.UsbBussesStub(MIC_CONTAIN)
        usb.busses = ubs
        ok_(self._instance.check_connection())

    def test_connection_ng1(self):
        ubs = usb_busses_stub.UsbBussesStub(MIC_NO_CONTAIN)
        usb.busses = ubs
        ok_(not self._instance.check_connection())

    def test_connection_ng2(self):
        ubs = usb_busses_stub.UsbBussesStub(MIC_VID_CONTAIN)
        usb.busses = ubs
        ok_(not self._instance.check_connection())

    def test_connection_ng3(self):
        ubs = usb_busses_stub.UsbBussesStub(MIC_PID_CONTAIN)
        usb.busses = ubs
        ok_(not self._instance.check_connection())

    @patch('os.remove')
    def test_user_ok(self, remove_mock):
        self.set_user_response_client('true')
        eq_(self._instance.check_interactive(self._fb, GoalHandleStub()), [])
        self.set_user_response_client('none')
        ok_(remove_mock.called)

    @patch('os.remove')
    def test_user_ng(self, remove_mock):
        self.set_user_response_client('false')
        eq_(self._instance.check_interactive(self._fb, GoalHandleStub()),
            ['Microphone Error'])
        self.set_user_response_client('none')
        ok_(remove_mock.called)

    @patch('os.remove')
    @patch('subprocess.call')
    def test_user_cancel(self, call_mock, remove_mock):
        start_stamp = self.node.get_clock().now()
        eq_(self._instance.check_interactive(self._fb, GoalHandleStub()),
            ['Diagnostic Canceled'])
        duration_sec = (self.node.get_clock().now() - start_stamp).nanoseconds / 1e9
        # Recording 5 sec, Playing 0 sec(mock), Waiting for response 10 sec
        assert_less(abs(duration_sec - 15.0), 1.0)
        ok_(call_mock.called)
        ok_(remove_mock.called)

        # Recording 5 sec, Playing 0 sec(mock), Waiting for response 3 sec
        wait_for_response_timeout = self.node.get_parameter(
            'wait_for_response_timeout').get_parameter_value().double_value
        self.node.set_parameters([
            rclpy.parameter.Parameter('wait_for_response_timeout',
                                      rclpy.Parameter.Type.DOUBLE,
                                      3.0)])
        start_stamp = self.node.get_clock().now()
        eq_(self._instance.check_interactive(self._fb, GoalHandleStub()),
            ['Diagnostic Canceled'])
        duration_sec = (self.node.get_clock().now() - start_stamp).nanoseconds / 1e9
        assert_less(abs(duration_sec - 8.0), 1.0)
        self.node.set_parameters([
            rclpy.parameter.Parameter('wait_for_response_timeout',
                                      rclpy.Parameter.Type.DOUBLE,
                                      wait_for_response_timeout)])
