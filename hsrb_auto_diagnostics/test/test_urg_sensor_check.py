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
import subprocess
import threading
import unittest

from hsrb_auto_diagnostics.urg_sensor_check import UrgSensorCheck
import launch
import launch_ros.actions
import launch_testing
from nose.tools import eq_, ok_
import pytest
import rclpy
from rclpy.executors import MultiThreadedExecutor

from .goal_handle_stub import GoalHandleStub
from .utils import update_val_client_impl

PKG = 'hsrb_auto_diagnostics'
NAME = 'test_urg_sensor_check'


def ping_connected(cmd):
    return True


def ping_not_connected(cmd):
    raise Exception


@pytest.mark.launch_test
def generate_test_description():
    urg_sensor_pub_stub_node = launch_ros.actions.Node(
        executable='test/urg_sensor_pub_stub_srv.py',
        name='urg_sensor_pub_stub_node',
        output='screen'
    )
    return launch.LaunchDescription([
        urg_sensor_pub_stub_node,
        launch_testing.actions.ReadyToTest()
    ])


class TestUrgSensorCheck(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node("test_urg_sensor_check",
                                      allow_undeclared_parameters=True,
                                      automatically_declare_parameters_from_overrides=True)
        self.node.declare_parameter("urg_ip", "10.255.255.1")

        self.executor = MultiThreadedExecutor()
        self.thread = threading.Thread(target=rclpy.spin, args=(self.node, self.executor), daemon=True)
        self.thread.start()

        self._instance = UrgSensorCheck(self.node)

    def tearDown(self):
        self.node.destroy_node()

    def update_val_client(self, rate, data_type):
        update_val_client_impl(self.node, '/urg_sensor_pub_stub_node/update_val',
                               device='urg_sensor', rate=rate, data_type=data_type)

    def test_ping_ok(self):
        subprocess.check_output = ping_connected
        ok_(self._instance.check_connection())

    def test_ping_ng(self):
        subprocess.check_output = ping_not_connected
        ok_(not self._instance.check_connection())

    def test_sub_msg_ok(self):
        self.update_val_client(40.0, 'ok')
        eq_(self._instance.check_sub_data(GoalHandleStub()), [])

    def test_sub_msg_unavailability(self):
        self.update_val_client(40.0, 'unavailability')
        eq_(self._instance.check_sub_data(GoalHandleStub()), ['Data Size Error'])

    def test_sub_msg_sluggish(self):
        self.update_val_client(20.0, 'sluggish')
        eq_(self._instance.check_sub_data(GoalHandleStub()),
            ['Subscribe Frequency Error'])

    def test_sub_msg_freezed(self):
        self.update_val_client(40.0, 'freezed')
        eq_(self._instance.check_sub_data(GoalHandleStub()), ['Data Freeze Error'])
