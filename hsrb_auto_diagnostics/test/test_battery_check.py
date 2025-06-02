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

from hsrb_auto_diagnostics.battery_check import BatteryCheck
from hsrb_auto_diagnostics_msgs.action import HsrbDeviceCheck
import launch
import launch_ros.actions
import launch_testing
from nose.tools import eq_
import pytest
import rclpy
from rclpy.executors import MultiThreadedExecutor

from .goal_handle_stub import GoalHandleStub
from .utils import update_val_client_impl

PKG = 'hsrb_auto_diagnostics'
NAME = 'test_battery_check'
TOPIC_NAME = '/battery_state'


@pytest.mark.launch_test
def generate_test_description():
    battery_state_pub_stub_srv_node = launch_ros.actions.Node(
        executable='test/battery_state_pub_stub_srv.py',
        name='battery_state_pub_stub_node',
        output='screen'
    )
    return launch.LaunchDescription([
        battery_state_pub_stub_srv_node,
        launch_testing.actions.ReadyToTest()
    ])


class TestBatteryCheck(unittest.TestCase):
    _feedback = HsrbDeviceCheck.Feedback()
    _result = HsrbDeviceCheck.Result()

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node("test_battery_check",
                                      allow_undeclared_parameters=True,
                                      automatically_declare_parameters_from_overrides=True)
        self.executor = MultiThreadedExecutor()
        self.thread = threading.Thread(target=rclpy.spin, args=(self.node, self.executor), daemon=True)
        self.thread.start()

        self._instance = BatteryCheck(self.node)

    def tearDown(self):
        self.node.destroy_node()

    def update_val_client(self, data_type):
        update_val_client_impl(self.node, '/battery_state_pub_stub_node/update_val',
                               device='battery', rate=1.0, data_type=data_type)

    def test_sub_msg_ok(self):
        self.update_val_client('ok')
        self.node.get_logger().info("test_sub_msg_ok:")
        result = self._instance.check_sub_data(GoalHandleStub())
        eq_(result, [])

    def test_sub_msg_high_temperature(self):
        self.update_val_client('high_temperature')
        self.node.get_logger().info("test_sub_msg_high_temperature:")
        result = self._instance.check_sub_data(GoalHandleStub())
        eq_(result, ['temperature:Data Range Error'])

    def test_sub_msg_error_occurred(self):
        self.update_val_client('error_occurred')
        self.node.get_logger().info("test_sub_msg_error_occured:")
        result = self._instance.check_sub_data(GoalHandleStub())
        eq_(result, ['Status Error'])

    def test_sub_msg_all_false(self):
        self.update_val_client('all_false')
        self.node.get_logger().info("test_sub_msg_all_false:")
        result = self._instance.check_sub_data(GoalHandleStub())
        eq_(result, ['temperature:Data Range Error', 'Status Error'])
