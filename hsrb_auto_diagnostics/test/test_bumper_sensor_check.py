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

from hsrb_auto_diagnostics.bumper_sensor_check import BumperSensorCheck
from hsrb_auto_diagnostics.check_action import CheckAction
import launch
import launch_ros.actions
import launch_testing
from nose.tools import eq_
import pytest
import rclpy
from rclpy.executors import MultiThreadedExecutor
from std_srvs.srv import Empty

from .goal_handle_stub import GoalHandleStub
from .utils import update_val_client_impl

PKG = 'hsrb_auto_diagnostics'
NAME = 'test_bumper_sensor_check'


@pytest.mark.launch_test
def generate_test_description():
    bumper_sensor_stub_node = launch_ros.actions.Node(
        executable='test/bumper_sensor_stub.py',
        name='bumper_sensor_stub_node',
        output='screen'
    )
    user_response_stub_node = launch_ros.actions.Node(
        executable='test/bool_pub_stub.py',
        name='user_response_stub_node',
        remappings=[('topic_name', '/user_response')],
        output='screen'
    )
    return launch.LaunchDescription([
        bumper_sensor_stub_node,
        user_response_stub_node,
        launch_testing.actions.ReadyToTest()
    ])


class TestBumperSensorCheck(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node("test_bumper_sensor_check",
                                      allow_undeclared_parameters=True,
                                      automatically_declare_parameters_from_overrides=True)
        self.executor = MultiThreadedExecutor()
        self.thread = threading.Thread(target=rclpy.spin, args=(self.node, self.executor), daemon=True)
        self.thread.start()

        self.node.set_parameters([
            rclpy.parameter.Parameter("check_time", rclpy.Parameter.Type.DOUBLE, 10.0)])
        self._check_action = CheckAction(self.node, 'bumper_sensor',
                                         BumperSensorCheck)
        self._bumper_reset_srv = self.node.create_service(
            Empty, '/bumper/reset', self.dummy_bumper_reset_cb)

    def tearDown(self):
        self.node.destroy_service(self._bumper_reset_srv)
        self.node.destroy_node()

    def dummy_bumper_reset_cb(self, req, res):
        return res

    def set_data_client(self, topic_name, device, data_type, rate=10.0):
        update_val_client_impl(self.node, topic_name,
                               device=device, rate=rate, data_type=data_type)

    def set_bumper_sensor(self, pattern):
        self.set_data_client('/bumper_sensor_stub_node/'
                             'set_bumper_sensor_data',
                             'bumper', pattern)

    def set_user_response(self, data):
        self.set_data_client('/user_response_stub_node/'
                             'set_bool_button_data',
                             'user', data)

    def test_wrong_front_sensor(self):
        self.set_bumper_sensor('true_false')
        self.set_user_response('none')
        self._check_action.execute_cb(GoalHandleStub())
        eq_(self._check_action._result.error_msg,
            'Front Bumper Sensor Error')

    def test_wrong_back_sensor(self):
        self.set_bumper_sensor('false_true')
        self.set_user_response('none')
        self._check_action.execute_cb(GoalHandleStub())
        eq_(self._check_action._result.error_msg,
            'Back Bumper Sensor Error')

    def test_user_timeout(self):
        self.set_bumper_sensor('false_false')
        self.set_user_response('none')
        self._check_action.execute_cb(GoalHandleStub())
        eq_(self._check_action._result.error_msg,
            'Diagnostic Canceled')

    def test_not_push_bumper(self):
        self.set_bumper_sensor('false_false')
        self.set_user_response('true')
        self._check_action.execute_cb(GoalHandleStub())
        eq_(self._check_action._result.error_msg,
            'Please push bumpers at least once')

    def test_ng(self):
        self.set_bumper_sensor('false_false')
        self.set_user_response('false')
        self.node.get_logger().info("test_ng")
        self._check_action.execute_cb(GoalHandleStub())
        eq_(self._check_action._result.error_msg, 'Bumper Error')
        self.node.get_logger().info("test_ng done")
