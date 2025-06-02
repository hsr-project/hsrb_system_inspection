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
# -*- coding: utf-8 -*-
import threading
import unittest
from unittest.mock import patch

from hsrb_auto_diagnostics.check_action import CheckAction
from hsrb_auto_diagnostics.multi_function_check import (
    MultifunctionalButtonCheck
)
import launch
import launch_ros.actions
import launch_testing
from nose.tools import eq_
import pytest
import rclpy
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import Bool
from tmc_voice_msgs.msg import Voice

from .goal_handle_stub import GoalHandleStub
from .utils import update_val_client_impl

PKG = 'hsrb_auto_diagnostics'
NAME = 'test_multifunctional_button_check'


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


class TestMultifunctionalButtonCheck(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node("test_multifunctional_button_check",
                                      allow_undeclared_parameters=True,
                                      automatically_declare_parameters_from_overrides=True)
        self.executor = MultiThreadedExecutor()
        self.thread = threading.Thread(target=rclpy.spin, args=(self.node, self.executor), daemon=True)
        self.thread.start()

        self._check_action = CheckAction(self.node,
                                         'multifunctional_button',
                                         MultifunctionalButtonCheck)
        self.node.set_parameters([
            rclpy.parameter.Parameter("check_time", rclpy.Parameter.Type.DOUBLE, 2.0)])

        self._led_sub_data = []
        self._talk_sub_data = None
        self._button_pub = self.node.create_publisher(
            Bool, "/hsrb/multi_functional_button", 1)
        self.node.create_subscription(
            Voice, "/talk_request", self.talk_callback, 10)
        self.node.create_subscription(
            Bool, "/hsrb/multifunctional_button_led", self.led_callback, 10)

    def tearDown(self):
        self.node.destroy_node()

    def led_callback(self, data):
        self._led_sub_data.append(data.data)

    def talk_callback(self, data):
        self._talk_sub_data = data.sentence

    def pub_mf_button(self, response):
        while self._button_pub.get_subscription_count() == 0:
            self.node.get_clock().sleep_for(Duration(seconds=0.1))
        self._button_pub.publish(Bool(data=True))
        self.set_user_response_client(response)

    def set_user_response_client(self, data_type, rate=10.0):
        update_val_client_impl(self.node, '/user_response_stub_node/set_bool_button_data',
                               device='none', rate=rate, data_type=data_type)

    def test_led_button_ok(self):
        self.set_user_response_client('true')
        self._check_action.execute_cb(GoalHandleStub())
        # Called in the order of LED on->off
        eq_(self._led_sub_data, [True, False])
        eq_(self._check_action._result.error_msg, '')
        self.set_user_response_client('none')

    def test_led_ng(self):
        self.set_user_response_client('false')
        self._check_action.execute_cb(GoalHandleStub())
        eq_(self._check_action._result.error_msg,
            'Multifunctional button LED Error')
        self.set_user_response_client('none')

    @patch(PKG + '.multi_function_check.MultifunctionalButtonCheck.'
           'wait_for_user_response_wrapper')
    def test_button_ng(self, user_response_mock):
        user_response_mock.side_effect = [True, True]
        self.set_user_response_client('false')
        self._check_action.execute_cb(GoalHandleStub())
        eq_(self._check_action._result.error_msg,
            'Multifunctional button Error')

    def test_speak_ok(self):
        # It will fail unless /user_response is stopped in advance
        self.set_user_response_client('none')

        # Set to a test setting that does not light up the LED
        self.node.set_parameters([
            rclpy.parameter.Parameter("is_check_mf_button_led", rclpy.Parameter.Type.BOOL, False)])

        # Press the MF button
        self.run_thread = threading.Thread(
            target=self.pub_mf_button, args=['true'])
        self.run_thread.setDaemon(True)
        self.run_thread.start()

        self._check_action.execute_cb(GoalHandleStub())
        eq_(self._talk_sub_data, 'multifunctional button OK')
        eq_(self._led_sub_data, [])
        eq_(self._check_action._result.error_msg, '')

    def test_speak_ng(self):
        self.node.set_parameters([
            rclpy.parameter.Parameter("is_check_mf_button_led", rclpy.Parameter.Type.BOOL, False)])

        self.run_thread = threading.Thread(
            target=self.pub_mf_button, args=['false'])
        self.run_thread.setDaemon(True)
        self.run_thread.start()

        self._check_action.execute_cb(GoalHandleStub())
        eq_(self._check_action._result.error_msg,
            'Multifunctional button Error')
