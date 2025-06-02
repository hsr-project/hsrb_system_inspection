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

from hsrb_auto_diagnostics.ac_adapter_check import (
    ACAdapterCheck
)
from hsrb_auto_diagnostics.check_action import CheckAction
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
NAME = 'test_ac_adapter_check'


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


class TestACAdapterCheck(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node("test_ac_adapter_check",
                                      allow_undeclared_parameters=True,
                                      automatically_declare_parameters_from_overrides=True)
        self.executor = MultiThreadedExecutor()
        self.thread = threading.Thread(target=rclpy.spin, args=(self.node, self.executor), daemon=True)
        self.thread.start()

    def tearDown(self):
        self.node.destroy_node()

    def test_check_data_pattern_ok(self):
        ok_data = [1, 1, 1, 1, 2, 2, 2, 2, 2]
        ng_lower_limit = [-1, 1, 1, 1, 2, 2, 2, 2, 2]
        ng_upper_limit = [1, 1, 1, 1, 2, 2, 2, 2, 10]
        ng_limits = [-1, 1, 1, 1, 2, 2, 2, 2, 10]
        ng_lower_limit2 = [1, 1, 1, 1, -1, 2, 2, 2, 2]
        ng_upper_limit2 = [1, 10, 1, 1, 2, 2, 2, 2, 2]
        ng_limits2 = [1, 1, -1, 1, 2, 2, 10, 2, 2]
        ng_oder = [1, 2, 1, 2, 1, 2, 1, 2, 1]
        ng_order2 = [2, 2, 2, 2, 2, 1, 1, 1, 1]
        check = ACAdapterCheck(self.node)
        ok_(check.check_data_pattern(ok_data, 1, 2))
        ok_(not check.check_data_pattern(ng_lower_limit, 1, 2))
        ok_(not check.check_data_pattern(ng_upper_limit, 1, 2))
        ok_(not check.check_data_pattern(ng_limits, 1, 2))
        ok_(not check.check_data_pattern(ng_lower_limit2, 1, 2))
        ok_(not check.check_data_pattern(ng_upper_limit2, 1, 2))
        ok_(not check.check_data_pattern(ng_limits2, 1, 2))
        ok_(not check.check_data_pattern(ng_oder, 1, 2))
        ok_(not check.check_data_pattern(ng_order2, 1, 2))

    def set_user_response_client(self, data_type, rate=10.0):
        update_val_client_impl(self.node, '/user_response_stub_node/set_bool_button_data',
                               device='none', rate=rate, data_type=data_type)

    @patch(PKG + '.ac_adapter_check.ACAdapterCheck.'
           'get_sub_data_with_duration')
    def test_ac_adapter_ok(self, sub_data_mock):
        check_action = CheckAction(self.node, 'ac_adapter', ACAdapterCheck)
        sub_data_mock.side_effect = [[6, 6, 6], [1, 1, 2]]
        self.set_user_response_client('true')
        check_action.execute_cb(GoalHandleStub())
        eq_(check_action._result.error_msg, '')
        self.set_user_response_client('none')

    @patch(PKG + '.ac_adapter_check.ACAdapterCheck.'
           'get_sub_data_with_duration')
    def test_ac_adapter_ok2(self, sub_data_mock):
        check_action = CheckAction(self.node, 'ac_adapter', ACAdapterCheck)
        sub_data_mock.side_effect = [[6, 6, 6], [2, 2, 2]]
        self.set_user_response_client('true')
        check_action.execute_cb(GoalHandleStub())
        eq_(check_action._result.error_msg, '')
        self.set_user_response_client('none')

    @patch(PKG + '.ac_adapter_check.ACAdapterCheck.'
           'get_sub_data_with_duration')
    def test_ac_adapter_connect_ng(self, sub_data_mock):
        check_action = CheckAction(self.node, 'ac_adapter', ACAdapterCheck)
        sub_data_mock.side_effect = [[6, 6, 1], [1, 1, 2]]
        self.set_user_response_client('true')
        check_action.execute_cb(GoalHandleStub())
        eq_(check_action._result.error_msg,
            'AC Adapter Error')
        self.set_user_response_client('none')

    @patch(PKG + '.ac_adapter_check.ACAdapterCheck.'
           'get_sub_data_with_duration')
    def test_button_ng(self, sub_data_mock):
        check_action = CheckAction(self.node, 'ac_adapter', ACAdapterCheck)
        sub_data_mock.side_effect = [[6, 6, 6], [2, 1, 2]]
        self.set_user_response_client('true')
        check_action.execute_cb(GoalHandleStub())
        eq_(check_action._result.error_msg,
            'AC Adapter Error')
        self.set_user_response_client('none')
