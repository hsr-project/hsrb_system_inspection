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

from hsrb_auto_diagnostics.check_action import CheckAction
from hsrb_auto_diagnostics.emergency_button_check import EmergencyButtonCheck
import launch
import launch_ros.actions
import launch_testing
from nose.tools import eq_
import pytest
import rclpy
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import Bool

from .goal_handle_stub import GoalHandleStub
from .utils import update_val_client_impl

PKG = 'hsrb_auto_diagnostics'
NAME = 'test_emergency_button_check'


@pytest.mark.launch_test
def generate_test_description():
    runstop_button_stub_node = launch_ros.actions.Node(
        executable='test/bool_pub_stub.py',
        name='runstop_button_stub_node',
        remappings=[('topic_name', '/runstop_button')],
        output='screen'
    )
    return launch.LaunchDescription([
        runstop_button_stub_node,
        launch_testing.actions.ReadyToTest()
    ])


class TestEmergencyButtonCheck(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node("test_emergency_button_check",
                                      allow_undeclared_parameters=True,
                                      automatically_declare_parameters_from_overrides=True)
        self.executor = MultiThreadedExecutor()
        self.thread = threading.Thread(target=rclpy.spin, args=(self.node, self.executor), daemon=True)
        self.thread.start()

        self._check_action = CheckAction(self.node, 'emergency_button',
                                         EmergencyButtonCheck)

        self.user_response_pub = self.node.create_publisher(Bool, "/user_response", 10)

    def tearDown(self):
        self.node.destroy_node()

    def set_data_client(self, topic_name, device, data_type, rate=10.0):
        update_val_client_impl(self.node, topic_name,
                               device=device, rate=rate, data_type=data_type)

    def stop_publish(self):
        self.set_data_client('/runstop_button_stub_node/'
                             'set_bool_button_data', 'none', 'none')
        self.set_data_client('/user_response_stub_node/'
                             'set_bool_button_data', 'user', 'none')

    def reaction1(self):
        question = ['?Lock the emergency switch. Then, push the OK button',
                    '?Unlock the  emergency switch. Then, push the OK button']

        while rclpy.ok():
            if self._check_action._feedback.feedback_msg == question[0]:
                self.set_data_client('/runstop_button_stub_node/'
                                     'set_bool_button_data',
                                     'none', 'true')
                self.node.get_clock().sleep_for(Duration(seconds=1.0))
                self.user_response_pub.publish(Bool(data=True))
            elif self._check_action._feedback.feedback_msg == question[1]:
                self.set_data_client('/runstop_button_stub_node/'
                                     'set_bool_button_data',
                                     'none', 'false')
                self.node.get_clock().sleep_for(Duration(seconds=1.0))
                self.user_response_pub.publish(Bool(data=True))
                break
            else:
                self.node.get_clock().sleep_for(Duration(seconds=0.1))
        self.stop_publish()

    def reaction2(self):
        question = ['?Lock the emergency switch. Then, push the OK button']

        while rclpy.ok():
            if self._check_action._feedback.feedback_msg == question[0]:
                self.set_data_client('/runstop_button_stub_node/'
                                     'set_bool_button_data',
                                     'none', 'false')
                self.node.get_clock().sleep_for(Duration(seconds=1.0))
                self.user_response_pub.publish(Bool(data=True))
                break
            else:
                self.node.get_clock().sleep_for(Duration(seconds=0.1))
        self.stop_publish()

    def reaction3(self):
        question = ['?Lock the emergency switch. Then, push the OK button',
                    '?Unlock the  emergency switch. Then, push the OK button']

        while rclpy.ok():
            if self._check_action._feedback.feedback_msg == question[0]:
                self.set_data_client('/runstop_button_stub_node/'
                                     'set_bool_button_data',
                                     'none', 'true')
                self.node.get_clock().sleep_for(Duration(seconds=1.0))
                self.user_response_pub.publish(Bool(data=True))
            elif self._check_action._feedback.feedback_msg == question[1]:
                self.set_data_client('/runstop_button_stub_node/'
                                     'set_bool_button_data',
                                     'none', 'true')
                self.node.get_clock().sleep_for(Duration(seconds=1.0))
                self.user_response_pub.publish(Bool(data=True))
                break
            else:
                self.node.get_clock().sleep_for(Duration(seconds=0.1))
        self.stop_publish()

    def test_ok(self):
        t = threading.Thread(target=self.reaction1)
        t.start()
        self._check_action.execute_cb(GoalHandleStub())
        eq_(self._check_action._result.error_msg, '')
        t.join()

    def test_first_user_response_timeout(self):
        self._check_action.execute_cb(GoalHandleStub())
        eq_(self._check_action._result.error_msg,
            'User response timeout(lock)')

    def test_wrong_sensor_data1(self):
        t = threading.Thread(target=self.reaction2)
        t.start()
        self._check_action.execute_cb(GoalHandleStub())
        eq_(self._check_action._result.error_msg,
            'Subscribed data is all False')
        t.join()

    def test_wrong_sensor_data2(self):
        t = threading.Thread(target=self.reaction3)
        t.start()
        self._check_action.execute_cb(GoalHandleStub())
        eq_(self._check_action._result.error_msg,
            'Value True is in subscribed data')
        t.join()
