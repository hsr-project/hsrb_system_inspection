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

from hsrb_auto_diagnostics.check_action import CheckAction
from hsrb_auto_diagnostics.magnetic_sensor_check import MagneticSensorCheck
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
NAME = 'test_magnetic_sensor_check'


@pytest.mark.launch_test
def generate_test_description():
    magnetic_sensor_stub_node = launch_ros.actions.Node(
        executable='test/magnetic_sensor_stub.py',
        name='magnetic_sensor_stub_node',
        output='screen'
    )
    return launch.LaunchDescription([
        magnetic_sensor_stub_node,
        launch_testing.actions.ReadyToTest()
    ])


class TestMagneticSensorCheck(unittest.TestCase):

    _NORMAL_REACTIONS = [
        ('?Please remove a magnet around the robot.'
         ' Then, push the OK button', (False, False)),
        ('?Please set a magnet front right of the '
         'robot. Then, push the OK button', (True, False)),
        ('?Please push the button if LED is blinking on and off', None),
        ('?Please remove a magnet and push the release '
         'button. Then, push the OK button', (False, False)),
        ('?Please push the button if LED is lighting-up', None),
        ('?Please set a magnet rear left of the '
         'robot. Then, push the OK button', (False, True))]

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node("test_magnetic_sensor_check",
                                      allow_undeclared_parameters=True,
                                      automatically_declare_parameters_from_overrides=True)
        self.executor = MultiThreadedExecutor()
        self.thread = threading.Thread(target=rclpy.spin, args=(self.node, self.executor), daemon=True)
        self.thread.start()

        self._check_action = CheckAction(self.node, 'magnetic_sensor',
                                         MagneticSensorCheck)
        self._is_reaction_running = True

    def tearDown(self):
        self.node.destroy_node()

    def set_data_client(self, topic_name, device, data_type, rate=10.0):
        update_val_client_impl(self.node, topic_name,
                               device=device, rate=rate, data_type=data_type)

    def set_sensor_data(self, front_right=False, rear_left=False):
        data_type = str(front_right).lower() + '_' + str(rear_left).lower()
        self.set_data_client(
            '/magnetic_sensor_stub_node/set_magnetic_sensor_data',
            'none', data_type)
        # For diagnostics action to receice sensor topic
        self.node.get_clock().sleep_for(Duration(seconds=1.5))

    def stop_publish(self):
        self.set_data_client('/magnetic_sensor_stub_node/'
                             'set_magnetic_sensor_data', 'none', 'none')

    def reaction(self, reaction_configs):
        response_pub = self.node.create_publisher(Bool, '/user_response', 1)
        while response_pub.get_subscription_count() == 0:
            self.node.get_clock().sleep_for(Duration(seconds=0.1))

        current_msg = None
        while rclpy.ok() and self._is_reaction_running:
            if current_msg == self._check_action._feedback.feedback_msg:
                continue
            else:
                current_msg = self._check_action._feedback.feedback_msg

            for config in reaction_configs:
                if current_msg == config[0]:
                    if config[1] is not None:
                        self.set_sensor_data(*config[1])
                    # If you send a response as soon as the feedback is updated, it will be ignored.
                    self.node.get_clock().sleep_for(Duration(seconds=0.5))
                    response_pub.publish(Bool(data=True))
                    self.node.get_clock().sleep_for(Duration(seconds=0.5))
                    break
        self.stop_publish()

    def _test_impl(self, reaction_configs, expected_result):
        t = threading.Thread(target=self.reaction, args=(reaction_configs,))
        t.start()
        self._check_action.execute_cb(GoalHandleStub())
        eq_(self._check_action._result.error_msg, expected_result)
        self._is_reaction_running = False
        t.join()

    def test_ok(self):
        self._test_impl(self._NORMAL_REACTIONS, '')

    def test_first_user_response_timeout(self):
        self._check_action.execute_cb(GoalHandleStub())
        eq_(self._check_action._result.error_msg,
            'Not ready to check magnetic sensor')

    def _test_not_removed_magnet(self, front_right, rear_left):
        reactions = [(self._NORMAL_REACTIONS[0][0],
                      (front_right, rear_left))] + self._NORMAL_REACTIONS[1:]
        self._test_impl(reactions,
                        'Before setting a magnet, sensors return wrong data')

    def test_not_removed_front_right_magnet(self):
        self._test_not_removed_magnet(True, False)

    def test_not_removed_rear_left_magnet(self):
        self._test_not_removed_magnet(False, True)

    def test_not_removed_both_magnet(self):
        self._test_not_removed_magnet(True, True)

    def test_not_set_front_right_magnet(self):
        reactions = [(self._NORMAL_REACTIONS[1][0], (False, False))] + \
            [self._NORMAL_REACTIONS[0]] + self._NORMAL_REACTIONS[2:]
        self._test_impl(
            reactions,
            'There are no responses of the front right magnetic sensor')

    def test_not_set_rear_left_magnet(self):
        reactions = [(self._NORMAL_REACTIONS[5][0], (False, False))] + \
            self._NORMAL_REACTIONS[:-1]
        self._test_impl(
            reactions,
            'There are no responses of the rear left magnetic sensor')

    def test_no_set_magnet_response(self):
        wait_for_placement_timeout = self.node.get_parameter(
            '~wait_for_placement_response_timeout').get_parameter_value().double_value
        self.node.set_parameters([
            rclpy.parameter.Parameter('wait_for_placement_response_timeout',
                                      rclpy.Parameter.Type.DOUBLE,
                                      5.0)])
        reactions = self._NORMAL_REACTIONS[0:2]
        self._test_impl(reactions, 'LED is not blinking')
        self.node.set_parameters([
            rclpy.parameter.Parameter('wait_for_placement_response_timeout',
                                      rclpy.Parameter.Type.DOUBLE,
                                      wait_for_placement_timeout)])

    def test_no_remove_magnet_response(self):
        wait_for_removal_response_timeout = self.node.get_parameter(
            '~wait_for_removal_response_timeout').get_parameter_value().double_value
        self.node.set_parameters([
            rclpy.parameter.Parameter('wait_for_removal_response_timeout',
                                      rclpy.Parameter.Type.DOUBLE,
                                      5.0)])
        reactions = self._NORMAL_REACTIONS[0:3]
        self._test_impl(reactions, 'Robot cannot recover')
        self.node.set_parameters([
            rclpy.parameter.Parameter('wait_for_removal_response_timeout',
                                      rclpy.Parameter.Type.DOUBLE,
                                      wait_for_removal_response_timeout)])
