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

from ament_index_python.packages import get_package_share_directory
from hsrb_auto_diagnostics.check_action import CheckAction
from hsrb_auto_diagnostics.speaker_check import SpeakerCheck
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
NAME = 'test_speaker_check'


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


class TestSpeakerCheck(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node("test_speaker_check",
                                      allow_undeclared_parameters=True,
                                      automatically_declare_parameters_from_overrides=True)

        package_dir = get_package_share_directory('hsrb_auto_diagnostics')
        self.node.declare_parameter("wav_file_name",
                                    package_dir + "/media/hsr-start.wav")

        self.executor = MultiThreadedExecutor()
        self.thread = threading.Thread(target=rclpy.spin, args=(self.node, self.executor), daemon=True)
        self.thread.start()

        self._check_action = CheckAction(self.node, 'speaker', SpeakerCheck)

    def tearDown(self):
        self.node.destroy_node()

    def set_user_response_client(self, data_type, rate=10.0):
        update_val_client_impl(self.node, '/user_response_stub_node/set_bool_button_data',
                               device='none', rate=rate, data_type=data_type)

    def test_user_ok(self):
        self.set_user_response_client('true')
        self._check_action.execute_cb(GoalHandleStub())
        eq_(self._check_action._result.error_msg, '')
        self.set_user_response_client('none')

    def test_user_ng(self):
        self.set_user_response_client('false')
        self._check_action.execute_cb(GoalHandleStub())
        eq_(self._check_action._result.error_msg,
            'Speaker Error')
        self.set_user_response_client('none')

    def test_user_cancel(self):
        self._check_action.execute_cb(GoalHandleStub())
        eq_(self._check_action._result.error_msg,
            'Diagnostic Canceled')
