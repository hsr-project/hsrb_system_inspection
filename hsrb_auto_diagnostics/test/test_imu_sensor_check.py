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
import os
import threading
import unittest

from hsrb_auto_diagnostics.imu_sensor_check import ImuSensorCheck
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
NAME = 'test_imu_sensor_check'


def path_exist(path):
    return True


def path_not_exist(path):
    return False


@pytest.mark.launch_test
def generate_test_description():
    imu_sensor_pub_stub_node = launch_ros.actions.Node(
        executable='test/imu_sensor_pub_stub_srv.py',
        name='imu_sensor_pub_stub_node',
        output='screen'
    )
    return launch.LaunchDescription([
        imu_sensor_pub_stub_node,
        launch_testing.actions.ReadyToTest()
    ])


class TestImuSensorCheck(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node("test_imu_sensor_check",
                                      allow_undeclared_parameters=True,
                                      automatically_declare_parameters_from_overrides=True)

        ros_parameters = {
            "imu_check_info_hsrb": {
                "ori_y": {
                    "do_freeze_check": True,
                    "lower_limit": -1.0,
                    "upper_limit": 1.0,
                },
                "ori_z": {
                    "do_freeze_check": False,
                    "lower_limit": 0.0,
                    "upper_limit": 0.0,
                },
                "ori_w": {
                    "do_freeze_check": False,
                    "lower_limit": -1.0,
                    "upper_limit": 1.0,
                },
                "ang_vel_x": {
                    "do_freeze_check": True,
                    "lower_limit": -0.3,
                    "upper_limit": 0.3,
                },
                "ang_vel_y": {
                    "do_freeze_check": True,
                    "lower_limit": -0.3,
                    "upper_limit": 0.3,
                },
                "ang_vel_z": {
                    "do_freeze_check": True,
                    "lower_limit": -0.3,
                    "upper_limit": 0.3,
                },
                "linear_acc_x": {
                    "do_freeze_check": True,
                    "lower_limit": -1.0,
                    "upper_limit": 1.0,
                },
                "linear_acc_y": {
                    "do_freeze_check": True,
                    "lower_limit": -1.0,
                    "upper_limit": 1.0,
                },
                "linear_acc_z": {
                    "do_freeze_check": True,
                    "lower_limit": 8.8,
                    "upper_limit": 10.8,
                },
            },
            "imu_check_info_hsrc": {
                "ori_y": {
                    "do_freeze_check": True,
                    "lower_limit": -1.0,
                    "upper_limit": 1.0,
                },
                "ori_z": {
                    "do_freeze_check": False,
                    "lower_limit": 0.0,
                    "upper_limit": 0.0,
                },
                "ori_w": {
                    "do_freeze_check": False,
                    "lower_limit": -1.0,
                    "upper_limit": 1.0,
                },
                "ang_vel_x": {
                    "do_freeze_check": True,
                    "lower_limit": -0.3,
                    "upper_limit": 0.3,
                },
                "ang_vel_y": {
                    "do_freeze_check": True,
                    "lower_limit": -0.3,
                    "upper_limit": 0.3,
                },
                "ang_vel_z": {
                    "do_freeze_check": True,
                    "lower_limit": -0.3,
                    "upper_limit": 0.3,
                },
                "linear_acc_x": {
                    "do_freeze_check": True,
                    "lower_limit": -1.0,
                    "upper_limit": 1.0,
                },
                "linear_acc_y": {
                    "do_freeze_check": True,
                    "lower_limit": -1.0,
                    "upper_limit": 1.0,
                },
                "linear_acc_z": {
                    "do_freeze_check": True,
                    "lower_limit": 8.8,
                    "upper_limit": 10.8,
                },
            },
        }

        # Assume that the parameter is always included
        robot_names = ["hsrb", "hsrc"]
        # The name must always be in this order.
        param_names = [
            "ori_y",
            "ori_z",
            "ori_w",
            "ang_vel_x",
            "ang_vel_y",
            "ang_vel_z",
            "linear_acc_x",
            "linear_acc_y",
            "linear_acc_z"
        ]
        for robot_name in robot_names:
            ns = "imu_check_info_" + robot_name
            for param_name in param_names:
                fullpath = ns + "." + param_name
                values = ros_parameters[ns][param_name]
                self.node.set_parameters([
                    rclpy.parameter.Parameter(fullpath + ".do_freeze_check",
                                              rclpy.Parameter.Type.BOOL,
                                              values["do_freeze_check"]),
                    rclpy.parameter.Parameter(fullpath + ".lower_limit",
                                              rclpy.Parameter.Type.DOUBLE,
                                              values["lower_limit"]),
                    rclpy.parameter.Parameter(fullpath + ".upper_limit",
                                              rclpy.Parameter.Type.DOUBLE,
                                              values["upper_limit"]),
                ])

        self.executor = MultiThreadedExecutor()
        self.thread = threading.Thread(target=rclpy.spin, args=(self.node, self.executor), daemon=True)
        self.thread.start()

        self._instance = ImuSensorCheck(self.node)

    def tearDown(self):
        self.node.destroy_node()

    def execute_cb(self):
        pass

    def update_val_client(self, rate, data_type):
        update_val_client_impl(self.node, '/imu_sensor_pub_stub_node/update_val',
                               device='imu_sensor', rate=rate, data_type=data_type)

    def test_connection_ok(self):
        os.path.exists = path_exist
        ok_(self._instance.check_connection())

    def test_connection_ng(self):
        os.path.exists = path_not_exist
        ok_(not self._instance.check_connection())

    def test_sub_msg_ok(self):
        os.environ['ROBOT_VERSION'] = 'HSRB-PHASE3'
        self.update_val_client(100.0, 'ok')
        eq_(self._instance.check_sub_data(GoalHandleStub()), [])

    def test_sub_msg_sluggish(self):
        os.environ['ROBOT_VERSION'] = 'HSRB-PHASE3'
        self.update_val_client(50.0, 'sluggish')
        eq_(self._instance.check_sub_data(GoalHandleStub()),
            ['Subscribe Frequency Error'])

    def test_sub_msg_freezed(self):
        os.environ['ROBOT_VERSION'] = 'HSRB-PHASE3'
        self.update_val_client(100.0, 'freezed')
        eq_(self._instance.check_sub_data(GoalHandleStub()),
            ['ori_y:Data Freeze Error',
             'ang_vel_x:Data Freeze Error',
             'ang_vel_y:Data Freeze Error',
             'ang_vel_z:Data Freeze Error',
             'linear_acc_x:Data Freeze Error',
             'linear_acc_y:Data Freeze Error',
             'linear_acc_z:Data Freeze Error'])

    def test_sub_msg_out_of_range(self):
        os.environ['ROBOT_VERSION'] = 'HSRB-PHASE3'
        self.update_val_client(100.0, 'out_of_range')
        eq_(self._instance.check_sub_data(GoalHandleStub()),
            ['ori_y:Data Range Error', 'ori_w:Data Range Error',
             'ang_vel_x:Data Range Error',
             'ang_vel_y:Data Range Error',
             'ang_vel_z:Data Range Error',
             'linear_acc_x:Data Range Error',
             'linear_acc_y:Data Range Error',
             'linear_acc_z:Data Range Error'])
