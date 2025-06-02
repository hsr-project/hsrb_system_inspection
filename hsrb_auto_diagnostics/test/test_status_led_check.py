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

from hsrb_auto_diagnostics.check_action import CheckAction
from hsrb_auto_diagnostics.status_led_check import StatusLEDCheck
import rclpy
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import ColorRGBA
from std_srvs.srv import Empty
from tmc_msgs.srv import SetColor

from .goal_handle_stub import GoalHandleStub

PKG = 'hsrb_auto_diagnostics'
NAME = 'test_status_led_check'


class TestStatusLEDCheck(unittest.TestCase):
    OFF = ColorRGBA()
    RED = ColorRGBA(r=1.0)
    GREEN = ColorRGBA(g=1.0)
    YELLOW = ColorRGBA(r=1.0, g=1.0)
    BLUE = ColorRGBA(b=1.0)
    PURPLE = ColorRGBA(r=1.0, b=1.0)
    LIGHT_BLUE = ColorRGBA(b=1.0, g=1.0)
    WHITE = ColorRGBA(r=1.0, g=1.0, b=1.0)

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node("test_status_led_check",
                                      allow_undeclared_parameters=True,
                                      automatically_declare_parameters_from_overrides=True)
        self.executor = MultiThreadedExecutor()
        self.thread = threading.Thread(target=rclpy.spin, args=(self.node, self.executor), daemon=True)
        self.thread.start()

        self._set_color_call_args_list = []
        self._activate_auto_call_args_list = []

        self._check_action = CheckAction(
            self.node, 'status_led', StatusLEDCheck)

        # set quick lighting time for test
        self.node.set_parameters([
            rclpy.parameter.Parameter("lighting_time", rclpy.Parameter.Type.DOUBLE, 0.1)])

        self._set_color_srv = self.node.create_service(
            SetColor, '/hsrb/status_led_node/set_color',
            self.dummy_set_color_cb)

        self._activate_auto_srv = self.node.create_service(
            Empty, '/hsrb/status_led_node/activate_auto_mode',
            self.dummy_activate_auto_cb)

    def tearDown(self):
        self.node.destroy_service(self._set_color_srv)
        self.node.destroy_service(self._activate_auto_srv)
        self.node.destroy_node()

    def dummy_set_color_cb(self, req, res):
        data = {"time": self.node.get_clock().now(), "data": req.color}
        self._set_color_call_args_list.append(data)
        return res

    def dummy_activate_auto_cb(self, req, res):
        data = {"time": self.node.get_clock().now(), "data": req}
        self._activate_auto_call_args_list.append(data)
        return res

    def check_order_of_called_srvs(self):
        self.assertEqual(self._set_color_call_args_list[0]["data"], self.OFF)
        self.assertEqual(self._set_color_call_args_list[1]["data"], self.RED)
        self.assertEqual(self._set_color_call_args_list[2]["data"], self.GREEN)
        self.assertEqual(
            self._set_color_call_args_list[3]["data"], self.YELLOW)
        self.assertEqual(self._set_color_call_args_list[4]["data"], self.BLUE)
        self.assertEqual(
            self._set_color_call_args_list[5]["data"], self.PURPLE)
        self.assertEqual(
            self._set_color_call_args_list[6]["data"], self.LIGHT_BLUE)
        self.assertEqual(self._set_color_call_args_list[7]["data"], self.WHITE)

        # restore mode to auto after calling all the set_color service
        self.assertTrue(
            (self._set_color_call_args_list[7]["time"]
             < self._activate_auto_call_args_list[0]["time"]), True)

    @patch(PKG + '.check_tools.Interactive.wait_for_user_response',
           return_value=[])
    def test_ok(self, response_mock):
        self._check_action.execute_cb(GoalHandleStub())

        # finish doing the test
        self.assertEqual(len(self._set_color_call_args_list), 8)
        self.assertEqual(len(self._activate_auto_call_args_list), 1)
        self.check_order_of_called_srvs()
        self.assertAlmostEqual(response_mock.call_args[0][4], 10.0)

        self.assertTrue(self._check_action._result.ok, True)

    @patch(PKG + '.check_tools.Interactive.wait_for_user_response',
           return_value=['LED_Error'])
    def test_ng(self, response_mock):
        wait_for_response_timeout = self.node.get_parameter(
            'wait_for_response_timeout').get_parameter_value().double_value
        self.node.set_parameters([
            rclpy.parameter.Parameter('wait_for_response_timeout',
                                      rclpy.Parameter.Type.DOUBLE,
                                      3.0)])
        self._check_action.execute_cb(GoalHandleStub())
        self.node.set_parameters([
            rclpy.parameter.Parameter('wait_for_response_timeout',
                                      rclpy.Parameter.Type.DOUBLE,
                                      wait_for_response_timeout)])

        self.assertEqual(len(self._set_color_call_args_list), 8)
        self.assertEqual(len(self._activate_auto_call_args_list), 1)
        self.check_order_of_called_srvs()
        self.assertAlmostEqual(response_mock.call_args[0][4], 3.0)

        self.assertFalse(self._check_action._result.ok, False)

    @patch(PKG + '.check_tools.Interactive.wait_for_user_response',
           return_value=['Diagnostic Canceled'])
    def test_cancel(self, response_mock):
        self._check_action.execute_cb(GoalHandleStub())

        self.assertEqual(len(self._set_color_call_args_list), 8)
        self.assertEqual(len(self._activate_auto_call_args_list), 1)
        self.check_order_of_called_srvs()

        self.assertFalse(self._check_action._result.ok, False)
