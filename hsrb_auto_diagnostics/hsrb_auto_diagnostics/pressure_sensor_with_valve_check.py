#! /usr/bin/env python
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
import hsrb_auto_diagnostics.check_tools as check_tools
from hsrb_auto_diagnostics.pressure_sensor_check import PressureSensorCheck
from rclpy.duration import Duration
from std_msgs.msg import Bool
from tmc_manipulation_msgs.srv import SafeJointChange

"""
PressureSensor with Valve Test Class
"""


class PressureSensorWithValveCheck(PressureSensorCheck):
    def __init__(self, *args):
        super().__init__(*args)

        self.node.declare_parameter("valve_topic", "/hsrb/valve")

    def pressure_sensor_callback(self, data):
        self._pressure_sensor = data.data

    def check_interactive(self, _fb, _as):
        err_msg = ['Valve Error', 'Pressure Sensor Error']
        question = ['?Push the OK button if a card is peeled '
                    'off the pump and dropped.']

        # Lower the arm to safely drop the suctioned object
        safe_pose_changer_client = self.node.create_client(
            SafeJointChange, '/safe_pose_changer/change_joint')
        safe_pose_changer_client.wait_for_service(timeout_sec=30.0)
        req = SafeJointChange.Request()
        req.ref_joint_state.name = [
            "arm_lift_joint", "arm_flex_joint", "arm_roll_joint",
            "wrist_flex_joint", "wrist_roll_joint", "head_pan_joint",
            "head_tilt_joint"]
        req.ref_joint_state.position = [0.3, -2.4, 0.0, 0.0, -1.57, 0.0, 0.0]
        self.node.get_logger().info("safe_pose_changer...")
        future = safe_pose_changer_client.call_async(req)
        init_time = self.node.get_clock().now()
        now = init_time
        while not future.done():
            now = self.node.get_clock().now()
            if (now - init_time) > Duration(seconds=3.0):
                self.node.get_logger().error("safe_pose_changer timeout.")
                future.cancel()
                break
            self.node.get_clock().sleep_for(Duration(seconds=0.1))
        self.node.get_logger().info("safe_pose_changer done.")

        tool = check_tools.Interactive(self.node, "/user_response", Bool)
        pressure_sensor_sub = self.node.create_subscription(
            Bool, '/hsrb/pressure_sensor',
            self.pressure_sensor_callback, 10)
        valve_pub = self.node.create_publisher(
            Bool,
            self.node.get_parameter("valve_topic").get_parameter_value().string_value,
            1)

        try:
            # Suction sensor test before R501
            suction_check_result = super(
                PressureSensorWithValveCheck, self).suction_test(
                    _fb, _as)
            if len(suction_check_result) > 0:
                self.node.get_logger().error(suction_check_result)
                return suction_check_result

            # Additional solenoid test after R501
            # Open the valve
            valve_pub.publish(Bool(data=True))
            valve_result = tool.wait_for_user_response(
                _fb, _as, question[0], err_msg[0], 30.0)
            # Suction sensor remains True (Mechanical specification)
            if not self._pressure_sensor:
                valve_result.append(err_msg[1])
                self.node.get_logger().error(valve_result)
            return valve_result
        except Exception as e:
            return [str(e)]
        finally:
            self.suction_control(False, Duration(0.0))
            valve_pub.publish(Bool(data=False))
            self.node.destroy_subscription(tool.sub)
            self.node.destroy_subscription(pressure_sensor_sub)
            req.ref_joint_state.position = [
                0.0, 0.0, -1.57, -1.57, 0.0, 0.0, 0.0]
            self.node.get_logger().info("safe_pose_changer...")
            future = safe_pose_changer_client.call_async(req)
            init_time = self.node.get_clock().now()
            now = init_time
            while not future.done():
                now = self.node.get_clock().now()
                if (now - init_time) > Duration(seconds=3.0):
                    self.node.get_logger().error("safe_pose_changer timeout.")
                    future.cancel()
                    break
                self.node.get_clock().sleep_for(Duration(seconds=0.1))
            self.node.get_logger().info("safe_pose_changer done.")
