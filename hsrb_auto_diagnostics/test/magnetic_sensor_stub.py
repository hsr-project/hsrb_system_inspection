#!/usr/bin/env python3
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

from hsrb_auto_diagnostics_msgs.srv import UpdateVal
import rclpy
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import Bool


class MagneticSensorStub(object):
    def __init__(self, node):
        self._node = node
        self.lock = threading.Lock()
        self._publish_on = False
        self._sensor_1_data = Bool()
        self._sensor_2_data = Bool()
        self._sensor_1_pub = self._node.create_publisher(
            Bool, '/base_magnetic_sensor_1', 10)
        self._sensor_2_pub = self._node.create_publisher(
            Bool, '/base_magnetic_sensor_2', 10)
        self._srv = self._node.create_service(
            UpdateVal, '~/set_magnetic_sensor_data', self.update_bool)

    def update_bool(self, req, res):
        with self.lock:
            if req.data_type == 'true_true':
                self._sensor_1_data.data = True
                self._sensor_2_data.data = True
            elif req.data_type == 'true_false':
                self._sensor_1_data.data = True
                self._sensor_2_data.data = False
            elif req.data_type == 'false_true':
                self._sensor_1_data.data = False
                self._sensor_2_data.data = True
            elif req.data_type == 'false_false':
                self._sensor_1_data.data = False
                self._sensor_2_data.data = False
            else:
                self._publish_on = False
            self._publish_on = True
        res.done = True
        return res

    def run(self):
        while rclpy.ok():
            current_time = self._node.get_clock().now()
            with self.lock:
                if self._publish_on:
                    self._sensor_1_pub.publish(self._sensor_1_data)
                    self._sensor_2_pub.publish(self._sensor_2_data)
            passed_time = (self._node.get_clock().now() - current_time).nanoseconds * 1e-9
            current_sleep = 0.1 - passed_time if 0.1 > passed_time else 1e-6
            self._node.get_clock().sleep_for(Duration(seconds=current_sleep))


def main():
    rclpy.init()
    node = rclpy.create_node('magnetic_sensor_stub_node')

    executor = MultiThreadedExecutor()
    thread = threading.Thread(target=rclpy.spin, args=(node, executor), daemon=True)
    thread.start()

    stub = MagneticSensorStub(node)
    stub.run()

    rclpy.try_shutdown()
    node.destroy_node()


if __name__ == '__main__':
    main()
