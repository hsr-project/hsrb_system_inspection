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

from geometry_msgs.msg import WrenchStamped
from hsrb_auto_diagnostics_msgs.srv import UpdateVal
from numpy import random
import rclpy
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor


class ForceTorqueSensorPub(object):
    def __init__(self, node):
        self._node = node
        self.lock = threading.Lock()
        self._wrench_pub = self._node.create_publisher(
            WrenchStamped, '/wrist_wrench/raw', 1)
        self._node.create_service(UpdateVal, '~/update_val', self.update_sensor_data)

        self._sleep_time = 1.0
        self._requested_data_type = None
        self._wrench_list = []
        self._wrench0 = WrenchStamped()
        self._wrench1 = WrenchStamped()
        self._wrench2 = WrenchStamped()

        self._force_ok = 200 * random.random_sample((3, 3)) - 100
        self._torque_ok = 10 * random.random_sample((3, 3)) - 5
        self._force_ng = 200 * random.random_sample((3, 3)) + 100
        self._torque_ng = 10 * random.random_sample((3, 3)) + 5

    def assign_data(self, wrench, force_data, torque_data):
        wrench.wrench.force.x = force_data[0]
        wrench.wrench.force.y = force_data[1]
        wrench.wrench.force.z = force_data[2]
        wrench.wrench.torque.x = torque_data[0]
        wrench.wrench.torque.y = torque_data[1]
        wrench.wrench.torque.z = torque_data[2]
        return wrench

    def set_data(self):
        self._wrench_list = []
        if self._requested_data_type == 'ok' or self._requested_data_type == 'sluggish':
            self._wrench_list.append(self.assign_data(
                                     self._wrench0,
                                     self._force_ok[0],
                                     self._torque_ok[0]))
            self._wrench_list.append(self.assign_data(
                                     self._wrench1,
                                     self._force_ok[1],
                                     self._torque_ok[1]))
            self._wrench_list.append(self.assign_data(
                                     self._wrench2,
                                     self._force_ok[2],
                                     self._torque_ok[2]))
        elif self._requested_data_type == 'freezed':
            self._wrench_list.append(self.assign_data(
                                     self._wrench0,
                                     self._force_ok[0],
                                     self._torque_ok[0]))
        elif self._requested_data_type == 'out_of_range':
            self._wrench_list.append(self.assign_data(
                                     self._wrench0,
                                     self._force_ng[0],
                                     self._torque_ng[0]))
            self._wrench_list.append(self.assign_data(
                                     self._wrench1,
                                     self._force_ng[1],
                                     self._torque_ng[1]))
            self._wrench_list.append(self.assign_data(
                                     self._wrench2,
                                     self._force_ng[2],
                                     self._torque_ng[2]))

    def update_sensor_data(self, req, res):
        with self.lock:
            self._sleep_time = 1.0 / req.rate
            self._requested_data_type = req.data_type
        res.done = True
        return res

    def run(self):
        pub_wrench = WrenchStamped()
        while rclpy.ok():
            with self.lock:
                sleep_time = self._sleep_time
                if self._requested_data_type is not None:
                    self.set_data()
                    self._requested_data_type = None

            if len(self._wrench_list) > 0:
                for wrench in self._wrench_list:
                    current_time = self._node.get_clock().now()
                    pub_wrench = wrench
                    pub_wrench.header.stamp = current_time.to_msg()
                    self._wrench_pub.publish(pub_wrench)
                    passed_time = (self._node.get_clock().now() - current_time).nanoseconds * 1e-9
                    current_sleep = sleep_time - passed_time if sleep_time > passed_time else 1e-6
                    self._node.get_clock().sleep_for(Duration(seconds=current_sleep))
            else:
                self._node.get_clock().sleep_for(Duration(seconds=0.1))


def main():
    rclpy.init()
    node = rclpy.create_node('force_torque_sensor_pub_stub_node')

    executor = MultiThreadedExecutor()
    thread = threading.Thread(target=rclpy.spin, args=(node, executor), daemon=True)
    thread.start()

    stub = ForceTorqueSensorPub(node)
    stub.run()

    rclpy.try_shutdown()
    node.destroy_node()


if __name__ == '__main__':
    main()
