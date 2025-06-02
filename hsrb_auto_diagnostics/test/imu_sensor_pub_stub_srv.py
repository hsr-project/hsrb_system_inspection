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
from numpy import random
import rclpy
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import Imu


class ImuSensorPub(object):
    def __init__(self, node):
        self._node = node
        self.lock = threading.Lock()
        self._imu_pub = self._node.create_publisher(
            Imu, '/imu/data', 1)
        self._node.create_service(UpdateVal, '~/update_val', self.update_imu_data)

        self._sleep_time = 1.0
        self._requested_data_type = None
        self._imu_list = []
        self._imu0 = Imu()
        self._imu1 = Imu()
        self._imu2 = Imu()

        # -1.0 <= data < 1.0
        self._ori_ok = 2.0 * random.random_sample((3, 2)) - 1.0
        # -0.3 <= data < 0.3
        self._ang_vel_ok = 0.6 * random.random_sample((3, 3)) - 0.3
        # -1.0 <= data < 1.0
        self._acc_xy_ok = 2.0 * random.random_sample((3, 2)) - 1.0
        # 8.8 <= data < 10.8
        self._acc_z_ok = 2.0 * random.random_sample((3, 1)) + 8.8

        # 1.05 <= data < 1.1
        self._ori_ng = 0.05 * random.random_sample((3, 2)) + 1.05
        # 0.35 <= data < 0.4
        self._ang_vel_ng = 0.05 * random.random_sample((3, 3)) + 0.35
        # 1.05 <= data < 1.1
        self._acc_xy_ng = 0.05 * random.random_sample((3, 2)) + 1.05
        # 10.85 <= data < 10.9
        self._acc_z_ng = 0.05 * random.random_sample((3, 1)) + 10.85

    def assign_data(self, imu, ori, ang_vel, acc_xy, acc_z):
        imu.orientation.y = ori[0]
        imu.orientation.w = ori[1]
        imu.angular_velocity.x = ang_vel[0]
        imu.angular_velocity.y = ang_vel[1]
        imu.angular_velocity.z = ang_vel[2]
        imu.linear_acceleration.x = acc_xy[0]
        imu.linear_acceleration.y = acc_xy[1]
        imu.linear_acceleration.z = acc_z[0]
        return imu

    def set_data(self):
        self._imu_list = []
        if self._requested_data_type == 'ok' or self._requested_data_type == 'sluggish':
            self._imu_list.append(self.assign_data(
                                  self._imu0,
                                  self._ori_ok[0],
                                  self._ang_vel_ok[0],
                                  self._acc_xy_ok[0],
                                  self._acc_z_ok[0]))
            self._imu_list.append(self.assign_data(
                                  self._imu1,
                                  self._ori_ok[1],
                                  self._ang_vel_ok[1],
                                  self._acc_xy_ok[1],
                                  self._acc_z_ok[1]))
            self._imu_list.append(self.assign_data(
                                  self._imu2,
                                  self._ori_ok[2],
                                  self._ang_vel_ok[2],
                                  self._acc_xy_ok[2],
                                  self._acc_z_ok[2]))
        elif self._requested_data_type == 'freezed':
            self._imu_list.append(self.assign_data(
                                  self._imu0,
                                  self._ori_ok[0],
                                  self._ang_vel_ok[0],
                                  self._acc_xy_ok[0],
                                  self._acc_z_ok[0]))
        elif self._requested_data_type == 'out_of_range':
            self._imu_list.append(self.assign_data(
                                  self._imu0,
                                  self._ori_ng[0],
                                  self._ang_vel_ng[0],
                                  self._acc_xy_ng[0],
                                  self._acc_z_ng[0]))
            self._imu_list.append(self.assign_data(
                                  self._imu1,
                                  self._ori_ng[1],
                                  self._ang_vel_ng[1],
                                  self._acc_xy_ng[1],
                                  self._acc_z_ng[1]))
            self._imu_list.append(self.assign_data(
                                  self._imu2,
                                  self._ori_ng[2],
                                  self._ang_vel_ng[2],
                                  self._acc_xy_ng[2],
                                  self._acc_z_ng[2]))

    def update_imu_data(self, req, res):
        with self.lock:
            self._requested_data_type = req.data_type
            self._sleep_time = 1.0 / req.rate
        res.done = True
        return res

    def run(self):
        pub_imu = Imu()
        while rclpy.ok():
            with self.lock:
                sleep_time = self._sleep_time
                if self._requested_data_type is not None:
                    self.set_data()
                    self._requested_data_type = None

            if len(self._imu_list) > 0:
                for imu in self._imu_list:
                    current_time = self._node.get_clock().now()
                    pub_imu = imu
                    pub_imu.header.stamp = current_time.to_msg()
                    self._imu_pub.publish(pub_imu)
                    passed_time = (self._node.get_clock().now() - current_time).nanoseconds * 1e-9
                    current_sleep = sleep_time - passed_time if sleep_time > passed_time else 1e-6
                    self._node.get_clock().sleep_for(Duration(seconds=current_sleep))
            else:
                self._node.get_clock().sleep_for(Duration(seconds=0.1))


def main():
    rclpy.init()
    node = rclpy.create_node('imu_sensor_pub_stub_node')

    executor = MultiThreadedExecutor()
    thread = threading.Thread(target=rclpy.spin, args=(node, executor), daemon=True)
    thread.start()

    stub = ImuSensorPub(node)
    stub.run()

    rclpy.try_shutdown()
    node.destroy_node()


if __name__ == '__main__':
    main()
