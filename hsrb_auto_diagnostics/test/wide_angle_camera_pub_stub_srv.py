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
import time

from hsrb_auto_diagnostics_msgs.srv import UpdateVal
from numpy.random import randint
import rclpy
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import Image


class WideAngleCameraPub(object):
    def __init__(self, node):
        self._node = node
        self.lock = threading.Lock()
        self._head_camera_data_pub = self._node.create_publisher(
            Image, '/head_center_camera/image_raw', 1)
        self._hand_camera_data_pub = self._node.create_publisher(
            Image, '/hand_camera/image_raw', 1)
        self._node.create_service(UpdateVal, '~/update_val', self.update_val)

        self._data = []
        self._sleep_time = 1.0
        self._device = 'unknown'
        self._requested_data_type = None

        self._empty_data = []
        self._unavailability_data = randint(0, 256, (3, 100)).tolist()
        self._freezed_data = randint(0, 256, (1, 480 * 640 * 2)).tolist()
        self._ok_data = randint(0, 256, (3, 480 * 640 * 2)).tolist()

        self._img = Image()

    def set_data(self):
        if self._requested_data_type == 'empty' or self._requested_data_type == 'empty_sluggish':
            self._data = self._empty_data
        elif (self._requested_data_type == 'unavailability' or self._requested_data_type == 'unavailability_sluggish'):
            self._data = self._unavailability_data
        elif self._requested_data_type == 'freezed' or self._requested_data_type == 'sluggish_freezed':
            self._data = self._freezed_data
        elif self._requested_data_type == 'ok' or self._requested_data_type == 'sluggish':
            self._data = self._ok_data

    def update_val(self, req, res):
        with self.lock:
            self._device = req.device
            self._sleep_time = 1.0 / req.rate
            self._requested_data_type = req.data_type
        time.sleep(0.5)
        res.done = True
        return res

    def run(self):
        while rclpy.ok():
            with self.lock:
                sleep_time = self._sleep_time
                device = self._device
                if self._requested_data_type is not None:
                    self.set_data()
                    self._requested_data_type = None

            if len(self._data) == 0:
                current_time = self._node.get_clock().now()
                self._img.header.stamp = current_time.to_msg()
                self._img.data = self._data
                if device == 'head_center_camera':
                    self._head_camera_data_pub.publish(self._img)
                elif device == 'hand_camera':
                    self._hand_camera_data_pub.publish(self._img)
                passed_time = (self._node.get_clock().now() - current_time).nanoseconds * 1e-9
                current_sleep = sleep_time - passed_time if sleep_time > passed_time else 1e-6
                self._node.get_clock().sleep_for(Duration(seconds=current_sleep))
            else:
                for data in self._data:
                    current_time = self._node.get_clock().now()
                    self._img.header.stamp = current_time.to_msg()
                    self._img.data = data
                    if device == 'head_center_camera':
                        self._head_camera_data_pub.publish(self._img)
                    elif device == 'hand_camera':
                        self._hand_camera_data_pub.publish(self._img)
                    passed_time = (self._node.get_clock().now() - current_time).nanoseconds * 1e-9
                    current_sleep = sleep_time - passed_time if sleep_time > passed_time else 1e-6
                    self._node.get_clock().sleep_for(Duration(seconds=current_sleep))


def main():
    rclpy.init()
    node = rclpy.create_node('wide_angle_camera_pub_stub_node')

    executor = MultiThreadedExecutor()
    thread = threading.Thread(target=rclpy.spin, args=(node, executor), daemon=True)
    thread.start()

    stub = WideAngleCameraPub(node)
    stub.run()

    rclpy.try_shutdown()
    node.destroy_node()


if __name__ == '__main__':
    main()
