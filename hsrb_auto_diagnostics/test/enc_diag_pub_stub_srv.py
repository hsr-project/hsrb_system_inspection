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

from diagnostic_msgs.msg import DiagnosticArray
from diagnostic_msgs.msg import DiagnosticStatus
from hsrb_auto_diagnostics_msgs.srv import UpdateVal
from numpy import random
import rclpy
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor


class EncDiagPub(object):
    def __init__(self, node):
        self._node = node
        self.lock = threading.Lock()
        self._enc_pub = self._node.create_publisher(
            DiagnosticArray, '/diagnostics_agg', 1)
        self._node.create_service(
            UpdateVal, '~/update_val', self.update_enc_diag)

        self._level0_status = DiagnosticStatus()
        self._level2_status = DiagnosticStatus()

        self._level0_status.name = '/Joints/Joint (joint_name)'
        self._level0_status.level = DiagnosticStatus.OK
        self._level0_status.message = 'OK'

        self._level2_status.name = '/Joints/Joint (joint_name)'
        self._level2_status.level = DiagnosticStatus.ERROR
        self._level2_status.message = 'Error message'

        self._enc_diag = DiagnosticArray()

    def update_enc_diag(self, req, res):
        with self.lock:
            self._enc_diag = DiagnosticArray()
            self._enc_diag.header.stamp = self._node.get_clock().now().to_msg()
            for num in range(11):
                self._enc_diag.status.append(self._level0_status)

            index_list = range(11)
            index = random.choice(index_list, 2, replace=False)

            if req.data_type == 'ok':
                pass
            elif req.data_type == 'has_error':
                self._enc_diag.status[index[0]] = self._level2_status
            elif req.data_type == 'has_errors':
                self._enc_diag.status[index[0]] = self._level2_status
                self._enc_diag.status[index[1]] = self._level2_status
        res.done = True
        return res

    def run(self):
        while rclpy.ok():
            current_time = self._node.get_clock().now()
            with self.lock:
                self._enc_pub.publish(self._enc_diag)
            passed_time = (self._node.get_clock().now() - current_time).nanoseconds * 1e-9
            current_sleep = 1.0 - passed_time if 1.0 > passed_time else 1e-6
            self._node.get_clock().sleep_for(Duration(seconds=current_sleep))


def main():
    rclpy.init()
    node = rclpy.create_node('enc_diag_pub_stub_node')

    executor = MultiThreadedExecutor()
    thread = threading.Thread(target=rclpy.spin, args=(node, executor), daemon=True)
    thread.start()

    stub = EncDiagPub(node)
    stub.run()

    rclpy.try_shutdown()
    node.destroy_node()


if __name__ == '__main__':
    main()
