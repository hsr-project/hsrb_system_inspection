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
from sensor_msgs.msg import BatteryState


class BatteryStatePub(object):
    def __init__(self, node):
        self._node = node
        self._lock = threading.Lock()
        self._battery_pub = self._node.create_publisher(
            BatteryState, '/battery_state', 1)
        self._node.create_service(
            UpdateVal, '~/update_val', self.update_battery_state)

        self._battery = BatteryState()

    def update_battery_state(self, req, res):
        # Current specifications of tmc_sanyo_battery
        # In case of error, power_supply_health = BatteryState.POWER_SUPPLY_HEALTH_UNKNOWN
        # If not an error, power_supply_health = BatteryState.POWER_SUPPLY_HEALTH_GOOD
        with self._lock:
            if req.data_type == 'ok':
                self._battery.temperature = 30.0
                self._battery.power_supply_health = BatteryState.POWER_SUPPLY_HEALTH_GOOD
            elif req.data_type == 'high_temperature':
                self._battery.temperature = 100.0
                self._battery.power_supply_health = BatteryState.POWER_SUPPLY_HEALTH_GOOD
            elif req.data_type == 'error_occurred':
                self._battery.temperature = 30.0
                self._battery.power_supply_health = BatteryState.POWER_SUPPLY_HEALTH_UNKNOWN
            elif req.data_type == 'all_false':
                self._battery.temperature = 100.0
                self._battery.power_supply_health = BatteryState.POWER_SUPPLY_HEALTH_UNKNOWN
        res.done = True
        return res

    def run(self):
        while rclpy.ok():
            current_time = self._node.get_clock().now()
            with self._lock:
                self._battery.header.stamp = current_time.to_msg()
                self._battery_pub.publish(self._battery)
            passed_time = (self._node.get_clock().now() - current_time).nanoseconds * 1e-9
            current_sleep = 1.0 - passed_time if 1.0 > passed_time else 1e-6
            self._node.get_clock().sleep_for(Duration(seconds=current_sleep))


def main():
    rclpy.init()
    node = rclpy.create_node('battery_state_pub_stub_node')

    executor = MultiThreadedExecutor()
    thread = threading.Thread(target=rclpy.spin, args=(node, executor), daemon=True)
    thread.start()

    stub = BatteryStatePub(node)
    stub.run()

    rclpy.try_shutdown()
    node.destroy_node()


if __name__ == '__main__':
    main()
