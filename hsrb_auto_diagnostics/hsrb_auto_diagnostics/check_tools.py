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
import os
import subprocess
from threading import Lock

from diagnostic_msgs.msg import DiagnosticStatus
import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.qos import QoSProfile
from rclpy.time import Time
from sensor_msgs.msg import BatteryState
import usb


def check_pyusb(vid, pid):
    return (int(vid, 16), int(pid, 16)) in [
        (int(device.idVendor), int(device.idProduct))
        for bus in usb.busses() for device in bus.devices]


def check_num_usb_devices(vid, pid):
    return len([1 for bus in usb.busses() for device in bus.devices
               if (int(vid, 16), int(pid, 16)) == (int(device.idVendor), int(device.idProduct))])


def check_dev_path(dev_path):
    return os.path.exists(dev_path)


def check_ping(ip):
    ping_cmd = ['ping', ip, '-c', '5']
    try:
        subprocess.check_output(ping_cmd)
        return True
    except Exception:
        return False


def make_result(result_list):
    while None in result_list:
        result_list.remove(None)
    return result_list


class SubCheck(object):
    def __init__(self, node, topic_name, topic_type):
        self._node = node
        self._topic_name = topic_name
        self._topic_type = topic_type
        self.subscribe_data = []
        self.subscribe_stamp = []
        self._error_msg = ['Subscribe Error',
                           'Data Size Error',
                           'Subscribe Frequency Error',
                           'Data Freeze Error',
                           'Data Range Error']
        self._size = 0
        self._range_min = 0.0
        self._range_max = 0.0

        self.BATTERY_MAX_TEMPERATURE = 50.0

        self.lock = Lock()

    def data_sub_cb(self, data):
        with self.lock:
            self.subscribe_data.append(data.data)
            if hasattr(data, 'header'):
                self.subscribe_stamp.append(data.header.stamp)
            else:
                self.subscribe_stamp.append(self._node.get_clock().now().to_msg())

    def enc_sub_cb(self, data):
        with self.lock:
            self.subscribe_data.append(data.status)
            self.subscribe_stamp.append(data.header.stamp)

    def battery_sub_cb(self, data):
        with self.lock:
            battery_state = [
                data.temperature,
                data.power_supply_health != BatteryState.POWER_SUPPLY_HEALTH_GOOD
            ]
            self.subscribe_data.append(battery_state)
            self.subscribe_stamp.append(data.header.stamp)

    def urg_sub_cb(self, data):
        with self.lock:
            self.subscribe_data.append(data.ranges)
            self.subscribe_stamp.append(data.header.stamp)
        if self._size == 0:
            self._size = int((data.angle_max - data.angle_min) / data.angle_increment) + 1

    def wrench_sub_cb(self, data):
        with self.lock:
            wrist_wrench = [data.wrench.force.x, data.wrench.force.y,
                            data.wrench.force.z, data.wrench.torque.x,
                            data.wrench.torque.y, data.wrench.torque.z]
            self.subscribe_data.append(wrist_wrench)
            self.subscribe_stamp.append(data.header.stamp)

    def imu_sub_cb(self, data):
        with self.lock:
            imu = [data.orientation.y, data.orientation.z,
                   data.orientation.w, data.angular_velocity.x,
                   data.angular_velocity.y, data.angular_velocity.z,
                   data.linear_acceleration.x,
                   data.linear_acceleration.y,
                   data.linear_acceleration.z]
            self.subscribe_data.append(imu)
            self.subscribe_stamp.append(data.header.stamp)

    def subscribe_topic(self, device, sub_duration,
                        _goal_handle,
                        qos_profile=QoSProfile(depth=1),
                        connection_timeout=30.0):
        sub = None
        if device == 'data':
            sub = self._node.create_subscription(
                self._topic_type,
                self._topic_name,
                self.data_sub_cb,
                qos_profile)
        elif device == 'battery':
            self._node.get_logger().info("battery subscriber for %s" % self._topic_name)
            sub = self._node.create_subscription(
                self._topic_type,
                self._topic_name,
                self.battery_sub_cb,
                qos_profile)
        elif device == 'enc':
            sub = self._node.create_subscription(
                self._topic_type,
                self._topic_name,
                self.enc_sub_cb,
                qos_profile)
        elif device == 'urg':
            sub = self._node.create_subscription(
                self._topic_type,
                self._topic_name,
                self.urg_sub_cb,
                qos_profile)
        elif device == 'wrench':
            sub = self._node.create_subscription(
                self._topic_type,
                self._topic_name,
                self.wrench_sub_cb,
                qos_profile)
        elif device == 'imu':
            sub = self._node.create_subscription(
                self._topic_type,
                self._topic_name,
                self.imu_sub_cb,
                qos_profile)
        with self.lock:
            self.subscribe_data = []
            self.subscribe_stamp = []

        init = self._node.get_clock().now()
        now = init

        self._node.get_logger().info("wait for message for %f senconds" % connection_timeout)
        while (len(self.subscribe_data) == 0) and rclpy.ok():
            if (now - init) < Duration(seconds=connection_timeout):
                now = self._node.get_clock().now()
                self._node.get_clock().sleep_for(Duration(seconds=0.1))
            else:
                self._node.get_logger().error("no message subscribed.")
                if sub is not None:
                    self._node.destroy_subscription(sub)
                return [], []

        init = now
        self._node.get_logger().info("subscribe for %f seconds" % sub_duration)
        while (now - init) < Duration(seconds=sub_duration):
            if _goal_handle.is_cancel_requested:
                raise Exception('Canceled')
            self._node.get_clock().sleep_for(Duration(seconds=0.1))
            now = self._node.get_clock().now()

        self._node.get_logger().info("subscribed")
        if sub is not None:
            self._node.destroy_subscription(sub)
        self._node.get_logger().info("  data length: %d" % len(self.subscribe_data))
        return self.subscribe_data, self.subscribe_stamp

    def sub_success_check(self, subscribed_data):
        if len(subscribed_data) > 0:
            return True

    def size_check(self, subscribed_data, expected_sizes):
        unavailability = [len(data) for data in subscribed_data
                          if len(data) not in expected_sizes]
        if len(unavailability) > 0:
            expected_str = str(expected_sizes[0])
            if len(expected_sizes) > 1:
                for i in range(1, len(expected_sizes) - 1):
                    expected_str = expected_str + ', ' + str(expected_sizes[i])
                expected_str = expected_str + ' or ' + str(expected_sizes[-1])
            self._node.get_logger().error(
                '[%s] Data size:%d (Expected:%s)' %
                (self._topic_name, unavailability[0], expected_str))
            return self._error_msg[1]

    def frequency_check(self, subscribed_data, subscribed_stamp,
                        sub_duration, expect_frequency):
        # NOTE: In the conventional method, the wait seconds were determined from the desired frequency
        #    and judged based on whether it reached the number of received messages.
        #   In this method, if one is missed before or after, it won't pass, and this happens frequently in ROS 2.
        # if len(subscribed_data) < sub_duration * expect_frequency:
        #     self._node.get_logger().error(
        #         '[%s] Frequency:%.2fHz (Expected:more than %.1fHz)' %
        #         (self._topic_name, len(subscribed_data) / sub_duration,
        #          expect_frequency))
        #     return self._error_msg[2]

        # NOTE: Obtain header.stmap at the same time, look at the timestamp, and calculate the frequency.
        assert len(subscribed_data) == len(subscribed_stamp)
        estimated_frequency = 0.0
        if len(subscribed_data) == 1:
            estimated_frequency = 1.0 / sub_duration
        elif len(subscribed_data) > 1:
            first_time_stamp = Time.from_msg(subscribed_stamp[0])
            last_time_stamp = Time.from_msg(subscribed_stamp[-1])
            estimated_duration = float((last_time_stamp - first_time_stamp).nanoseconds) * 1e-9
            if (estimated_duration > 1e-6):
                estimated_frequency = float(len(subscribed_data) - 1) / estimated_duration

        # NOTE: Also want to confirm continuous reception over the entire section, so also check the number of receptions.
        expect_data_length = sub_duration * expect_frequency
        # NOTE: Judgment criteria: Accept up to floor(desired number of receptions * 0.99) and (desired frequency * 0.95).
        check_result = ((len(subscribed_data) >= int(expect_data_length * 0.99))
                        and (estimated_frequency >= (expect_frequency * 0.95)))

        # estimated_frequency = len(subscribed_data) / sub_duration
        if not check_result:
            self._node.get_logger().error(
                '[%s] Data length:%d (Expected:more than %d)' %
                (self._topic_name, len(subscribed_data), expect_data_length))
            self._node.get_logger().error(
                '[%s] Frequency:%.2fHz (Expected:more than %.1fHz)' %
                (self._topic_name, estimated_frequency, expect_frequency))
            return self._error_msg[2]

    def freeze_check(self, arg_name, subscribed_data):
        if len(subscribed_data) > 1:
            if all([data == subscribed_data[0]
                    for data in subscribed_data[1:]]):
                return arg_name + self._error_msg[3]

    def range_check(self, arg_name, subscribed_data, range_min, range_max):
        out_of_range = [data for data in subscribed_data
                        if data < range_min or data > range_max]
        if len(out_of_range) > 0:
            self._node.get_logger().error(
                '[%s] %s%.4f (Expected data range:%.1f~%.1f)' %
                (self._topic_name, arg_name, out_of_range[0],
                 range_min, range_max))
            return arg_name + self._error_msg[4]

    def check_camera_msg(self, expected_data_sizes, _as,
                         expect_frequency,
                         qos_profile=QoSProfile(depth=1),
                         sub_duration=3.0):
        """Three types of check for Image or PointCloud2 data

        data size
        frequency
        freeze or not
        """
        result = []
        subscribed_data, subscribed_stamp = self.subscribe_topic('data', sub_duration, _as, qos_profile=qos_profile)
        if self.sub_success_check(subscribed_data):
            result.append(self.size_check(subscribed_data,
                                          expected_data_sizes))
            result.append(self.frequency_check(subscribed_data, subscribed_stamp,
                          sub_duration, expect_frequency))
            result.append(self.freeze_check('', subscribed_data))
        else:
            result.append(self._error_msg[0])
            self._node.get_logger().error('Cannot subscribe to %s' % self._topic_name)
        return make_result(result)

    def check_battery_msg(self, _goal_handle, sub_duration=3.0):
        result = []
        subscribed_data, _ = self.subscribe_topic('battery', sub_duration, _goal_handle)
        if self.sub_success_check(subscribed_data):
            temperature = []
            error = []
            for data in subscribed_data:
                temperature.append(data[0])
                error.append(data[1])
            result.append(self.range_check('temperature:', temperature, 0.0,
                          self.BATTERY_MAX_TEMPERATURE))
            if any(error):
                result.append('Status Error')
        else:
            result.append(self._error_msg[0])
            self._node.get_logger().error('Cannot subscribe to %s' % self._topic_name)
        self._node.get_logger().info("  result: %s" % result)
        return make_result(result)

    def check_enc_msg(self, _as, sub_duration=5.0):
        result = []
        subscribed_data, _ = self.subscribe_topic(
            'enc', sub_duration, _as)

        if self.sub_success_check(subscribed_data):
            for data in subscribed_data[0]:
                if 'Joints/Joint' in data.name:
                    if data.level == DiagnosticStatus.ERROR:
                        self._node.get_logger().error(
                            'ERROR:%s -> %s' % (data.name[15:-1], data.message))
                        result = ['Diagnostic Error']
        else:
            result.append(self._error_msg[0])
            self._node.get_logger().error('Cannot subscribe to %s' % self._topic_name)

        return result

    def check_urg_msg(self, _as, expect_frequency, sub_duration=3.0):
        result = []
        subscribed_data, subscribed_stamp = self.subscribe_topic('urg', sub_duration, _as)
        if self.sub_success_check(subscribed_data):
            result.append(self.size_check(subscribed_data, [self._size]))
            result.append(self.frequency_check(subscribed_data, subscribed_stamp,
                          sub_duration, expect_frequency))
            result.append(self.freeze_check('', subscribed_data))
        else:
            result.append(self._error_msg[0])
            self._node.get_logger().error('Cannot subscribe to %s' % self._topic_name)
        return make_result(result)

    def check_wrench_msg(self, _as, expect_frequency, sub_duration=2.0):
        subscribed_data, subscribed_stamp = self.subscribe_topic('wrench', sub_duration, _as)
        axis = ['Fx:', 'Fy:', 'Fz:', 'Mx:', 'My:', 'Mz:']
        result = []
        if self.sub_success_check(subscribed_data):
            result.append(self.frequency_check(subscribed_data, subscribed_stamp,
                          sub_duration, expect_frequency))

            for i in range(6):
                data_piece = []
                result_piece = []
                for data in subscribed_data:
                    data_piece.append(data[i])
                if i < 3:
                    result_piece.append(self.freeze_check(axis[i], data_piece))
                    result_piece.append(self.range_check(axis[i], data_piece,
                                        -100.0, 100.0))
                else:
                    result_piece.append(self.freeze_check(axis[i], data_piece))
                    result_piece.append(self.range_check(axis[i], data_piece,
                                        -5.0, 5.0))
                result.extend(result_piece)
        else:
            result.append(self._error_msg[0])
            self._node.get_logger().error('Cannot subscribe to %s' % self._topic_name)
        return make_result(result)

    def check_imu_msg(self, _as, expect_frequency,
                      test_info, sub_duration=2.0):
        subscribed_data, subscribed_stamp = self.subscribe_topic('imu', sub_duration, _as)
        result = []
        # Message subscription and periodic check
        if self.sub_success_check(subscribed_data):
            result.append(self.frequency_check(subscribed_data, subscribed_stamp,
                          sub_duration, expect_frequency))

            # Generate a list for each axis
            np_sub_data = np.array(subscribed_data)
            np_reshaped_sub_data = np_sub_data.transpose()
            subscribed_data = np_reshaped_sub_data.tolist()

            # Data range check
            # test_key = ["name", do_freeze_check, l_limit, u_limit]
            result_piece = []
            for i, test_key in enumerate(test_info):
                key_name = test_key[0]
                do_freeze_check = test_key[1]
                lower_limit = test_key[2]
                upper_limit = test_key[3]
                if do_freeze_check:
                    result_piece.append(
                        self.freeze_check(key_name, subscribed_data[i]))
                # NOTE: Because it is now impossible to store None in double type param, set an invalid value separately
                # if lower_limit != 'None' and upper_limit != 'None':
                if upper_limit - lower_limit > 1e-5:
                    result_piece.append(
                        self.range_check(key_name, subscribed_data[i],
                                         lower_limit, upper_limit))
            result.extend(result_piece)
        else:
            result.append(self._error_msg[0])
            self._node.get_logger().error('Cannot subscribe to %s' % self._topic_name)
        return make_result(result)


class Interactive(object):
    def __init__(self, node, topic_name, topic_type):
        self._node = node
        self._topic_name = topic_name
        self._topic_type = topic_type
        self.subscribe_data = []
        self.lock = Lock()
        self.sub = self._node.create_subscription(
            topic_type, topic_name, self.sub_cb, 1)

    def sub_cb(self, data):
        with self.lock:
            self.subscribe_data.append(data.data)

    def subscribe_user_response(self, _as, timeout):
        with self.lock:
            self.subscribe_data = []

        init = self._node.get_clock().now()
        now = init
        while (len(self.subscribe_data) == 0) and rclpy.ok():
            self._node.get_clock().sleep_for(Duration(seconds=0.033))
            if now - init < Duration(seconds=timeout):
                now = self._node.get_clock().now()
            else:
                return []
        return self.subscribe_data

    def publish_feedback(self, msg, _fb, _goal_handle):
        _fb.feedback_msg = msg
        _goal_handle.publish_feedback(_fb)

    def wait_for_user_response(self, _fb, _goal_handle, fb_msg, err_msg, timeout):
        list_of_result = []
        self.publish_feedback(fb_msg, _fb, _goal_handle)
        result = self.subscribe_user_response(_goal_handle, timeout)
        if len(result) == 0:
            list_of_result.append('Diagnostic Canceled')
        elif result[0]:
            list_of_result.append(None)
        else:
            list_of_result.append(err_msg)
        return make_result(list_of_result)

    def action_client(self, client, goal):
        if client.wait_for_server(Duration(seconds=3.0)):
            future = client.send_goal_async(goal)
            init_time = self._node.get_clock().now()
            now = init_time
            while not future.done():
                now = self._node.get_clock().now()
                if (now - init_time) > Duration(seconds=3.0):
                    future.cancel()
                    return False
                self._node.get_clock().sleep_for(Duration(seconds=0.1))
            return True
        else:
            return False
