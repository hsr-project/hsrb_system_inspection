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

from hsrb_auto_diagnostics_msgs.srv import UpdateVal
from rclpy.duration import Duration


def update_val_client_impl(node, service_name, device, rate, data_type):
    update_val = node.create_client(UpdateVal, service_name)
    node.get_logger().info("wait_for_service")
    update_val.wait_for_service(timeout_sec=3.0)
    node.get_logger().info("wait_for_service done")
    try:
        future = update_val.call_async(
            UpdateVal.Request(device=device, rate=rate, data_type=data_type))
        node.get_logger().info("wait for future")
        init_time = node.get_clock().now()
        now = init_time
        while not future.done():
            now = node.get_clock().now()
            if (now - init_time) > Duration(seconds=3.0):
                node.get_logger().error("update_val timeout.")
                future.cancel()
                break
            node.get_clock().sleep_for(Duration(seconds=0.1))
        node.get_logger().info("wait for future done")
    except Exception as e:
        node.get_logger().error("Service call failed: %s" % str(e))
    finally:
        node.destroy_client(update_val)
