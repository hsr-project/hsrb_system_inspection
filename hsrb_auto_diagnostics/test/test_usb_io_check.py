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

from hsrb_auto_diagnostics.usb_io_check import USBIOCheck
from nose.tools import ok_
import rclpy
from rclpy.executors import MultiThreadedExecutor
import usb

from . import usb_busses_stub

USBIO_CONTAIN = (('0x403', '0x6001'), ('0x3353', '0x5345'))
USBIO_NO_CONTAIN = (('0x3345', '0x3245'), ('0x3353', '0x5345'))
USBIO_VID_CONTAIN = (('0x403', '0x8491'), )
USBIO_PID_CONTAIN = (('0x1489', '0x6001'), ('0x3353', '0x5345'),
                     ('0x1d7a', '0x126'))


class TestUSBIOCheck(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node("test_usb_io_check",
                                      allow_undeclared_parameters=True,
                                      automatically_declare_parameters_from_overrides=True)
        self.executor = MultiThreadedExecutor()
        self.thread = threading.Thread(target=rclpy.spin, args=(self.node, self.executor), daemon=True)
        self.thread.start()

        self._instance = USBIOCheck(self.node)

    def tearDown(self):
        self.node.destroy_node()

    def test_connection_ok(self):
        ubs = usb_busses_stub.UsbBussesStub(USBIO_CONTAIN)
        usb.busses = ubs
        ok_(self._instance.check_connection())

    def test_connection_ng1(self):
        ubs = usb_busses_stub.UsbBussesStub(USBIO_NO_CONTAIN)
        usb.busses = ubs
        ok_(not self._instance.check_connection())

    def test_connection_ng2(self):
        ubs = usb_busses_stub.UsbBussesStub(USBIO_VID_CONTAIN)
        usb.busses = ubs
        ok_(not self._instance.check_connection())

    def test_connection_ng3(self):
        ubs = usb_busses_stub.UsbBussesStub(USBIO_PID_CONTAIN)
        usb.busses = ubs
        ok_(not self._instance.check_connection())
