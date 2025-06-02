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
import glob
import os

from setuptools import setup

package_name = 'hsrb_auto_diagnostics'

setup(
    name=package_name,
    version='2.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        (os.path.join('share', package_name), ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob.glob("launch/*.launch.py")),
        (os.path.join('share', package_name, 'launch', 'include'), glob.glob("launch/include/*.py")),
        (os.path.join('share', package_name, 'config'), glob.glob('config/*.yaml')),
        (os.path.join('share', package_name, 'media'), ['media/hsr-start.wav']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='HSR Support',
    maintainer_email='xr-hsr-support@mail.toyota.co.jp',
    description='Package providing a diagnostic of hsrb devices',
    license='BSD 3-clause Clear License',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ac_adapter_check = hsrb_auto_diagnostics.ac_adapter_check_node:main',
            'battery_check = hsrb_auto_diagnostics.battery_check_node:main',
            'bumper_sensor_check = hsrb_auto_diagnostics.bumper_sensor_check_node:main',
            'drive_power_check = hsrb_auto_diagnostics.drive_power_check_node:main',
            'emergency_button_check = hsrb_auto_diagnostics.emergency_button_check_node:main',
            'force_torque_sensor_check = hsrb_auto_diagnostics.force_torque_sensor_check_node:main',
            'hand_camera_check = hsrb_auto_diagnostics.hand_camera_check_node:main',
            'head_center_camera_check = hsrb_auto_diagnostics.head_center_camera_check_node:main',
            'imu_sensor_check = hsrb_auto_diagnostics.imu_sensor_check_node:main',
            'magnetic_sensor_check = hsrb_auto_diagnostics.magnetic_sensor_check_node:main',
            'microphone_check = hsrb_auto_diagnostics.microphone_check_node:main',
            'motor_amp_check = hsrb_auto_diagnostics.motor_amp_check_node:main',
            'multi_function_check = hsrb_auto_diagnostics.multi_function_check_node:main',
            'pressure_sensor_check = hsrb_auto_diagnostics.pressure_sensor_check_node:main',
            'pressure_sensor_with_valve_check = hsrb_auto_diagnostics.pressure_sensor_with_valve_check_node:main',
            'speaker_check = hsrb_auto_diagnostics.speaker_check_node:main',
            'speaker_with_mute_check = hsrb_auto_diagnostics.speaker_with_mute_check_node:main',
            'status_led_check = hsrb_auto_diagnostics.status_led_check_node:main',
            'stereo_camera_check = hsrb_auto_diagnostics.stereo_camera_check_node:main',
            'tk1_check = hsrb_auto_diagnostics.tk1_check_node:main',
            'urg_sensor_check = hsrb_auto_diagnostics.urg_sensor_check_node:main',
            'usb_io_check = hsrb_auto_diagnostics.usb_io_check_node:main',
            'xtion_check = hsrb_auto_diagnostics.xtion_check_node:main',
            'check_client = hsrb_auto_diagnostics.check_client:main',
        ],
    },
)
