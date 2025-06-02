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
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('hsrb_auto_diagnostics'),
        'config',
        'test_info.yaml')

    battery_check_node = Node(
        package='hsrb_auto_diagnostics',
        executable='battery_check',
        name='battery_check',
        namespace='diag',
        output='screen',
    )
    force_torque_sensor_check_node = Node(
        package='hsrb_auto_diagnostics',
        executable='force_torque_sensor_check',
        name='force_torque_sensor_check',
        namespace='diag',
        output='screen',
    )
    imu_sensor_check_node = Node(
        package='hsrb_auto_diagnostics',
        executable='imu_sensor_check',
        name='imu_sensor_check',
        namespace='diag',
        output='screen',
        parameters=[
            config,
            {
                'dev_path': '/dev/ttyCTI3',
            }],
    )
    hand_camera_check_node = Node(
        package='hsrb_auto_diagnostics',
        executable='hand_camera_check',
        name='hand_camera_check',
        namespace='diag',
        output='screen',
    )
    motor_amp_check_node = Node(
        package='hsrb_auto_diagnostics',
        executable='motor_amp_check',
        name='motor_amp_check',
        namespace='diag',
        output='screen',
    )
    stereo_camera_check_node = Node(
        package='hsrb_auto_diagnostics',
        executable='stereo_camera_check',
        name='stereo_camera_check',
        namespace='diag',
        output='screen',
        parameters=[{
            'product_id': '0x33000',
        }],
    )
    tk1_check_node = Node(
        package='hsrb_auto_diagnostics',
        executable='tk1_check',
        name='tk1_check',
        namespace='diag',
        output='screen',
        parameters=[{
            'tk1_hostname': 'hsrb-tk1.local',
        }],
    )
    urg_sensor_check_node = Node(
        package='hsrb_auto_diagnostics',
        executable='urg_sensor_check',
        name='urg_sensor_check',
        namespace='diag',
        output='screen',
        parameters=[{
            'urg_ip': '10.255.255.2',
        }],
    )
    rear_urg_sensor_check_node = Node(
        package='hsrb_auto_diagnostics',
        executable='urg_sensor_check',
        name='rear_urg_sensor_check',
        namespace='diag',
        output='screen',
        parameters=[{
            'urg_ip': '10.255.255.3',
            'action_name': 'rear_urg_sensor',
        }],
        remappings=[
            ('/base_scan', '/base_rear_scan'),
        ],
    )
    usb_io_check_node = Node(
        package='hsrb_auto_diagnostics',
        executable='usb_io_check',
        name='usb_io_check',
        namespace='diag',
        output='screen',
    )
    xtion_check_node = Node(
        package='hsrb_auto_diagnostics',
        executable='xtion_check',
        name='xtion_check',
        namespace='diag',
        output='screen',
    )

    bumper_sensor_check_node = Node(
        package='hsrb_auto_diagnostics',
        executable='bumper_sensor_check',
        name='bumper_sensor_check',
        namespace='diag_interactive',
        output='screen',
        remappings=[
            ('command_status_led_rgb', '/command_status_led_rgb'),
        ],
    )
    microphone_check_node = Node(
        package='hsrb_auto_diagnostics',
        executable='microphone_check',
        name='microphone_check',
        namespace='diag_interactive',
        output='screen',
        parameters=[{
            'vendor_id': '0xffff',
            'product_id': '0xfffd',
        }],
    )
    # pressure_sensor_check_node = Node(
    #     package='hsrb_auto_diagnostics',
    #     executable='pressure_sensor_with_valve_check',
    #     name='pressure_sensor_with_valve_check',
    #     namespace='diag_interactive',
    #     output='screen',
    #     parameters=[{
    #         'valve_topic': '/command_valve',
    #     }],
    # )
    speaker_with_mute_check_node = Node(
        package='hsrb_auto_diagnostics',
        executable='speaker_with_mute_check',
        name='speaker_with_mute_check',
        namespace='diag_interactive',
        output='screen',
        parameters=[{
            'mute_srv': '/sound/mute',
            'wav_file_name':
                get_package_share_directory('hsrb_auto_diagnostics') + '/media/hsr-start.wav',
        }],
    )
    # status_led_check_node = Node(
    #     package='hsrb_auto_diagnostics',
    #     executable='status_led_check',
    #     name='status_led_check',
    #     namespace='diag_interactive',
    #     output='screen',
    # )
    multi_function_check_node = Node(
        package='hsrb_auto_diagnostics',
        executable='multi_function_check',
        name='multi_function_check',
        namespace='diag_interactive',
        output='screen',
        parameters=[{
            'multifunctional_button_topic': '/switch_input',
            'multifunctional_button_led_topic': '/switch_led',
        }],
    )
    ac_adapter_check_node = Node(
        package='hsrb_auto_diagnostics',
        executable='ac_adapter_check',
        name='ac_adapter_check',
        namespace='diag_interactive',
        output='screen',
        parameters=[{
            'ac_adapter_topic': '/charger/state',
        }],
    )

    return LaunchDescription([
        battery_check_node,
        force_torque_sensor_check_node,
        imu_sensor_check_node,
        hand_camera_check_node,
        motor_amp_check_node,
        stereo_camera_check_node,
        tk1_check_node,
        urg_sensor_check_node,
        rear_urg_sensor_check_node,
        usb_io_check_node,
        xtion_check_node,
        bumper_sensor_check_node,
        microphone_check_node,
        # pressure_sensor_check_node,
        speaker_with_mute_check_node,
        # status_led_check_node,
        multi_function_check_node,
        ac_adapter_check_node,
    ])
