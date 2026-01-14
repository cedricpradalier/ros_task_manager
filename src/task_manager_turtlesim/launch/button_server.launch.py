# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#
# Software License Agreement (BSD License 2.0)
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of the Willow Garage nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import os
import sys
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription

import launch_ros.actions
import launch_ros.descriptions


def generate_launch_description():
    return LaunchDescription([
        launch_ros.actions.Node(
            package='button_server', executable='button_server_node',
            name='button_server',
            parameters=[
                {'root': os.path.join(get_package_share_directory('button_server'),".")},
                {'port': 5180},
                {'num_buttons': 4},
                {'button0': "Go"},
                {'button0_text': "Go"},
                {'button0_color': "lightgreen"},
                {'button1': 'Pause'},
                {'button1_text': 'Pause'},
                {'button1_style': 'background-color:#202020;color:white;'},
                {'button2': 'Home'},
                {'button2_text': 'Go Home'},
                {'button2_color': 'lightblue'},
                {'button3': 'Abort'},
                {'button3_text': 'Abort'},
                {'button3_style': 'background-color:red;color:black;'},
                ],
            remappings=[
                #('~/clouds3d', '/points'),
                ],
            output='screen'),

    ])
