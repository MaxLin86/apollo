 
 
#!/usr/bin/env python3

# ****************************************************************************
# Copyright 2019 The Apollo Authors. All Rights Reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
#  You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
# ****************************************************************************
# -*- coding: utf-8 -*-
"""Module for example of listener."""

import time
from cyber_py3 import cyber 
 
from modules.monitor.proto.system_status_pb2 import SystemStatus

rev_seq = 0 

def callback_monitor(data):
    """
    Reader message callback.
    """
    print("=" * 80)
    global rev_seq;
    rev_seq +=1;
    print(rev_seq," py:reader /apollo/monitor/system_status->:") 
    print(data)
    print("=" * 80)

def listener_monitor():
    """
    Reader message.
    """
    print("=" * 120)
    test_node = cyber.Node("listener_monitor")
    test_node.create_reader("/apollo/monitor/system_status", SystemStatus, callback_monitor) 
    test_node.spin()

if __name__ == '__main__':
    cyber.init() 
    listener_monitor()
    cyber.shutdown()
