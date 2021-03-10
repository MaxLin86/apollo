#!/usr/bin/env python

###############################################################################
# Copyright 2017 The Apollo Authors. All Rights Reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
###############################################################################

#import rospy #shzhw
from cyber_py3 import cyber
import sys
import json
from modules.map.relative_map.proto import navigation_pb2
import time

if __name__ == '__main__':
    navi_files = sys.argv[1:]
    #rospy.init_node("navigator_offline", anonymous=True)#shzhw
    #cyber.init_node("navigator_offline", anonymous=True)
    cyber.init()
    test_node = cyber.Node("navigator_offline")
    #navigation_pub = rospy.Publisher( #shzhw
    navigation_pub = test_node.create_writer(
        "/apollo/navigation", navigation_pb2.NavigationInfo, 1)
    
    # generate navigation info
    navigation_info = navigation_pb2.NavigationInfo()
    priority = 0
    for fdata in navi_files:
        print("processing " + fdata)
        navigation_path = navigation_info.navigation_path.add()
        navigation_path.path_priority = priority
        priority += 1
        navigation_path.path.name = "navigation"

        f = open(fdata, 'r')
        cnt = 0
        for line in f:
            cnt += 1
            if cnt < 3:
                continue
            json_point = json.loads(line)
            point = navigation_path.path.path_point.add()
            point.x = json_point['x']
            point.y = json_point['y']
            point.s = json_point['s']
            point.theta = json_point['theta']
            point.kappa = json_point['kappa']
            point.dkappa = json_point['dkappa']
        f.close()

    # send navigation info to /apollo/navigation
    #r = rospy.Rate(0.5)  # 0.5Hz #shzhw
    #r = cyber.Rate(0.5)  # 0.5Hz
    #while not rospy.is_shutdown(): #shzhw
    while not cyber.is_shutdown():
        time.sleep(0.5)
        navigation_pub.write(navigation_info)
        time.sleep(0.5)
        break
