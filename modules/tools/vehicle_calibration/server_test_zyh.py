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
"""
Data Collector
"""
import os
import sys
import time
import signal
import time

from cyber_py import cyber
from cyber_py import cyber_time

from plot_data import Plotter

from modules.canbus.proto import chassis_pb2
from modules.control.proto import control_cmd_pb2
from modules.localization.proto import localization_pb2
import argparse, socket, time
import struct


class DataCollector(object):
    """
    DataCollector Class
    """

    def __init__(self, node):
        self.sequence_num = 0
        self.control_pub = node.create_writer('/apollo/control',
                                              control_cmd_pb2.ControlCommand)
        time.sleep(0.3)
        self.controlcmd = control_cmd_pb2.ControlCommand()
        self.outfile = ""

    def run(self, cmd):
        signal.signal(signal.SIGINT, self.signal_handler)
        self.in_session = True
        self.cmd = map(float, cmd)
        out = ''
        if self.cmd[0] > 0:
            out += 't'
        else:
            out += 'b'
        out = out + str(int(self.cmd[0]))
        if self.cmd[2] > 0:
            out += 't'
        else:
            out += 'b'
        out += str(int(self.cmd[2])) + 'r'
        i = 0
        self.outfile = out + str(i) + '_recorded.csv'
        while os.path.exists(self.outfile):
            i += 1
            self.outfile = out + str(i) + '_recorded.csv'
        self.file = open(self.outfile, 'w')
        self.file.write(
            "time,io,ctlmode,ctlbrake,ctlthrottle,ctlgear_location," +
            "vehicle_speed,engine_rpm,driving_mode,throttle_percentage," +
            "brake_percentage,gear_location,imu\n"
        )

        print('Send Reset Command.')
        self.controlcmd.header.module_name = "control"
        self.controlcmd.header.sequence_num = self.sequence_num
        self.sequence_num = self.sequence_num + 1
        self.controlcmd.header.timestamp_sec = cyber_time.Time.now().to_sec()
        self.controlcmd.pad_msg.action = 2
        self.control_pub.write(self.controlcmd)

        time.sleep(0.2)
        # Set Default Message
        print('Send Default Command.')
        self.controlcmd.pad_msg.action = 1
        self.controlcmd.throttle = 0
        self.controlcmd.brake = 0
        self.controlcmd.steering_rate = 100
        self.controlcmd.steering_target = 0
        self.controlcmd.gear_location = chassis_pb2.Chassis.GEAR_DRIVE

        self.canmsg_received = False

        while self.in_session:
            now = cyber_time.Time.now().to_sec()
            self.publish_control()
            sleep_time = 0.01 - (cyber_time.Time.now().to_sec() - now)
            if sleep_time > 0:
                time.sleep(sleep_time)

    def callback_canbus(self, data):
        """
        New CANBUS
        """
        if not self.localization_received:
            print('No Localization Message Yet')
            return
        timenow = data.header.timestamp_sec
        self.vehicle_speed = data.speed_mps
        self.engine_rpm = data.engine_rpm
        self.throttle_percentage = data.throttle_percentage
        self.brake_percentage = data.brake_percentage
        self.gear_location = data.gear_location
        self.driving_mode = data.driving_mode

        self.canmsg_received = True
        if self.in_session:
            self.write_file(timenow, 0)

    def publish_control(self):
        """
        New Control Command
        """
        if not self.canmsg_received:
            print('No CAN Message Yet')
            return

        self.controlcmd.header.sequence_num = self.sequence_num
        self.sequence_num += 1

        if self.case == 'a':
            if self.cmd[0] > 0:
                self.controlcmd.throttle = self.cmd[0]
                self.controlcmd.brake = 0
            else:
                self.controlcmd.throttle = 0
                self.controlcmd.brake = -self.cmd[0]
            if self.vehicle_speed >= self.cmd[1]:
                self.case = 'd'
        elif self.case == 'd':
            if self.cmd[2] > 0:
                self.controlcmd.throttle = self.cmd[0]
                self.controlcmd.brake = 0
            else:
                self.controlcmd.throttle = 0
                self.controlcmd.brake = -self.cmd[2]
            if self.vehicle_speed == 0:
                self.in_session = False

        self.controlcmd.header.timestamp_sec = cyber_time.Time.now().to_sec()
        self.control_pub.write(self.controlcmd)
        self.write_file(self.controlcmd.header.timestamp_sec, 1)
        if self.in_session == False:
            self.file.close()

    def write_file(self, time, io):
        """
        Write Message to File
        """
        self.file.write(
            "%.4f,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s\n" %
            (time, io, 1, self.controlcmd.brake, self.controlcmd.throttle,
             self.controlcmd.gear_location, self.vehicle_speed, self.engine_rpm,
             self.driving_mode, self.throttle_percentage, self.brake_percentage,
             self.gear_location, self.acceleration))

def main():
    """
    Main function
    """
    node = cyber.Node("data_collector")
    data_collector = DataCollector(node)
    MAX_BYTES = 65535

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(('192.168.95.146', 10000))
    print('Listening at {}'.format(sock.getsockname()))
    while True:
        Control_data, address = sock.recvfrom(MAX_BYTES)

        data_Sequence, AD_StatusReq, AD_GearReq, AD_TorqueReqPercent, AD_MaxSpeedLimitReq, \
        AD_VCU_SW_Version_VIN_Req, AD_BrakeReq, AD_BrakePercent, AD_EPB, AD_ADComponentStatus, AD_DTC, \
        AD_SteeringWheelTurnAngleCmd, AD_WorkModeCmd, AD_ADUStatus, AD_TurnCmdValid, AD_TurnAddedTorque, \
        AD_WheelSpeed, AD_WarningLampRequest, AD_LeftTurnLampRequest, AD_RightTurnLampRequest, AD_PositionLampRequest, \
        AD_DippedHeadLampRequest, AD_HeadLightFullBeamRequest, AD_FrontFogLampRequest, AD_RearFogLampRequest = struct.unpack(
            '>HBBHHBBHBBHhBBBBHBBBBBBBB', Control_data)

        data_collector.controlcmd.brake = AD_BrakePercent
        data_collector.controlcmd.throttle = AD_TorqueReqPercent
        data_collector.controlcmd.gear_location = AD_GearReq
        #data_collector.controlcmd.driving_mode = AD_StatusReq
        #data_collector.controlcmd.steering_rate = (AD_WheelSpeed*100)/540.0
        data_collector.controlcmd.steering_target = AD_SteeringWheelTurnAngleCmd/10
        
        data_collector.controlcmd.header.module_name = "control"
        data_collector.controlcmd.header.sequence_num = data_collector.sequence_num
        data_collector.sequence_num = data_collector.sequence_num + 1
        data_collector.controlcmd.header.timestamp_sec = cyber_time.Time.now().to_sec()
        
        data_collector.control_pub.write(data_collector.controlcmd)

        print (data_collector.controlcmd)


if __name__ == '__main__':
    cyber.init()
    main()
    cyber.shutdown()
