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
        #print("init test %f" %(control_cmd_pb2.ControlCommand().throttle))
        self.canmsg_received = False    
        self.localization_received = True # change by why    

        self.case = 'a'
        self.in_session = False

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
        #self.controlcmd.pad_msg.action = 2  #test by why 
        #self.control_pub.write(self.controlcmd)
        #print(self.controlcmd.driving_mode)

        time.sleep(0.2)
        # Set Default Message
        print('Send Default Command.')
        #self.case == 'a'
        self.controlcmd.pad_msg.action = 1
        self.controlcmd.throttle = 0
        self.controlcmd.brake = 0
        self.controlcmd.steering_rate = 100
        self.controlcmd.steering_target = 0


        #print("gear: %s", self.controlcmd.gear_location)
        #print("gear2: %s",chassis_pb2.Chassis.GEAR_NEUTRAL)
        #if (self.controlcmd.gear_location == chassis_pb2.Chassis.GEAR_NEUTRAL):
        # if (self.gear_location == chassis_pb2.Chassis.GEAR_NEUTRAL):
        #     print("11111111111")
        #     time.sleep(0.2)
        #     for i in range(5):
        #         self.controlcmd.brake = 50
        #         self.controlcmd.gear_location = chassis_pb2.Chassis.GEAR_NEUTRAL
        #         self.control_pub.write(self.controlcmd)
        #     print("first gear %d" %(self.controlcmd.gear_location))
        #     time.sleep(0.2)
        #     for j in range(5):
        #         self.controlcmd.brake = 50
        #         self.controlcmd.gear_location = chassis_pb2.Chassis.GEAR_DRIVE
        #         self.control_pub.write(self.controlcmd)
        #     print("second gear %d" %(self.controlcmd.gear_location))
        # self.controlcmd.gear_location = chassis_pb2.Chassis.GEAR_DRIVE  # test by why 
        # self.controlcmd.brake = 0
        self.control_pub.write(self.controlcmd)
        #print("third gear %d" %(self.controlcmd.gear_location))
        #self.canmsg_received = False  # test by why

        while self.in_session:
            print("cmd gear: %s", self.controlcmd.gear_location)
            print("Chassis gear2: %s",chassis_pb2.Chassis.GEAR_NEUTRAL)
            #if (self.controlcmd.gear_location == chassis_pb2.Chassis.GEAR_NEUTRAL):
            if (self.gear_location == chassis_pb2.Chassis.GEAR_NEUTRAL):
                print("11111111111")
                for i in range(50):
                    self.controlcmd.throttle = 0
                    self.controlcmd.brake = 60
                    self.controlcmd.gear_location = chassis_pb2.Chassis.GEAR_NEUTRAL
                    self.control_pub.write(self.controlcmd)
                    time.sleep(0.1)
                print("first gear %d" %(self.controlcmd.gear_location))
                for j in range(50):
                    self.controlcmd.throttle = 0
                    self.controlcmd.brake = 60
                    self.controlcmd.gear_location = chassis_pb2.Chassis.GEAR_DRIVE
                    self.control_pub.write(self.controlcmd)
                    time.sleep(0.1)
                print("second gear %d" %(self.controlcmd.gear_location))
            # #self.controlcmd.gear_location = chassis_pb2.Chassis.GEAR_DRIVE  # test by why 
            self.controlcmd.brake = 0
            #self.control_pub.write(self.controlcmd)
            #print("third gear %d" %(self.controlcmd.gear_location))
            #self.canmsg_received = False  # test by why

            #print("this is a test2")
            now = cyber_time.Time.now().to_sec()
            self.publish_control()
            sleep_time = 0.01 - (cyber_time.Time.now().to_sec() - now)
            if sleep_time > 0:
                time.sleep(sleep_time)

    def signal_handler(self, signal, frame):
        self.in_session = False

    def callback_localization(self, data):
        """
        New Localization
        """
        self.acceleration = data.pose.linear_acceleration_vrf.x
        #self.acceleration = 0.1  # test by why 
        self.localization_received = True
        #print("data.pose.linear_acceleration_vrf.y: %f" %(data.pose.linear_acceleration_vrf.y))
        #print("self.localization_received %s" %(self.localization_received))
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
        #print(data)
        #print("this is speed_mps %f" %(data.speed_mps))
        #print("this is gear_location %f" %(data.gear_location))
        #print("this is brake_percentage %f" %(data.brake_percentage))
        #print("this is throttle_percentage %f" %(data.throttle_percentage))
        self.canmsg_received = True
        #print("self.canmsg_received: %s" %(self.canmsg_received))
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
        #print("this is a tmp_test, %s",self.case)
        if self.case == 'a':
            if self.cmd[0] > 0:
                #print("3333333333333")
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
            if self.vehicle_speed <= 0.00001:
                #print("44444444444")
                self.case = 'a'
                self.in_session = False
                
        self.controlcmd.steering_target = 0
        self.controlcmd.header.timestamp_sec = cyber_time.Time.now().to_sec()
        self.control_pub.write(self.controlcmd)
        #print(self.controlcmd)
        #print("this is a test3")
        self.write_file(self.controlcmd.header.timestamp_sec, 1)
        if self.in_session == False:
            #self.case == 'a'
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
    plotter = Plotter()
    print("create data_collector")
    node.create_reader('/apollo/localization/pose',
                        localization_pb2.LocalizationEstimate,
                        data_collector.callback_localization)
    node.create_reader('/apollo/canbus/chassis', chassis_pb2.Chassis,
                       data_collector.callback_canbus)

    print('Enter q to quit.')
    print('Enter p to plot result from last run.')
    print('Enter x to remove result from last run.')
    print('Enter x y z, where x is acceleration command, ' +
          'y is speed limit, z is decceleration command.')
    print('Positive number for throttle and negative number for brake.')

    while True:
        #data_collector.case == 'a'
        # if (self.controlcmd.gear_location == chassis_pb2.Chassis.GEAR_NEUTRAL):
        #     self.controlcmd.brake = 50
        #     time.sleep(0.5)
        #     self.controlcmd.gear_location = chassis_pb2.Chassis.GEAR_NEUTRAL
        #     self.control_pub.write(self.controlcmd)
        #     print("first gear %d" %(self.controlcmd.gear_location))
        #     time.sleep(0.5)
        #     self.controlcmd.gear_location = chassis_pb2.Chassis.GEAR_DRIVE
        #     self.control_pub.write(self.controlcmd)
        #     print("second gear %d" %(self.controlcmd.gear_location))
        # #self.controlcmd.gear_location = chassis_pb2.Chassis.GEAR_DRIVE  # test by why 
        # self.controlcmd.brake = 0
        #self.control_pub.write(self.controlcmd)
        #print("third gear %d" %(self.controlcmd.gear_location))
        #self.canmsg_received = False  # test by why

        cmd = raw_input("Enter commands: ").split()
        if len(cmd) == 0:
            print('Quiting.')
            break
        elif len(cmd) == 1:
            if cmd[0] == "q":
                break
            elif cmd[0] == "p":
                print('Plotting result.')
                if os.path.exists(data_collector.outfile):
                    plotter.process_data(data_collector.outfile)
                    plotter.plot_result()
                else:
                    print('File does not exist: %s' % data_collector.outfile)
            elif cmd[0] == "x":
                print('Removing last result.')
                if os.path.exists(data_collector.outfile):
                    os.remove(data_collector.outfile)
                else:
                    print('File does not exist: %s' % date_collector.outfile)
        elif len(cmd) == 3:
            data_collector.run(cmd)

if __name__ == '__main__':
    cyber.init()
    main()
    cyber.shutdown()
