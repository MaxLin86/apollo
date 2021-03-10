#!/usr/bin/env python

###############################################################################
# Copyright 2018 The Apollo Authors. All Rights Reserved.
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

import sys
from datetime import datetime
from cyber_py.record import RecordReader
from modules.localization.proto import localization_pb2
from modules.routing.proto import routing_pb2
from modules.canbus.proto import chassis_pb2
from modules.drivers.gnss.proto import imu_pb2
from modules.drivers.gnss.proto import gnss_best_pose_pb2
from modules.drivers.gnss.proto import heading_pb2
import csv


if __name__ == '__main__':
    if len(sys.argv) < 2:
        print("usage: python record_extractor.py record_file1 record_file2 ...")

    frecords = sys.argv[1:]
    now = datetime.now().strftime("%Y-%m-%d_%H.%M.%S")

    
    
    f_localization = open("localization_" + frecords[0].split('/')[-1] + ".csv", 'wb')
    csv_writer_localization = csv.writer(f_localization)
    csv_writer_localization.writerow(["timestamp", "x", "y", "vx", "vy", "ax", "ay",  "heading", "std_dev_x", "std_dev_y"])

    f_chassis = open("chassis_" + frecords[0].split('/')[-1] + ".csv", 'wb')
    csv_writer_chassis = csv.writer(f_chassis)
    csv_writer_chassis.writerow(["timestamp", "speed", "steering_percentage"])

    f_routing = open("routing_" + frecords[0].split('/')[-1] + ".csv", 'wb')
    csv_writer_routing = csv.writer(f_routing)
    csv_writer_routing.writerow(["timestamp", "guidepost_x", "guidepost_y", "use_guidepost", "cur_id", "end_id"])

    f_imu = open("imu_" + frecords[0].split('/')[-1] + ".csv", 'wb')
    csv_writer_imu = csv.writer(f_imu)
    csv_writer_imu.writerow(["timestamp", "acc_x", "acc_y", "acc_z", "gry_x", "gry_y", "gry_z"])

    f_gps = open("gps_" + frecords[0].split('/')[-1] + ".csv", 'wb')
    csv_writer_gps = csv.writer(f_gps)
    csv_writer_gps.writerow(["timestamp", "lat", "lon", "height","measure_timestamp"])

    f_heading = open("heading_" + frecords[0].split('/')[-1] + ".csv", 'wb')
    csv_writer_heading = csv.writer(f_heading)
    csv_writer_heading.writerow(["timestamp", "heading","measure_timestamp"])


    #with open("path_" + frecords[0].split('/')[-1] + ".txt", 'w') as f:
    for frecord in frecords:
        print("processing " + frecord)
        reader = RecordReader(frecord)
        for msg in reader.read_messages():
            if msg.topic == "/apollo/localization/pose":
                localization = localization_pb2.LocalizationEstimate()
                localization.ParseFromString(msg.message)
                loc_time = localization.header.timestamp_sec
                x = localization.pose.position.x
                y = localization.pose.position.y
                vx = localization.pose.linear_velocity.x
                vy = localization.pose.linear_velocity.y
                ax = localization.pose.linear_acceleration.x
                ay = localization.pose.linear_acceleration.y
                heading = localization.pose.heading

                std_dev_x = localization.uncertainty.position_std_dev.x
                std_dev_y = localization.uncertainty.position_std_dev.y
            
                #f.write(str(loc_time) + "," + str(x) + "," + str(y) + "\n")
                csv_writer_localization.writerow([loc_time, x, y, vx, vy, ax, ay, heading, std_dev_x, std_dev_y])

            if msg.topic == "/apollo/canbus/chassis":
                chassis = chassis_pb2.Chassis()
                chassis.ParseFromString(msg.message)
                chassis_time = chassis.header.timestamp_sec
                speed = chassis.speed_mps
                steering_percentage = chassis.steering_percentage
            
                csv_writer_chassis.writerow([chassis_time, speed, steering_percentage])

            if msg.topic == "/apollo/routing_response":
                routingResponse = routing_pb2.RoutingResponse()
                routingResponse.ParseFromString(msg.message)
                routing_time = routingResponse.header.timestamp_sec                
                guidepost_x = routingResponse.routing_request.waypoint[1].pose.x
                guidepost_y = routingResponse.routing_request.waypoint[1].pose.y
                is_use_guidepost = routingResponse.routing_request.is_use_guidepost
                cur_id = routingResponse.routing_request.waypoint[1].id
                end_id = routingResponse.routing_request.end_id            

                csv_writer_routing.writerow([routing_time, guidepost_x, guidepost_y, is_use_guidepost+0, cur_id, end_id])

            if msg.topic == "/apollo/sensor/gnss/best_pose":
                gnssBestPost = gnss_best_pose_pb2.GnssBestPose()
                gnssBestPost.ParseFromString(msg.message)
                gnss_best_post_time = gnssBestPost.header.timestamp_sec                
                gnss_best_post_lat = gnssBestPost.latitude
                gnss_best_post_lon = gnssBestPost.longitude
                gnss_best_post_hei = gnssBestPost.height_msl
                gnss_best_post_measure_time = gnssBestPost.measurement_time
                csv_writer_gps.writerow([gnss_best_post_time, gnss_best_post_lat, gnss_best_post_lon, gnss_best_post_hei,gnss_best_post_measure_time])

            if msg.topic == "/apollo/sensor/gnss/imu":
                imuInfo =imu_pb2.Imu()
                imuInfo.ParseFromString(msg.message)
                imu_time = imuInfo.header.timestamp_sec                
                imu_accx = imuInfo.linear_acceleration.x
                imu_accy = imuInfo.linear_acceleration.y
                imu_accz = imuInfo.linear_acceleration.z

                imu_gryx = imuInfo.angular_velocity.x
                imu_gryy = imuInfo.angular_velocity.y
                imu_gryz = imuInfo.angular_velocity.z

                csv_writer_imu.writerow([imu_time, imu_accx, imu_accy, imu_accz, imu_gryx, imu_gryy, imu_gryz])

            if msg.topic == "/apollo/sensor/gnss/heading":
                headingInfo =heading_pb2.Heading()
                headingInfo.ParseFromString(msg.message)
                heading_time = headingInfo.header.timestamp_sec     
                heading = headingInfo.heading
                measure_time = headingInfo.measurement_time

                csv_writer_heading.writerow([heading_time,heading,measure_time])

    f_localization.close()
    f_chassis.close()
    f_routing.close()
    f_heading.close()
