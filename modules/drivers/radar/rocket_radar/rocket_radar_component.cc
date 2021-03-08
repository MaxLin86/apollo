/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/
#include "modules/drivers/radar/rocket_radar/rocket_radar_component.h"

#include "modules/drivers/radar/rocket_radar/driver/include/sra.h"
#include "modules/drivers/radar/rocket_radar/driver/include/connection.h"
#include "modules/drivers/radar/rocket_radar/driver/include/scanning.h"
#include "modules/drivers/radar/rocket_radar/driver/include/scanobject.h"
#include "modules/drivers/radar/rocket_radar/driver/include/detections.h"
#include "modules/drivers/radar/rocket_radar/driver/include/pointcloud.h"
#include "modules/drivers/radar/rocket_radar/driver/system-radar-software/engine/scp-src/eng-api/uhmath.h"
#include "modules/drivers/radar/rocket_radar/driver/include/userlogagents.h"

#include <string>
#include <thread>
#include "modules/common/adapters/adapter_gflags.h"

#include "modules/common/configs/config_gflags.h"//add by shzhw
#include "modules/common/util/message_util.h"

#include<iostream>
#include <fstream>
#include <iomanip>
#include <math.h>

namespace apollo {
namespace drivers {
namespace uhnder_radar {

#define PI (3.1415926)

bool RocketRadarComponent::Init() {
  AINFO << "RocketRadarComponent init";
  if (!GetProtoConfig(&uhnder_radar_config_)) {
    return false;
  }

  radar_id_ = uhnder_radar_config_.radar_id();
  radar_ip_ = uhnder_radar_config_.radar_ip().c_str();
  install_agle_ = (uhnder_radar_config_.install_angle()) / 180.0 * PI;
  install_pos_x_ = uhnder_radar_config_.install_pos_x();
  install_pos_y_ = uhnder_radar_config_.install_pos_y();
  install_pos_z_ = uhnder_radar_config_.install_pos_z();

  radarMsg_.Clear();
  if (radar_id_ == "middle")
  {
    radarMsg_.set_radar_id(0);
  } else if (radar_id_ == "left")
  {
    radarMsg_.set_radar_id(1);
  } else if (radar_id_ == "right")
  {
    radarMsg_.set_radar_id(-1);
  }else if (radar_id_ == "right_corner")
  {
    radarMsg_.set_radar_id(-2);
  } else if (radar_id_ == "left_corner")
  {
    radarMsg_.set_radar_id(2);
  }else if(radar_id_ == "top"){
    radarMsg_.set_radar_id(5);
  }
  
  frame_count = 0;
  // ThresholdPreset thre = uhnder_radar_config_.threshold();
  // if (thre == LOW) {
  //   threshold_ = LOW_SENSITIVITY;
  // } else if (thre == HIGH) {
  //   threshold_ = HIGH_SENSITIVITY;
  // } else {
  //   threshold_ = MID_SENSITIVITY;
  // }

  // RDC_Scan_Type scanType = uhnder_radar_config_.scan_type();
  // selectScanType(scanType);

  // scan_delay_offset_us_ = uhnder_radar_config_.scan_delay_offset_us();
  
  positive_ = uhnder_radar_config_.positive();

  uhnder_radar_writer_ =
      node_->CreateWriter<ContiRadar>(uhnder_radar_config_.radar_channel());

  stop_radar_writer_ =
      node_->CreateWriter<Ultra_Radar>(FLAGS_ultra_radar_topic);

  local_writer_ =
      node_->CreateWriter<localization::LocalizationEstimate>(FLAGS_localization_topic);

  device_thread_ = std::thread(&RocketRadarComponent::poll, this);
  device_thread_.detach();
   
  track_process_ = new RadarTrackProcess();
  track_process_->initRadarTrack(install_pos_x_,install_pos_y_,install_agle_);
  return true;
}



bool RocketRadarComponent::poll() {
  uint32_t radar_ip;

  inet_pton(AF_INET, radar_ip_, &radar_ip);

  ConsoleLogger logger;
  Connection* con = make_good_connection(radar_ip, logger);

  if (!con) {
    printf("unable to connect\n");
    return false;
  }

  RDC_ThresholdControl thresh;
  thresh.defaults();
  thresh.apply_preset(LOW_SENSITIVITY);
  con->configure_detection_thresholds(&thresh);

  UhdpCaptureControl cap_ctrl;
  memset(&cap_ctrl, 0, sizeof(cap_ctrl));
  cap_ctrl.capture_mode = DL_NORMAL;
  cap_ctrl.enable_mask = DL_DET;

  RDC_ScanControl scanctrl;
  scanctrl.defaults();

  scanctrl.scan_loop_count = 1;
  RDC_ScanDescriptor desc;

  if (radar_id_ == "top")
  {
    if (con->is_sabine_b())
    {
      desc.set_defaults(VP105);
      desc.antenna_config_id =
          con->get_basic_antenna_config_id(BAC_TRADITIONAL_1D);
    }
  }
  else
  {
    if (con->is_sabine_b())
    {
      scanctrl.frame_interval_us = 50 * 1000;
      desc.set_defaults(VP101);
      desc.antenna_config_id =
          con->get_basic_antenna_config_id(BAC_TRADITIONAL_1D);
    }
  }
  Scanning* s = con->setup_scanning(scanctrl, &desc, cap_ctrl);
  
  // scanctrl.scan_loop_count = 2;
  // RDC_ScanDescriptor desc[2];

  // if (con->is_sabine_b()) {
  //   desc[0].set_defaults(VP105);
  //   desc[0].antenna_config_id =
  //       con->get_basic_antenna_config_id(BAC_TRADITIONAL_1D);
  //   desc[1].set_defaults(VP105);
  //   desc[1].antenna_config_id =
  //       con->get_basic_antenna_config_id(BAC_TRADITIONAL_2D);
  // }
  // Scanning* s = con->setup_scanning(scanctrl, desc, cap_ctrl);

  if (!s) {
    con->close_and_release();
    printf("Unable to start scan(s)\n");
    return false;
  }
  
  // std::string filename("result");
  // filename =filename + "_" + radar_id_ + "_track.txt";
  // std::ofstream outfile;
  // outfile.open(filename, std::ios::out);
  // outfile << std::setprecision(3);

  std::string filename1("result");
  filename1 = filename1 + "_" + radar_id_ + "_meas.txt";
  std::ofstream outfile2;
  outfile2.open(filename1, std::ios::out);
  outfile2 << std::setprecision(3);

  ScanObject* scan = s->wait_for_scan();
  while (!apollo::cyber::IsShutdown()) {
    frame_count++; 
    double timestamp = apollo::common::time::Clock::NowInSeconds();
    double delta = timestamp - track_process_->timestamp_;
    if(delta > 0.2){
      delta = 0.1;
    }
    track_process_->delta_time_ = delta;
    track_process_->timestamp_ = timestamp;
    track_process_->MeasLists_.clear();
    
    fflush(NULL);
    radarMsg_.clear_contiobs();
    Detections* dets = scan->get_detections();
    if (dets) {
      uint32_t count = dets->get_count();
      // printf("measure object num:   %d\n",count);
      for (uint32_t i = 0; i < count; i++) {
        const DetectionData& d = dets->get_detection(i);
        measure obj;
        obj.range = d.range;
        obj.azimuth = positive_ * d.azimuth;
        obj.elevation = d.elevation;
        obj.speed = d.doppler;
        obj.mag = d.magnitude;
        obj.snr = d.snr;
        obj.flag = d.flags;
        obj.pos_x = d.pos_x;
        obj.pos_y = positive_ * d.pos_y;

        obj.pos_z = positive_ * d.pos_z;
        obj.match_status = false;
        obj.clutter = false;
        obj.rcs = d.rcs;
        track_process_->MeasLists_.push_back(obj);
        // printf("pos_z:    %.3f\n", d.pos_z);
        if (radar_id_ == "top") {
//	   outfile2 << i << std::setw(15) << d.range << std::setw(15)
//                 << d.azimuth << std::setw(15) << d.doppler << std::setw(15)
//                 << d.elevation << std::setw(15) << d.magnitude
//                 << std::setw(15) << d.snr << std::setw(15) << d.pos_x
//                 << std::setw(15) << d.pos_y << std::setw(15) << d.pos_z
//                 << std::setw(15) << d.flags << std::endl;
 
            drivers::ContiRadarObs* obs = radarMsg_.add_contiobs();
            obs->set_range(d.range);
            obs->set_azimuth(d.azimuth);
            obs->set_elevation(d.elevation);
            obs->set_doppler(d.doppler);
            obs->set_magnitude(d.magnitude);
            obs->set_snr(d.snr);
            obs->set_uhn_rcs(d.rcs);
            obs->set_pos_x(d.pos_x);
            obs->set_pos_y(d.pos_y);
            obs->set_pos_z(d.pos_z);
            obs->set_fags(d.flags);
   
        }
      }
    }
    track_process_->radarTrackMainProcess(trackObjects_);

    float pos_x = 0.0f;
    float pos_y = 0.0f;
    float pos_z = 0.0f;
    float vel_x = 0.0f;
    float vel_y = 0.0f;
    

    // printf("track object num:   %d\n",trackObjects_.size());
    for (uint16_t i = 0; i < trackObjects_.size(); ++i) {
      pos_x = trackObjects_[i].pos_x * cos(install_agle_) -
              trackObjects_[i].pos_y * sin(install_agle_) + install_pos_x_;
      pos_y = trackObjects_[i].pos_x * sin(install_agle_) +
              trackObjects_[i].pos_y * cos(install_agle_) + install_pos_y_;
      pos_z = install_pos_z_ + trackObjects_[i].pos_z;

      vel_x = trackObjects_[i].vel_x * cos(install_agle_) -
              trackObjects_[i].vel_y * sin(install_agle_) ;
      vel_y = trackObjects_[i].vel_x * sin(install_agle_) +
              trackObjects_[i].vel_y * cos(install_agle_);

      if (radar_id_ != "top") {
        drivers::ContiRadarObs* obs = radarMsg_.add_contiobs();
        obs->set_obstacle_id(trackObjects_[i].id);
        obs->set_pos_x(pos_x);
        obs->set_pos_y(pos_y);
        obs->set_pos_z(pos_z);
        obs->set_longitude_dist(pos_x);
        obs->set_lateral_dist(pos_y);
        obs->set_longitude_vel(vel_x);
        obs->set_lateral_vel(vel_y);
        obs->set_longitude_accel(trackObjects_[i].acc_x);
        obs->set_lateral_accel(trackObjects_[i].acc_y);
        obs->set_fags(false);
      }

      // if (frame_count > 10000 || trackObjects_.size() > 300) {
      //   outfile << i << std::setw(15) << trackObjects_[i].id << std::setw(15)
      //           << pos_x << std::setw(15) << pos_y << std::setw(15) << pos_z
      //           << std::setw(15) << vel_x << std::setw(15) << vel_y
      //           << std::setw(15) << trackObjects_[i].acc_x << std::setw(15)
      //           << trackObjects_[i].acc_y << std::setw(15)
      //           << trackObjects_[i].lifetime << std::setw(15)
      //           << trackObjects_[i].count << std::endl;
      // }
    }

    common::util::FillHeader(node_->Name(), &radarMsg_);
    uhnder_radar_writer_->Write(radarMsg_);
    if (radar_id_ != "top") {
      tmpradarMsg_.mutable_radarobs()->CopyFrom(radarMsg_);
      stop_radar_writer_->Write(tmpradarMsg_);
    }

    // if(FLAGS_enable_radar_write_localization_msg) { //add by shzhw
    //   if(FLAGS_use_sim_localization) {
    //     AERROR << "localization message is already writeed by simulation module.";
    //   } else{
    //     localization::LocalizationEstimate localization;
    //     localization.mutable_header()->set_timestamp_sec(timestamp);

    //     localization.mutable_pose()->mutable_position()->set_x(0.0);
    //     localization.mutable_pose()->mutable_position()->set_y(0.0);
    //     localization.mutable_pose()->mutable_position()->set_z(0.0);

    //     localization.mutable_pose()->mutable_linear_velocity()->set_x(0.0);
    //     localization.mutable_pose()->mutable_linear_velocity()->set_y(0.0);
    //     localization.mutable_pose()->mutable_linear_velocity()->set_z(0.0);

    //     localization.mutable_pose()->mutable_linear_acceleration()->set_x(0.0);
    //     localization.mutable_pose()->mutable_linear_acceleration()->set_y(0.0);
    //     localization.mutable_pose()->mutable_linear_acceleration()->set_z(0.0);
    //     /**/
    //     localization.mutable_pose()->mutable_angular_velocity()->set_x(0.0);
    //     localization.mutable_pose()->mutable_angular_velocity()->set_y(0.0);
    //     localization.mutable_pose()->mutable_angular_velocity()->set_z(0.0);

    //     localization.mutable_pose()->mutable_linear_acceleration_vrf()->set_x(0.0);
    //     localization.mutable_pose()->mutable_linear_acceleration_vrf()->set_y(0.0);
    //     localization.mutable_pose()->mutable_linear_acceleration_vrf()->set_z(0.0);

    //     localization.mutable_pose()->mutable_angular_velocity_vrf()->set_x(0.0);
    //     localization.mutable_pose()->mutable_angular_velocity_vrf()->set_y(0.0);
    //     localization.mutable_pose()->mutable_angular_velocity_vrf()->set_z(0.0);

    //     localization.mutable_pose()->mutable_orientation()->set_qw(0.0);
    //     localization.mutable_pose()->mutable_orientation()->set_qx(0.0);
    //     localization.mutable_pose()->mutable_orientation()->set_qy(0.0);
    //     localization.mutable_pose()->mutable_orientation()->set_qz(0.0);

    //     localization.mutable_pose()->set_heading(0.0);
        
    //     local_writer_->Write(localization);
    //   }
    // }
    //----------------------------------------------------------

    scan->release();
    scan = s->wait_for_scan();
  }
  outfile2.close();
  // outfile.close();
  return true;
}

}  // namespace uhnder_radar
}  // namespace drivers
}  // namespace apollo
