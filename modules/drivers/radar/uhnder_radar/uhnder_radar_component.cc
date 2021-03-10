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
#include "modules/drivers/radar/uhnder_radar/uhnder_radar_component.h"

#include "modules/drivers/radar/uhnder_radar/driver/include/sra.h"
#include "modules/drivers/radar/uhnder_radar/driver/include/connection.h"
#include "modules/drivers/radar/uhnder_radar/driver/include/scanning.h"
#include "modules/drivers/radar/uhnder_radar/driver/include/scanobject.h"
#include "modules/drivers/radar/uhnder_radar/driver/include/detections.h"
#include "modules/drivers/radar/uhnder_radar/driver/include/pointcloud.h"
#include "modules/drivers/radar/uhnder_radar/driver/system-radar-software/engine/scp-src/eng-api/uhmath.h"
#include "modules/drivers/radar/uhnder_radar/driver/include/userlogagents.h"

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

bool UhnderRadarComponent::Init() {
  AINFO << "UhnderRadarComponent init";
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
  if (radar_id_ == "right_corner")
  {
    radarMsg_.set_radar_id(-2);
  }
  else if (radar_id_ == "left_corner")
  {
    radarMsg_.set_radar_id(2);
  }


  ThresholdPreset thre = uhnder_radar_config_.threshold();
  if (thre == LOW) {
    threshold_ = LOW_SENSITIVITY;
  } else if (thre == HIGH) {
    threshold_ = HIGH_SENSITIVITY;
  } else {
    threshold_ = MID_SENSITIVITY;
  }

  RDC_Scan_Type scanType = uhnder_radar_config_.scan_type();
  selectScanType(scanType);

  scan_delay_offset_us_ = uhnder_radar_config_.scan_delay_offset_us();
  
  positive_ = uhnder_radar_config_.positive();

  uhnder_radar_writer_ =
      node_->CreateWriter<ContiRadar>(uhnder_radar_config_.radar_channel());

  stop_radar_writer_ =
        node_->CreateWriter<Ultra_Radar>(FLAGS_ultra_radar_topic);

  local_writer_ =
      node_->CreateWriter<localization::LocalizationEstimate>(FLAGS_localization_topic);

  device_thread_ = std::thread(&UhnderRadarComponent::poll, this);
  device_thread_.detach();
   
  track_process_ = new RadarTrackProcess();
  track_process_->initRadarTrack(install_pos_x_,install_pos_y_,install_agle_);
  return true;
}

void UhnderRadarComponent::selectScanType(RDC_Scan_Type p){
  switch (p) {
    case Type_VP1a:
      scan_type_ = VP1a;
      break;
    case Type_VP1as:
      scan_type_ = VP1as;
      break;
    case Type_VP1b:
      scan_type_ = VP1b;
      break;
    case Type_VP1bb:
      scan_type_ = VP1bb;
      break;
    case Type_VP1c:
      scan_type_ = VP1c;
      break;
    case Type_VP4:
      scan_type_ = VP4;
      break;
    case Type_VP8:
      scan_type_ = VP8;
      break;
    case Type_VP9:
      scan_type_ = VP9;
      break;
    case Type_VP9f:
      scan_type_ = VP9f;
      break;
    case Type_VP11:
      scan_type_ = VP11;
      break;
    case Type_VP12:
      scan_type_ = VP12;
      break;
    case Type_VP13:
      scan_type_ = VP13;
      break;
    case Type_VP14:
      scan_type_ = VP14;
      break;
    case Type_VP14f:
      scan_type_ = VP14f;
      break;
    case Type_VP14ff:
      scan_type_ = VP14ff;
      break;
    case Type_VP15f:
      scan_type_ = VP15f;
      break;
    case Type_VP15m:
      scan_type_ = VP15m;
      break;
    case Type_VP15s:
      scan_type_ = VP15s;
      break;
    case Type_VP16:
      scan_type_ = VP16;
      break;
    case Type_CP1a:
      scan_type_ = CP1a;
      break;
    case Type_CP1as:
      scan_type_ = CP1as;
      break;
    case Type_CP1b:
      scan_type_ = CP1b;
      break;
    case Type_CP1bb:
      scan_type_ = CP1bb;
      break;
    case Type_CP1c:
      scan_type_ = CP1c;
      break;
    case Type_CP2:
      scan_type_ = CP2;
      break;
    case Type_NUM_PRESETS:
      break;
    default:
      break;
  }
}

bool UhnderRadarComponent::poll() {
  uint32_t radar_ip, local_ip;

  inet_pton(AF_INET, radar_ip_, &radar_ip);
  inet_pton(AF_INET, "192.168.95.146", &local_ip);

  ConsoleLogger logger;
  Connection* con = make_connection(radar_ip, local_ip, 3, logger, false);

  if (!con) {
    printf("Unable to connect\n");
    return false;
  }

  ///////scan starting time base for this radar (offset from global time base)
  timeval base_scan_time;
  gettimeofday(&base_scan_time, NULL);
  timeval t = base_scan_time;
  t.tv_usec += scan_delay_offset_us_;
  con->synchronize_scan_interval(t);

  UhdpThresholdControl thresh;
  thresh.defaults();
  thresh.apply_preset(threshold_);
  con->configure_detection_thresholds(thresh);

  UhdpCaptureControl cap_ctrl;
  memset(&cap_ctrl, 0, sizeof(cap_ctrl));
  cap_ctrl.capture_mode = DL_NORMAL;
  cap_ctrl.enable_mask = DL_DET;

  RDC_ScanControl scanctrl;
  scanctrl.defaults();

  //scanctrl.scan_loop_count = 2;
  //RDC_ScanDescriptor desc[2];
  //desc[0].set_defaults(scan_type_);
  //desc[0].antenna_config_id =
  //    con->get_basic_antenna_config_id(BAC_TRADITIONAL_1D);
  //desc[1].set_defaults(scan_type_);
  //desc[1].antenna_config_id =
  //    con->get_basic_antenna_config_id(BAC_TRADITIONAL_2D);
  //Scanning* s = con->setup_scanning(scanctrl, &desc[0], cap_ctrl);

   scanctrl.scan_loop_count = 1;
   RDC_ScanDescriptor desc;
   desc.set_defaults(scan_type_);
   desc.antenna_config_id = con->get_basic_antenna_config_id(BAC_TRADITIONAL_1D);
   Scanning* s = con->setup_scanning(scanctrl, &desc, cap_ctrl);

  if (!s) {
    con->close_and_release();
    printf("Unable to start scan(s)\n");
    return false;
  }
  
  ScanObject* scan = s->wait_for_scan();

  while (!apollo::cyber::IsShutdown()) { 
    double timestamp = apollo::common::time::Clock::NowInSeconds();
    double delta = timestamp - track_process_->timestamp_;
    if(delta > 0.2){
      delta = 0.04;
    }
    track_process_->delta_time_ = delta;
    track_process_->timestamp_ = timestamp;
    track_process_->MeasLists_.clear();
    
    fflush(NULL);

    Detections* dets = scan->get_detections();
    if (dets) {
      uint32_t count = dets->get_count();
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

        // outfile2 << i << std::setw(15) << d.range << std::setw(15) << d.azimuth
        //          << std::setw(15) << d.doppler << std::setw(15) << d.elevation
        //          << std::setw(15) << d.magnitude << std::setw(15) << d.snr
        //          << std::setw(15) << d.pos_x << std::setw(15) << d.pos_y
        //          << std::setw(15) << d.pos_z << std::setw(15) << d.flags
        //          << std::endl;
      }
    }
    track_process_->radarTrackMainProcess(trackObjects_);

    float pos_x = 0.0f;
    float pos_y = 0.0f;
    float pos_z = 0.0f;
    float vel_x = 0.0f;
    float vel_y = 0.0f;
    
    radarMsg_.clear_contiobs();
    for (uint16_t i = 0; i < trackObjects_.size(); ++i) {
      pos_x = trackObjects_[i].pos_x * cos(install_agle_) -
              trackObjects_[i].pos_y * sin(install_agle_) + install_pos_x_;
      pos_y = trackObjects_[i].pos_x * sin(install_agle_) +
              trackObjects_[i].pos_y * cos(install_agle_) + install_pos_y_;
      pos_z = 0.0; // 

      vel_x = trackObjects_[i].vel_x * cos(install_agle_) -
              trackObjects_[i].vel_y * sin(install_agle_) ;
      vel_y = trackObjects_[i].vel_x * sin(install_agle_) +
              trackObjects_[i].vel_y * cos(install_agle_); 
      
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
      obs->set_flags(false);
      obs->set_counters(1);
      obs->set_lifetime(trackObjects_[i].lifetime);
    }
    common::util::FillHeader(node_->Name(), &radarMsg_);
    tmpradarMsg_.mutable_radarobs()->CopyFrom(radarMsg_);
    stop_radar_writer_->Write(tmpradarMsg_);
    uhnder_radar_writer_->Write(radarMsg_);

    if(FLAGS_enable_radar_write_localization_msg) { //add by shzhw
      if(FLAGS_use_sim_localization) {
        AERROR << "localization message is already writeed by simulation module.";
      } else{
        localization::LocalizationEstimate localization;
        localization.mutable_header()->set_timestamp_sec(timestamp);

        localization.mutable_pose()->mutable_position()->set_x(0.0);
        localization.mutable_pose()->mutable_position()->set_y(0.0);
        localization.mutable_pose()->mutable_position()->set_z(0.0);

        localization.mutable_pose()->mutable_linear_velocity()->set_x(0.0);
        localization.mutable_pose()->mutable_linear_velocity()->set_y(0.0);
        localization.mutable_pose()->mutable_linear_velocity()->set_z(0.0);

        localization.mutable_pose()->mutable_linear_acceleration()->set_x(0.0);
        localization.mutable_pose()->mutable_linear_acceleration()->set_y(0.0);
        localization.mutable_pose()->mutable_linear_acceleration()->set_z(0.0);
        /**/
        localization.mutable_pose()->mutable_angular_velocity()->set_x(0.0);
        localization.mutable_pose()->mutable_angular_velocity()->set_y(0.0);
        localization.mutable_pose()->mutable_angular_velocity()->set_z(0.0);

        localization.mutable_pose()->mutable_linear_acceleration_vrf()->set_x(0.0);
        localization.mutable_pose()->mutable_linear_acceleration_vrf()->set_y(0.0);
        localization.mutable_pose()->mutable_linear_acceleration_vrf()->set_z(0.0);

        localization.mutable_pose()->mutable_angular_velocity_vrf()->set_x(0.0);
        localization.mutable_pose()->mutable_angular_velocity_vrf()->set_y(0.0);
        localization.mutable_pose()->mutable_angular_velocity_vrf()->set_z(0.0);

        localization.mutable_pose()->mutable_orientation()->set_qw(0.0);
        localization.mutable_pose()->mutable_orientation()->set_qx(0.0);
        localization.mutable_pose()->mutable_orientation()->set_qy(0.0);
        localization.mutable_pose()->mutable_orientation()->set_qz(0.0);

        localization.mutable_pose()->set_heading(0.0);
        
        local_writer_->Write(localization);
      }
    }
    //----------------------------------------------------------

    scan->release();
    scan = s->wait_for_scan();
  }
 
  return true;
}

}  // namespace uhnder_radar
}  // namespace drivers
}  // namespace apollo
