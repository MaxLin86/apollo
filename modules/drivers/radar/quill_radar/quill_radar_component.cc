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
#include "modules/drivers/radar/quill_radar/quill_radar_component.h"

#include "modules/drivers/radar/quill_radar/driver/include/rra.h"
#include "modules/drivers/radar/quill_radar/driver/include/connection.h"
#include "modules/drivers/radar/quill_radar/driver/include/scanning.h"
#include "modules/drivers/radar/quill_radar/driver/include/scanobject.h"
#include "modules/drivers/radar/quill_radar/driver/include/detections.h"
#include "modules/drivers/radar/quill_radar/driver/include/pointcloud.h"
#include "modules/drivers/radar/quill_radar/driver/system-radar-software/engine/scp-src/eng-api/uhmath.h"
#include "modules/drivers/radar/quill_radar/driver/include/userlogagents.h"

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

bool QuillRadarComponent::Init() {
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
  rawRadarMsg_.Clear();
  if (radar_id_ == "front_middle")
  {
    radarMsg_.set_radar_id(0);
    rawRadarMsg_.set_radar_id(0);
  } 
  //else if (radar_id_ == "front_left")
  //{
  //  radarMsg_.set_radar_id(1);
  //  rawRadarMsg_.set_radar_id(1);
  //} else if (radar_id_ == "front_right")
  //{
  //  radarMsg_.set_radar_id(-1);
  //  rawRadarMsg_.set_radar_id(-1);
  //} else if (radar_id_ == "front_left_corner")
  //{
  //  radarMsg_.set_radar_id(2);
  //  rawRadarMsg_.set_radar_id(2);
  //}  else if (radar_id_ == "front_right_corner")
  //{
  //  radarMsg_.set_radar_id(-2);
  //  rawRadarMsg_.set_radar_id(-2);
  //}else if (radar_id_ == "rear_left_corner")
  //{
  //  radarMsg_.set_radar_id(3);
  //  rawRadarMsg_.set_radar_id(3);
  //}else if (radar_id_ == "rear_right_corner")
  //{
  //  radarMsg_.set_radar_id(-3);
  //  rawRadarMsg_.set_radar_id(-3);
  //}
  
  frame_count = 0;
  
  positive_ = uhnder_radar_config_.positive();

  uhnder_radar_writer_ =
      node_->CreateWriter<ContiRadar>(uhnder_radar_config_.radar_channel());

  stop_radar_writer_ =
      node_->CreateWriter<Ultra_Radar>(FLAGS_ultra_radar_topic);

  local_writer_ = node_->CreateWriter<localization::LocalizationEstimate>(
      FLAGS_localization_topic);

  detection_radar_writer_ =
      node_->CreateWriter<ContiRadar>(uhnder_radar_config_.detect_channel());

  device_thread_ = std::thread(&QuillRadarComponent::poll, this);
  device_thread_.detach();

  track_process_ = new RadarTrackProcess();
  track_process_->initRadarTrack(install_pos_x_, install_pos_y_, install_agle_);

  return true;
}



bool QuillRadarComponent::poll() {
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
  thresh.apply_preset(HIGH_SENSITIVITY);
  con->configure_detection_thresholds(&thresh);

  UhdpCaptureControl cap_ctrl;
  memset(&cap_ctrl, 0, sizeof(cap_ctrl));
  cap_ctrl.capture_mode = DL_NORMAL;
  cap_ctrl.enable_mask = DL_DET;

  RDC_ScanControl scanctrl;
  scanctrl.defaults();
  Scanning* s = NULL;

    ////// 1D 模式
  // RDC_ScanDescriptor desc;
  // scanctrl.scan_loop_count = 1;
  // desc.set_defaults(VP105);
  // desc.antenna_config_id = con->get_basic_antenna_config_id(BAC_TRADITIONAL_1D);
  // s = con->setup_scanning(scanctrl, &desc, cap_ctrl);

    
  ////// ２D 模式
  // RDC_ScanDescriptor desc;
  // scanctrl.scan_loop_count = 1;
  // desc.set_defaults(VP105);
  // desc.antenna_config_id = con->get_basic_antenna_config_id(BAC_TRADITIONAL_２D);
  // s = con->setup_scanning(scanctrl, &desc, cap_ctrl);

  //////　１Ｄ　２Ｄ切换
  scanctrl.scan_loop_count = 2;
  RDC_ScanDescriptor desc[2];
  desc[0].set_defaults(VP105);
  desc[0].antenna_config_id =
      con->get_basic_antenna_config_id(BAC_TRADITIONAL_1D);
  desc[1].set_defaults(VP105);
  desc[1].antenna_config_id =
      con->get_basic_antenna_config_id(BAC_TRADITIONAL_2D);

  s = con->setup_scanning(scanctrl, desc, cap_ctrl);
  
  if (!s) {
    con->close_and_release();
    printf("Unable to start scan(s)\n");
    return false;
  }

  ScanObject* scan = s->wait_for_scan();
  
  //track_process_->Flag_2D_ = false;

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
    
    radarMsg_.clear_contiobs();
    rawRadarMsg_.clear_contiobs();

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
        
        // // 发送原始数据，注释掉２３５～２４６
        drivers::ContiRadarObs* obs = rawRadarMsg_.add_contiobs();
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
      }
    }
    track_process_->radarTrackMainProcess(trackObjects_);
//    track_process_->Flag_2D_  = !track_process_->Flag_2D_ ;

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

      if(std::fabs(trackObjects_[i].pos_z) < 0.0001){
        pos_z =  0.0;
      }else{
        pos_z = trackObjects_[i].pos_z + install_pos_z_ ;
      }

      vel_x = trackObjects_[i].vel_x * cos(install_agle_) -
              trackObjects_[i].vel_y * sin(install_agle_);
      vel_y = trackObjects_[i].vel_x * sin(install_agle_) +
              trackObjects_[i].vel_y * cos(install_agle_);


      // // 发送跟踪数据，注释掉１９７～２０８
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
      obs->set_snr(trackObjects_[i].snr);
      obs->set_lifetime(trackObjects_[i].lifetime);
    }

    common::util::FillHeader(node_->Name(), &radarMsg_);
    uhnder_radar_writer_->Write(radarMsg_);

    tmpradarMsg_.mutable_radarobs()->CopyFrom(radarMsg_);
    stop_radar_writer_->Write(tmpradarMsg_);

    common::util::FillHeader(node_->Name(), &rawRadarMsg_);
    detection_radar_writer_->Write(rawRadarMsg_);

    scan->release();
    scan = s->wait_for_scan();
  }

  return true;
}

}  // namespace uhnder_radar
}  // namespace drivers
}  // namespace apollo
