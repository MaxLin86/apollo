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
  install_angle_ = (uhnder_radar_config_.install_angle()) / 180.0 * PI;
  install_pos_x_ = uhnder_radar_config_.install_pos_x();
  install_pos_y_ = uhnder_radar_config_.install_pos_y();
  install_pos_z_ = uhnder_radar_config_.install_pos_z();

  radarMsg_.Clear();
  rawRadarMsg_.Clear();

  positive_ = uhnder_radar_config_.positive();

  uhnder_radar_writer_ =
      node_->CreateWriter<ContiRadar>(uhnder_radar_config_.radar_channel());

  stop_radar_writer_ =
      node_->CreateWriter<Ultra_Radar>(FLAGS_ultra_radar_topic);

  if (!uhnder_radar_config_.need_track()) {
    if (radar_id_ == "top_middle") {
      radarMsg_.set_radar_id(5);
    } else if (radar_id_ == "top_left") {
      radarMsg_.set_radar_id(-6);
    } else if (radar_id_ == "top_right") {
      radarMsg_.set_radar_id(6);
    }
    device_thread_ = std::thread(&RocketRadarComponent::poll, this);
    device_thread_.detach();
  } else {
    if (radar_id_ == "front_middle")   {
      radarMsg_.set_radar_id(0);
      rawRadarMsg_.set_radar_id(0);
    }  else if (radar_id_ == "front_left"){
      radarMsg_.set_radar_id(1);
      rawRadarMsg_.set_radar_id(1);
    } else if (radar_id_ == "front_right"){
      radarMsg_.set_radar_id(-1);
      rawRadarMsg_.set_radar_id(-1);
    } else if (radar_id_ == "front_left_corner"){
      radarMsg_.set_radar_id(2);
      rawRadarMsg_.set_radar_id(2);
    }  else if (radar_id_ == "front_right_corner") {
      radarMsg_.set_radar_id(-2);
      rawRadarMsg_.set_radar_id(-2);
    }else if (radar_id_ == "rear_left_corner") {
      radarMsg_.set_radar_id(3);
      rawRadarMsg_.set_radar_id(3);
    }else if (radar_id_ == "rear_right_corner") {
      radarMsg_.set_radar_id(-3);
      rawRadarMsg_.set_radar_id(-3);
    }
    
    track_process_ = new RadarTrackProcess();
    track_process_->initRadarTrack(install_pos_x_, install_pos_y_,
                                   install_angle_);
    detection_radar_writer_ =
        node_->CreateWriter<ContiRadar>(uhnder_radar_config_.detect_channel());

    device_thread_ = std::thread(&RocketRadarComponent::pullData, this);
    device_thread_.detach();

    /*cyber::ReaderConfig chassis_reader_config;
    chassis_reader_config.channel_name = FLAGS_chassis_topic;
    chassis_reader_config.pending_queue_size = 5;

    chassis_reader_ = node_->CreateReader<apollo::canbus::Chassis>(
        chassis_reader_config, nullptr);
    CHECK(chassis_reader_ != nullptr);*/
  }

  return true;
}



bool RocketRadarComponent::poll() {

  uint32_t radar_ip;

  inet_pton(AF_INET, radar_ip_, &radar_ip);

  ConsoleLogger logger;
  con = make_good_connection(radar_ip, logger);

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
  scanctrl.frame_interval_us = 50 * 1000;
  RDC_ScanDescriptor desc;
  desc.set_defaults(VP105);
  desc.antenna_config_id = con->get_basic_antenna_config_id(BAC_TRADITIONAL_1D);
  s = con->setup_scanning(scanctrl, &desc, cap_ctrl);

  // if (!s) {
  //   con->close_and_release();
  //   printf("Unable to start scan(s)\n");
  //   return false;
  // }
  
  while(!s){
    con->close_and_release();
    con = make_good_connection(radar_ip, logger);
    con->configure_detection_thresholds(&thresh);
    s = con->setup_scanning(scanctrl, &desc, cap_ctrl);
  }
  
  ScanObject* scan = s->wait_for_scan();
  while (!apollo::cyber::IsShutdown() ) {
    fflush(NULL);
    radarMsg_.clear_contiobs();
    Detections* dets = scan->get_detections();
    if (dets) {
      uint32_t count = dets->get_count();
      for (uint32_t i = 0; i < count; i++) {
        const DetectionData& d = dets->get_detection(i);
        drivers::ContiRadarObs* obs = radarMsg_.add_contiobs();
        obs->set_range(d.range);
        obs->set_azimuth(d.azimuth);
        obs->set_elevation(d.elevation);
        obs->set_doppler(d.doppler);
        obs->set_magnitude(d.magnitude);
        obs->set_snr(d.snr);
        obs->set_uhn_rcs(d.rcs);
        obs->set_pos_x(d.pos_x);
        obs->set_pos_y(d.pos_y * positive_);
        obs->set_pos_z(d.pos_z);
      }
    }

    common::util::FillHeader(node_->Name(), &radarMsg_);
    uhnder_radar_writer_->Write(radarMsg_);

    scan->release();
    scan = s->wait_for_scan();
  }
  
  return true;
}

bool RocketRadarComponent::pullData(){
  
  uint32_t radar_ip;
  inet_pton(AF_INET, radar_ip_, &radar_ip);

  ConsoleLogger logger;
  con = make_good_connection(radar_ip, logger);

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

  /////　１Ｄ　２Ｄ切换
  scanctrl.scan_loop_count = 2;
  scanctrl.frame_interval_us = 100 * 1000;

  RDC_ScanDescriptor desc[2];
  desc[0].set_defaults(VP114);
  desc[0].antenna_config_id =
      con->get_basic_antenna_config_id(BAC_TRADITIONAL_1D);
  desc[1].set_defaults(VP114);
  desc[1].antenna_config_id =
      con->get_basic_antenna_config_id(BAC_TRADITIONAL_2D);

  s = con->setup_scanning(scanctrl, desc, cap_ctrl);
  
  // if (!s) {
  //   con->close_and_release();
  //   printf("Unable to start scan(s)\n");
  //   return false;
  // }
  
  while(!s){
    con->close_and_release();
    con = make_good_connection(radar_ip, logger);
    con->configure_detection_thresholds(&thresh);
    s = con->setup_scanning(scanctrl, desc, cap_ctrl);
  }

  ScanObject* scan = s->wait_for_scan();
  
  while (!apollo::cyber::IsShutdown() ) {
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

    float pos_x = 0.0f;
    float pos_y = 0.0f;
    float pos_z = 0.0f;
    float vel_x = 0.0f;
    float vel_y = 0.0f;

    for (uint16_t i = 0; i < trackObjects_.size(); ++i) {
      pos_x = trackObjects_[i].pos_x * cos(install_angle_) -
              trackObjects_[i].pos_y * sin(install_angle_) + install_pos_x_;
      pos_y = trackObjects_[i].pos_x * sin(install_angle_) +
              trackObjects_[i].pos_y * cos(install_angle_) + install_pos_y_; 

      if(std::fabs(trackObjects_[i].pos_z) < 0.0001){
        pos_z =  0.0;
      }else{
        pos_z = trackObjects_[i].pos_z + install_pos_z_ ;
      }

      vel_x = trackObjects_[i].vel_x * cos(install_angle_) -
              trackObjects_[i].vel_y * sin(install_angle_);
      vel_y = trackObjects_[i].vel_x * sin(install_angle_) +
              trackObjects_[i].vel_y * cos(install_angle_);


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

    /*chassis_reader_->Observe();
    auto msg = chassis_reader_->GetLatestObserved();

    float cur_speed = 0.0;
    if (msg) {
      cur_speed = msg->speed_mps();
      if (msg->gear_location() == apollo::canbus::Chassis::GEAR_REVERSE) {
        cur_speed = -1.0 * cur_speed;
      }
    }
    vec3f_t vehicle_vel;
    vehicle_vel.x = cur_speed;
    vehicle_vel.y = -0;
    vehicle_vel.z = 0;
    vec3f_t radar_vel;
    radar_vel.x =
        vehicle_vel.x * cos(install_angle_) - vehicle_vel.y * sin(install_angle_);
    radar_vel.y =
        vehicle_vel.x * sin(install_angle_) + vehicle_vel.y * cos(install_angle_);
    radar_vel.z = vehicle_vel.z;

    UhdpTelemetryData telem;
    memset(&telem, 0, sizeof(telem));
    telem.ego_linear_velocity =
        radar_vel *
        vec3f_t(-1);             // meters per second of world relative to radar
    telem.telemetry_age_us = 0;  // age of telemetry data, in microseconds
    con->configure_radar_telemetry(telem);*/

    scan->release();
    scan = s->wait_for_scan();
  }
  

  return true;
}


}  // namespace uhnder_radar
}  // namespace drivers
}  // namespace apollo
