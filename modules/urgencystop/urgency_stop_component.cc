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
#include "modules/urgencystop/urgency_stop_component.h"

#include <utility>
#include "modules/common/adapters/adapter_gflags.h"
#include <math.h>
#include "modules/common/util/message_util.h"
#include "modules/common/configs/vehicle_config_helper.h"

// #include "modules/drivers/canbus_api.h"
#include <fstream>  //æä»¶æµåºå½æ°
#include <iomanip>
#include <iostream>

namespace apollo {
namespace urgency {

#define LEFT_CORNER_RADAR 1
#define LEFT_RADAR 2
#define FRONT_RADAR 4
#define RIGHT_RADAR 8
#define RIGHT_CORNER_RADAR 16
#define TOP_RADAR 32

bool UrgencyStop::Init() {
  if (!GetProtoConfig(&urgenStopConf_)) {
    return false;
  }

  origin_dist_ = urgenStopConf_.dist_to_origin();
  safety_zone_left_ = urgenStopConf_.safety_zone_left();
  safety_zone_right_ = urgenStopConf_.safety_zone_right();
  
  check_frame_ = urgenStopConf_.check_frame();
  match_size_ = urgenStopConf_.match_size();
  check_count_ = 0;
  brakeFlag_ = false;

  radarBrakeFlag_ = false;
  ultraBrakeFlag_ = false;

  stop_count_ = 0;
  begin_stop_dis_ = 0.0;

  fence_flag_ = false;
  steer_flag_ = false;
  CorD_ = false;
  fence_ratio_ = 0;
  fence_dist_ = 0;
  dis2fence_ = 0;

  control_guardian_writer_ =
      node_->CreateWriter<AEBCommand>(FLAGS_aeb_command_topic);
  ACHECK(control_guardian_writer_ != nullptr);

  radar_writer_ =
      node_->CreateWriter<drivers::ContiRadar>(FLAGS_fused_radar_topic);
  ACHECK(radar_writer_ != nullptr);

  cyber::ReaderConfig chassis_reader_config;
  chassis_reader_config.channel_name = FLAGS_chassis_topic;
  chassis_reader_config.pending_queue_size = 5;

  chassis_reader_ = node_->CreateReader<apollo::canbus::Chassis>(
      chassis_reader_config, nullptr);
  CHECK(chassis_reader_ != nullptr);

  cyber::ReaderConfig localreader_config;
  localreader_config.channel_name = FLAGS_localization_topic;
  localreader_config.pending_queue_size = 5;
  localization_reader_ =
      node_->CreateReader<apollo::localization::LocalizationEstimate>(
          localreader_config, nullptr);

  for (int i = 0; i < urgenStopConf_.veh_speed_size(); i++) {
    veh_speed_.push_back((urgenStopConf_.veh_speed(i) / 3.60));
    stop_dis_.push_back(urgenStopConf_.stop_dis(i));
  }

  steer_ratio_ =
      common::VehicleConfigHelper::GetConfig().vehicle_param().steer_ratio();
  wheel_base_ =
      common::VehicleConfigHelper::GetConfig().vehicle_param().wheel_base();
  max_steer_angle_ = common::VehicleConfigHelper::GetConfig()
                         .vehicle_param()
                         .max_steer_angle();
  return true;
}

bool UrgencyStop::Proc(const std::shared_ptr<Ultra_Radar>& ultraRadarMessage) {
  std::lock_guard<std::mutex> lock(mutex_);
  double timestamp = apollo::common::time::Clock::NowInSeconds();
  static double leftCornerTime = timestamp;
  static double leftRadarTime = timestamp;
  static double rightCornerTime = timestamp;
  static double rightRadarTime = timestamp;
  static double frontRadarTime = timestamp;
  static double topRadarTime = timestamp;
  static double ultrtime = timestamp;
  static bool missRadar = false;
  static uint16_t miss_pos = 0;

  bool missRadarFlag = false;
  uint16_t miss_radar_pos = 0;
  
  float miss_time = 1.0f;

  if (std::fabs(timestamp - frontRadarTime) > miss_time) {
    missRadarFlag = true;
    miss_radar_pos |= FRONT_RADAR;
  }

  if (std::fabs(timestamp - leftRadarTime) > miss_time) {
    missRadarFlag = true;
    miss_radar_pos |= LEFT_RADAR;
  }
  if (std::fabs(timestamp - leftCornerTime) > miss_time) {
    //        missRadarFlag = true;
    miss_radar_pos |= LEFT_CORNER_RADAR;
  }
  if (std::fabs(timestamp - rightRadarTime) > miss_time) {
    missRadarFlag = true;
    miss_radar_pos |= RIGHT_RADAR;
  }
  if (std::fabs(timestamp - rightCornerTime) > miss_time) {
    // missRadarFlag = true;
    miss_radar_pos |= RIGHT_CORNER_RADAR;
  }

  if (std::fabs(timestamp - topRadarTime) > miss_time) {
    // missRadarFlag = true;
    miss_radar_pos |= TOP_RADAR;
  }

  missRadar = missRadarFlag;
  miss_pos = miss_radar_pos;

  if (ultraRadarMessage->has_radarobs()) {
    auto radarMessage = ultraRadarMessage->radarobs();
    if (radarMessage.radar_id() == 1 ||
        radarMessage.header().module_name() == "radar_left") {
      messageCollection(radarMessage, &leftObs_);
      leftRadarTime = timestamp;
    } else if (radarMessage.radar_id() == 2 ||
               radarMessage.header().module_name() == "radar_left_corner") {
      messageCollection(radarMessage, &leftCornerObs_);
      leftCornerTime = timestamp;
    } else if (radarMessage.radar_id() == -1 ||
               radarMessage.header().module_name() == "radar_right") {
      messageCollection(radarMessage, &rightObs_);
      rightRadarTime = timestamp;
    } else if (radarMessage.radar_id() == -2 ||
               radarMessage.header().module_name() == "radar_right_corner") {
      messageCollection(radarMessage, &rightCornerObs_);
      rightCornerTime = timestamp;
    } else if (radarMessage.radar_id() == 5 ||
               radarMessage.header().module_name() == "radar_top") {
      topRadarTime = timestamp;
    } else if (radarMessage.radar_id() == 0 ||
               radarMessage.header().module_name() == "radar_front") {
      time_ = apollo::common::time::Clock::NowInSeconds();
      frontRadarTime = timestamp;
      // // fuse object list
      if (std::fabs(timestamp - leftRadarTime) < 0.1) {
        radarObjectFuse(leftObs_, radarMessage);
      }
      if (std::fabs(timestamp - leftCornerTime) < 0.1) {
        radarObjectFuse(leftCornerObs_, radarMessage);
      }
      if (std::fabs(timestamp - rightRadarTime) < 0.1) {
        radarObjectFuse(rightObs_, radarMessage);
      }
      if (std::fabs(timestamp - rightCornerTime) < 0.1) {
        radarObjectFuse(rightCornerObs_, radarMessage);
      }

      radarBrakeFlag_ = judgeStopRules(radarMessage);
      fence_flag_ = polyfitLine(radarMessage);
      
      outputFusedObj(radarMessage);
    }
  }

  if (ultraRadarMessage->has_ultrasonic_obs()) {
    auto ultraMessage = ultraRadarMessage->ultrasonic_obs();
    ultraBrakeFlag_ = ultrasonicJudge(ultraMessage);
    ultrtime = timestamp;
  } else {
    if(fabs(timestamp - ultrtime) > 1.0f) {
      ultraBrakeFlag_ = false;
    }
  }

  if ((ultraRadarMessage->has_radarobs() &&
       ultraRadarMessage->radarobs().radar_id() == 0) ||
      ultraRadarMessage->has_ultrasonic_obs() || missRadar) {
    AEBCommand tmpControlCommand;
    if (fence_flag_ || radarBrakeFlag_ || ultraBrakeFlag_ || missRadar) {
    
      tmpControlCommand.mutable_control_command()->set_brake(urgenStopConf_.normal_brake());
      if (radarBrakeFlag_) {
        tmpControlCommand.set_stop_px(stop_px_);
        tmpControlCommand.set_stop_py(stop_py_);
        tmpControlCommand.set_stop_pz(stop_pz_);
        tmpControlCommand.set_ttc(ttc_);
        tmpControlCommand.set_thw(thw_);
        tmpControlCommand.set_dy(dy_);
        tmpControlCommand.set_ratio(ratio_);
        if (stop_px_ > 10.0) {
       //   tmpControlCommand.mutable_control_command()->set_brake(60.0);
        }
        if (stop_px_ > 5.0 && stop_px_ < 10.0) {
       //   tmpControlCommand.mutable_control_command()->set_brake(100.0);
        }
      }
    } else {
      tmpControlCommand.mutable_control_command()->set_brake(0);
    }
    tmpControlCommand.set_miss_radar(miss_pos);
    chassis_reader_->Observe();
    auto msg = chassis_reader_->GetLatestObserved();
    float cur_steering_percentage = 0.0;
    float cur_speed = 0.0;
    if(msg){
      cur_steering_percentage = msg->steering_percentage();
      cur_speed = msg->speed_mps();
    }

    steer_flag_ = false; // true: correct steer angle

    if(fence_flag_) {
      float correct_steering_target;
      if(CorD_) {
        // C max: 10°(27%)
        if((cur_steering_percentage < 27.0f) && (cur_speed > 0.001f)) {
          steer_flag_ = true;
          correct_steering_target = cur_steering_percentage + urgenStopConf_.steering_pencentage_test();
          tmpControlCommand.mutable_control_command()->set_steering_target(correct_steering_target);
        } 
      } else {
        // D min: -10°(27%)
        if((cur_steering_percentage > -27.0) && (cur_speed > 0.001f)) {
          steer_flag_ = true;
          correct_steering_target = cur_steering_percentage - urgenStopConf_.steering_pencentage_test();
          tmpControlCommand.mutable_control_command()->set_steering_target(correct_steering_target);
        } 
      }
      tmpControlCommand.mutable_control_command()->set_brake(urgenStopConf_.fence_brake());
    }

    tmpControlCommand.set_steer_flag(steer_flag_);
    tmpControlCommand.set_fence_ratio(fence_ratio_);
    tmpControlCommand.set_fence_dist(fence_dist_);
    tmpControlCommand.set_dist2fence(dis2fence_);

    common::util::FillHeader(node_->Name(), &tmpControlCommand);
    control_guardian_writer_->Write(tmpControlCommand);
  }

  return true;
}

void UrgencyStop::radarObjectFuse(
    std::vector<drivers::ContiRadarObs>& Obsvector,
    drivers::ContiRadar& tmp_message) {
  float high_snr = 45.0;
  // multi radar object list fuse
  uint32_t sub_size = Obsvector.size();
  std::vector<int> sub_match;
  sub_match.resize(sub_size);
  int k = 0;

  for (auto& main_obs : tmp_message.contiobs()) {
    if (main_obs.flags() == false) {
      if (main_obs.pos_x() > 30.0) {
        k++;
        continue;
      }

      for (uint32_t i = 0; i < sub_size; i++) {
        drivers::ContiRadarObs sub_obs = Obsvector[i];

        float px_m = main_obs.pos_x();
        float py_m = main_obs.pos_y();
        float px_s = sub_obs.pos_x();
        float py_s = sub_obs.pos_y();
        float d_error =
            sqrt((px_m - px_s) * (px_m - px_s) + (py_m - py_s) * (py_m - py_s));
        
        px_m = px_m - origin_dist_;
        if((px_m < 0.75 && px_m > 0.465) ){
          tmp_message.mutable_contiobs(k)->set_flags(true);
        }

        if (d_error < match_size_) {
          sub_match[i] = 1;
          int counters = main_obs.counters();
          counters++;
          tmp_message.mutable_contiobs(k)->set_counters(counters);

          if ((px_m < 3.0 && px_m > 0.75 ) || px_m > 10.0 ) {
            if (counters > 1) {
              tmp_message.mutable_contiobs(k)->set_flags(true);
            }
          } else {
            if (counters > 2) {
              tmp_message.mutable_contiobs(k)->set_flags(true);
            }
          }

          // tmp_message.mutable_contiobs(k)->set_snr(
          //     std::min(sub_obs.snr(), main_obs.snr()));
          if (sub_obs.snr() > high_snr || main_obs.snr() > high_snr) {
            tmp_message.mutable_contiobs(k)->set_snr(
                std::max(sub_obs.snr(), main_obs.snr()));
            tmp_message.mutable_contiobs(k)->set_flags(true);
          } else {
            tmp_message.mutable_contiobs(k)->set_snr(
                std::min(sub_obs.snr(), main_obs.snr()));
          }

          tmp_message.mutable_contiobs(k)->set_pos_z(
              std::max(sub_obs.pos_z(), main_obs.pos_z()));
          tmp_message.mutable_contiobs(k)->set_lifetime(
              std::max(sub_obs.lifetime(), main_obs.lifetime()));
        }
      }

      if(urgenStopConf_.double_check() == false){
       if (tmp_message.contiobs(k).counters() > 1) {
          tmp_message.mutable_contiobs(k)->set_flags(true);
        }
      }

    }
    k++;
  }
  for (uint32_t i = 0; i < sub_size; i++) {
    if (sub_match[i] == 0) {
      drivers::ContiRadarObs* obs = tmp_message.add_contiobs();
      drivers::ContiRadarObs sub_obs = Obsvector[i];
      obs->set_range(sub_obs.range());
      obs->set_azimuth(sub_obs.azimuth());
      obs->set_elevation(sub_obs.elevation());
      obs->set_doppler(sub_obs.doppler());
      obs->set_magnitude(sub_obs.magnitude());
      obs->set_snr(sub_obs.snr());
      obs->set_uhn_rcs(sub_obs.rcs());
      obs->set_pos_x(sub_obs.pos_x());
      obs->set_pos_y(sub_obs.pos_y());
      obs->set_pos_z(sub_obs.pos_z());
      obs->set_longitude_vel(sub_obs.longitude_vel());
      obs->set_lateral_vel(sub_obs.lateral_vel());
      obs->set_longitude_accel(sub_obs.longitude_accel());
      obs->set_lateral_accel(sub_obs.lateral_accel());
      obs->set_counters(sub_obs.counters());
      obs->set_lifetime(sub_obs.lifetime());
      obs->set_flags(sub_obs.flags());
    }
  }
}

void UrgencyStop::messageCollection(
    drivers::ContiRadar& sub_message,
    std::vector<drivers::ContiRadarObs>* Obsvector) {
  // collect radar detection object list
  Obsvector->clear();
  for (int i = 0; i < sub_message.contiobs().size(); i++) {
    const drivers::ContiRadarObs sub_obs = sub_message.contiobs(i);
      drivers::ContiRadarObs obs;
      obs.set_obstacle_id(sub_obs.obstacle_id());
      obs.set_pos_x(sub_obs.pos_x());
      obs.set_pos_y(sub_obs.pos_y());
      obs.set_pos_z(sub_obs.pos_z());
      obs.set_snr(sub_obs.snr());
      obs.set_longitude_vel(sub_obs.longitude_vel());
      obs.set_lateral_vel(sub_obs.lateral_vel());
      obs.set_longitude_accel(sub_obs.longitude_accel());
      obs.set_lateral_accel(sub_obs.lateral_accel());
      obs.set_counters(sub_obs.counters());
      obs.set_lifetime(sub_obs.lifetime());
      obs.set_flags(sub_obs.flags());
      Obsvector->push_back(obs);
  }
}

bool UrgencyStop::judgeStopRules(drivers::ContiRadar& message) {
  chassis_reader_->Observe();
  auto msg = chassis_reader_->GetLatestObserved();

  float cur_speed = 0.0;
  float cur_steer_percent = 0.0;
  // float cur_brake = 0.0;
  if (msg) {
    cur_speed = msg->speed_mps();
    cur_steer_percent = msg->steering_percentage();
    // cur_brake = msg->brake_percentage();
    if (msg->gear_location() == canbus::Chassis::GEAR_REVERSE) {
      return false;
    }
  }

  float steer_theta =
      cur_steer_percent / 100.0 * max_steer_angle_ / steer_ratio_;
  float steer_radius = 0;
  float ratio = 0;  //
  if (std::fabs(steer_theta) > 0.001) {
    steer_radius = wheel_base_ / sin(steer_theta);
    ratio = 1 / steer_radius;
  }

  float stop_dis = stop_dis_[0];
  if (stop_count_ == 0) {
    for (uint32_t i = 1; i < veh_speed_.size(); i++) {
      if (cur_speed < veh_speed_[i] && cur_speed > veh_speed_[i - 1]) {
        stop_dis = stop_dis_[i];
      }
    }
  } else {
    stop_dis = begin_stop_dis_;
  }

  AERROR << "steering_percentage:  " << cur_steer_percent << "   ,steer_theta: " << steer_theta
   << "   ,steer_radius:  " << steer_radius <<"   ,ratio:  " << ratio ; 
  AERROR << "stop_dis:  " << stop_dis << "   ,check_count: " << check_count_ << "   ,AEBFlag:  " << brakeFlag_ ;

  localization_reader_->Observe();
  auto local_msg = localization_reader_->GetLatestObserved();
  bool strict_flag = true;
  if (local_msg) {
    auto veh_pose = local_msg->pose().position();
    double veh_px = veh_pose.x() * cos(-1.24) + veh_pose.y() * sin(-1.24);
    double veh_py = -veh_pose.x() * sin(-1.24) + veh_pose.y() * cos(-1.24);

    if (veh_px > urgenStopConf_.line_xmin() &&
        veh_px < urgenStopConf_.line_xmax() &&
        veh_py > urgenStopConf_.line_ymin() &&
        veh_py < urgenStopConf_.line_ymax()) {
      strict_flag = false;
    }
  }
  
  float height_thre = urgenStopConf_.height_thre();

  float py_left = safety_zone_left_;
  float py_right = safety_zone_right_;

  if (strict_flag) {
    py_left = py_left + urgenStopConf_.safety_offset();
    py_right = py_right - urgenStopConf_.safety_offset();
  }
  

  bool stopFlag = false;
  bool tmpFlag = false;
  ContiRadar raw_obstacles = message;

  uint32_t objsize = raw_obstacles.contiobs_size();

  stop_px_ = 100.0f;

  for (uint32_t i = 0; i < objsize; i++) {
    const drivers::ContiRadarObs obs = raw_obstacles.contiobs(i);
    float vel = obs.longitude_vel();
    if (vel > 1.0) {
      continue;
    }
    float px = obs.pos_x();
    float py = obs.pos_y();
    float pz = obs.pos_z();
    float pxx = px ;
    px = px - origin_dist_;

    float ttc = -px / vel;
    float dy = ratio / 2 * pxx * pxx - py;
    float thw = px / cur_speed;

    float snr_thre = 0.0;
    if (!strict_flag) {
      if (px < 5.0 && px > 0.0) {
        snr_thre = urgenStopConf_.snr_thre();  
      } else if (px > 5.0 && px < 10.0) {
        snr_thre = urgenStopConf_.snr_thre() - 2.5;
      }
    }

    if (dy < py_left && dy > py_right && px < stop_dis && px < stop_px_ &&   
        pz > height_thre && (px > 0.3 && obs.flags() == true) && obs.snr() > snr_thre) {
      tmpFlag = true;
      stop_px_ = px;
      stop_py_ = py;
      stop_pz_ = pz;
      ttc_ = ttc;
      thw_ = thw;
      dy_ = dy;
      ratio_ = ratio;
      // AERROR << "px :" << px << ";   py : " << py << ";   pz : "<< pz << ";  ratio: "<< ratio <<
      //  ";  dy: "<< dy << ";  vel_x:  " << obs.longitude_vel() <<
      //  ";  vel_y :" << obs.lateral_vel() << ";  snr: " << obs.snr() << " ; lifetime :" << obs.lifetime();
    }
  }

  if (tmpFlag) {
    check_count_++;
    if (check_count_ > check_frame_) {
      check_count_ = check_frame_;
    }
  } else {
    if (check_count_ > 0) {
      check_count_--;
    } else {
      check_count_ = 0;
    }
  }

  if (!brakeFlag_) {
    if (check_count_ >= check_frame_) {
      brakeFlag_ = true;
      if (stop_count_ == 0) {
        begin_stop_dis_ = stop_dis;
        stop_count_++;
      }
    }
  } else {
    if (check_count_ == 0) {
      brakeFlag_ = false;
      stop_count_ = 0;
    }
  }

  stopFlag = brakeFlag_;

  return stopFlag;
}

void UrgencyStop::outputFusedObj(drivers::ContiRadar& message) {
  drivers::ContiRadar radarObj;
  uint16_t objsize = message.contiobs_size();
  float px;
  
  for (uint16_t i = 0; i < objsize; ++i) {
    drivers::ContiRadarObs sub_obs = message.contiobs(i);
    px = sub_obs.pos_x() - origin_dist_;

    if ((px > 0.3&& sub_obs.flags() == true)) {
      drivers::ContiRadarObs* obs = radarObj.add_contiobs();
      obs->set_range(sub_obs.range());
      obs->set_azimuth(sub_obs.azimuth());
      obs->set_elevation(sub_obs.elevation());
      obs->set_doppler(sub_obs.doppler());
      obs->set_magnitude(sub_obs.magnitude());
      obs->set_snr(sub_obs.snr());
      obs->set_uhn_rcs(sub_obs.rcs());
      obs->set_pos_x(sub_obs.pos_x());
      obs->set_pos_y(sub_obs.pos_y());
      obs->set_pos_z(sub_obs.pos_z());
      obs->set_longitude_vel(sub_obs.longitude_vel());
      obs->set_lateral_vel(sub_obs.lateral_vel());
      obs->set_longitude_accel(sub_obs.longitude_accel());
      obs->set_lateral_accel(sub_obs.lateral_accel());
      obs->set_flags(sub_obs.flags());
    }
  }
  radar_writer_->Write(radarObj);
}

bool UrgencyStop::ultrasonicJudge(drivers::Ultrasonic& message) {
  bool tmpFlag = false;

  chassis_reader_->Observe();
  auto msg = chassis_reader_->GetLatestObserved();
  if (msg) {
    if (msg->gear_location() == canbus::Chassis::GEAR_REVERSE) {
      return false;
    }
  }

  // for (int i = 0; i < 4; i++) {
  //   float range = message.ranges(i);
  //   if (range > 0.1f && range < 0.9f) {
  //     tmpFlag = true;
  //   }
  // }

  for (int i = 0; i < message.ultr_obstacle_size(); i++) {
    auto obj = message.ultr_obstacle(i);
    // float x = obj.x();
    float y = obj.y();
    if((y <= 1.691f) && (y > -1.688f)) {
      tmpFlag = true;
      if(((obj.index() == 9) || (obj.index() == 6)) && obj.range() >= 1.0) {
        tmpFlag = false;
      }
      if(tmpFlag) {
        fflush(NULL);
        AERROR << "ultrasonic stop : " << obj.index() << " , " << obj.range();
      }

    } 
  }

  /*static uint32_t ultra_check_count_ = 0;
  static bool ultra_flag = false;
  if (tmpFlag) {
    ultra_check_count_++;
    if (ultra_check_count_ > check_frame_) {
      ultra_check_count_ = check_frame_;
    }
  } else {
    if (ultra_check_count_ > 0) {
      ultra_check_count_--;
    } else {
      ultra_check_count_ = 0;
    }
  }

  if (!ultra_flag) {
    if (ultra_check_count_ >= check_frame_) {
      ultra_flag = true;
    }
  } else {
    if (ultra_check_count_ == 0) {
      ultra_flag = false;
    }
  }
  // TODO side ultrasonic rules

  return ultra_flag;*/
  return tmpFlag;
}


bool UrgencyStop::polyfitLine(drivers::ContiRadar& message){
  
  localization_reader_->Observe();
  auto local_msg = localization_reader_->GetLatestObserved();
  bool strict_flag = true;
  if (local_msg) {
    auto veh_pose = local_msg->pose().position();
    double veh_px = veh_pose.x() * cos(-1.24) + veh_pose.y() * sin(-1.24);
    double veh_py = -veh_pose.x() * sin(-1.24) + veh_pose.y() * cos(-1.24);

    if (veh_px > urgenStopConf_.line_xmin() &&
        veh_px < urgenStopConf_.line_xmax() &&
        veh_py > urgenStopConf_.line_ymin() &&
        veh_py < urgenStopConf_.line_ymax()) {
      strict_flag = false;
    }
  }

   float vehSpeed = 0.0; 
   chassis_reader_->Observe();
   auto msg = chassis_reader_->GetLatestObserved();
   if (msg) {
    vehSpeed = msg->speed_mps();
  if (msg->gear_location() == canbus::Chassis::GEAR_REVERSE) {
      return false;
    }
   }

  double dist = vehSpeed * 3.0; 

#if 1

  if (!strict_flag) {
    float best_a = -100;
    float best_b = -100;
    float a = 0;
    float b = 0;
    uint16_t max_num = 0;
    uint16_t num = 0;
    uint16_t objsize = message.contiobs_size();
    for (uint16_t i = 0; i < objsize; i++) {
      auto obj_i = message.contiobs(i);
      if (obj_i.pos_x() > 20) {
        continue;
      }
      for (uint16_t j = i + 1; j < objsize; j++) {
        auto obj_j = message.contiobs(j);
        if (obj_j.pos_x() > 20) {
          continue;
        }
        a = (obj_i.pos_y() - obj_j.pos_y()) / (obj_i.pos_x() - obj_j.pos_x());
        b = obj_i.pos_y() - a * obj_i.pos_x();
        num = 0;
        for (uint16_t k = 0; k < objsize; k++) {
          auto obj_k = message.contiobs(k);
          float sigma = a * obj_k.pos_x() + b - obj_k.pos_y();
          if (std::fabs(sigma) <  0.2) {
            num++;
          }
        }
        if (num > max_num) {
          max_num = num;
          best_a = a;
          best_b = b;
        }
      }
    }

    if (max_num > 15) {
      //点到直线的距离
      float distance = 0.0;
      best_a = best_a * 0.85 + fence_ratio_ * 0.15;
      best_b = best_b * 0.85 + fence_dist_ * 0.15;

      if (best_b > 0) {
        CorD_ = false; // D
        distance = std::fabs((best_a * 4.92 + best_b - 1.25) /
                             (std::sqrt(best_a * best_a + (-1) * (-1))));
      } else {
        CorD_ = true; // C
        distance = std::fabs((best_a * 4.92 + best_b - (-1.25)) /
                             (std::sqrt(best_a * best_a + (-1) * (-1))));
      }
  
      fence_ratio_ = best_a;
      fence_dist_ = best_b;
      dis2fence_ = distance;
      if (dis2fence_ < urgenStopConf_.fence_distance() ||
          (std::fabs(fence_dist_/fence_ratio_ ) < std::max(urgenStopConf_.inter_distance(),dist))) { 
        return true;
      }
    }
  }  else {
    fence_ratio_ = 0;
    fence_dist_ = 0;
    dis2fence_ = 0;
  }
#else
  if (!strict_flag) {
  std::vector<drivers::ContiRadarObs> object;
  uint16_t objsize1 = message.contiobs_size();
  for (uint16_t i = 0; i < objsize1; i++) {
    auto obj_i = message.contiobs(i);
    if (obj_i.pos_x() < 20 &&
        std::fabs(obj_i.longitude_vel() - veh_speed) < 1.0)) {
      object.push_back((obj_i);
      }
  }
  }
#endif

  return false;
}

}  //  namespace  urgency
}  // namespace apollo
