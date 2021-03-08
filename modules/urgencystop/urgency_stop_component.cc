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
#include<iostream>
#include <fstream>  //æä»¶æµåºå½æ°
#include <iomanip>


namespace apollo {
namespace urgency {

#define LEFT_RADAR          1
#define LEFT_CORNER_RADAR   2
#define RIGHT_RADAR         4
#define RIGHT_CORNER_RADAR  8

bool UrgencyStop::Init() {
  if (!GetProtoConfig(&urgenStopConf_)) {
    return false;
  }

  origin_dist_ = urgenStopConf_.dist_to_origin();
  safety_zone_left_ = urgenStopConf_.safety_zone_left();
  safety_zone_right_ = urgenStopConf_.safety_zone_right();
  safety_dist_ = urgenStopConf_.safety_distance();
  turn_radius_ = urgenStopConf_.turn_radius();
  check_frame_ = urgenStopConf_.check_frame();
  match_size_  = urgenStopConf_.match_size();
  check_count_ = 0;
  brakeFlag_ = false;
  
  radarBrakeFlag_ = false;
  ultraBrakeFlag_ = false;

  stop_count_ = 0;
  begin_stop_dis_ = 0.0;

  control_guardian_writer_ = node_->CreateWriter<AEBCommand>(FLAGS_aeb_command_topic);
  ACHECK(control_guardian_writer_ != nullptr);

   radar_writer_ = node_->CreateWriter<drivers::ContiRadar>(FLAGS_fused_radar_topic);
   ACHECK(radar_writer_ != nullptr);

  cyber::ReaderConfig chassis_reader_config;
  chassis_reader_config.channel_name = FLAGS_chassis_topic;
  chassis_reader_config.pending_queue_size = 10;

  chassis_reader_ =
      node_->CreateReader<apollo::canbus::Chassis>(chassis_reader_config, nullptr);
  CHECK(chassis_reader_ != nullptr);

  cyber::ReaderConfig localreader_config;
  localreader_config.channel_name = FLAGS_localization_topic;
  localreader_config.pending_queue_size = 10;
  localization_reader_ =  node_->CreateReader<apollo::localization::LocalizationEstimate>(localreader_config,nullptr);

  for (int i = 0; i < urgenStopConf_.veh_speed_size(); i++) {
    veh_speed_.push_back((urgenStopConf_.veh_speed(i)/3.60));
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
  static double leftCornerTime = 0.0;
  static double leftRadarTime = 0.0;
  static double rightCornerTime = 0.0;
  static double rightRadarTime = 0.0;

  bool missRadarFlag = false;
  uint16_t miss_radar_pos = 0;
  static bool missRadar = false;
  static uint16_t miss_pos =0;
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
    } else if (radarMessage.radar_id() == 0 ||
               radarMessage.header().module_name() == "radar_front") {
      time_ = apollo::common::time::Clock::NowInSeconds();
      fflush(NULL);
      // // fuse object list
      if (std::fabs(timestamp - leftRadarTime) < 0.5) {
        radarObjectFuse(leftObs_, radarMessage);
      }else{
        missRadarFlag = true;
        miss_radar_pos |= LEFT_RADAR;
      }
      if (std::fabs(timestamp - leftCornerTime) < 0.5) {
        radarObjectFuse(leftCornerObs_, radarMessage);
      }else{
      //        missRadarFlag = true;
        miss_radar_pos |= LEFT_CORNER_RADAR;
      }
      if (std::fabs(timestamp - rightRadarTime) < 0.5) {
        radarObjectFuse(rightObs_, radarMessage);
      } else{
        missRadarFlag = true;
        miss_radar_pos |= RIGHT_RADAR;
      }
      if (std::fabs(timestamp - rightCornerTime) < 0.5) {
        radarObjectFuse(rightCornerObs_, radarMessage);
      }else{
        // missRadarFlag = true;
        miss_radar_pos |= RIGHT_CORNER_RADAR;
      }

      radarBrakeFlag_ = judgeStopRules(radarMessage);
      outputFusedObj(radarMessage);
      missRadar = missRadarFlag;
      miss_pos = miss_radar_pos;
    }
  }

  if (ultraRadarMessage->has_ultrasonic_obs()) {
    auto ultraMessage = ultraRadarMessage->ultrasonic_obs();
    ultraBrakeFlag_ = ultrasonicJudge(ultraMessage);
  }

  if ((ultraRadarMessage->has_radarobs() &&
       ultraRadarMessage->radarobs().radar_id() == 0) ||
      ultraRadarMessage->has_ultrasonic_obs()) {
    AEBCommand tmpControlCommand;
    if (radarBrakeFlag_ || ultraBrakeFlag_ || missRadar) {
      // tmpControlCommand.set_throttle(0.0);
      tmpControlCommand.mutable_control_command()->set_brake(100);
      if (radarBrakeFlag_) {
        tmpControlCommand.set_stop_px(stop_px_);
        tmpControlCommand.set_stop_py(stop_py_);
        tmpControlCommand.set_stop_pz(stop_pz_);
        tmpControlCommand.set_ttc(ttc_);
        tmpControlCommand.set_thw(thw_);
        tmpControlCommand.set_dy(dy_);
        tmpControlCommand.set_ratio(ratio_);
      }
      //printf("Time is:   %.5f,stop the truck!!!\n",timestamp);
    } else {
      tmpControlCommand.mutable_control_command()->set_brake(0);
    }
    tmpControlCommand.set_miss_radar(miss_pos);
    common::util::FillHeader(node_->Name(), &tmpControlCommand);
    control_guardian_writer_->Write(tmpControlCommand);
  }

  return true;
}

void UrgencyStop::radarObjectFuse(
    std::vector<drivers::ContiRadarObs>& Obsvector,
    drivers::ContiRadar& tmp_message) {
  // multi radar object list fuse
  uint32_t sub_size = Obsvector.size();
  std::vector<int> sub_match;
  sub_match.resize(sub_size);
  int k = 0;
  for (auto& main_obs : tmp_message.contiobs()) {
    if (main_obs.fags() == false) {
      for (uint32_t i = 0; i < sub_size; i++) {
        if (sub_match[i] == 1) {
          continue;
        }
        drivers::ContiRadarObs sub_obs = Obsvector[i];
        float px_m = main_obs.pos_x();
        float py_m = main_obs.pos_y();
        float px_s = sub_obs.pos_x();
        float py_s = sub_obs.pos_y();
        float d_error =
            sqrt((px_m - px_s) * (px_m - px_s) + (py_m - py_s) * (py_m - py_s));
        if (d_error < match_size_) {
          sub_match[i] = 1;
          tmp_message.mutable_contiobs(k)->set_fags(true);
          tmp_message.mutable_contiobs(k)->set_pos_z(sub_obs.pos_z());
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
      obs->set_fags(sub_obs.fags());
    }
  }
}

void UrgencyStop::messageCollection(
    drivers::ContiRadar &sub_message,
    std::vector<drivers::ContiRadarObs>* Obsvector) {
  // collect radar detection object list 
  Obsvector->clear();
  for (int i = 0; i < sub_message.contiobs().size(); i++) {
    drivers::ContiRadarObs obs;
    const drivers::ContiRadarObs sub_obs = sub_message.contiobs(i);
    obs.set_obstacle_id(sub_obs.obstacle_id());
    obs.set_pos_x(sub_obs.pos_x());
    obs.set_pos_y(sub_obs.pos_y());
    obs.set_pos_z(sub_obs.pos_z());
    obs.set_longitude_vel(sub_obs.longitude_vel());
    obs.set_lateral_vel(sub_obs.lateral_vel());
    obs.set_longitude_accel(sub_obs.longitude_accel());
    obs.set_lateral_accel(sub_obs.lateral_accel());
    obs.set_fags(false);
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

  // localization_reader_->Observe();
  // auto local_msg = localization_reader_->GetLatestObserved();

  float stop_dis = stop_dis_[0];
  if (stop_count_ == 0)
  {
    for (uint32_t i = 1; i < veh_speed_.size(); i++)
    {
      if (cur_speed < veh_speed_[i] && cur_speed > veh_speed_[i - 1])
      {
        stop_dis = stop_dis_[i];
      }
    }
  }
  else
  {
    stop_dis = begin_stop_dis_;
  }

  bool stopFlag = false;
  bool tmpFlag = false;
  ContiRadar raw_obstacles = message;

  float height_thre = 0.0f;
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

    float dy = ratio/2 * px * px - py;
    px = px - origin_dist_;
    float ttc = -px / vel;
    float thw = px/cur_speed;
    if (dy < safety_zone_left_ && dy > safety_zone_right_ && px < stop_dis &&
        px < stop_px_ && pz > height_thre &&
        ((px > 2.0 && obs.fags() == true) ||
         (px < 2.0 && px > 0.5))) {
      tmpFlag = true;
      stop_px_ = px;
      stop_py_ = py;
      stop_pz_ = pz;
      ttc_ = ttc;
      thw_ = thw;
      dy_ = dy;
      ratio_ = ratio;
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
      if(stop_count_ == 0){
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
  // if (stopFlag) {
  //   std::ofstream outfile;
  //   outfile.open("result.txt", std::ios::app);
  //   outfile << std::fixed;
  //   outfile << std::setprecision(9);
  //   double timestamp = apollo::common::time::Clock::NowInSeconds();
  //   outfile << time_ << std::setw(20) << timestamp << std::setw(20) << cur_speed << std::setw(20)
  //           << cur_brake << std::setw(20) << stop_px << std::setw(20) << stop_py
  //           << std::setw(20) << stop_pz << std::setw(20);

  //   if (local_msg) {
  //     outfile << local_msg->pose().position().x() << std::setw(20)
  //             << local_msg->pose().position().y() << std::setw(20)
  //             << local_msg->pose().position().z() << std::setw(20)
  //             << local_msg->pose().linear_velocity().x() << std::setw(20)
  //             << local_msg->pose().linear_velocity().y() << std::setw(20)
  //             << local_msg->pose().linear_velocity().z() << std::endl;
  //   } else {
  //     outfile << std::endl;
  //   }

  //   outfile.close();
  // }

  return stopFlag;
}


void UrgencyStop::outputFusedObj(drivers::ContiRadar& message){
  drivers::ContiRadar radarObj;
  uint16_t objsize = message.contiobs_size();
  for (uint16_t i = 0; i < objsize; ++i) {
    if (message.contiobs(i).fags() ||
        (message.contiobs(i).pos_x() < 2.0 &&
         message.contiobs(i).pos_x() > 0.5 &&
         std::fabs(message.contiobs(i).pos_y()) < 1.5)) {
      drivers::ContiRadarObs* obs = radarObj.add_contiobs();
      drivers::ContiRadarObs sub_obs = message.contiobs(i);
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
      obs->set_fags(sub_obs.fags());
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

  for (int i = 0; i < 4; i++) {
    float range = message.ranges(i);
    if (range > 0.1f && range < 0.9f) {
      tmpFlag = true;
    }
  }

  static uint32_t ultra_check_count_ = 0;
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

  return ultra_flag;
}

}  //  namespace  urgency
}  // namespace apollo
