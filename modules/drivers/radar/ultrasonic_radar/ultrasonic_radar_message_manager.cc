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

/**
 * @file ultrasonic_radar_message_manager.h
 * @brief The class of UltrasonicRadarMessageManager
 */

#include "modules/drivers/radar/ultrasonic_radar/ultrasonic_radar_message_manager.h"
#include "modules/common/util/message_util.h"
#include "modules/drivers/radar/ultrasonic_radar/protocol/ultra401.h"

namespace apollo {
namespace drivers {
namespace ultrasonic_radar {

UltrasonicRadarMessageManager::UltrasonicRadarMessageManager(
    const UltrasonicRadarConf ultrasonic_radar_conf,
    const std::shared_ptr<::apollo::cyber::Writer<Ultrasonic>> &writer,
    const std::shared_ptr<::apollo::cyber::Writer<Ultra_Radar>>
        &ultra_radar_writer)
    : ultrasonic_radar_conf_(ultrasonic_radar_conf),
      ultrasonic_radar_writer_(writer),
      ultra_radar_writer_(ultra_radar_writer) {
  // sensor_dat<a_.mutable_ranges()->Resize(entrance_num_, 12.7);
  for(int i = 0; i < ultrasonic_radar_conf_.origin_size(); i++) {
    origin_.push_back(ultrasonic_radar_conf_.origin(i));
  }
  ultrobjs_.resize(ultrasonic_radar_conf_.entrance_num());
  AddRecvProtocolData<Ultra600, true>();
  AddRecvProtocolData<Ultra601, true>();
  AddRecvProtocolData<Ultra602, true>();
}

void UltrasonicRadarMessageManager::set_can_client(
    std::shared_ptr<CanClient> can_client) {
  can_client_ = can_client;
}

void UltrasonicRadarMessageManager::Parse(const uint32_t message_id,
                                          const uint8_t *data, int32_t length) {
  static int64_t TimeParse600 = 0;
  static int64_t TimeParse601 = 0;
  static int64_t TimeParse602 = 0;
  
  if (message_id == Ultra600::ID) {
    uint8_t state = ultr_send_[0];
    TimeParse600 = absl::ToUnixMicros(Clock::Now());
    int id_base = 0;
    for (int i = 0; i < 6; i++) {
      uint8_t tmp = data[i];
      int index = i + id_base;
      if((index == 3)||(index == 4)) {
        index = index + 1;
      } else if(index == 5) {
        index = 3;
      }
      bool valid = tmp & 0x80;
      float range = 10.0f;
      if (valid) {
        bool enable = (state >> i) & 0x01;
        if(enable) {
          range = (tmp & 0x7F) * 0.1;
          if(range < 12.7f) {
            AERROR << "ID " << index << ":" << range;
            // sensor_data_.set_ranges(index, range);
            ultrobjs_[index].range = range;
            ultrobjs_[index].x = origin_[index].x();//(index <= 4) ? (origin_[index].x()):(origin_[index].x() + range * sin(25));
            ultrobjs_[index].y = origin_[index].y() - range;//(index <= 4) ? (origin_[index].y() - range):(origin_[index].y() - range * cos(25));
            ultrobjs_[index].z = origin_[index].z();
            unObjFrameCnt_[index] = 0;
          } else {
            unObjFrameCnt_[index] = unObjFrameCnt_[index] + 1;
            if(unObjFrameCnt_[index] >= 4) {
              unObjFrameCnt_[index] = 0;
              // sensor_data_.set_ranges(index, range);
              ultrobjs_[index].range = range;
            }
          }
        }
      } else {
        AERROR << "UltraRadar " << index << " is invalid";
      }
    }
  } 
  if (message_id == Ultra601::ID) {
    TimeParse601 = absl::ToUnixMicros(Clock::Now());
    uint8_t state = ultr_send_[1];
    int id_base = 6;
    for (int i = 0; i < 5; i++) {
      uint8_t tmp = data[i];
      int index = i + id_base;
      bool valid = tmp & 0x80;
      float range = 10.0f;
      if (valid) {
        bool enable = (state >> i) & 0x01;
        if(enable) {
          range = (tmp & 0x7F) * 0.1;
          if(range < 12.7f) {
            AERROR << "ID " << index << ":" << range;
            // sensor_data_.set_ranges(index, range);
            ultrobjs_[index].range = range;
            ultrobjs_[index].x = (index <= 9) ? (origin_[index].x() + range):(origin_[index].x());;
            ultrobjs_[index].y = (index <= 9) ? (origin_[index].y()):(origin_[index].y() + range);
            ultrobjs_[index].z = origin_[index].z();
            unObjFrameCnt_[index] = 0;
          } else {
            unObjFrameCnt_[index] = unObjFrameCnt_[index] + 1;
            if(unObjFrameCnt_[index] >= 4) {
              unObjFrameCnt_[index] = 0;
              // sensor_data_.set_ranges(index, range);
              ultrobjs_[index].range = range;
            }
          }
        } 
      } else {
        AERROR << "UltraRadar " << index << " is invalid";
      }
    }
  }

  if (message_id == Ultra602::ID) {
    uint8_t state = ultr_send_[2];
    TimeParse602 = absl::ToUnixMicros(Clock::Now());
    int id_base = 11;
    for (int i = 0; i < 5; i++) {
      uint8_t tmp = data[i];
      int index = i + id_base;
      bool valid = tmp & 0x80;
      float range = 10.0f;
      if (valid) {
        bool enable = (state >> i) & 0x01;
        if(enable) {
          range = (tmp & 0x7F) * 0.1;
          if(range < 12.7f) {
            AERROR << "ID " << index << ":" << range;
            // sensor_data_.set_ranges(index, range);
            ultrobjs_[index].range = range;
            ultrobjs_[index].x = origin_[index].x();
            ultrobjs_[index].y = origin_[index].y() + range;
            ultrobjs_[index].z = origin_[index].z();
            unObjFrameCnt_[index] = 0;
          } else {
            unObjFrameCnt_[index] = unObjFrameCnt_[index] + 1;
            if(unObjFrameCnt_[index] >= 4) {
              unObjFrameCnt_[index] = 0;
              // sensor_data_.set_ranges(index, range);
              ultrobjs_[index].range = range;
            }
          }
        }
      } else {
        AERROR << "UltraRadar " << index << " is invalid";
      }
    }
  }

  sensor_data_.clear_ultr_obstacle();
  for(size_t i = 0; i < ultrobjs_.size(); i++) {
    if(ultrobjs_[i].range < 10.0f) {
      UltrObstacle* ultrobj = sensor_data_.add_ultr_obstacle();
      ultrobj->set_index(i);
      ultrobj->set_range(ultrobjs_[i].range);
      ultrobj->set_x(ultrobjs_[i].x);
      ultrobj->set_y(ultrobjs_[i].y);
      ultrobj->set_z(ultrobjs_[i].z);
    }
  }

  if((TimeParse600 > 0) && (TimeParse601 > 0) && (TimeParse602 > 0) &&
    (abs(TimeParse602 - TimeParse601) < 50000) && (abs(TimeParse601 - TimeParse600) < 50000)) {
    TimeParse600 = 0;
    TimeParse601 = 0;
    TimeParse602 = 0;

    Ultra401 tmpValue;
    SenderMessage<Ultrasonic> sender_message(Ultra401::ID,&tmpValue);
    memcpy(ultr_send_, sender_message.CanFrame().data, sizeof(uint8_t) * 3);
    can_client_->SendSingleFrame({sender_message.CanFrame()});
  }

  common::util::FillHeader("ultrasonic_radar", &sensor_data_); //&sensor_data_
  ultrasonic_radar_writer_->Write(sensor_data_);
  tmpMsg.mutable_ultrasonic_obs()->CopyFrom(sensor_data_);
  ultra_radar_writer_->Write(tmpMsg);
  
  received_ids_.insert(message_id);
  // check if need to check period
  const auto it = check_ids_.find(message_id);
  if (it != check_ids_.end()) {
    const int64_t time = absl::ToUnixMicros(Clock::Now());
    it->second.real_period = time - it->second.last_time;
    // if period 1.5 large than base period, inc error_count
    const double period_multiplier = 1.5;
    if (it->second.real_period >
        (static_cast<double>(it->second.period) * period_multiplier)) {
      it->second.error_count += 1;
    } else {
      it->second.error_count = 0;
    }
    it->second.last_time = time;
  }
}

}  // namespace ultrasonic_radar
}  // namespace drivers
}  // namespace apollo
