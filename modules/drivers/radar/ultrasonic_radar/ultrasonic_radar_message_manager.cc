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

namespace apollo {
namespace drivers {
namespace ultrasonic_radar {

UltrasonicRadarMessageManager::UltrasonicRadarMessageManager(
    const int entrance_num,
    const std::shared_ptr<::apollo::cyber::Writer<Ultrasonic>> &writer,
    const std::shared_ptr<::apollo::cyber::Writer<Ultra_Radar>>
        &ultra_radar_writer)
    : entrance_num_(entrance_num),
      ultrasonic_radar_writer_(writer),
      ultra_radar_writer_(ultra_radar_writer) {
  sensor_data_.mutable_ranges()->Resize(entrance_num_, 0.0);
  sensor_data_.mutable_ranges_x()->Resize(entrance_num_, 0.0);
  sensor_data_.mutable_ranges_y()->Resize(entrance_num_, 0.0);
  sensor_data_.mutable_ranges_z()->Resize(entrance_num_, 0.0);
  AddRecvProtocolData<Ultra525, true>();
}

void UltrasonicRadarMessageManager::set_can_client(
    std::shared_ptr<CanClient> can_client) {
  can_client_ = can_client;
}

void UltrasonicRadarMessageManager::Parse(const uint32_t message_id,
                                          const uint8_t *data, int32_t length) {
  //gqs
  // const int64_t ReceivedTime = absl::ToUnixMicros(Clock::Now());
  // printf("ReceivedTime = %lld\n", ReceivedTime);
  // if(lastRevTime_ != 0){
  //  int64_t period = ReceivedTime - lastRevTime_;
  //  printf("period = %lld\n", period);
  // }
  // lastRevTime_ = ReceivedTime;

  if (message_id == 0x525) {
    fflush(NULL);
    for (int i = 0; i < 8; i++) {
      uint8_t tmp = data[i] & 0x1F;
      float range = 0.0f;
      if(tmp == 0x01){
        range = 0.35;
      }
      if (tmp > 0x01 && tmp < 0x1A) {
        range = (tmp - 0x02) * 0.05 + 0.35;
      }
      sensor_data_.set_ranges(i, range);
      printf("index:    %d,  range:  %.3f\n",i,range);
      //gqs
      sensor_data_.set_ranges_x(i, (range + 4.92));
      if(i == 0){
        sensor_data_.set_ranges_y(i, 0.7);
      } else if(i == 1){
        sensor_data_.set_ranges_y(i, 0.25);
      } else if(i == 2){
        sensor_data_.set_ranges_y(i, -0.25);
      } else if(i == 3){
        sensor_data_.set_ranges_y(i, -0.7);
      }
      sensor_data_.set_ranges_z(i, 0);
    }
    common::util::FillHeader("ultrasonic_radar", &sensor_data_);
    ultrasonic_radar_writer_->Write(sensor_data_);
    tmpMsg.mutable_ultrasonic_obs()->CopyFrom(sensor_data_);
    ultra_radar_writer_->Write(tmpMsg);
  }

  received_ids_.insert(message_id);
  // check if need to check period
  const auto it = check_ids_.find(message_id);
  if (it != check_ids_.end()) {
    const int64_t time = absl::ToUnixMicros(Clock::Now());
    it->second.real_period = time - it->second.last_time;
    printf("real_period:  %lld\n", it->second.real_period);
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
