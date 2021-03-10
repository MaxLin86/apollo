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
 * @file tire_pressure_message_manager.h
 * @brief The class of TirePressureMessageManager
 */

#include "modules/drivers/tire_pressure/tire_pressure_message_manager.h"
#include "modules/common/util/message_util.h"

namespace apollo {
namespace drivers {
namespace tire_pressure {

TirePressureMessageManager::TirePressureMessageManager(
    const std::shared_ptr<::apollo::cyber::Writer<Tire_Pressure>> &tire_pressure_writer)
    : tire_pressure_writer_(tire_pressure_writer) {
  AddRecvProtocolData<Tire18FEF433, true>();
}

void TirePressureMessageManager::set_can_client(
    std::shared_ptr<CanClient> can_client) {
  can_client_ = can_client;
}

void TirePressureMessageManager::Parse(const uint32_t message_id,
                                          const uint8_t *data, int32_t length) {
  if (message_id == 0x98FEF433) {
    fflush(NULL);
    uint8_t location = data[0];
    Tire_Pressure::Location tire_location = Tire_Pressure::ERROR;
    switch(location) {
      case 0:
        tire_location = Tire_Pressure::FRONTLEFT;// front_left
        break;
      case 3:
        tire_location = Tire_Pressure::FRONTRIGHT;// front_right
        break;
      case 16:
        tire_location = Tire_Pressure::REARLEFTOUT;//rear_left_out
        break;
      case 17:
        tire_location = Tire_Pressure::REARLEFTINNER;// rear_left_inner
        break;
      case 18:
        tire_location = Tire_Pressure::REARRIGHTINNER;//rear_right_inner
        break;
      case 19:
        tire_location = Tire_Pressure::REARRIGHTOUT;//rear_right_out
        break;
      default:
        tire_location = Tire_Pressure::ERROR;// fault
        break;
    }
    sensor_data_.set_tire_location(tire_location);
    // AERROR << "tire_location: " << tire_location;

    uint8_t pressure = data[1];
    float tire_pressure = pressure * 5.5;//kpa
    sensor_data_.set_tire_pressure(tire_pressure);
    // AERROR << "tire_pressure: " << tire_pressure;

    uint16_t temperature = (((uint16_t)(data[3])) << 8) | (uint16_t)(data[2]);
    float tire_temperature = temperature * 0.03125 - 273.0; //℃
    sensor_data_.set_tire_temperature(tire_temperature);
    // AERROR << "tire_temperature: " << tire_temperature;

    // uint8_t CTIStatus = data[4];
    uint8_t sensor_loss = (data[4] & 0x10) >> 4;
    sensor_data_.set_sensor_loss(sensor_loss);
    // AERROR << "sensor_loss: " << sensor_loss;
    uint8_t leakage_warning = (data[4] & 0x04) >> 2;
    sensor_data_.set_leakage_warning(leakage_warning);
    uint8_t high_temp_warning = (data[4] & 0x01);
    sensor_data_.set_high_temp_warning(high_temp_warning);

    uint8_t pressure_warning = data[7] & 0xE0;
    Tire_Pressure::WarnType tire_pressure_warning = Tire_Pressure::NORMAL;
    switch(pressure_warning) {
      case 0:
        tire_pressure_warning = Tire_Pressure::HIGHWARN;
        break;
      case 32:
        tire_pressure_warning = Tire_Pressure::HIGHPREWARN;
        break;
      case 64:
        tire_pressure_warning = Tire_Pressure::NORMAL;
        break;
      case 96:
        tire_pressure_warning = Tire_Pressure::LOWPREWARN;
        break;
      case 128:
        tire_pressure_warning = Tire_Pressure::LOWWARN;
        break;
      default:
        tire_pressure_warning = Tire_Pressure::ABNORMAL;
        break;
    }
    sensor_data_.set_presure_warning(tire_pressure_warning);
    AERROR << "tire_pressure_warning: " << tire_pressure_warning;

    common::util::FillHeader("tire_pressure", &sensor_data_);
    tire_pressure_writer_->Write(sensor_data_);
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

}  // namespace tire_pressure
}  // namespace drivers
}  // namespace apollo
