/******************************************************************************
 * Copyright 2019 The Apollo Authors. All Rights Reserved.
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

#include "modules/canbus/vehicle/ch/protocol/adu_brakectrl_8c04eb0a.h"

#include "modules/drivers/canbus/common/byte.h"

namespace apollo {
namespace canbus {
namespace ch {

using ::apollo::drivers::canbus::Byte;

const int32_t Adubrakectrl8c04eb0a::ID = 0x8c04eb0a;

// public
Adubrakectrl8c04eb0a::Adubrakectrl8c04eb0a() { Reset(); }

uint32_t Adubrakectrl8c04eb0a::GetPeriod() const {
  // TODO(All) :  modify every protocol's period manually
  static const uint32_t PERIOD = 20 * 1000;
  return PERIOD;
}

inline void Adubrakectrl8c04eb0a::set_msg_flag(uint8_t* data) {
  Byte to_set(data + 0);
  to_set.set_value(0x0B, 0, 8);

  Byte to_set1(data + 1);
  to_set1.set_value(0x0E, 0, 8);

  Byte to_set2(data + 2);
  to_set2.set_value(0xEB, 0, 8);
}

void Adubrakectrl8c04eb0a::UpdateData(uint8_t* data) {
  set_p_adu_brake1_air_pressure(data, adu_brake1_air_pressure_);
  set_p_adu_brake2_air_pressure(data, adu_brake2_air_pressure_);
  set_msg_flag(data);
}

void Adubrakectrl8c04eb0a::Reset() {
  adu_brake1_air_pressure_ = 0;
  adu_brake2_air_pressure_ = 0;
}


Adubrakectrl8c04eb0a* Adubrakectrl8c04eb0a::set_adu_brake1_air_pressure(
    int adu_brake1_air_pressure) {
  adu_brake1_air_pressure_ = adu_brake1_air_pressure;
  return this;
}

// config detail: {'name': 'AD_BrakePercent', 'offset': 0.0, 'precision': 1.0, 'len': 7, 'is_signed_var': False, 'physical_range': '[0|100]', 'bit': 33, 'type': 'int', 'order': 'intel', 'physical_unit': '%'}
void Adubrakectrl8c04eb0a::set_p_adu_brake1_air_pressure(uint8_t* data,
    int adu_brake1_air_pressure) {
  /* Modify@20201223: according to Rev13 matrix, unit change from /100 to /1000 */
  adu_brake1_air_pressure = ProtocolData::BoundedValue(0, 1000, adu_brake1_air_pressure);
  int x = adu_brake1_air_pressure;
  
  uint8_t half = 0;
  half = static_cast<uint8_t>(x & 0xFF);
  Byte to_set0(data + 3);
  to_set0.set_value(half, 0, 8);
  x >>= 8;

  half = static_cast<uint8_t>(x & 0xFF);
  Byte to_set1(data + 4);
  to_set1.set_value(half, 0, 8);
}


Adubrakectrl8c04eb0a* Adubrakectrl8c04eb0a::set_adu_brake2_air_pressure(
    int adu_brake2_air_pressure) {
  adu_brake2_air_pressure_ = adu_brake2_air_pressure;
  return this;
}

// config detail: {'name': 'AD_BrakePercent', 'offset': 0.0, 'precision': 1.0, 'len': 7, 'is_signed_var': False, 'physical_range': '[0|100]', 'bit': 33, 'type': 'int', 'order': 'intel', 'physical_unit': '%'}
void Adubrakectrl8c04eb0a::set_p_adu_brake2_air_pressure(uint8_t* data,
    int adu_brake2_air_pressure) {
  /* Modify@20201223: according to Rev13 matrix, unit change from /100 to /1000 */
  adu_brake2_air_pressure = ProtocolData::BoundedValue(0, 1000, adu_brake2_air_pressure);
  int x = adu_brake2_air_pressure;
  
  uint8_t half = 0;
  half = static_cast<uint8_t>(x & 0xFF);
  Byte to_set0(data + 5);
  to_set0.set_value(half, 0, 8);
  x >>= 8;

  half = static_cast<uint8_t>(x & 0xFF);
  Byte to_set1(data + 6);
  to_set1.set_value(half, 0, 8);
}

}  // namespace ch
}  // namespace canbus
}  // namespace apollo
