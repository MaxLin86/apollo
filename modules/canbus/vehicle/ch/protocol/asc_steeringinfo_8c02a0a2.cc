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

#include "modules/canbus/vehicle/ch/protocol/asc_steeringinfo_8c02a0a2.h"

#include "glog/logging.h"

#include "modules/drivers/canbus/common/byte.h"
#include "modules/drivers/canbus/common/canbus_consts.h"

namespace apollo {
namespace canbus {
namespace ch {

using ::apollo::drivers::canbus::Byte;

Ascsteeringinfo8c02a0a2::Ascsteeringinfo8c02a0a2() {}
const int32_t Ascsteeringinfo8c02a0a2::ID = 0x8c02a0a2;

void Ascsteeringinfo8c02a0a2::Parse(const std::uint8_t* bytes, int32_t length,
                         ChassisDetail* chassis) const {
  chassis->mutable_ch()->mutable_asc_steeringinfo_8c02a0a2()->set_asc_counter(asc_counter(bytes, length));
  chassis->mutable_ch()->mutable_asc_steeringinfo_8c02a0a2()->set_asc_faultlevel(asc_faultlevel(bytes, length));
  chassis->mutable_ch()->mutable_asc_steeringinfo_8c02a0a2()->set_asc_ascstatus(asc_ascstatus(bytes, length));
  chassis->mutable_ch()->mutable_asc_steeringinfo_8c02a0a2()->set_asc_wheeltorque(asc_wheeltorque(bytes, length));
  chassis->mutable_ch()->mutable_asc_steeringinfo_8c02a0a2()->set_asc_wheelspeedfeedback(asc_wheelspeedfeedback(bytes, length));
  chassis->mutable_ch()->mutable_asc_steeringinfo_8c02a0a2()->set_asc_assisttorque(asc_assisttorque(bytes, length));
  chassis->mutable_ch()->mutable_asc_steeringinfo_8c02a0a2()->set_asc_steeringwheelturnangle(asc_steeringwheelturnangle(bytes, length));
}

// config detail: {'name': 'asc_counter', 'offset': 0.0, 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'physical_range': '[0|255]', 'bit': 56, 'type': 'int', 'order': 'intel', 'physical_unit': ''}
int Ascsteeringinfo8c02a0a2::asc_counter(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 7);
  int32_t x = t0.get_byte(0, 8);

  int ret = x;
  return ret;
}

// config detail: {'name': 'asc_faultlevel', 'enum': {0: 'ASC_FAULTLEVEL_NO_FAILURE', 1: 'ASC_FAULTLEVEL_L1_FAULT', 2: 'ASC_FAULTLEVEL_L2_FAULT'}, 'precision': 1.0, 'len': 4, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|15]', 'bit': 52, 'type': 'enum', 'order': 'intel', 'physical_unit': ''}
Asc_steeringinfo_8c02a0a2::Asc_faultlevelType Ascsteeringinfo8c02a0a2::asc_faultlevel(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 6);
  int32_t x = t0.get_byte(4, 4);

  Asc_steeringinfo_8c02a0a2::Asc_faultlevelType ret =  static_cast<Asc_steeringinfo_8c02a0a2::Asc_faultlevelType>(x);
  return ret;
}

// config detail: {'name': 'asc_ascstatus', 'enum': {0: 'ASC_ASCSTATUS_MANUAL_CONTROL_MODE', 1: 'ASC_ASCSTATUS_AD_MODE', 2: 'ASC_ASCSTATUS_RESERVE', 3: 'ASC_ASCSTATUS_TORQUE_ADDED_MODE', 4: 'ASC_ASCSTATUS_EPS_ASSIST_MODE', 5: 'ASC_ASCSTATUS_HUMAN_INTERRUPT_MODE', 6: 'ASC_ASCSTATUS_WARNING_MODE', 7: 'ASC_ASCSTATUS_FAULT'}, 'precision': 1.0, 'len': 4, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|15]', 'bit': 48, 'type': 'enum', 'order': 'intel', 'physical_unit': ''}
Asc_steeringinfo_8c02a0a2::Asc_ascstatusType Ascsteeringinfo8c02a0a2::asc_ascstatus(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 6);
  int32_t x = t0.get_byte(0, 4);

  Asc_steeringinfo_8c02a0a2::Asc_ascstatusType ret =  static_cast<Asc_steeringinfo_8c02a0a2::Asc_ascstatusType>(x);
  return ret;
}

// config detail: {'name': 'asc_wheeltorque', 'offset': 0.0, 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'physical_range': '[0|255]', 'bit': 40, 'type': 'int', 'order': 'intel', 'physical_unit': ''}
int Ascsteeringinfo8c02a0a2::asc_wheeltorque(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 5);
  int32_t x = t0.get_byte(0, 8);

  int ret = x;
  return ret;
}

// config detail: {'name': 'asc_wheelspeedfeedback', 'offset': 0.0, 'precision': 10.0, 'len': 8, 'is_signed_var': False, 'physical_range': '[50|540]', 'bit': 32, 'type': 'double', 'order': 'intel', 'physical_unit': '\xa1\xe3/s'}
double Ascsteeringinfo8c02a0a2::asc_wheelspeedfeedback(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 4);
  int32_t x = t0.get_byte(0, 8);

  double ret = x * 10.000000;
  return ret;
}

// config detail: {'name': 'asc_assisttorque', 'offset': -5.0, 'precision': 0.000152, 'len': 16, 'is_signed_var': False, 'physical_range': '[-5|5]', 'bit': 16, 'type': 'double', 'order': 'intel', 'physical_unit': 'nm'}
double Ascsteeringinfo8c02a0a2::asc_assisttorque(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 3);
  int32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 2);
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  double ret = x * 0.000152 + -5.000000;
  return ret;
}

// config detail: {'name': 'asc_steeringwheelturnangle', 'offset': -1575.0, 'precision': 0.1, 'len': 16, 'is_signed_var': False, 'physical_range': '[-1575|1575]', 'bit': 0, 'type': 'double', 'order': 'intel', 'physical_unit': '\xa1\xe3'}
double Ascsteeringinfo8c02a0a2::asc_steeringwheelturnangle(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 0);
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  double ret = x * 0.100000 + -1575.000000;
  return ret;
}
}  // namespace ch
}  // namespace canbus
}  // namespace apollo
