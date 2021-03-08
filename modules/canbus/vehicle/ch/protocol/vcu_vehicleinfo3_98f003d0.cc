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

#include "modules/canbus/vehicle/ch/protocol/vcu_vehicleinfo3_98f003d0.h"

#include "glog/logging.h"

#include "modules/drivers/canbus/common/byte.h"
#include "modules/drivers/canbus/common/canbus_consts.h"

namespace apollo {
namespace canbus {
namespace ch {

using ::apollo::drivers::canbus::Byte;

Vcuvehicleinfo398f003d0::Vcuvehicleinfo398f003d0() {}
const int32_t Vcuvehicleinfo398f003d0::ID = 0x98f003d0;

void Vcuvehicleinfo398f003d0::Parse(const std::uint8_t* bytes, int32_t length,
                         ChassisDetail* chassis) const {
  chassis->mutable_ch()->mutable_vcu_vehicleinfo3_98f003d0()->set_vcu_lifecounter(vcu_lifecounter(bytes, length));
  chassis->mutable_ch()->mutable_vcu_vehicleinfo3_98f003d0()->set_vcu_batterytemp(vcu_batterytemp(bytes, length));
  chassis->mutable_ch()->mutable_vcu_vehicleinfo3_98f003d0()->set_vcu_batteryvoltage(vcu_batteryvoltage(bytes, length));
  chassis->mutable_ch()->mutable_vcu_vehicleinfo3_98f003d0()->set_vcu_lvbusvoltage(vcu_lvbusvoltage(bytes, length));
  chassis->mutable_ch()->mutable_vcu_vehicleinfo3_98f003d0()->set_vcu_vehiclespeed(vcu_vehiclespeed(bytes, length));
  chassis->mutable_ch()->mutable_vcu_vehicleinfo3_98f003d0()->set_vcu_coolantstatus(vcu_coolantstatus(bytes, length));
  chassis->mutable_ch()->mutable_vcu_vehicleinfo3_98f003d0()->set_vcu_vehiclepowerstatus(vcu_vehiclepowerstatus(bytes, length));
  chassis->mutable_ch()->mutable_vcu_vehicleinfo3_98f003d0()->set_vcu_gearstatus(vcu_gearstatus(bytes, length));
}

// config detail: {'name': 'vcu_lifecounter', 'offset': 0.0, 'precision': 1.0, 'len': 4, 'is_signed_var': False, 'physical_range': '[0|15]', 'bit': 60, 'type': 'int', 'order': 'intel', 'physical_unit': ''}
int Vcuvehicleinfo398f003d0::vcu_lifecounter(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 7);
  int32_t x = t0.get_byte(4, 4);

  int ret = x;
  return ret;
}

// config detail: {'name': 'vcu_batterytemp', 'offset': -100.0, 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'physical_range': '[-100|155]', 'bit': 48, 'type': 'int', 'order': 'intel', 'physical_unit': '\xa1\xe6'}
int Vcuvehicleinfo398f003d0::vcu_batterytemp(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 6);
  int32_t x = t0.get_byte(0, 8);

  int ret = x + -100.000000;
  return ret;
}

// config detail: {'name': 'vcu_batteryvoltage', 'offset': 0.0, 'precision': 3.0, 'len': 8, 'is_signed_var': False, 'physical_range': '[0|765]', 'bit': 40, 'type': 'double', 'order': 'intel', 'physical_unit': 'V'}
double Vcuvehicleinfo398f003d0::vcu_batteryvoltage(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 5);
  int32_t x = t0.get_byte(0, 8);

  double ret = x * 3.000000;
  return ret;
}

// config detail: {'name': 'vcu_lvbusvoltage', 'offset': 0.0, 'precision': 0.2, 'len': 8, 'is_signed_var': False, 'physical_range': '[0|40]', 'bit': 32, 'type': 'double', 'order': 'intel', 'physical_unit': 'V'}
double Vcuvehicleinfo398f003d0::vcu_lvbusvoltage(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 4);
  int32_t x = t0.get_byte(0, 8);

  double ret = x * 0.200000;
  return ret;
}

// config detail: {'name': 'vcu_vehiclespeed', 'offset': 0.0, 'precision': 0.00390625, 'len': 16, 'is_signed_var': False, 'physical_range': '[0|255]', 'bit': 8, 'type': 'double', 'order': 'intel', 'physical_unit': 'km/h'}
double Vcuvehicleinfo398f003d0::vcu_vehiclespeed(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 2);
  int32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 1);
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  double ret = x * 0.003906;
  return ret;
}

// config detail: {'name': 'vcu_coolantstatus', 'enum': {0: 'VCU_COOLANTSTATUS_NORMAL', 1: 'VCU_COOLANTSTATUS_COOLANT_LOW', 2: 'VCU_COOLANTSTATUS_ERROR_INDICATOR', 3: 'VCU_COOLANTSTATUS_NOT_AVAILABLE'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 4, 'type': 'enum', 'order': 'intel', 'physical_unit': ''}
Vcu_vehicleinfo3_98f003d0::Vcu_coolantstatusType Vcuvehicleinfo398f003d0::vcu_coolantstatus(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(4, 2);

  Vcu_vehicleinfo3_98f003d0::Vcu_coolantstatusType ret =  static_cast<Vcu_vehicleinfo3_98f003d0::Vcu_coolantstatusType>(x);
  return ret;
}

// config detail: {'name': 'vcu_vehiclepowerstatus', 'enum': {0: 'VCU_VEHICLEPOWERSTATUS_LV_UP', 1: 'VCU_VEHICLEPOWERSTATUS_HV_UP_WITH_FAULT', 2: 'VCU_VEHICLEPOWERSTATUS_READY', 3: 'VCU_VEHICLEPOWERSTATUS_OTHER'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 2, 'type': 'enum', 'order': 'intel', 'physical_unit': ''}
Vcu_vehicleinfo3_98f003d0::Vcu_vehiclepowerstatusType Vcuvehicleinfo398f003d0::vcu_vehiclepowerstatus(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(2, 2);

  Vcu_vehicleinfo3_98f003d0::Vcu_vehiclepowerstatusType ret =  static_cast<Vcu_vehicleinfo3_98f003d0::Vcu_vehiclepowerstatusType>(x);
  return ret;
}

// config detail: {'name': 'vcu_gearstatus', 'enum': {0: 'VCU_GEARSTATUS_NEUTRAL', 1: 'VCU_GEARSTATUS_DRIVE', 2: 'VCU_GEARSTATUS_REVERSE', 3: 'VCU_GEARSTATUS_PARK'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 0, 'type': 'enum', 'order': 'intel', 'physical_unit': ''}
Vcu_vehicleinfo3_98f003d0::Vcu_gearstatusType Vcuvehicleinfo398f003d0::vcu_gearstatus(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(0, 2);

  Vcu_vehicleinfo3_98f003d0::Vcu_gearstatusType ret =  static_cast<Vcu_vehicleinfo3_98f003d0::Vcu_gearstatusType>(x);
  return ret;
}
}  // namespace ch
}  // namespace canbus
}  // namespace apollo
