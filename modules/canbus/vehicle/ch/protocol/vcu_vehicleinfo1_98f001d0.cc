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

#include "modules/canbus/vehicle/ch/protocol/vcu_vehicleinfo1_98f001d0.h"

#include "glog/logging.h"

#include "modules/drivers/canbus/common/byte.h"
#include "modules/drivers/canbus/common/canbus_consts.h"

namespace apollo {
namespace canbus {
namespace ch {

using ::apollo::drivers::canbus::Byte;

Vcuvehicleinfo198f001d0::Vcuvehicleinfo198f001d0() {}
const int32_t Vcuvehicleinfo198f001d0::ID = 0x98f001d0;

void Vcuvehicleinfo198f001d0::Parse(const std::uint8_t* bytes, int32_t length,
                         ChassisDetail* chassis) const {
  chassis->mutable_ch()->mutable_vcu_vehicleinfo1_98f001d0()->set_vcu_dtc(vcu_dtc(bytes, length));
  chassis->mutable_ch()->mutable_vcu_vehicleinfo1_98f001d0()->set_vcu_cumulatedmileage(vcu_cumulatedmileage(bytes, length));
  chassis->mutable_ch()->mutable_vcu_vehicleinfo1_98f001d0()->set_vcu_modulestatus(vcu_modulestatus(bytes, length));
  chassis->mutable_ch()->mutable_vcu_vehicleinfo1_98f001d0()->set_vcu_batterysoh(vcu_batterysoh(bytes, length));
  chassis->mutable_ch()->mutable_vcu_vehicleinfo1_98f001d0()->set_vcu_batterysoc(vcu_batterysoc(bytes, length));
  chassis->mutable_ch()->mutable_vcu_vehicleinfo1_98f001d0()->set_vcu_availabletorque(vcu_availabletorque(bytes, length));
  chassis->mutable_ch()->mutable_vcu_vehicleinfo1_98f001d0()->set_vcu_vehiclemode(vcu_vehiclemode(bytes, length));
  chassis->mutable_ch()->mutable_vcu_vehicleinfo1_98f001d0()->set_vcu_vehicletype(vcu_vehicletype(bytes, length));
}

// config detail: {'name': 'vcu_dtc', 'offset': 0.0, 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'physical_range': '[0|255]', 'bit': 40, 'type': 'int', 'order': 'intel', 'physical_unit': ''}
int Vcuvehicleinfo198f001d0::vcu_dtc(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 5);
  int32_t x = t0.get_byte(0, 8);

  int ret = x;
  return ret;
}

// config detail: {'name': 'vcu_cumulatedmileage', 'offset': 0.0, 'precision': 20.0, 'len': 16, 'is_signed_var': False, 'physical_range': '[0|1310000]', 'bit': 48, 'type': 'double', 'order': 'intel', 'physical_unit': 'km'}
double Vcuvehicleinfo198f001d0::vcu_cumulatedmileage(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 7);
  int32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 6);
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  double ret = x * 20.000000;
  return ret;
}

// config detail: {'name': 'vcu_modulestatus', 'enum': {0: 'VCU_MODULESTATUS_NORMAL', 1: 'VCU_MODULESTATUS_L1_WARNING', 2: 'VCU_MODULESTATUS_L2_FAULT', 3: 'VCU_MODULESTATUS_L3_FAULT', 4: 'VCU_MODULESTATUS_OTHER'}, 'precision': 1.0, 'len': 3, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|7]', 'bit': 32, 'type': 'enum', 'order': 'intel', 'physical_unit': ''}
Vcu_vehicleinfo1_98f001d0::Vcu_modulestatusType Vcuvehicleinfo198f001d0::vcu_modulestatus(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 4);
  int32_t x = t0.get_byte(0, 3);

  Vcu_vehicleinfo1_98f001d0::Vcu_modulestatusType ret =  static_cast<Vcu_vehicleinfo1_98f001d0::Vcu_modulestatusType>(x);
  return ret;
}

// config detail: {'name': 'vcu_batterysoh', 'offset': 0.0, 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'physical_range': '[0|100]', 'bit': 24, 'type': 'int', 'order': 'intel', 'physical_unit': '%'}
int Vcuvehicleinfo198f001d0::vcu_batterysoh(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 3);
  int32_t x = t0.get_byte(0, 8);

  int ret = x;
  return ret;
}

// config detail: {'name': 'vcu_batterysoc', 'offset': 0.0, 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'physical_range': '[0|100]', 'bit': 16, 'type': 'int', 'order': 'intel', 'physical_unit': '%'}
int Vcuvehicleinfo198f001d0::vcu_batterysoc(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 2);
  int32_t x = t0.get_byte(0, 8);

  int ret = x;
  return ret;
}

// config detail: {'name': 'vcu_availabletorque', 'offset': 0.0, 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'physical_range': '[0|100]', 'bit': 8, 'type': 'int', 'order': 'intel', 'physical_unit': '%'}
int Vcuvehicleinfo198f001d0::vcu_availabletorque(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(0, 8);

  int ret = x;
  return ret;
}

// config detail: {'name': 'vcu_vehiclemode', 'enum': {0: 'VCU_VEHICLEMODE_CHARGE', 1: 'VCU_VEHICLEMODE_AD_TRACTION', 2: 'VCU_VEHICLEMODE_MANUAL_CONTROL', 3: 'VCU_VEHICLEMODE_PARK'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 5, 'type': 'enum', 'order': 'intel', 'physical_unit': ''}
Vcu_vehicleinfo1_98f001d0::Vcu_vehiclemodeType Vcuvehicleinfo198f001d0::vcu_vehiclemode(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(5, 2);

  Vcu_vehicleinfo1_98f001d0::Vcu_vehiclemodeType ret =  static_cast<Vcu_vehicleinfo1_98f001d0::Vcu_vehiclemodeType>(x);
  return ret;
}

// config detail: {'name': 'vcu_vehicletype', 'enum': {0: 'VCU_VEHICLETYPE_AVG', 1: 'VCU_VEHICLETYPE_TRUCK_40', 2: 'VCU_VEHICLETYPE_TRUCK_20'}, 'precision': 1.0, 'len': 5, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|31]', 'bit': 0, 'type': 'enum', 'order': 'intel', 'physical_unit': ''}
Vcu_vehicleinfo1_98f001d0::Vcu_vehicletypeType Vcuvehicleinfo198f001d0::vcu_vehicletype(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(0, 5);

  Vcu_vehicleinfo1_98f001d0::Vcu_vehicletypeType ret =  static_cast<Vcu_vehicleinfo1_98f001d0::Vcu_vehicletypeType>(x);
  return ret;
}
}  // namespace ch
}  // namespace canbus
}  // namespace apollo
