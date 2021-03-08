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

#include "modules/canbus/vehicle/ch/protocol/vcu_vehicleinfo4_98f005d0.h"

#include "glog/logging.h"

#include "modules/drivers/canbus/common/byte.h"
#include "modules/drivers/canbus/common/canbus_consts.h"

namespace apollo {
namespace canbus {
namespace ch {

using ::apollo::drivers::canbus::Byte;

Vcuvehicleinfo498f005d0::Vcuvehicleinfo498f005d0() {}
const int32_t Vcuvehicleinfo498f005d0::ID = 0x98f005d0;

void Vcuvehicleinfo498f005d0::Parse(const std::uint8_t* bytes, int32_t length,
                         ChassisDetail* chassis) const {
  chassis->mutable_ch()->mutable_vcu_vehicleinfo4_98f005d0()->set_vcu_vernum4(vcu_vernum4(bytes, length));
  chassis->mutable_ch()->mutable_vcu_vehicleinfo4_98f005d0()->set_vcu_vernum3(vcu_vernum3(bytes, length));
  chassis->mutable_ch()->mutable_vcu_vehicleinfo4_98f005d0()->set_vcu_vernum2(vcu_vernum2(bytes, length));
  chassis->mutable_ch()->mutable_vcu_vehicleinfo4_98f005d0()->set_vcu_vernum1(vcu_vernum1(bytes, length));
}

// config detail: {'name': 'vcu_vernum4', 'offset': 0.0, 'precision': 1.0, 'len': 16, 'is_signed_var': False, 'physical_range': '[0|65535]', 'bit': 48, 'type': 'int', 'order': 'intel', 'physical_unit': ''}
int Vcuvehicleinfo498f005d0::vcu_vernum4(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 7);
  int32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 6);
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  int ret = x;
  return ret;
}

// config detail: {'name': 'vcu_vernum3', 'offset': 0.0, 'precision': 1.0, 'len': 16, 'is_signed_var': False, 'physical_range': '[0|65535]', 'bit': 32, 'type': 'int', 'order': 'intel', 'physical_unit': ''}
int Vcuvehicleinfo498f005d0::vcu_vernum3(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 5);
  int32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 4);
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  int ret = x;
  return ret;
}

// config detail: {'name': 'vcu_vernum2', 'offset': 0.0, 'precision': 1.0, 'len': 16, 'is_signed_var': False, 'physical_range': '[0|65535]', 'bit': 16, 'type': 'int', 'order': 'intel', 'physical_unit': ''}
int Vcuvehicleinfo498f005d0::vcu_vernum2(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 3);
  int32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 2);
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  int ret = x;
  return ret;
}

// config detail: {'name': 'vcu_vernum1', 'offset': 0.0, 'precision': 1.0, 'len': 16, 'is_signed_var': False, 'physical_range': '[0|65535]', 'bit': 0, 'type': 'int', 'order': 'intel', 'physical_unit': ''}
int Vcuvehicleinfo498f005d0::vcu_vernum1(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 0);
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  int ret = x;
  return ret;
}
}  // namespace ch
}  // namespace canbus
}  // namespace apollo
