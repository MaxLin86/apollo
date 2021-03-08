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

#include "modules/canbus/vehicle/ch/protocol/vcu_vehicleinfo2_98f002d0.h"

#include "glog/logging.h"

#include "modules/drivers/canbus/common/byte.h"
#include "modules/drivers/canbus/common/canbus_consts.h"

namespace apollo {
namespace canbus {
namespace ch {

using ::apollo::drivers::canbus::Byte;

Vcuvehicleinfo298f002d0::Vcuvehicleinfo298f002d0() {}
const int32_t Vcuvehicleinfo298f002d0::ID = 0x98f002d0;

void Vcuvehicleinfo298f002d0::Parse(const std::uint8_t* bytes, int32_t length,
                         ChassisDetail* chassis) const {
  chassis->mutable_ch()->mutable_vcu_vehicleinfo2_98f002d0()->set_vcu_vinnum16(vcu_vinnum16(bytes, length));
  chassis->mutable_ch()->mutable_vcu_vehicleinfo2_98f002d0()->set_vcu_vinnum15(vcu_vinnum15(bytes, length));
  chassis->mutable_ch()->mutable_vcu_vehicleinfo2_98f002d0()->set_vcu_vinnum14(vcu_vinnum14(bytes, length));
  chassis->mutable_ch()->mutable_vcu_vehicleinfo2_98f002d0()->set_vcu_vinnum13(vcu_vinnum13(bytes, length));
  chassis->mutable_ch()->mutable_vcu_vehicleinfo2_98f002d0()->set_vcu_vinnum12(vcu_vinnum12(bytes, length));
  chassis->mutable_ch()->mutable_vcu_vehicleinfo2_98f002d0()->set_vcu_vinnum11(vcu_vinnum11(bytes, length));
  chassis->mutable_ch()->mutable_vcu_vehicleinfo2_98f002d0()->set_vcu_vinnum10(vcu_vinnum10(bytes, length));
  chassis->mutable_ch()->mutable_vcu_vehicleinfo2_98f002d0()->set_vcu_vinnum9(vcu_vinnum9(bytes, length));
  chassis->mutable_ch()->mutable_vcu_vehicleinfo2_98f002d0()->set_vcu_vinnum8(vcu_vinnum8(bytes, length));
  chassis->mutable_ch()->mutable_vcu_vehicleinfo2_98f002d0()->set_vcu_vinnum7(vcu_vinnum7(bytes, length));
  chassis->mutable_ch()->mutable_vcu_vehicleinfo2_98f002d0()->set_vcu_vinnum6(vcu_vinnum6(bytes, length));
  chassis->mutable_ch()->mutable_vcu_vehicleinfo2_98f002d0()->set_vcu_vinnum5(vcu_vinnum5(bytes, length));
  chassis->mutable_ch()->mutable_vcu_vehicleinfo2_98f002d0()->set_vcu_vinnum4(vcu_vinnum4(bytes, length));
  chassis->mutable_ch()->mutable_vcu_vehicleinfo2_98f002d0()->set_vcu_vinnum3(vcu_vinnum3(bytes, length));
  chassis->mutable_ch()->mutable_vcu_vehicleinfo2_98f002d0()->set_vcu_vinnum2(vcu_vinnum2(bytes, length));
  chassis->mutable_ch()->mutable_vcu_vehicleinfo2_98f002d0()->set_vcu_vinnum1(vcu_vinnum1(bytes, length));
}

// config detail: {'name': 'vcu_vinnum16', 'offset': 0.0, 'precision': 1.0, 'len': 4, 'is_signed_var': False, 'physical_range': '[0|15]', 'bit': 60, 'type': 'int', 'order': 'intel', 'physical_unit': ''}
int Vcuvehicleinfo298f002d0::vcu_vinnum16(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 7);
  int32_t x = t0.get_byte(4, 4);

  int ret = x;
  return ret;
}

// config detail: {'name': 'vcu_vinnum15', 'offset': 0.0, 'precision': 1.0, 'len': 4, 'is_signed_var': False, 'physical_range': '[0|15]', 'bit': 56, 'type': 'int', 'order': 'intel', 'physical_unit': ''}
int Vcuvehicleinfo298f002d0::vcu_vinnum15(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 7);
  int32_t x = t0.get_byte(0, 4);

  int ret = x;
  return ret;
}

// config detail: {'name': 'vcu_vinnum14', 'offset': 0.0, 'precision': 1.0, 'len': 4, 'is_signed_var': False, 'physical_range': '[0|15]', 'bit': 52, 'type': 'int', 'order': 'intel', 'physical_unit': ''}
int Vcuvehicleinfo298f002d0::vcu_vinnum14(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 6);
  int32_t x = t0.get_byte(4, 4);

  int ret = x;
  return ret;
}

// config detail: {'name': 'vcu_vinnum13', 'offset': 0.0, 'precision': 1.0, 'len': 4, 'is_signed_var': False, 'physical_range': '[0|15]', 'bit': 48, 'type': 'int', 'order': 'intel', 'physical_unit': ''}
int Vcuvehicleinfo298f002d0::vcu_vinnum13(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 6);
  int32_t x = t0.get_byte(0, 4);

  int ret = x;
  return ret;
}

// config detail: {'name': 'vcu_vinnum12', 'offset': 0.0, 'precision': 1.0, 'len': 4, 'is_signed_var': False, 'physical_range': '[0|15]', 'bit': 44, 'type': 'int', 'order': 'intel', 'physical_unit': ''}
int Vcuvehicleinfo298f002d0::vcu_vinnum12(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 5);
  int32_t x = t0.get_byte(4, 4);

  int ret = x;
  return ret;
}

// config detail: {'name': 'vcu_vinnum11', 'offset': 0.0, 'precision': 1.0, 'len': 4, 'is_signed_var': False, 'physical_range': '[0|15]', 'bit': 40, 'type': 'int', 'order': 'intel', 'physical_unit': ''}
int Vcuvehicleinfo298f002d0::vcu_vinnum11(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 5);
  int32_t x = t0.get_byte(0, 4);

  int ret = x;
  return ret;
}

// config detail: {'name': 'vcu_vinnum10', 'offset': 0.0, 'precision': 1.0, 'len': 4, 'is_signed_var': False, 'physical_range': '[0|15]', 'bit': 36, 'type': 'int', 'order': 'intel', 'physical_unit': ''}
int Vcuvehicleinfo298f002d0::vcu_vinnum10(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 4);
  int32_t x = t0.get_byte(4, 4);

  int ret = x;
  return ret;
}

// config detail: {'name': 'vcu_vinnum9', 'offset': 0.0, 'precision': 1.0, 'len': 4, 'is_signed_var': False, 'physical_range': '[0|15]', 'bit': 32, 'type': 'int', 'order': 'intel', 'physical_unit': ''}
int Vcuvehicleinfo298f002d0::vcu_vinnum9(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 4);
  int32_t x = t0.get_byte(0, 4);

  int ret = x;
  return ret;
}

// config detail: {'name': 'vcu_vinnum8', 'offset': 0.0, 'precision': 1.0, 'len': 4, 'is_signed_var': False, 'physical_range': '[0|15]', 'bit': 28, 'type': 'int', 'order': 'intel', 'physical_unit': ''}
int Vcuvehicleinfo298f002d0::vcu_vinnum8(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 3);
  int32_t x = t0.get_byte(4, 4);

  int ret = x;
  return ret;
}

// config detail: {'name': 'vcu_vinnum7', 'offset': 0.0, 'precision': 1.0, 'len': 4, 'is_signed_var': False, 'physical_range': '[0|15]', 'bit': 24, 'type': 'int', 'order': 'intel', 'physical_unit': ''}
int Vcuvehicleinfo298f002d0::vcu_vinnum7(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 3);
  int32_t x = t0.get_byte(0, 4);

  int ret = x;
  return ret;
}

// config detail: {'name': 'vcu_vinnum6', 'offset': 0.0, 'precision': 1.0, 'len': 4, 'is_signed_var': False, 'physical_range': '[0|15]', 'bit': 20, 'type': 'int', 'order': 'intel', 'physical_unit': ''}
int Vcuvehicleinfo298f002d0::vcu_vinnum6(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 2);
  int32_t x = t0.get_byte(4, 4);

  int ret = x;
  return ret;
}

// config detail: {'name': 'vcu_vinnum5', 'offset': 0.0, 'precision': 1.0, 'len': 4, 'is_signed_var': False, 'physical_range': '[0|15]', 'bit': 16, 'type': 'int', 'order': 'intel', 'physical_unit': ''}
int Vcuvehicleinfo298f002d0::vcu_vinnum5(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 2);
  int32_t x = t0.get_byte(0, 4);

  int ret = x;
  return ret;
}

// config detail: {'name': 'vcu_vinnum4', 'offset': 0.0, 'precision': 1.0, 'len': 4, 'is_signed_var': False, 'physical_range': '[0|15]', 'bit': 12, 'type': 'int', 'order': 'intel', 'physical_unit': ''}
int Vcuvehicleinfo298f002d0::vcu_vinnum4(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(4, 4);

  int ret = x;
  return ret;
}

// config detail: {'name': 'vcu_vinnum3', 'offset': 0.0, 'precision': 1.0, 'len': 4, 'is_signed_var': False, 'physical_range': '[0|15]', 'bit': 8, 'type': 'int', 'order': 'intel', 'physical_unit': ''}
int Vcuvehicleinfo298f002d0::vcu_vinnum3(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(0, 4);

  int ret = x;
  return ret;
}

// config detail: {'name': 'vcu_vinnum2', 'offset': 0.0, 'precision': 1.0, 'len': 4, 'is_signed_var': False, 'physical_range': '[0|15]', 'bit': 4, 'type': 'int', 'order': 'intel', 'physical_unit': ''}
int Vcuvehicleinfo298f002d0::vcu_vinnum2(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(4, 4);

  int ret = x;
  return ret;
}

// config detail: {'name': 'vcu_vinnum1', 'offset': 0.0, 'precision': 1.0, 'len': 4, 'is_signed_var': False, 'physical_range': '[0|15]', 'bit': 0, 'type': 'int', 'order': 'intel', 'physical_unit': ''}
int Vcuvehicleinfo298f002d0::vcu_vinnum1(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(0, 4);

  int ret = x;
  return ret;
}
}  // namespace ch
}  // namespace canbus
}  // namespace apollo
