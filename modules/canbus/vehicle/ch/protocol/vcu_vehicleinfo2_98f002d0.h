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

#pragma once

#include "modules/drivers/canbus/can_comm/protocol_data.h"
#include "modules/canbus/proto/chassis_detail.pb.h"

namespace apollo {
namespace canbus {
namespace ch {

class Vcuvehicleinfo298f002d0 : public ::apollo::drivers::canbus::ProtocolData<
                    ::apollo::canbus::ChassisDetail> {
 public:
  static const int32_t ID;
  Vcuvehicleinfo298f002d0();
  void Parse(const std::uint8_t* bytes, int32_t length,
                     ChassisDetail* chassis) const override;

 private:

  // config detail: {'name': 'VCU_VINNum16', 'offset': 0.0, 'precision': 1.0, 'len': 4, 'is_signed_var': False, 'physical_range': '[0|15]', 'bit': 60, 'type': 'int', 'order': 'intel', 'physical_unit': ''}
  int vcu_vinnum16(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'VCU_VINNum15', 'offset': 0.0, 'precision': 1.0, 'len': 4, 'is_signed_var': False, 'physical_range': '[0|15]', 'bit': 56, 'type': 'int', 'order': 'intel', 'physical_unit': ''}
  int vcu_vinnum15(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'VCU_VINNum14', 'offset': 0.0, 'precision': 1.0, 'len': 4, 'is_signed_var': False, 'physical_range': '[0|15]', 'bit': 52, 'type': 'int', 'order': 'intel', 'physical_unit': ''}
  int vcu_vinnum14(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'VCU_VINNum13', 'offset': 0.0, 'precision': 1.0, 'len': 4, 'is_signed_var': False, 'physical_range': '[0|15]', 'bit': 48, 'type': 'int', 'order': 'intel', 'physical_unit': ''}
  int vcu_vinnum13(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'VCU_VINNum12', 'offset': 0.0, 'precision': 1.0, 'len': 4, 'is_signed_var': False, 'physical_range': '[0|15]', 'bit': 44, 'type': 'int', 'order': 'intel', 'physical_unit': ''}
  int vcu_vinnum12(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'VCU_VINNum11', 'offset': 0.0, 'precision': 1.0, 'len': 4, 'is_signed_var': False, 'physical_range': '[0|15]', 'bit': 40, 'type': 'int', 'order': 'intel', 'physical_unit': ''}
  int vcu_vinnum11(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'VCU_VINNum10', 'offset': 0.0, 'precision': 1.0, 'len': 4, 'is_signed_var': False, 'physical_range': '[0|15]', 'bit': 36, 'type': 'int', 'order': 'intel', 'physical_unit': ''}
  int vcu_vinnum10(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'VCU_VINNum9', 'offset': 0.0, 'precision': 1.0, 'len': 4, 'is_signed_var': False, 'physical_range': '[0|15]', 'bit': 32, 'type': 'int', 'order': 'intel', 'physical_unit': ''}
  int vcu_vinnum9(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'VCU_VINNum8', 'offset': 0.0, 'precision': 1.0, 'len': 4, 'is_signed_var': False, 'physical_range': '[0|15]', 'bit': 28, 'type': 'int', 'order': 'intel', 'physical_unit': ''}
  int vcu_vinnum8(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'VCU_VINNum7', 'offset': 0.0, 'precision': 1.0, 'len': 4, 'is_signed_var': False, 'physical_range': '[0|15]', 'bit': 24, 'type': 'int', 'order': 'intel', 'physical_unit': ''}
  int vcu_vinnum7(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'VCU_VINNum6', 'offset': 0.0, 'precision': 1.0, 'len': 4, 'is_signed_var': False, 'physical_range': '[0|15]', 'bit': 20, 'type': 'int', 'order': 'intel', 'physical_unit': ''}
  int vcu_vinnum6(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'VCU_VINNum5', 'offset': 0.0, 'precision': 1.0, 'len': 4, 'is_signed_var': False, 'physical_range': '[0|15]', 'bit': 16, 'type': 'int', 'order': 'intel', 'physical_unit': ''}
  int vcu_vinnum5(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'VCU_VINNum4', 'offset': 0.0, 'precision': 1.0, 'len': 4, 'is_signed_var': False, 'physical_range': '[0|15]', 'bit': 12, 'type': 'int', 'order': 'intel', 'physical_unit': ''}
  int vcu_vinnum4(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'VCU_VINNum3', 'offset': 0.0, 'precision': 1.0, 'len': 4, 'is_signed_var': False, 'physical_range': '[0|15]', 'bit': 8, 'type': 'int', 'order': 'intel', 'physical_unit': ''}
  int vcu_vinnum3(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'VCU_VINNum2', 'offset': 0.0, 'precision': 1.0, 'len': 4, 'is_signed_var': False, 'physical_range': '[0|15]', 'bit': 4, 'type': 'int', 'order': 'intel', 'physical_unit': ''}
  int vcu_vinnum2(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'VCU_VINNum1', 'offset': 0.0, 'precision': 1.0, 'len': 4, 'is_signed_var': False, 'physical_range': '[0|15]', 'bit': 0, 'type': 'int', 'order': 'intel', 'physical_unit': ''}
  int vcu_vinnum1(const std::uint8_t* bytes, const int32_t length) const;
};

}  // namespace ch
}  // namespace canbus
}  // namespace apollo


