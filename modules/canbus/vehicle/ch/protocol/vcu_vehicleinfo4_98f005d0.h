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

class Vcuvehicleinfo498f005d0 : public ::apollo::drivers::canbus::ProtocolData<
                    ::apollo::canbus::ChassisDetail> {
 public:
  static const int32_t ID;
  Vcuvehicleinfo498f005d0();
  void Parse(const std::uint8_t* bytes, int32_t length,
                     ChassisDetail* chassis) const override;

 private:

  // config detail: {'name': 'VCU_VerNum4', 'offset': 0.0, 'precision': 1.0, 'len': 16, 'is_signed_var': False, 'physical_range': '[0|65535]', 'bit': 48, 'type': 'int', 'order': 'intel', 'physical_unit': ''}
  int vcu_vehCumMileage(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'VCU_VerNum3', 'offset': 0.0, 'precision': 1.0, 'len': 16, 'is_signed_var': False, 'physical_range': '[0|65535]', 'bit': 32, 'type': 'int', 'order': 'intel', 'physical_unit': ''}
  int vcu_brakeAirPressure(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'VCU_VerNum2', 'offset': 0.0, 'precision': 1.0, 'len': 16, 'is_signed_var': False, 'physical_range': '[0|65535]', 'bit': 16, 'type': 'int', 'order': 'intel', 'physical_unit': ''}
  int vcu_motorBrakeTorque(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'VCU_VerNum1', 'offset': 0.0, 'precision': 1.0, 'len': 16, 'is_signed_var': False, 'physical_range': '[0|65535]', 'bit': 0, 'type': 'int', 'order': 'intel', 'physical_unit': ''}
  int vcu_torqueReqPercent(const std::uint8_t* bytes, const int32_t length) const;
};

}  // namespace ch
}  // namespace canbus
}  // namespace apollo


