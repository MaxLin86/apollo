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

class Ascsteeringinfo8c02a0a2 : public ::apollo::drivers::canbus::ProtocolData<
                    ::apollo::canbus::ChassisDetail> {
 public:
  static const int32_t ID;
  Ascsteeringinfo8c02a0a2();
  void Parse(const std::uint8_t* bytes, int32_t length,
                     ChassisDetail* chassis) const override;

 private:

  // config detail: {'name': 'ASC_Counter', 'offset': 0.0, 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'physical_range': '[0|255]', 'bit': 56, 'type': 'int', 'order': 'intel', 'physical_unit': ''}
  int asc_counter(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'ASC_FaultLevel', 'enum': {0: 'ASC_FAULTLEVEL_NO_FAILURE', 1: 'ASC_FAULTLEVEL_L1_FAULT', 2: 'ASC_FAULTLEVEL_L2_FAULT'}, 'precision': 1.0, 'len': 4, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|15]', 'bit': 52, 'type': 'enum', 'order': 'intel', 'physical_unit': ''}
  Asc_steeringinfo_8c02a0a2::Asc_faultlevelType asc_faultlevel(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'ASC_ASCStatus', 'enum': {0: 'ASC_ASCSTATUS_MANUAL_CONTROL_MODE', 1: 'ASC_ASCSTATUS_AD_MODE', 2: 'ASC_ASCSTATUS_RESERVE', 3: 'ASC_ASCSTATUS_TORQUE_ADDED_MODE', 4: 'ASC_ASCSTATUS_EPS_ASSIST_MODE', 5: 'ASC_ASCSTATUS_HUMAN_INTERRUPT_MODE', 6: 'ASC_ASCSTATUS_WARNING_MODE', 7: 'ASC_ASCSTATUS_FAULT'}, 'precision': 1.0, 'len': 4, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|15]', 'bit': 48, 'type': 'enum', 'order': 'intel', 'physical_unit': ''}
  Asc_steeringinfo_8c02a0a2::Asc_ascstatusType asc_ascstatus(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'ASC_WheelTorque', 'offset': 0.0, 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'physical_range': '[0|255]', 'bit': 40, 'type': 'int', 'order': 'intel', 'physical_unit': ''}
  int asc_wheeltorque(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'ASC_WheelSpeedFeedback', 'offset': 0.0, 'precision': 10.0, 'len': 8, 'is_signed_var': False, 'physical_range': '[50|540]', 'bit': 32, 'type': 'double', 'order': 'intel', 'physical_unit': '\xa1\xe3/s'}
  double asc_wheelspeedfeedback(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'ASC_AssistTorque', 'offset': -5.0, 'precision': 0.000152, 'len': 16, 'is_signed_var': False, 'physical_range': '[-5|5]', 'bit': 16, 'type': 'double', 'order': 'intel', 'physical_unit': 'nm'}
  double asc_assisttorque(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'ASC_SteeringWheelTurnAngle', 'offset': -1575.0, 'precision': 0.1, 'len': 16, 'is_signed_var': False, 'physical_range': '[-1575|1575]', 'bit': 0, 'type': 'double', 'order': 'intel', 'physical_unit': '\xa1\xe3'}
  double asc_steeringwheelturnangle(const std::uint8_t* bytes, const int32_t length) const;
};

}  // namespace ch
}  // namespace canbus
}  // namespace apollo


