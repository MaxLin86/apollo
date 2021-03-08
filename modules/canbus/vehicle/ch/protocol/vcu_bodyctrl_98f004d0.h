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

class Vcubodyctrl98f004d0 : public ::apollo::drivers::canbus::ProtocolData<
                    ::apollo::canbus::ChassisDetail> {
 public:
  static const int32_t ID;
  Vcubodyctrl98f004d0();
  void Parse(const std::uint8_t* bytes, int32_t length,
                     ChassisDetail* chassis) const override;

 private:

  // config detail: {'name': 'VCU_LightCounter', 'offset': 0.0, 'precision': 1.0, 'len': 4, 'is_signed_var': False, 'physical_range': '[0|15]', 'bit': 60, 'type': 'int', 'order': 'intel', 'physical_unit': ''}
  int vcu_lightcounter(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'VCU_BackwardLampStatus', 'enum': {0: 'VCU_BACKWARDLAMPSTATUS_CLOSE', 1: 'VCU_BACKWARDLAMPSTATUS_OPEN', 2: 'VCU_BACKWARDLAMPSTATUS_RESERVE1', 3: 'VCU_BACKWARDLAMPSTATUS_RESERVE2'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 34, 'type': 'enum', 'order': 'intel', 'physical_unit': ''}
  Vcu_bodyctrl_98f004d0::Vcu_backwardlampstatusType vcu_backwardlampstatus(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'VCU_WarningLampStatus', 'enum': {0: 'VCU_WARNINGLAMPSTATUS_CLOSE', 1: 'VCU_WARNINGLAMPSTATUS_OPEN', 2: 'VCU_WARNINGLAMPSTATUS_RESERVE1', 3: 'VCU_WARNINGLAMPSTATUS_RESERVE2'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 32, 'type': 'enum', 'order': 'intel', 'physical_unit': ''}
  Vcu_bodyctrl_98f004d0::Vcu_warninglampstatusType vcu_warninglampstatus(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'VCU_RearFogLampStatus', 'enum': {0: 'VCU_REARFOGLAMPSTATUS_CLOSE', 1: 'VCU_REARFOGLAMPSTATUS_OPEN', 2: 'VCU_REARFOGLAMPSTATUS_RESERVE1', 3: 'VCU_REARFOGLAMPSTATUS_RESERVE2'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 30, 'type': 'enum', 'order': 'intel', 'physical_unit': ''}
  Vcu_bodyctrl_98f004d0::Vcu_rearfoglampstatusType vcu_rearfoglampstatus(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'VCU_FrontFogLampStatus', 'enum': {0: 'VCU_FRONTFOGLAMPSTATUS_CLOSE', 1: 'VCU_FRONTFOGLAMPSTATUS_OPEN', 2: 'VCU_FRONTFOGLAMPSTATUS_RESERVE1', 3: 'VCU_FRONTFOGLAMPSTATUS_RESERVE2'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 28, 'type': 'enum', 'order': 'intel', 'physical_unit': ''}
  Vcu_bodyctrl_98f004d0::Vcu_frontfoglampstatusType vcu_frontfoglampstatus(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'VCU_BrakeLampStatus', 'enum': {0: 'VCU_BRAKELAMPSTATUS_CLOSE', 1: 'VCU_BRAKELAMPSTATUS_OPEN', 2: 'VCU_BRAKELAMPSTATUS_RESERVE1', 3: 'VCU_BRAKELAMPSTATUS_RESERVE2'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 26, 'type': 'enum', 'order': 'intel', 'physical_unit': ''}
  Vcu_bodyctrl_98f004d0::Vcu_brakelampstatusType vcu_brakelampstatus(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'VCU_RightTurnLampStatus', 'enum': {0: 'VCU_RIGHTTURNLAMPSTATUS_CLOSE', 1: 'VCU_RIGHTTURNLAMPSTATUS_OPEN', 2: 'VCU_RIGHTTURNLAMPSTATUS_RESERVE1', 3: 'VCU_RIGHTTURNLAMPSTATUS_RESERVE2'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 24, 'type': 'enum', 'order': 'intel', 'physical_unit': ''}
  Vcu_bodyctrl_98f004d0::Vcu_rightturnlampstatusType vcu_rightturnlampstatus(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'VCU_LeftTurnLampStatus', 'enum': {0: 'VCU_LEFTTURNLAMPSTATUS_CLOSE', 1: 'VCU_LEFTTURNLAMPSTATUS_OPEN', 2: 'VCU_LEFTTURNLAMPSTATUS_RESERVE1', 3: 'VCU_LEFTTURNLAMPSTATUS_RESERVE2'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 22, 'type': 'enum', 'order': 'intel', 'physical_unit': ''}
  Vcu_bodyctrl_98f004d0::Vcu_leftturnlampstatusType vcu_leftturnlampstatus(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'VCU_HeadLightFullBeamStatus', 'enum': {0: 'VCU_HEADLIGHTFULLBEAMSTATUS_CLOSE', 1: 'VCU_HEADLIGHTFULLBEAMSTATUS_OPEN', 2: 'VCU_HEADLIGHTFULLBEAMSTATUS_RESERVE1', 3: 'VCU_HEADLIGHTFULLBEAMSTATUS_RESERVE2'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 20, 'type': 'enum', 'order': 'intel', 'physical_unit': ''}
  Vcu_bodyctrl_98f004d0::Vcu_headlightfullbeamstatusType vcu_headlightfullbeamstatus(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'VCU_DippedHeadLampStatus', 'enum': {0: 'VCU_DIPPEDHEADLAMPSTATUS_CLOSE', 1: 'VCU_DIPPEDHEADLAMPSTATUS_OPEN', 2: 'VCU_DIPPEDHEADLAMPSTATUS_RESERVE1', 3: 'VCU_DIPPEDHEADLAMPSTATUS_RESERVE2'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 18, 'type': 'enum', 'order': 'intel', 'physical_unit': ''}
  Vcu_bodyctrl_98f004d0::Vcu_dippedheadlampstatusType vcu_dippedheadlampstatus(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'VCU_PositionLampStatus', 'enum': {0: 'VCU_POSITIONLAMPSTATUS_CLOSE', 1: 'VCU_POSITIONLAMPSTATUS_OPEN', 2: 'VCU_POSITIONLAMPSTATUS_RESERVE1', 3: 'VCU_POSITIONLAMPSTATUS_RESERVE2'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 16, 'type': 'enum', 'order': 'intel', 'physical_unit': ''}
  Vcu_bodyctrl_98f004d0::Vcu_positionlampstatusType vcu_positionlampstatus(const std::uint8_t* bytes, const int32_t length) const;
};

}  // namespace ch
}  // namespace canbus
}  // namespace apollo


