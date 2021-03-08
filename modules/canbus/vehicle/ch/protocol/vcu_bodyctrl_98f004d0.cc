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

#include "modules/canbus/vehicle/ch/protocol/vcu_bodyctrl_98f004d0.h"

#include "glog/logging.h"

#include "modules/drivers/canbus/common/byte.h"
#include "modules/drivers/canbus/common/canbus_consts.h"

namespace apollo {
namespace canbus {
namespace ch {

using ::apollo::drivers::canbus::Byte;

Vcubodyctrl98f004d0::Vcubodyctrl98f004d0() {}
const int32_t Vcubodyctrl98f004d0::ID = 0x98f004d0;

void Vcubodyctrl98f004d0::Parse(const std::uint8_t* bytes, int32_t length,
                         ChassisDetail* chassis) const {
  chassis->mutable_ch()->mutable_vcu_bodyctrl_98f004d0()->set_vcu_lightcounter(vcu_lightcounter(bytes, length));
  chassis->mutable_ch()->mutable_vcu_bodyctrl_98f004d0()->set_vcu_backwardlampstatus(vcu_backwardlampstatus(bytes, length));
  chassis->mutable_ch()->mutable_vcu_bodyctrl_98f004d0()->set_vcu_warninglampstatus(vcu_warninglampstatus(bytes, length));
  chassis->mutable_ch()->mutable_vcu_bodyctrl_98f004d0()->set_vcu_rearfoglampstatus(vcu_rearfoglampstatus(bytes, length));
  chassis->mutable_ch()->mutable_vcu_bodyctrl_98f004d0()->set_vcu_frontfoglampstatus(vcu_frontfoglampstatus(bytes, length));
  chassis->mutable_ch()->mutable_vcu_bodyctrl_98f004d0()->set_vcu_brakelampstatus(vcu_brakelampstatus(bytes, length));
  chassis->mutable_ch()->mutable_vcu_bodyctrl_98f004d0()->set_vcu_rightturnlampstatus(vcu_rightturnlampstatus(bytes, length));
  chassis->mutable_ch()->mutable_vcu_bodyctrl_98f004d0()->set_vcu_leftturnlampstatus(vcu_leftturnlampstatus(bytes, length));
  chassis->mutable_ch()->mutable_vcu_bodyctrl_98f004d0()->set_vcu_headlightfullbeamstatus(vcu_headlightfullbeamstatus(bytes, length));
  chassis->mutable_ch()->mutable_vcu_bodyctrl_98f004d0()->set_vcu_dippedheadlampstatus(vcu_dippedheadlampstatus(bytes, length));
  chassis->mutable_ch()->mutable_vcu_bodyctrl_98f004d0()->set_vcu_positionlampstatus(vcu_positionlampstatus(bytes, length));
}

// config detail: {'name': 'vcu_lightcounter', 'offset': 0.0, 'precision': 1.0, 'len': 4, 'is_signed_var': False, 'physical_range': '[0|15]', 'bit': 60, 'type': 'int', 'order': 'intel', 'physical_unit': ''}
int Vcubodyctrl98f004d0::vcu_lightcounter(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 7);
  int32_t x = t0.get_byte(4, 4);

  int ret = x;
  return ret;
}

// config detail: {'name': 'vcu_backwardlampstatus', 'enum': {0: 'VCU_BACKWARDLAMPSTATUS_CLOSE', 1: 'VCU_BACKWARDLAMPSTATUS_OPEN', 2: 'VCU_BACKWARDLAMPSTATUS_RESERVE1', 3: 'VCU_BACKWARDLAMPSTATUS_RESERVE2'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 34, 'type': 'enum', 'order': 'intel', 'physical_unit': ''}
Vcu_bodyctrl_98f004d0::Vcu_backwardlampstatusType Vcubodyctrl98f004d0::vcu_backwardlampstatus(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 4);
  int32_t x = t0.get_byte(2, 2);

  Vcu_bodyctrl_98f004d0::Vcu_backwardlampstatusType ret =  static_cast<Vcu_bodyctrl_98f004d0::Vcu_backwardlampstatusType>(x);
  return ret;
}

// config detail: {'name': 'vcu_warninglampstatus', 'enum': {0: 'VCU_WARNINGLAMPSTATUS_CLOSE', 1: 'VCU_WARNINGLAMPSTATUS_OPEN', 2: 'VCU_WARNINGLAMPSTATUS_RESERVE1', 3: 'VCU_WARNINGLAMPSTATUS_RESERVE2'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 32, 'type': 'enum', 'order': 'intel', 'physical_unit': ''}
Vcu_bodyctrl_98f004d0::Vcu_warninglampstatusType Vcubodyctrl98f004d0::vcu_warninglampstatus(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 4);
  int32_t x = t0.get_byte(0, 2);

  Vcu_bodyctrl_98f004d0::Vcu_warninglampstatusType ret =  static_cast<Vcu_bodyctrl_98f004d0::Vcu_warninglampstatusType>(x);
  return ret;
}

// config detail: {'name': 'vcu_rearfoglampstatus', 'enum': {0: 'VCU_REARFOGLAMPSTATUS_CLOSE', 1: 'VCU_REARFOGLAMPSTATUS_OPEN', 2: 'VCU_REARFOGLAMPSTATUS_RESERVE1', 3: 'VCU_REARFOGLAMPSTATUS_RESERVE2'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 30, 'type': 'enum', 'order': 'intel', 'physical_unit': ''}
Vcu_bodyctrl_98f004d0::Vcu_rearfoglampstatusType Vcubodyctrl98f004d0::vcu_rearfoglampstatus(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 3);
  int32_t x = t0.get_byte(6, 2);

  Vcu_bodyctrl_98f004d0::Vcu_rearfoglampstatusType ret =  static_cast<Vcu_bodyctrl_98f004d0::Vcu_rearfoglampstatusType>(x);
  return ret;
}

// config detail: {'name': 'vcu_frontfoglampstatus', 'enum': {0: 'VCU_FRONTFOGLAMPSTATUS_CLOSE', 1: 'VCU_FRONTFOGLAMPSTATUS_OPEN', 2: 'VCU_FRONTFOGLAMPSTATUS_RESERVE1', 3: 'VCU_FRONTFOGLAMPSTATUS_RESERVE2'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 28, 'type': 'enum', 'order': 'intel', 'physical_unit': ''}
Vcu_bodyctrl_98f004d0::Vcu_frontfoglampstatusType Vcubodyctrl98f004d0::vcu_frontfoglampstatus(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 3);
  int32_t x = t0.get_byte(4, 2);

  Vcu_bodyctrl_98f004d0::Vcu_frontfoglampstatusType ret =  static_cast<Vcu_bodyctrl_98f004d0::Vcu_frontfoglampstatusType>(x);
  return ret;
}

// config detail: {'name': 'vcu_brakelampstatus', 'enum': {0: 'VCU_BRAKELAMPSTATUS_CLOSE', 1: 'VCU_BRAKELAMPSTATUS_OPEN', 2: 'VCU_BRAKELAMPSTATUS_RESERVE1', 3: 'VCU_BRAKELAMPSTATUS_RESERVE2'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 26, 'type': 'enum', 'order': 'intel', 'physical_unit': ''}
Vcu_bodyctrl_98f004d0::Vcu_brakelampstatusType Vcubodyctrl98f004d0::vcu_brakelampstatus(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 3);
  int32_t x = t0.get_byte(2, 2);

  Vcu_bodyctrl_98f004d0::Vcu_brakelampstatusType ret =  static_cast<Vcu_bodyctrl_98f004d0::Vcu_brakelampstatusType>(x);
  return ret;
}

// config detail: {'name': 'vcu_rightturnlampstatus', 'enum': {0: 'VCU_RIGHTTURNLAMPSTATUS_CLOSE', 1: 'VCU_RIGHTTURNLAMPSTATUS_OPEN', 2: 'VCU_RIGHTTURNLAMPSTATUS_RESERVE1', 3: 'VCU_RIGHTTURNLAMPSTATUS_RESERVE2'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 24, 'type': 'enum', 'order': 'intel', 'physical_unit': ''}
Vcu_bodyctrl_98f004d0::Vcu_rightturnlampstatusType Vcubodyctrl98f004d0::vcu_rightturnlampstatus(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 3);
  int32_t x = t0.get_byte(0, 2);

  Vcu_bodyctrl_98f004d0::Vcu_rightturnlampstatusType ret =  static_cast<Vcu_bodyctrl_98f004d0::Vcu_rightturnlampstatusType>(x);
  return ret;
}

// config detail: {'name': 'vcu_leftturnlampstatus', 'enum': {0: 'VCU_LEFTTURNLAMPSTATUS_CLOSE', 1: 'VCU_LEFTTURNLAMPSTATUS_OPEN', 2: 'VCU_LEFTTURNLAMPSTATUS_RESERVE1', 3: 'VCU_LEFTTURNLAMPSTATUS_RESERVE2'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 22, 'type': 'enum', 'order': 'intel', 'physical_unit': ''}
Vcu_bodyctrl_98f004d0::Vcu_leftturnlampstatusType Vcubodyctrl98f004d0::vcu_leftturnlampstatus(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 2);
  int32_t x = t0.get_byte(6, 2);

  Vcu_bodyctrl_98f004d0::Vcu_leftturnlampstatusType ret =  static_cast<Vcu_bodyctrl_98f004d0::Vcu_leftturnlampstatusType>(x);
  return ret;
}

// config detail: {'name': 'vcu_headlightfullbeamstatus', 'enum': {0: 'VCU_HEADLIGHTFULLBEAMSTATUS_CLOSE', 1: 'VCU_HEADLIGHTFULLBEAMSTATUS_OPEN', 2: 'VCU_HEADLIGHTFULLBEAMSTATUS_RESERVE1', 3: 'VCU_HEADLIGHTFULLBEAMSTATUS_RESERVE2'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 20, 'type': 'enum', 'order': 'intel', 'physical_unit': ''}
Vcu_bodyctrl_98f004d0::Vcu_headlightfullbeamstatusType Vcubodyctrl98f004d0::vcu_headlightfullbeamstatus(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 2);
  int32_t x = t0.get_byte(4, 2);

  Vcu_bodyctrl_98f004d0::Vcu_headlightfullbeamstatusType ret =  static_cast<Vcu_bodyctrl_98f004d0::Vcu_headlightfullbeamstatusType>(x);
  return ret;
}

// config detail: {'name': 'vcu_dippedheadlampstatus', 'enum': {0: 'VCU_DIPPEDHEADLAMPSTATUS_CLOSE', 1: 'VCU_DIPPEDHEADLAMPSTATUS_OPEN', 2: 'VCU_DIPPEDHEADLAMPSTATUS_RESERVE1', 3: 'VCU_DIPPEDHEADLAMPSTATUS_RESERVE2'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 18, 'type': 'enum', 'order': 'intel', 'physical_unit': ''}
Vcu_bodyctrl_98f004d0::Vcu_dippedheadlampstatusType Vcubodyctrl98f004d0::vcu_dippedheadlampstatus(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 2);
  int32_t x = t0.get_byte(2, 2);

  Vcu_bodyctrl_98f004d0::Vcu_dippedheadlampstatusType ret =  static_cast<Vcu_bodyctrl_98f004d0::Vcu_dippedheadlampstatusType>(x);
  return ret;
}

// config detail: {'name': 'vcu_positionlampstatus', 'enum': {0: 'VCU_POSITIONLAMPSTATUS_CLOSE', 1: 'VCU_POSITIONLAMPSTATUS_OPEN', 2: 'VCU_POSITIONLAMPSTATUS_RESERVE1', 3: 'VCU_POSITIONLAMPSTATUS_RESERVE2'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 16, 'type': 'enum', 'order': 'intel', 'physical_unit': ''}
Vcu_bodyctrl_98f004d0::Vcu_positionlampstatusType Vcubodyctrl98f004d0::vcu_positionlampstatus(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 2);
  int32_t x = t0.get_byte(0, 2);

  Vcu_bodyctrl_98f004d0::Vcu_positionlampstatusType ret =  static_cast<Vcu_bodyctrl_98f004d0::Vcu_positionlampstatusType>(x);
  return ret;
}
}  // namespace ch
}  // namespace canbus
}  // namespace apollo
