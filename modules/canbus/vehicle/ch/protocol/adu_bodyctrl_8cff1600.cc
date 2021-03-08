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

#include "modules/canbus/vehicle/ch/protocol/adu_bodyctrl_8cff1600.h"

#include "modules/drivers/canbus/common/byte.h"

namespace apollo {
namespace canbus {
namespace ch {

using ::apollo::drivers::canbus::Byte;

const int32_t Adubodyctrl8cff1600::ID = 0x8cff1600;

// public
Adubodyctrl8cff1600::Adubodyctrl8cff1600() { Reset(); }

uint32_t Adubodyctrl8cff1600::GetPeriod() const {
  // TODO(All) :  modify every protocol's period manually
  static const uint32_t PERIOD = 200 * 1000;
  return PERIOD;
}

void Adubodyctrl8cff1600::UpdateData(uint8_t* data) {
  set_p_ad_rearfoglampreq(data, ad_rearfoglampreq_);
  set_p_ad_frontfoglampreq(data, ad_frontfoglampreq_);
  set_p_ad_headlightfullbeamreq(data, ad_headlightfullbeamreq_);
  set_p_ad_dippedheadlampreq(data, ad_dippedheadlampreq_);
  set_p_ad_positionlampreq(data, ad_positionlampreq_);
  set_p_ad_rightturnlampreq(data, ad_rightturnlampreq_);
  set_p_ad_leftturnlampreq(data, ad_leftturnlampreq_);
  set_p_ad_warninglampreq(data, ad_warninglampreq_);
}

void Adubodyctrl8cff1600::Reset() {
  // TODO(All) :  you should check this manually
  ad_rearfoglampreq_ = Adu_bodyctrl_8cff1600::AD_REARFOGLAMPREQ_CLOSE;
  ad_frontfoglampreq_ = Adu_bodyctrl_8cff1600::AD_FRONTFOGLAMPREQ_CLOSE;
  ad_headlightfullbeamreq_ = Adu_bodyctrl_8cff1600::AD_HEADLIGHTFULLBEAMREQ_CLOSE;
  ad_dippedheadlampreq_ = Adu_bodyctrl_8cff1600::AD_DIPPEDHEADLAMPREQ_OPEN;
  ad_positionlampreq_ = Adu_bodyctrl_8cff1600::AD_POSITIONLAMPREQ_CLOSE;
  ad_rightturnlampreq_ = Adu_bodyctrl_8cff1600::AD_RIGHTTURNLAMPREQ_CLOSE;
  ad_leftturnlampreq_ = Adu_bodyctrl_8cff1600::AD_LEFTTURNLAMPREQ_CLOSE;
  ad_warninglampreq_ = Adu_bodyctrl_8cff1600::AD_WARNINGLAMPREQ_CLOSE;
}

Adubodyctrl8cff1600* Adubodyctrl8cff1600::set_ad_rearfoglampreq(
    Adu_bodyctrl_8cff1600::Ad_rearfoglampreqType ad_rearfoglampreq) {
  ad_rearfoglampreq_ = ad_rearfoglampreq;
  return this;
 }

// config detail: {'name': 'AD_RearFogLampReq', 'enum': {0: 'AD_REARFOGLAMPREQ_CLOSE', 1: 'AD_REARFOGLAMPREQ_OPEN', 2: 'AD_REARFOGLAMPREQ_RESERVE1', 3: 'AD_REARFOGLAMPREQ_RESERVE2'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 26, 'type': 'enum', 'order': 'intel', 'physical_unit': ''}
void Adubodyctrl8cff1600::set_p_ad_rearfoglampreq(uint8_t* data,
    Adu_bodyctrl_8cff1600::Ad_rearfoglampreqType ad_rearfoglampreq) {
  int x = ad_rearfoglampreq;

  Byte to_set(data + 3);
  to_set.set_value(x, 2, 2);
}


Adubodyctrl8cff1600* Adubodyctrl8cff1600::set_ad_frontfoglampreq(
    Adu_bodyctrl_8cff1600::Ad_frontfoglampreqType ad_frontfoglampreq) {
  ad_frontfoglampreq_ = ad_frontfoglampreq;
  return this;
 }

// config detail: {'name': 'AD_FrontFogLampReq', 'enum': {0: 'AD_FRONTFOGLAMPREQ_CLOSE', 1: 'AD_FRONTFOGLAMPREQ_OPEN', 2: 'AD_FRONTFOGLAMPREQ_RESERVE1', 3: 'AD_FRONTFOGLAMPREQ_RESERVE2'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 24, 'type': 'enum', 'order': 'intel', 'physical_unit': ''}
void Adubodyctrl8cff1600::set_p_ad_frontfoglampreq(uint8_t* data,
    Adu_bodyctrl_8cff1600::Ad_frontfoglampreqType ad_frontfoglampreq) {
  int x = ad_frontfoglampreq;

  Byte to_set(data + 3);
  to_set.set_value(x, 0, 2);
}


Adubodyctrl8cff1600* Adubodyctrl8cff1600::set_ad_headlightfullbeamreq(
    Adu_bodyctrl_8cff1600::Ad_headlightfullbeamreqType ad_headlightfullbeamreq) {
  ad_headlightfullbeamreq_ = ad_headlightfullbeamreq;
  return this;
 }

// config detail: {'name': 'AD_HeadLightFullBeamReq', 'enum': {0: 'AD_HEADLIGHTFULLBEAMREQ_CLOSE', 1: 'AD_HEADLIGHTFULLBEAMREQ_OPEN', 2: 'AD_HEADLIGHTFULLBEAMREQ_RESERVE1', 3: 'AD_HEADLIGHTFULLBEAMREQ_RESERVE2'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 22, 'type': 'enum', 'order': 'intel', 'physical_unit': ''}
void Adubodyctrl8cff1600::set_p_ad_headlightfullbeamreq(uint8_t* data,
    Adu_bodyctrl_8cff1600::Ad_headlightfullbeamreqType ad_headlightfullbeamreq) {
  int x = ad_headlightfullbeamreq;

  Byte to_set(data + 2);
  to_set.set_value(x, 6, 2);
}


Adubodyctrl8cff1600* Adubodyctrl8cff1600::set_ad_dippedheadlampreq(
    Adu_bodyctrl_8cff1600::Ad_dippedheadlampreqType ad_dippedheadlampreq) {
  ad_dippedheadlampreq_ = ad_dippedheadlampreq;
  return this;
 }

// config detail: {'name': 'AD_DippedHeadLampReq', 'enum': {0: 'AD_DIPPEDHEADLAMPREQ_CLOSE', 1: 'AD_DIPPEDHEADLAMPREQ_OPEN', 2: 'AD_DIPPEDHEADLAMPREQ_RESERVE1', 3: 'AD_DIPPEDHEADLAMPREQ_RESERVE2'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 20, 'type': 'enum', 'order': 'intel', 'physical_unit': ''}
void Adubodyctrl8cff1600::set_p_ad_dippedheadlampreq(uint8_t* data,
    Adu_bodyctrl_8cff1600::Ad_dippedheadlampreqType ad_dippedheadlampreq) {
  int x = ad_dippedheadlampreq;

  Byte to_set(data + 2);
  to_set.set_value(x, 4, 2);
}


Adubodyctrl8cff1600* Adubodyctrl8cff1600::set_ad_positionlampreq(
    Adu_bodyctrl_8cff1600::Ad_positionlampreqType ad_positionlampreq) {
  ad_positionlampreq_ = ad_positionlampreq;
  return this;
 }

// config detail: {'name': 'AD_PositionLampReq', 'enum': {0: 'AD_POSITIONLAMPREQ_CLOSE', 1: 'AD_POSITIONLAMPREQ_OPEN', 2: 'AD_POSITIONLAMPREQ_RESERVE1', 3: 'AD_POSITIONLAMPREQ_RESERVE2'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 18, 'type': 'enum', 'order': 'intel', 'physical_unit': ''}
void Adubodyctrl8cff1600::set_p_ad_positionlampreq(uint8_t* data,
    Adu_bodyctrl_8cff1600::Ad_positionlampreqType ad_positionlampreq) {
  int x = ad_positionlampreq;

  Byte to_set(data + 2);
  to_set.set_value(x, 2, 2);
}


Adubodyctrl8cff1600* Adubodyctrl8cff1600::set_ad_rightturnlampreq(
    Adu_bodyctrl_8cff1600::Ad_rightturnlampreqType ad_rightturnlampreq) {
  ad_rightturnlampreq_ = ad_rightturnlampreq;
  return this;
 }

// config detail: {'name': 'AD_RightTurnLampReq', 'enum': {0: 'AD_RIGHTTURNLAMPREQ_CLOSE', 1: 'AD_RIGHTTURNLAMPREQ_OPEN', 2: 'AD_RIGHTTURNLAMPREQ_RESERVE1', 3: 'AD_RIGHTTURNLAMPREQ_RESERVE2'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 12, 'type': 'enum', 'order': 'intel', 'physical_unit': ''}
void Adubodyctrl8cff1600::set_p_ad_rightturnlampreq(uint8_t* data,
    Adu_bodyctrl_8cff1600::Ad_rightturnlampreqType ad_rightturnlampreq) {
  int x = ad_rightturnlampreq;

  Byte to_set(data + 1);
  to_set.set_value(x, 4, 2);
}


Adubodyctrl8cff1600* Adubodyctrl8cff1600::set_ad_leftturnlampreq(
    Adu_bodyctrl_8cff1600::Ad_leftturnlampreqType ad_leftturnlampreq) {
  ad_leftturnlampreq_ = ad_leftturnlampreq;
  return this;
 }

// config detail: {'name': 'AD_LeftTurnLampReq', 'enum': {0: 'AD_LEFTTURNLAMPREQ_CLOSE', 1: 'AD_LEFTTURNLAMPREQ_OPEN', 2: 'AD_LEFTTURNLAMPREQ_RESERVE1', 3: 'AD_LEFTTURNLAMPREQ_RESERVE2'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 10, 'type': 'enum', 'order': 'intel', 'physical_unit': ''}
void Adubodyctrl8cff1600::set_p_ad_leftturnlampreq(uint8_t* data,
    Adu_bodyctrl_8cff1600::Ad_leftturnlampreqType ad_leftturnlampreq) {
  int x = ad_leftturnlampreq;

  Byte to_set(data + 1);
  to_set.set_value(x, 2, 2);
}


Adubodyctrl8cff1600* Adubodyctrl8cff1600::set_ad_warninglampreq(
    Adu_bodyctrl_8cff1600::Ad_warninglampreqType ad_warninglampreq) {
  ad_warninglampreq_ = ad_warninglampreq;
  return this;
 }

// config detail: {'name': 'AD_WarningLampReq', 'enum': {0: 'AD_WARNINGLAMPREQ_CLOSE', 1: 'AD_WARNINGLAMPREQ_OPEN', 2: 'AD_WARNINGLAMPREQ_RESERVE1', 3: 'AD_WARNINGLAMPREQ_RESERVE2'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 8, 'type': 'enum', 'order': 'intel', 'physical_unit': ''}
void Adubodyctrl8cff1600::set_p_ad_warninglampreq(uint8_t* data,
    Adu_bodyctrl_8cff1600::Ad_warninglampreqType ad_warninglampreq) {
  int x = ad_warninglampreq;

  Byte to_set(data + 1);
  to_set.set_value(x, 0, 2);
}

}  // namespace ch
}  // namespace canbus
}  // namespace apollo
