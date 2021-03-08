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

#include "modules/canbus/vehicle/ch/protocol/adu_steeringctrl_8cff1500.h"

#include "modules/drivers/canbus/common/byte.h"

namespace apollo {
namespace canbus {
namespace ch {

using ::apollo::drivers::canbus::Byte;

const int32_t Adusteeringctrl8cff1500::ID = 0x8cff1500;

// public
Adusteeringctrl8cff1500::Adusteeringctrl8cff1500() { Reset(); }

uint32_t Adusteeringctrl8cff1500::GetPeriod() const {
  // TODO(All) :  modify every protocol's period manually
  static const uint32_t PERIOD = 10 * 1000;
  return PERIOD;
}

void Adusteeringctrl8cff1500::UpdateData(uint8_t* data) {
  set_p_ad_counter(data, ad_counter_);
  set_p_ad_wheelspeed(data, ad_wheelspeed_);
  set_p_ad_turnaddedtorque(data, ad_turnaddedtorque_);
  set_p_ad_turncmdvalid(data, ad_turncmdvalid_);
  set_p_ad_adustatus(data, ad_adustatus_);
  set_p_ad_workmode(data, ad_workmode_);
  set_p_ad_steeringturnangle(data, ad_steeringturnangle_);
}

void Adusteeringctrl8cff1500::Reset() {
  // TODO(All) :  you should check this manually
  ad_counter_ = 0;
  ad_wheelspeed_ = 0.0;
  ad_turnaddedtorque_ = 0x80;
  ad_turncmdvalid_ = Adu_steeringctrl_8cff1500::AD_TURNCMDVALID_ABNORMAL;
  ad_adustatus_ = Adu_steeringctrl_8cff1500::AD_ADUSTATUS_ABNORMAL;
  ad_workmode_ = Adu_steeringctrl_8cff1500::AD_WORKMODE_AD_MODE;
  ad_steeringturnangle_ = 0.0;
}

Adusteeringctrl8cff1500* Adusteeringctrl8cff1500::set_ad_counter(
    int ad_counter) {
  ad_counter_ = ad_counter;
  return this;
 }

// config detail: {'name': 'AD_Counter', 'offset': 0.0, 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'physical_range': '[0|255]', 'bit': 56, 'type': 'int', 'order': 'intel', 'physical_unit': ''}
void Adusteeringctrl8cff1500::set_p_ad_counter(uint8_t* data,
    int ad_counter) {
  ad_counter = ProtocolData::BoundedValue(0, 255, ad_counter);
  int x = ad_counter;

  Byte to_set(data + 7);
  to_set.set_value(x, 0, 8);
}


Adusteeringctrl8cff1500* Adusteeringctrl8cff1500::set_ad_wheelspeed(
    double ad_wheelspeed) {
  ad_wheelspeed_ = ad_wheelspeed;
  return this;
 }

// config detail: {'name': 'AD_WheelSpeed', 'offset': 0.0, 'precision': 10.0, 'len': 8, 'is_signed_var': False, 'physical_range': '[50|540]', 'bit': 32, 'type': 'double', 'order': 'intel', 'physical_unit': '\xa1\xe3/s'}
void Adusteeringctrl8cff1500::set_p_ad_wheelspeed(uint8_t* data,
    double ad_wheelspeed) {
  ad_wheelspeed = ProtocolData::BoundedValue(50.0, 540.0, ad_wheelspeed);
  int x = ad_wheelspeed / 10.000000;

  Byte to_set(data + 4);
  to_set.set_value(x, 0, 8);
}


Adusteeringctrl8cff1500* Adusteeringctrl8cff1500::set_ad_turnaddedtorque(
    int ad_turnaddedtorque) {
  ad_turnaddedtorque_ = ad_turnaddedtorque;
  return this;
 }

// config detail: {'name': 'AD_TurnAddedTorque', 'offset': 0.0, 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'physical_range': '[0|255]', 'bit': 24, 'type': 'int', 'order': 'intel', 'physical_unit': ''}
void Adusteeringctrl8cff1500::set_p_ad_turnaddedtorque(uint8_t* data,
    int ad_turnaddedtorque) {
  ad_turnaddedtorque = ProtocolData::BoundedValue(0, 255, ad_turnaddedtorque);
  int x = ad_turnaddedtorque;

  Byte to_set(data + 3);
  to_set.set_value(x, 0, 8);
}


Adusteeringctrl8cff1500* Adusteeringctrl8cff1500::set_ad_turncmdvalid(
    Adu_steeringctrl_8cff1500::Ad_turncmdvalidType ad_turncmdvalid) {
  ad_turncmdvalid_ = ad_turncmdvalid;
  return this;
 }

// config detail: {'name': 'AD_TurnCmdValid', 'enum': {0: 'AD_TURNCMDVALID_ABNORMAL', 1: 'AD_TURNCMDVALID_NORMAL'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 21, 'type': 'enum', 'order': 'intel', 'physical_unit': ''}
void Adusteeringctrl8cff1500::set_p_ad_turncmdvalid(uint8_t* data,
    Adu_steeringctrl_8cff1500::Ad_turncmdvalidType ad_turncmdvalid) {
  int x = ad_turncmdvalid;

  Byte to_set(data + 2);
  to_set.set_value(x, 5, 1);
}


Adusteeringctrl8cff1500* Adusteeringctrl8cff1500::set_ad_adustatus(
    Adu_steeringctrl_8cff1500::Ad_adustatusType ad_adustatus) {
  ad_adustatus_ = ad_adustatus;
  return this;
 }

// config detail: {'name': 'AD_ADUStatus', 'enum': {0: 'AD_ADUSTATUS_ABNORMAL', 1: 'AD_ADUSTATUS_NORMAL'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 20, 'type': 'enum', 'order': 'intel', 'physical_unit': ''}
void Adusteeringctrl8cff1500::set_p_ad_adustatus(uint8_t* data,
    Adu_steeringctrl_8cff1500::Ad_adustatusType ad_adustatus) {
  int x = ad_adustatus;

  Byte to_set(data + 2);
  to_set.set_value(x, 4, 1);
}


Adusteeringctrl8cff1500* Adusteeringctrl8cff1500::set_ad_workmode(
    Adu_steeringctrl_8cff1500::Ad_workmodeType ad_workmode) {
  ad_workmode_ = ad_workmode;
  return this;
 }

// config detail: {'name': 'AD_WorkMode', 'enum': {0: 'AD_WORKMODE_READY', 1: 'AD_WORKMODE_AD_MODE', 2: 'AD_WORKMODE_RESERVE', 3: 'AD_WORKMODE_TORQUE_ADDED', 4: 'AD_WORKMODE_MANUAL_CONTROL', 5: 'AD_WORKMODE_MANUAL_RETURN', 6: 'AD_WORKMODE_FAULT_RESET'}, 'precision': 1.0, 'len': 4, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|15]', 'bit': 16, 'type': 'enum', 'order': 'intel', 'physical_unit': ''}
void Adusteeringctrl8cff1500::set_p_ad_workmode(uint8_t* data,
    Adu_steeringctrl_8cff1500::Ad_workmodeType ad_workmode) {
  int x = ad_workmode;

  Byte to_set(data + 2);
  to_set.set_value(x, 0, 4);
}


Adusteeringctrl8cff1500* Adusteeringctrl8cff1500::set_ad_steeringturnangle(
    double ad_steeringturnangle) {
  ad_steeringturnangle_ = ad_steeringturnangle;
  return this;
 }

// config detail: {'name': 'AD_SteeringTurnAngle', 'offset': -1575.0, 'precision': 0.1, 'len': 16, 'is_signed_var': False, 'physical_range': '[-850|850]', 'bit': 0, 'type': 'double', 'order': 'intel', 'physical_unit': '\xa1\xe3'}
void Adusteeringctrl8cff1500::set_p_ad_steeringturnangle(uint8_t* data,
    double ad_steeringturnangle) {
  // TODO  
  if(std::isnan(ad_steeringturnangle)){
    ad_steeringturnangle = 0.0;
  }
  ad_steeringturnangle = ProtocolData::BoundedValue(-850.0, 850.0, ad_steeringturnangle);  
  int x = (ad_steeringturnangle - -1575.000000) / 0.100000;
  uint8_t t = 0;

  t = x & 0xFF;
  Byte to_set0(data + 0);
  to_set0.set_value(t, 0, 8);
  x >>= 8;

  t = x & 0xFF;
  Byte to_set1(data + 1);
  to_set1.set_value(t, 0, 8);
}

}  // namespace ch
}  // namespace canbus
}  // namespace apollo
