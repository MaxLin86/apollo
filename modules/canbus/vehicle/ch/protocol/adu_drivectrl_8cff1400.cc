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

#include "modules/canbus/vehicle/ch/protocol/adu_drivectrl_8cff1400.h"

#include "modules/drivers/canbus/common/byte.h"

namespace apollo {
namespace canbus {
namespace ch {

using ::apollo::drivers::canbus::Byte;

const int32_t Adudrivectrl8cff1400::ID = 0x8cff1400;

// public
Adudrivectrl8cff1400::Adudrivectrl8cff1400() { Reset(); }

uint32_t Adudrivectrl8cff1400::GetPeriod() const {
  // TODO(All) :  modify every protocol's period manually
  static const uint32_t PERIOD = 10 * 1000;
  return PERIOD;
}

void Adudrivectrl8cff1400::UpdateData(uint8_t* data) {
  set_p_ad_vcuswverreq(data, ad_vcuswverreq_);
  set_p_ad_vehiclespeed(data, ad_vehiclespeed_);
  set_p_ad_dtc(data, ad_dtc_);
  set_p_ad_adcstatus(data, ad_adcstatus_);
  set_p_ad_epb(data, ad_epb_);
  set_p_ad_brakepercent(data, ad_brakepercent_);
  set_p_ad_brakereq(data, ad_brakereq_);
  set_p_ad_maxspeedlimitreq(data, ad_maxspeedlimitreq_);
  set_p_ad_torquepercent(data, ad_torquepercent_);
  set_p_ad_gearreq(data, ad_gearreq_);
  set_p_ad_statusreq(data, ad_statusreq_);
}

void Adudrivectrl8cff1400::Reset() {
  // TODO(All) :  you should check this manually
  ad_vcuswverreq_ = Adu_drivectrl_8cff1400::AD_VCUSWVERREQ_NO_REQUEST;
  ad_vehiclespeed_ = 0;
  ad_dtc_ = 0;
  ad_adcstatus_ = Adu_drivectrl_8cff1400::AD_ADCSTATUS_NORMAL;
  ad_epb_ = Adu_drivectrl_8cff1400::AD_EPB_STANDBY;
  // Modified by shijh
  ad_brakepercent_ = 500;
  ad_brakereq_ = Adu_drivectrl_8cff1400::AD_BRAKEREQ_APPLY_BRAKE;
  // Modified by shijh
  ad_maxspeedlimitreq_ = 27.0; //20.0; 
  ad_torquepercent_ = 0;
  ad_gearreq_ = Adu_drivectrl_8cff1400::AD_GEARREQ_NATURAL;
  ad_statusreq_ = Adu_drivectrl_8cff1400::AD_STATUSREQ_VEHICLE_IGNITION_REQUEST;
}

Adudrivectrl8cff1400* Adudrivectrl8cff1400::set_ad_vcuswverreq(
    Adu_drivectrl_8cff1400::Ad_vcuswverreqType ad_vcuswverreq) {
  ad_vcuswverreq_ = ad_vcuswverreq;
  return this;
 }

// config detail: {'name': 'AD_VCUSWVerReq', 'enum': {0: 'AD_VCUSWVERREQ_NO_REQUEST', 1: 'AD_VCUSWVERREQ_REQUEST'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 24, 'type': 'enum', 'order': 'intel', 'physical_unit': ''}
void Adudrivectrl8cff1400::set_p_ad_vcuswverreq(uint8_t* data,
    Adu_drivectrl_8cff1400::Ad_vcuswverreqType ad_vcuswverreq) {
  int x = ad_vcuswverreq;
  
  Byte to_set(data + 3);
  to_set.set_value(x, 0, 1);
}


Adudrivectrl8cff1400* Adudrivectrl8cff1400::set_ad_vehiclespeed(
    int ad_vehiclespeed) {
  ad_vehiclespeed_ = ad_vehiclespeed;
  return this;
 }

// config detail: {'name': 'AD_VehicleSpeed', 'offset': 0.0, 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'physical_range': '[0|200]', 'bit': 56, 'type': 'int', 'order': 'intel', 'physical_unit': 'km/h'}
void Adudrivectrl8cff1400::set_p_ad_vehiclespeed(uint8_t* data,
    int ad_vehiclespeed) {
  ad_vehiclespeed = ProtocolData::BoundedValue(0, 200, ad_vehiclespeed);
  int x = ad_vehiclespeed;

  Byte to_set(data + 7);
  to_set.set_value(x, 0, 8);
}


Adudrivectrl8cff1400* Adudrivectrl8cff1400::set_ad_dtc(
    int ad_dtc) {
  ad_dtc_ = ad_dtc;
  return this;
 }

// config detail: {'name': 'AD_DTC', 'offset': 0.0, 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'physical_range': '[0|255]', 'bit': 48, 'type': 'int', 'order': 'intel', 'physical_unit': ''}
void Adudrivectrl8cff1400::set_p_ad_dtc(uint8_t* data,
    int ad_dtc) {
  ad_dtc = ProtocolData::BoundedValue(0, 255, ad_dtc);
  int x = ad_dtc;

  Byte to_set(data + 6);
  to_set.set_value(x, 0, 8);
}


Adudrivectrl8cff1400* Adudrivectrl8cff1400::set_ad_adcstatus(
    Adu_drivectrl_8cff1400::Ad_adcstatusType ad_adcstatus) {
  ad_adcstatus_ = ad_adcstatus;
  return this;
 }

// config detail: {'name': 'AD_ADCStatus', 'enum': {0: 'AD_ADCSTATUS_NORMAL', 1: 'AD_ADCSTATUS_L1_WARNING', 2: 'AD_ADCSTATUS_L2_FAULT', 3: 'AD_ADCSTATUS_L3_FAULT', 4: 'AD_ADCSTATUS_OTHERS'}, 'precision': 1.0, 'len': 3, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|4]', 'bit': 41, 'type': 'enum', 'order': 'intel', 'physical_unit': ''}
void Adudrivectrl8cff1400::set_p_ad_adcstatus(uint8_t* data,
    Adu_drivectrl_8cff1400::Ad_adcstatusType ad_adcstatus) {
  int x = ad_adcstatus;

  Byte to_set(data + 5);
  to_set.set_value(x, 1, 3);
}


Adudrivectrl8cff1400* Adudrivectrl8cff1400::set_ad_epb(
    Adu_drivectrl_8cff1400::Ad_epbType ad_epb) {
  ad_epb_ = ad_epb;
  return this;
 }

// config detail: {'name': 'AD_EPB', 'enum': {0: 'AD_EPB_STANDBY', 1: 'AD_EPB_APPLY_PARKING_BRAKE'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 40, 'type': 'enum', 'order': 'intel', 'physical_unit': ''}
void Adudrivectrl8cff1400::set_p_ad_epb(uint8_t* data,
    Adu_drivectrl_8cff1400::Ad_epbType ad_epb) {
  int x = ad_epb;

  Byte to_set(data + 5);
  to_set.set_value(x, 0, 1);
}


Adudrivectrl8cff1400* Adudrivectrl8cff1400::set_ad_brakepercent(
    int ad_brakepercent) {
  ad_brakepercent_ = ad_brakepercent;
  return this;
 }

// config detail: {'name': 'AD_BrakePercent', 'offset': 0.0, 'precision': 1.0, 'len': 7, 'is_signed_var': False, 'physical_range': '[0|100]', 'bit': 33, 'type': 'int', 'order': 'intel', 'physical_unit': '%'}
void Adudrivectrl8cff1400::set_p_ad_brakepercent(uint8_t* data,
    int ad_brakepercent) {
  /* Modify@20201223: according to Rev13 matrix, unit change from /100 to /1000 */
  ad_brakepercent = ProtocolData::BoundedValue(0, 1000, ad_brakepercent);
  int x = ad_brakepercent;
  
  uint8_t half = 0;
  half = static_cast<uint8_t>(x & 0x0F);
  Byte to_set0(data + 3);
  to_set0.set_value(half, 4, 4);
  x >>= 4;

  half = static_cast<uint8_t>(x & 0xFF);
  Byte to_set1(data + 4);
  to_set1.set_value(half, 0, 8);
}


Adudrivectrl8cff1400* Adudrivectrl8cff1400::set_ad_brakereq(
    Adu_drivectrl_8cff1400::Ad_brakereqType ad_brakereq) {
  ad_brakereq_ = ad_brakereq;
  return this;
 }

// config detail: {'name': 'AD_BrakeReq', 'enum': {0: 'AD_BRAKEREQ_NO_BRAKE', 1: 'AD_BRAKEREQ_APPLY_BRAKE'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 32, 'type': 'enum', 'order': 'intel', 'physical_unit': ''}
void Adudrivectrl8cff1400::set_p_ad_brakereq(uint8_t* data,
    Adu_drivectrl_8cff1400::Ad_brakereqType ad_brakereq) {
  int x = ad_brakereq;
  
  /* Modify@20201223: according to Rev13 matrix */
  Byte to_set(data + 3);
  to_set.set_value(x, 1, 1);
}


Adudrivectrl8cff1400* Adudrivectrl8cff1400::set_ad_maxspeedlimitreq(
    double ad_maxspeedlimitreq) {
  ad_maxspeedlimitreq_ = ad_maxspeedlimitreq;
  return this;
 }

// config detail: {'name': 'AD_MaxSpeedLimitReq', 'offset': 0.0, 'precision': 0.5, 'len': 8, 'is_signed_var': False, 'physical_range': '[0|120]', 'bit': 16, 'type': 'double', 'order': 'intel', 'physical_unit': 'km/h'}
void Adudrivectrl8cff1400::set_p_ad_maxspeedlimitreq(uint8_t* data,
    double ad_maxspeedlimitreq) {
  ad_maxspeedlimitreq = ProtocolData::BoundedValue(0.0, 120.0, ad_maxspeedlimitreq);
  int x = ad_maxspeedlimitreq / 0.500000;

  Byte to_set(data + 2);
  to_set.set_value(x, 0, 8);
}


Adudrivectrl8cff1400* Adudrivectrl8cff1400::set_ad_torquepercent(
    int ad_torquepercent) {
  ad_torquepercent_ = ad_torquepercent;
  return this;
 }

// config detail: {'name': 'AD_TorquePercent', 'offset': 0.0, 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'physical_range': '[0|100]', 'bit': 8, 'type': 'int', 'order': 'intel', 'physical_unit': '%'}
void Adudrivectrl8cff1400::set_p_ad_torquepercent(uint8_t* data,
    int ad_torquepercent) {
  /* Modify@20201223: according to Rev13 matrix, unit change from /100 to /1000 */
  ad_torquepercent = ProtocolData::BoundedValue(0, 1000, ad_torquepercent);
  int x = ad_torquepercent;
  
  uint8_t half = 0;
  half = static_cast<uint8_t>(x & 0x0F);
  Byte to_set0(data + 0);
  to_set0.set_value(half, 4, 4);
  x >>= 4;

  half = static_cast<uint8_t>(x & 0xFF);
  Byte to_set1(data + 1);
  to_set1.set_value(half, 0, 8);
}


Adudrivectrl8cff1400* Adudrivectrl8cff1400::set_ad_gearreq(
    Adu_drivectrl_8cff1400::Ad_gearreqType ad_gearreq) {
  ad_gearreq_ = ad_gearreq;
  return this;
 }

// config detail: {'name': 'AD_GearReq', 'enum': {0: 'AD_GEARREQ_NATURAL', 1: 'AD_GEARREQ_DRIVE', 2: 'AD_GEARREQ_REVERSE', 3: 'AD_GEARREQ_PARK'}, 'precision': 1.0, 'len': 4, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|15]', 'bit': 4, 'type': 'enum', 'order': 'intel', 'physical_unit': ''}
void Adudrivectrl8cff1400::set_p_ad_gearreq(uint8_t* data,
    Adu_drivectrl_8cff1400::Ad_gearreqType ad_gearreq) {
  int x = ad_gearreq;

  /* Modify@20201223: according to Rev13 matrix */
  Byte to_set(data + 0);
  to_set.set_value(x, 2, 2);
}


Adudrivectrl8cff1400* Adudrivectrl8cff1400::set_ad_statusreq(
    Adu_drivectrl_8cff1400::Ad_statusreqType ad_statusreq) {
  ad_statusreq_ = ad_statusreq;
  return this;
 }

// config detail: {'name': 'AD_StatusReq', 'enum': {0: 'AD_STATUSREQ_VEHICLE_IGNITION_REQUEST', 1: 'AD_STATUSREQ_AD_REQUEST', 2: 'AD_STATUSREQ_FAULT_RESET', 3: 'AD_STATUSREQ_VEHICLE_HV_POWEROFF', 10: 'AD_STATUSREQ_REMOTE_CONTROL'}, 'precision': 1.0, 'len': 4, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|15]', 'bit': 0, 'type': 'enum', 'order': 'intel', 'physical_unit': ''}
void Adudrivectrl8cff1400::set_p_ad_statusreq(uint8_t* data,
    Adu_drivectrl_8cff1400::Ad_statusreqType ad_statusreq) {
  int x = ad_statusreq;

  /* Modify@20201223: according to Rev13 matrix */
  Byte to_set(data + 0);
  to_set.set_value(x, 0, 2);
}

}  // namespace ch
}  // namespace canbus
}  // namespace apollo
