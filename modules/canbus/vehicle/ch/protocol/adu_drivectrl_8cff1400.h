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

class Adudrivectrl8cff1400 : public ::apollo::drivers::canbus::ProtocolData<
                    ::apollo::canbus::ChassisDetail> {
 public:
  static const int32_t ID;

  Adudrivectrl8cff1400();

  uint32_t GetPeriod() const override;

  void UpdateData(uint8_t* data) override;

  void Reset() override;

  // config detail: {'name': 'AD_VCUSWVerReq', 'enum': {0: 'AD_VCUSWVERREQ_NO_REQUEST', 1: 'AD_VCUSWVERREQ_REQUEST'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 24, 'type': 'enum', 'order': 'intel', 'physical_unit': ''}
  Adudrivectrl8cff1400* set_ad_vcuswverreq(Adu_drivectrl_8cff1400::Ad_vcuswverreqType ad_vcuswverreq);

  // config detail: {'name': 'AD_VehicleSpeed', 'offset': 0.0, 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'physical_range': '[0|200]', 'bit': 56, 'type': 'int', 'order': 'intel', 'physical_unit': 'km/h'}
  Adudrivectrl8cff1400* set_ad_vehiclespeed(int ad_vehiclespeed);

  // config detail: {'name': 'AD_DTC', 'offset': 0.0, 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'physical_range': '[0|255]', 'bit': 48, 'type': 'int', 'order': 'intel', 'physical_unit': ''}
  Adudrivectrl8cff1400* set_ad_dtc(int ad_dtc);

  // config detail: {'name': 'AD_ADCStatus', 'enum': {0: 'AD_ADCSTATUS_NORMAL', 1: 'AD_ADCSTATUS_L1_WARNING', 2: 'AD_ADCSTATUS_L2_FAULT', 3: 'AD_ADCSTATUS_L3_FAULT', 4: 'AD_ADCSTATUS_OTHERS'}, 'precision': 1.0, 'len': 3, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|4]', 'bit': 41, 'type': 'enum', 'order': 'intel', 'physical_unit': ''}
  Adudrivectrl8cff1400* set_ad_adcstatus(Adu_drivectrl_8cff1400::Ad_adcstatusType ad_adcstatus);

  // config detail: {'name': 'AD_EPB', 'enum': {0: 'AD_EPB_STANDBY', 1: 'AD_EPB_APPLY_PARKING_BRAKE'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 40, 'type': 'enum', 'order': 'intel', 'physical_unit': ''}
  Adudrivectrl8cff1400* set_ad_epb(Adu_drivectrl_8cff1400::Ad_epbType ad_epb);

  // config detail: {'name': 'AD_BrakePercent', 'offset': 0.0, 'precision': 1.0, 'len': 7, 'is_signed_var': False, 'physical_range': '[0|100]', 'bit': 33, 'type': 'int', 'order': 'intel', 'physical_unit': '%'}
  Adudrivectrl8cff1400* set_ad_brakepercent(int ad_brakepercent);

  // config detail: {'name': 'AD_BrakeReq', 'enum': {0: 'AD_BRAKEREQ_NO_BRAKE', 1: 'AD_BRAKEREQ_APPLY_BRAKE'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 32, 'type': 'enum', 'order': 'intel', 'physical_unit': ''}
  Adudrivectrl8cff1400* set_ad_brakereq(Adu_drivectrl_8cff1400::Ad_brakereqType ad_brakereq);

  // config detail: {'name': 'AD_MaxSpeedLimitReq', 'offset': 0.0, 'precision': 0.5, 'len': 8, 'is_signed_var': False, 'physical_range': '[0|120]', 'bit': 16, 'type': 'double', 'order': 'intel', 'physical_unit': 'km/h'}
  Adudrivectrl8cff1400* set_ad_maxspeedlimitreq(double ad_maxspeedlimitreq);

  // config detail: {'name': 'AD_TorquePercent', 'offset': 0.0, 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'physical_range': '[0|100]', 'bit': 8, 'type': 'int', 'order': 'intel', 'physical_unit': '%'}
  Adudrivectrl8cff1400* set_ad_torquepercent(int ad_torquepercent);

  // config detail: {'name': 'AD_GearReq', 'enum': {0: 'AD_GEARREQ_NATURAL', 1: 'AD_GEARREQ_DRIVE', 2: 'AD_GEARREQ_REVERSE', 3: 'AD_GEARREQ_PARK'}, 'precision': 1.0, 'len': 4, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|15]', 'bit': 4, 'type': 'enum', 'order': 'intel', 'physical_unit': ''}
  Adudrivectrl8cff1400* set_ad_gearreq(Adu_drivectrl_8cff1400::Ad_gearreqType ad_gearreq);

  // config detail: {'name': 'AD_StatusReq', 'enum': {0: 'AD_STATUSREQ_VEHICLE_IGNITION_REQUEST', 1: 'AD_STATUSREQ_AD_REQUEST', 2: 'AD_STATUSREQ_FAULT_RESET', 3: 'AD_STATUSREQ_VEHICLE_HV_POWEROFF', 10: 'AD_STATUSREQ_REMOTE_CONTROL'}, 'precision': 1.0, 'len': 4, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|15]', 'bit': 0, 'type': 'enum', 'order': 'intel', 'physical_unit': ''}
  Adudrivectrl8cff1400* set_ad_statusreq(Adu_drivectrl_8cff1400::Ad_statusreqType ad_statusreq);

 private:

  // config detail: {'name': 'AD_VCUSWVerReq', 'enum': {0: 'AD_VCUSWVERREQ_NO_REQUEST', 1: 'AD_VCUSWVERREQ_REQUEST'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 24, 'type': 'enum', 'order': 'intel', 'physical_unit': ''}
  void set_p_ad_vcuswverreq(uint8_t* data, Adu_drivectrl_8cff1400::Ad_vcuswverreqType ad_vcuswverreq);

  // config detail: {'name': 'AD_VehicleSpeed', 'offset': 0.0, 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'physical_range': '[0|200]', 'bit': 56, 'type': 'int', 'order': 'intel', 'physical_unit': 'km/h'}
  void set_p_ad_vehiclespeed(uint8_t* data, int ad_vehiclespeed);

  // config detail: {'name': 'AD_DTC', 'offset': 0.0, 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'physical_range': '[0|255]', 'bit': 48, 'type': 'int', 'order': 'intel', 'physical_unit': ''}
  void set_p_ad_dtc(uint8_t* data, int ad_dtc);

  // config detail: {'name': 'AD_ADCStatus', 'enum': {0: 'AD_ADCSTATUS_NORMAL', 1: 'AD_ADCSTATUS_L1_WARNING', 2: 'AD_ADCSTATUS_L2_FAULT', 3: 'AD_ADCSTATUS_L3_FAULT', 4: 'AD_ADCSTATUS_OTHERS'}, 'precision': 1.0, 'len': 3, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|4]', 'bit': 41, 'type': 'enum', 'order': 'intel', 'physical_unit': ''}
  void set_p_ad_adcstatus(uint8_t* data, Adu_drivectrl_8cff1400::Ad_adcstatusType ad_adcstatus);

  // config detail: {'name': 'AD_EPB', 'enum': {0: 'AD_EPB_STANDBY', 1: 'AD_EPB_APPLY_PARKING_BRAKE'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 40, 'type': 'enum', 'order': 'intel', 'physical_unit': ''}
  void set_p_ad_epb(uint8_t* data, Adu_drivectrl_8cff1400::Ad_epbType ad_epb);

  // config detail: {'name': 'AD_BrakePercent', 'offset': 0.0, 'precision': 1.0, 'len': 7, 'is_signed_var': False, 'physical_range': '[0|100]', 'bit': 33, 'type': 'int', 'order': 'intel', 'physical_unit': '%'}
  void set_p_ad_brakepercent(uint8_t* data, int ad_brakepercent);

  // config detail: {'name': 'AD_BrakeReq', 'enum': {0: 'AD_BRAKEREQ_NO_BRAKE', 1: 'AD_BRAKEREQ_APPLY_BRAKE'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 32, 'type': 'enum', 'order': 'intel', 'physical_unit': ''}
  void set_p_ad_brakereq(uint8_t* data, Adu_drivectrl_8cff1400::Ad_brakereqType ad_brakereq);

  // config detail: {'name': 'AD_MaxSpeedLimitReq', 'offset': 0.0, 'precision': 0.5, 'len': 8, 'is_signed_var': False, 'physical_range': '[0|120]', 'bit': 16, 'type': 'double', 'order': 'intel', 'physical_unit': 'km/h'}
  void set_p_ad_maxspeedlimitreq(uint8_t* data, double ad_maxspeedlimitreq);

  // config detail: {'name': 'AD_TorquePercent', 'offset': 0.0, 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'physical_range': '[0|100]', 'bit': 8, 'type': 'int', 'order': 'intel', 'physical_unit': '%'}
  void set_p_ad_torquepercent(uint8_t* data, int ad_torquepercent);

  // config detail: {'name': 'AD_GearReq', 'enum': {0: 'AD_GEARREQ_NATURAL', 1: 'AD_GEARREQ_DRIVE', 2: 'AD_GEARREQ_REVERSE', 3: 'AD_GEARREQ_PARK'}, 'precision': 1.0, 'len': 4, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|15]', 'bit': 4, 'type': 'enum', 'order': 'intel', 'physical_unit': ''}
  void set_p_ad_gearreq(uint8_t* data, Adu_drivectrl_8cff1400::Ad_gearreqType ad_gearreq);

  // config detail: {'name': 'AD_StatusReq', 'enum': {0: 'AD_STATUSREQ_VEHICLE_IGNITION_REQUEST', 1: 'AD_STATUSREQ_AD_REQUEST', 2: 'AD_STATUSREQ_FAULT_RESET', 3: 'AD_STATUSREQ_VEHICLE_HV_POWEROFF', 10: 'AD_STATUSREQ_REMOTE_CONTROL'}, 'precision': 1.0, 'len': 4, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|15]', 'bit': 0, 'type': 'enum', 'order': 'intel', 'physical_unit': ''}
  void set_p_ad_statusreq(uint8_t* data, Adu_drivectrl_8cff1400::Ad_statusreqType ad_statusreq);

 private:
  Adu_drivectrl_8cff1400::Ad_vcuswverreqType ad_vcuswverreq_;
  int ad_vehiclespeed_;
  int ad_dtc_;
  Adu_drivectrl_8cff1400::Ad_adcstatusType ad_adcstatus_;
  Adu_drivectrl_8cff1400::Ad_epbType ad_epb_;
  int ad_brakepercent_;
  Adu_drivectrl_8cff1400::Ad_brakereqType ad_brakereq_;
  double ad_maxspeedlimitreq_;
  int ad_torquepercent_;
  Adu_drivectrl_8cff1400::Ad_gearreqType ad_gearreq_;
  Adu_drivectrl_8cff1400::Ad_statusreqType ad_statusreq_;
};

}  // namespace ch
}  // namespace canbus
}  // namespace apollo


