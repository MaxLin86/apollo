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

class Adusteeringctrl8cff1500 : public ::apollo::drivers::canbus::ProtocolData<
                    ::apollo::canbus::ChassisDetail> {
 public:
  static const int32_t ID;

  Adusteeringctrl8cff1500();

  uint32_t GetPeriod() const override;

  void UpdateData(uint8_t* data) override;

  void Reset() override;

  // config detail: {'name': 'AD_Counter', 'offset': 0.0, 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'physical_range': '[0|255]', 'bit': 56, 'type': 'int', 'order': 'intel', 'physical_unit': ''}
  Adusteeringctrl8cff1500* set_ad_counter(int ad_counter);

  // config detail: {'name': 'AD_WheelSpeed', 'offset': 0.0, 'precision': 10.0, 'len': 8, 'is_signed_var': False, 'physical_range': '[50|540]', 'bit': 32, 'type': 'double', 'order': 'intel', 'physical_unit': '\xa1\xe3/s'}
  Adusteeringctrl8cff1500* set_ad_wheelspeed(double ad_wheelspeed);

  // config detail: {'name': 'AD_TurnAddedTorque', 'offset': 0.0, 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'physical_range': '[0|255]', 'bit': 24, 'type': 'int', 'order': 'intel', 'physical_unit': ''}
  Adusteeringctrl8cff1500* set_ad_turnaddedtorque(int ad_turnaddedtorque);

  // config detail: {'name': 'AD_TurnCmdValid', 'enum': {0: 'AD_TURNCMDVALID_ABNORMAL', 1: 'AD_TURNCMDVALID_NORMAL'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 21, 'type': 'enum', 'order': 'intel', 'physical_unit': ''}
  Adusteeringctrl8cff1500* set_ad_turncmdvalid(Adu_steeringctrl_8cff1500::Ad_turncmdvalidType ad_turncmdvalid);

  // config detail: {'name': 'AD_ADUStatus', 'enum': {0: 'AD_ADUSTATUS_ABNORMAL', 1: 'AD_ADUSTATUS_NORMAL'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 20, 'type': 'enum', 'order': 'intel', 'physical_unit': ''}
  Adusteeringctrl8cff1500* set_ad_adustatus(Adu_steeringctrl_8cff1500::Ad_adustatusType ad_adustatus);

  // config detail: {'name': 'AD_WorkMode', 'enum': {0: 'AD_WORKMODE_READY', 1: 'AD_WORKMODE_AD_MODE', 2: 'AD_WORKMODE_RESERVE', 3: 'AD_WORKMODE_TORQUE_ADDED', 4: 'AD_WORKMODE_MANUAL_CONTROL', 5: 'AD_WORKMODE_MANUAL_RETURN', 6: 'AD_WORKMODE_FAULT_RESET'}, 'precision': 1.0, 'len': 4, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|15]', 'bit': 16, 'type': 'enum', 'order': 'intel', 'physical_unit': ''}
  Adusteeringctrl8cff1500* set_ad_workmode(Adu_steeringctrl_8cff1500::Ad_workmodeType ad_workmode);

  // config detail: {'name': 'AD_SteeringTurnAngle', 'offset': -1575.0, 'precision': 0.1, 'len': 16, 'is_signed_var': False, 'physical_range': '[-850|850]', 'bit': 0, 'type': 'double', 'order': 'intel', 'physical_unit': '\xa1\xe3'}
  Adusteeringctrl8cff1500* set_ad_steeringturnangle(double ad_steeringturnangle);

 private:

  // config detail: {'name': 'AD_Counter', 'offset': 0.0, 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'physical_range': '[0|255]', 'bit': 56, 'type': 'int', 'order': 'intel', 'physical_unit': ''}
  void set_p_ad_counter(uint8_t* data, int ad_counter);

  // config detail: {'name': 'AD_WheelSpeed', 'offset': 0.0, 'precision': 10.0, 'len': 8, 'is_signed_var': False, 'physical_range': '[50|540]', 'bit': 32, 'type': 'double', 'order': 'intel', 'physical_unit': '\xa1\xe3/s'}
  void set_p_ad_wheelspeed(uint8_t* data, double ad_wheelspeed);

  // config detail: {'name': 'AD_TurnAddedTorque', 'offset': 0.0, 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'physical_range': '[0|255]', 'bit': 24, 'type': 'int', 'order': 'intel', 'physical_unit': ''}
  void set_p_ad_turnaddedtorque(uint8_t* data, int ad_turnaddedtorque);

  // config detail: {'name': 'AD_TurnCmdValid', 'enum': {0: 'AD_TURNCMDVALID_ABNORMAL', 1: 'AD_TURNCMDVALID_NORMAL'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 21, 'type': 'enum', 'order': 'intel', 'physical_unit': ''}
  void set_p_ad_turncmdvalid(uint8_t* data, Adu_steeringctrl_8cff1500::Ad_turncmdvalidType ad_turncmdvalid);

  // config detail: {'name': 'AD_ADUStatus', 'enum': {0: 'AD_ADUSTATUS_ABNORMAL', 1: 'AD_ADUSTATUS_NORMAL'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 20, 'type': 'enum', 'order': 'intel', 'physical_unit': ''}
  void set_p_ad_adustatus(uint8_t* data, Adu_steeringctrl_8cff1500::Ad_adustatusType ad_adustatus);

  // config detail: {'name': 'AD_WorkMode', 'enum': {0: 'AD_WORKMODE_READY', 1: 'AD_WORKMODE_AD_MODE', 2: 'AD_WORKMODE_RESERVE', 3: 'AD_WORKMODE_TORQUE_ADDED', 4: 'AD_WORKMODE_MANUAL_CONTROL', 5: 'AD_WORKMODE_MANUAL_RETURN', 6: 'AD_WORKMODE_FAULT_RESET'}, 'precision': 1.0, 'len': 4, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|15]', 'bit': 16, 'type': 'enum', 'order': 'intel', 'physical_unit': ''}
  void set_p_ad_workmode(uint8_t* data, Adu_steeringctrl_8cff1500::Ad_workmodeType ad_workmode);

  // config detail: {'name': 'AD_SteeringTurnAngle', 'offset': -1575.0, 'precision': 0.1, 'len': 16, 'is_signed_var': False, 'physical_range': '[-850|850]', 'bit': 0, 'type': 'double', 'order': 'intel', 'physical_unit': '\xa1\xe3'}
  void set_p_ad_steeringturnangle(uint8_t* data, double ad_steeringturnangle);

 private:
  int ad_counter_;
  double ad_wheelspeed_;
  int ad_turnaddedtorque_;
  Adu_steeringctrl_8cff1500::Ad_turncmdvalidType ad_turncmdvalid_;
  Adu_steeringctrl_8cff1500::Ad_adustatusType ad_adustatus_;
  Adu_steeringctrl_8cff1500::Ad_workmodeType ad_workmode_;
  double ad_steeringturnangle_;
};

}  // namespace ch
}  // namespace canbus
}  // namespace apollo


