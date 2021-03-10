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

class Vcuvehicleinfo398f003d0 : public ::apollo::drivers::canbus::ProtocolData<
                    ::apollo::canbus::ChassisDetail> {
 public:
  static const int32_t ID;
  Vcuvehicleinfo398f003d0();
  void Parse(const std::uint8_t* bytes, int32_t length,
                     ChassisDetail* chassis) const override;

 private:

  // config detail: {'name': 'VCU_LifeCounter', 'offset': 0.0, 'precision': 1.0, 'len': 4, 'is_signed_var': False, 'physical_range': '[0|15]', 'bit': 60, 'type': 'int', 'order': 'intel', 'physical_unit': ''}
  int vcu_lifecounter(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'VCU_BatteryTemp', 'offset': -100.0, 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'physical_range': '[-100|155]', 'bit': 48, 'type': 'int', 'order': 'intel', 'physical_unit': '\xa1\xe6'}
  int vcu_batterytemp(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'VCU_BatteryVoltage', 'offset': 0.0, 'precision': 3.0, 'len': 8, 'is_signed_var': False, 'physical_range': '[0|765]', 'bit': 40, 'type': 'double', 'order': 'intel', 'physical_unit': 'V'}
  double vcu_batteryvoltage(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'VCU_LVBusVoltage', 'offset': 0.0, 'precision': 0.2, 'len': 8, 'is_signed_var': False, 'physical_range': '[0|40]', 'bit': 32, 'type': 'double', 'order': 'intel', 'physical_unit': 'V'}
  double vcu_lvbusvoltage(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'VCU_VehicleSpeed', 'offset': 0.0, 'precision': 0.00390625, 'len': 16, 'is_signed_var': False, 'physical_range': '[0|255]', 'bit': 8, 'type': 'double', 'order': 'intel', 'physical_unit': 'km/h'}
  double vcu_vehiclespeed(const std::uint8_t* bytes, const int32_t length) const;

  int vcu_handbrakestatus(const std::uint8_t* bytes, int32_t length) const;

  // config detail: {'name': 'VCU_CoolantStatus', 'enum': {0: 'VCU_COOLANTSTATUS_NORMAL', 1: 'VCU_COOLANTSTATUS_COOLANT_LOW', 2: 'VCU_COOLANTSTATUS_ERROR_INDICATOR', 3: 'VCU_COOLANTSTATUS_NOT_AVAILABLE'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 4, 'type': 'enum', 'order': 'intel', 'physical_unit': ''}
  Vcu_vehicleinfo3_98f003d0::Vcu_coolantstatusType vcu_coolantstatus(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'VCU_VehiclePowerStatus', 'enum': {0: 'VCU_VEHICLEPOWERSTATUS_LV_UP', 1: 'VCU_VEHICLEPOWERSTATUS_HV_UP_WITH_FAULT', 2: 'VCU_VEHICLEPOWERSTATUS_READY', 3: 'VCU_VEHICLEPOWERSTATUS_OTHER'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 2, 'type': 'enum', 'order': 'intel', 'physical_unit': ''}
  Vcu_vehicleinfo3_98f003d0::Vcu_vehiclepowerstatusType vcu_vehiclepowerstatus(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'VCU_GearStatus', 'enum': {0: 'VCU_GEARSTATUS_NEUTRAL', 1: 'VCU_GEARSTATUS_DRIVE', 2: 'VCU_GEARSTATUS_REVERSE', 3: 'VCU_GEARSTATUS_PARK'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 0, 'type': 'enum', 'order': 'intel', 'physical_unit': ''}
  Vcu_vehicleinfo3_98f003d0::Vcu_gearstatusType vcu_gearstatus(const std::uint8_t* bytes, const int32_t length) const;
};

}  // namespace ch
}  // namespace canbus
}  // namespace apollo


