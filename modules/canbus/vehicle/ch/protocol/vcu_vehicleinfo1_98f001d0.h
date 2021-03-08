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

class Vcuvehicleinfo198f001d0 : public ::apollo::drivers::canbus::ProtocolData<
                    ::apollo::canbus::ChassisDetail> {
 public:
  static const int32_t ID;
  Vcuvehicleinfo198f001d0();
  void Parse(const std::uint8_t* bytes, int32_t length,
                     ChassisDetail* chassis) const override;

 private:

  // config detail: {'name': 'VCU_DTC', 'offset': 0.0, 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'physical_range': '[0|255]', 'bit': 40, 'type': 'int', 'order': 'intel', 'physical_unit': ''}
  int vcu_dtc(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'VCU_CumulatedMileage', 'offset': 0.0, 'precision': 20.0, 'len': 16, 'is_signed_var': False, 'physical_range': '[0|1310000]', 'bit': 48, 'type': 'double', 'order': 'intel', 'physical_unit': 'km'}
  double vcu_cumulatedmileage(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'VCU_ModuleStatus', 'enum': {0: 'VCU_MODULESTATUS_NORMAL', 1: 'VCU_MODULESTATUS_L1_WARNING', 2: 'VCU_MODULESTATUS_L2_FAULT', 3: 'VCU_MODULESTATUS_L3_FAULT', 4: 'VCU_MODULESTATUS_OTHER'}, 'precision': 1.0, 'len': 3, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|7]', 'bit': 32, 'type': 'enum', 'order': 'intel', 'physical_unit': ''}
  Vcu_vehicleinfo1_98f001d0::Vcu_modulestatusType vcu_modulestatus(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'VCU_BatterySoH', 'offset': 0.0, 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'physical_range': '[0|100]', 'bit': 24, 'type': 'int', 'order': 'intel', 'physical_unit': '%'}
  int vcu_batterysoh(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'VCU_BatterySoC', 'offset': 0.0, 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'physical_range': '[0|100]', 'bit': 16, 'type': 'int', 'order': 'intel', 'physical_unit': '%'}
  int vcu_batterysoc(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'VCU_AvailableTorque', 'offset': 0.0, 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'physical_range': '[0|100]', 'bit': 8, 'type': 'int', 'order': 'intel', 'physical_unit': '%'}
  int vcu_availabletorque(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'VCU_VehicleMode', 'enum': {0: 'VCU_VEHICLEMODE_CHARGE', 1: 'VCU_VEHICLEMODE_AD_TRACTION', 2: 'VCU_VEHICLEMODE_MANUAL_CONTROL', 3: 'VCU_VEHICLEMODE_PARK'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 5, 'type': 'enum', 'order': 'intel', 'physical_unit': ''}
  Vcu_vehicleinfo1_98f001d0::Vcu_vehiclemodeType vcu_vehiclemode(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'VCU_VehicleType', 'enum': {0: 'VCU_VEHICLETYPE_AVG', 1: 'VCU_VEHICLETYPE_TRUCK_40', 2: 'VCU_VEHICLETYPE_TRUCK_20'}, 'precision': 1.0, 'len': 5, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|31]', 'bit': 0, 'type': 'enum', 'order': 'intel', 'physical_unit': ''}
  Vcu_vehicleinfo1_98f001d0::Vcu_vehicletypeType vcu_vehicletype(const std::uint8_t* bytes, const int32_t length) const;
};

}  // namespace ch
}  // namespace canbus
}  // namespace apollo


