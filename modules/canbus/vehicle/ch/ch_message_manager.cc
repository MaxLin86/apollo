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

#include "modules/canbus/vehicle/ch/ch_message_manager.h"

#include "modules/canbus/vehicle/ch/protocol/adu_bodyctrl_8cff1600.h"
#include "modules/canbus/vehicle/ch/protocol/adu_drivectrl_8cff1400.h"
//#include "modules/canbus/vehicle/ch/protocol/adu_steeringctrl_8cff1500.h"
#include "modules/canbus/vehicle/ch/protocol/adu_autosteering_98ffefe8.h"
#include "modules/canbus/vehicle/ch/protocol/adu_brakectrl_8c04eb0a.h"

//#include "modules/canbus/vehicle/ch/protocol/asc_steeringinfo_8c02a0a2.h"
#include "modules/canbus/vehicle/ch/protocol/asc_steeringinfo2_98fff013.h"
#include "modules/canbus/vehicle/ch/protocol/asc_steeringinfo1_98fff113.h"
#include "modules/canbus/vehicle/ch/protocol/vcu_bodyctrl_98f004d0.h"
#include "modules/canbus/vehicle/ch/protocol/vcu_vehicleinfo1_98f001d0.h"
#include "modules/canbus/vehicle/ch/protocol/vcu_vehicleinfo2_98f002d0.h"
#include "modules/canbus/vehicle/ch/protocol/vcu_vehicleinfo3_98f003d0.h"
#include "modules/canbus/vehicle/ch/protocol/vcu_vehicleinfo4_98f005d0.h"
#include "modules/canbus/vehicle/ch/protocol/vcu_brakeinfo1_8c04cb0a.h"
#include "modules/canbus/vehicle/ch/protocol/vcu_brakeinfo2_8c04db0a.h"


namespace apollo {
namespace canbus {
namespace ch {

ChMessageManager::ChMessageManager() {
  // Control Messages
  AddSendProtocolData<Adubodyctrl8cff1600, true>();
  AddSendProtocolData<Adudrivectrl8cff1400, true>();
  //AddSendProtocolData<Adusteeringctrl8cff1500, true>();
  AddSendProtocolData<Aduautosteering98ffefe8, true>();
  AddSendProtocolData<Adubrakectrl8c04eb0a, true>();

  // Report Messages
  //AddRecvProtocolData<Ascsteeringinfo8c02a0a2, true>();
  AddRecvProtocolData<Ascsteeringinfo298fff013,true>();
  AddRecvProtocolData<Ascsteeringinfo198fff113,true>();
  AddRecvProtocolData<Vcubodyctrl98f004d0, true>();
  AddRecvProtocolData<Vcuvehicleinfo198f001d0, true>();
  AddRecvProtocolData<Vcuvehicleinfo298f002d0, true>();
  AddRecvProtocolData<Vcuvehicleinfo398f003d0, true>();
  AddRecvProtocolData<Vcuvehicleinfo498f005d0, true>();
  AddRecvProtocolData<Vcubrakeinfo18c04cb0a, true>();
  AddRecvProtocolData<Vcubrakeinfo28c04db0a, true>();
}

ChMessageManager::~ChMessageManager() {}

}  // namespace ch
}  // namespace canbus
}  // namespace apollo
