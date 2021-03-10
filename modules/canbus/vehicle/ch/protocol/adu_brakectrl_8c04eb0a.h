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

class Adubrakectrl8c04eb0a : public ::apollo::drivers::canbus::ProtocolData<
                    ::apollo::canbus::ChassisDetail> {
 public:
  static const int32_t ID;

  Adubrakectrl8c04eb0a();

  uint32_t GetPeriod() const override;

  void UpdateData(uint8_t* data) override;

  void Reset() override;

  Adubrakectrl8c04eb0a* set_adu_brake1_air_pressure(int adu_brake1_air_pressure) ;
  Adubrakectrl8c04eb0a* set_adu_brake2_air_pressure(int adu_brake2_air_pressure) ;

 private:
  void set_p_adu_brake1_air_pressure(uint8_t* data, int adu_brake1_air_pressure);
  void set_p_adu_brake2_air_pressure(uint8_t* data, int adu_brake2_air_pressure);
  void set_msg_flag(uint8_t* data);

 private:
  int adu_brake1_air_pressure_;
  int adu_brake2_air_pressure_;

};

}  // namespace ch
}  // namespace canbus
}  // namespace apollo


