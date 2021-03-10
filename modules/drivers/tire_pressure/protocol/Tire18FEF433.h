/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
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

#include <cmath>

#include "modules/drivers/canbus/can_comm/protocol_data.h"
#include "modules/drivers/proto/tire_pressure.pb.h"

namespace apollo {
namespace drivers {
namespace tire_pressure {

using apollo::drivers::Tire_Pressure;

class Tire18FEF433 : public apollo::drivers::canbus::ProtocolData<Tire_Pressure> {
 public:
  static const uint32_t ID;
  Tire18FEF433();
  ~Tire18FEF433();
  void Parse(const std::uint8_t* bytes, int32_t length,
             Tire_Pressure* tire_pressure) const override;
};
}  // namespace tire_pressure
}  // namespace drivers
}  // namespace apollo
