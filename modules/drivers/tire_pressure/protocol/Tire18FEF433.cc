/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
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

#include "modules/drivers/tire_pressure/protocol/Tire18FEF433.h"

#include "modules/drivers/canbus/common/byte.h"

namespace apollo {
namespace drivers {
namespace tire_pressure {

using apollo::drivers::Tire_Pressure;
using apollo::drivers::canbus::Byte;

const uint32_t Tire18FEF433::ID = 0x18FEF433;

Tire18FEF433::Tire18FEF433() {}
Tire18FEF433::~Tire18FEF433() {}


void Tire18FEF433::Parse(const std::uint8_t* bytes, int32_t length,
                          Tire_Pressure* tire_pressure) const {
  
}


}  // namespace conti_radar
}  // namespace drivers
}  // namespace apollo
