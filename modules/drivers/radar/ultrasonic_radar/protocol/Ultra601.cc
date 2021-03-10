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

#include "modules/drivers/radar/ultrasonic_radar/protocol/Ultra601.h"

#include "modules/drivers/canbus/common/byte.h"

namespace apollo {
namespace drivers {
namespace ultrasonic_radar {

using apollo::drivers::Ultrasonic;
using apollo::drivers::canbus::Byte;

const uint32_t Ultra601::ID = 0x601;

Ultra601::Ultra601() {}
Ultra601::~Ultra601() {}


void Ultra601::Parse(const std::uint8_t* bytes, int32_t length,
                          Ultrasonic* ultrasonic_radar) const {
  
}


}  // namespace conti_radar
}  // namespace drivers
}  // namespace apollo
