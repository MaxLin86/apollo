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

#include "modules/drivers/radar/ultrasonic_radar/protocol/ultra431.h"

#include "modules/drivers/canbus/common/byte.h"

namespace apollo {
namespace drivers {
namespace ultrasonic_radar {

using apollo::drivers::Ultrasonic;
using apollo::drivers::canbus::Byte;

const uint32_t Ultra431::ID = 0x431;

Ultra431::Ultra431() {}
Ultra431::~Ultra431() {}

uint32_t Ultra431::GetPeriod() const {
  static const uint32_t PERIOD = 20 * 1000;
  return PERIOD;
}

/**
 * @brief update the data
 * @param data a pointer to the data to be updated
 */
void Ultra431::UpdateData(uint8_t* data) {
  Byte tmpFrame(data + 4);
  tmpFrame.set_value(static_cast<unsigned char>(1 & 0xff), 3, 1);
}

/**
 * @brief reset the private variables
 */
void Ultra431::Reset() { }


}  // namespace conti_radar
}  // namespace drivers
}  // namespace apollo
