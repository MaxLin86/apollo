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

#include "modules/canbus/vehicle/ch/protocol/vcu_brakeinfo2_8c04db0a.h"

#include "glog/logging.h"

#include "modules/drivers/canbus/common/byte.h"
#include "modules/drivers/canbus/common/canbus_consts.h"

namespace apollo {
namespace canbus {
namespace ch {

using ::apollo::drivers::canbus::Byte;

Vcubrakeinfo28c04db0a::Vcubrakeinfo28c04db0a() {}
const int32_t Vcubrakeinfo28c04db0a::ID = 0x8c04db0a;

void Vcubrakeinfo28c04db0a::Parse(const std::uint8_t* bytes, int32_t length,
                         ChassisDetail* chassis) const {
    chassis->mutable_ch()->mutable_vcu_brakeinfo2_8c04db0a()->set_vcu_brake2_air_pressure(vcu_brake2_air_pressure(bytes, length));
}

int Vcubrakeinfo28c04db0a::vcu_brake2_air_pressure(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 7);
  int32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 6);
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;
  return x;
} 


}  // namespace ch
}  // namespace canbus
}  // namespace apollo
