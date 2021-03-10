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

#include "modules/canbus/vehicle/ch/protocol/asc_steeringinfo2_98fff013.h"

#include "glog/logging.h"

#include "modules/drivers/canbus/common/byte.h"
#include "modules/drivers/canbus/common/canbus_consts.h"

namespace apollo {
namespace canbus {
namespace ch {

using ::apollo::drivers::canbus::Byte;

Ascsteeringinfo298fff013::Ascsteeringinfo298fff013() {}
const int32_t Ascsteeringinfo298fff013::ID = 0x98fff013;

void Ascsteeringinfo298fff013::Parse(const std::uint8_t* bytes, int32_t length,
                         ChassisDetail* chassis) const {
    chassis->mutable_ch()->mutable_asc_steeringinfo2_98fff013()->set_asc_absolute_steering_position(asc_absolute_steering_position(bytes, length)); 
    chassis->mutable_ch()->mutable_asc_steeringinfo2_98fff013()->set_asc_external_input_command_echo(asc_external_input_command_echo(bytes, length)); 
    chassis->mutable_ch()->mutable_asc_steeringinfo2_98fff013()->set_asc_status_error(asc_status_error(bytes, length)); 
    chassis->mutable_ch()->mutable_asc_steeringinfo2_98fff013()->set_asc_actual_mode(asc_actual_mode(bytes, length)); 
    chassis->mutable_ch()->mutable_asc_steeringinfo2_98fff013()->set_asc_request_mode(asc_request_mode(bytes, length)); 
    chassis->mutable_ch()->mutable_asc_steeringinfo2_98fff013()->set_asc_absolute_steering_position_qf(asc_absolute_steering_position_qf(bytes, length)); 
    chassis->mutable_ch()->mutable_asc_steeringinfo2_98fff013()->set_asc_message_checksum(asc_message_checksum(bytes, length)); 
    chassis->mutable_ch()->mutable_asc_steeringinfo2_98fff013()->set_asc_message_counter(asc_message_counter(bytes, length)); 
}
  

int Ascsteeringinfo298fff013::asc_message_check(const std::uint8_t* bytes, int32_t length) const {
  uint32_t id = ID - 0x80000000;
  uint8_t message_checksum_recv = *(bytes + 7) & 0x0F;
  uint8_t message_counter = ((*(bytes + 7) >> 4) & 0x0F);

  uint32_t message_checksum = 0;
  for (int i = 0; i < 7; i++) {
    message_checksum += *(bytes + 1);
  }
  message_checksum += (message_counter & 0x0F);
  message_checksum += ((id & 0xFF) + ((id >> 8) & 0xFF) + ((id >> 16) & 0xFF) + ((id >> 24) & 0xFF));
  message_checksum = ((message_checksum >> 4) + message_checksum) & 0x0F;

  if (message_checksum == message_checksum_recv) {
    return 1;
  } else {
    AERROR << "Ascsteeringinfo: " << std::hex << ID << " check err: "
        << std::hex << static_cast<uint8_t>(*(bytes + 0)) 
        << std::hex << static_cast<uint8_t>(*(bytes + 1)) 
        << std::hex << static_cast<uint8_t>(*(bytes + 2)) 
        << std::hex << static_cast<uint8_t>(*(bytes + 3)) 
        << std::hex << static_cast<uint8_t>(*(bytes + 4)) 
        << std::hex << static_cast<uint8_t>(*(bytes + 5)) 
        << std::hex << static_cast<uint8_t>(*(bytes + 6)) 
        << std::hex << static_cast<uint8_t>(*(bytes + 7));
    return 0;
  }
} 

inline double Ascsteeringinfo298fff013::asc_get_sign_value_2bytes(const std::uint8_t* bytes, 
                            int32_t idx_low_byte,
                            int32_t idx_high_byte,
                            double factor) const {
#if 1
  Byte t0(bytes + idx_high_byte);
  int16_t high = t0.get_byte(0, 8);

  Byte t1(bytes + idx_low_byte);
  int16_t low = t1.get_byte(0, 8);

  high = (((high & 0xff) << 8) + (low & 0xff));
  return (high * factor);
#else
  Byte t0(bytes + idx_high_byte);
  int32_t x = t0.get_byte(0, 8);
  int32_t sign = (x >> 7) & 0x1;
  x &= ~(1 << 7);

  Byte t1(bytes + idx_low_byte);
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  if (sign) {
    x = ((~x) & 0x7FFF) + 1;
  }

  double ret = x * factor;
  if (sign) {
    ret = -ret;
  }
  return ret;
#endif

}

double Ascsteeringinfo298fff013::asc_absolute_steering_position(const std::uint8_t* bytes, int32_t length) const { 
  return asc_get_sign_value_2bytes(bytes, 0, 1, 0.0625);
}

double Ascsteeringinfo298fff013::asc_external_input_command_echo(const std::uint8_t* bytes, int32_t length) const { 
  return asc_get_sign_value_2bytes(bytes, 2, 3, 0.0625);
}

int Ascsteeringinfo298fff013::asc_status_error(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 4);
  int32_t x = t0.get_byte(0, 8); 
  return x;
} 

Asc_steeringinfo2_98fff013::Asc_actual_mode Ascsteeringinfo298fff013::asc_actual_mode(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 5);
  int32_t x = t0.get_byte(0, 4);
  Asc_steeringinfo2_98fff013::Asc_actual_mode mode = static_cast<Asc_steeringinfo2_98fff013::Asc_actual_mode>(x);
  return mode;
} 
 
int Ascsteeringinfo298fff013::asc_request_mode(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 5);
  int32_t x = t0.get_byte(4, 4); 
  return x;
} 


int Ascsteeringinfo298fff013::asc_absolute_steering_position_qf(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 6);
  int32_t x = t0.get_byte(0, 1); 
  return x;
} 

int Ascsteeringinfo298fff013::asc_message_checksum(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 7);
  int32_t x = t0.get_byte(0, 4); 
  return x;
} 

int Ascsteeringinfo298fff013::asc_message_counter(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 7);
  int32_t x = t0.get_byte(4, 4); 
  return x;
} 



}  // namespace ch
}  // namespace canbus
}  // namespace apollo