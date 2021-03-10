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

#include "modules/canbus/vehicle/ch/protocol/adu_autosteering_98ffefe8.h"

#include "modules/drivers/canbus/common/byte.h"

namespace apollo {
namespace canbus {
namespace ch {

using ::apollo::drivers::canbus::Byte;

const int32_t Aduautosteering98ffefe8::ID = 0x98ffefe8;

// public
Aduautosteering98ffefe8::Aduautosteering98ffefe8() { Reset(); }


uint32_t Aduautosteering98ffefe8::GetPeriod() const {
  // TODO(All) :  modify every protocol's period manually
  static const uint32_t PERIOD = 10 * 1000;
  return PERIOD;
}


void Aduautosteering98ffefe8::Reset() {
   ad_desiredoperatingmode_ = Adu_autosteering_98ffefe8::AD_DESIREDOPERATINGMODE_DEFAULT;
   ad_command_from_external_controller_ = 0.0;
   ad_message_counter_ = 0;
   ad_message_checksum_ = 0;
 
}


void Aduautosteering98ffefe8::UpdateData(uint8_t* data) {  
    set_p_ad_desiredoperatingmode(data, ad_desiredoperatingmode_);
    set_p_ad_command_from_external_controller(data, ad_command_from_external_controller_);

    /* note: first counter, next checksum */
    /* set_p_ad_message_counter(data, ad_message_counter_);
    set_p_ad_message_checksum(data, ad_message_checksum_); */ 
}


inline void Aduautosteering98ffefe8::asc_set_sign_value_2bytes(uint8_t* data, 
                            double value,
                            double min,
                            double max,
                            int32_t idx_low_byte,
                            int32_t idx_high_byte,
                            double factor){
  value = ProtocolData::BoundedValue(min, max, value);
#if 1
  int16_t x = value / factor;
#else
  int32_t x = value / factor;
  uint8_t sign = (x < 0) ? 1 : 0;
  x = std::abs(x);

  if(1 == sign){
    x = ((~x) + 1) & 0x7FFF;
    x |= (1 << 15);
  }  
#endif

  uint8_t t = 0;
  t = static_cast<uint8_t>(x & 0xFF);
  Byte to_set0(data + idx_low_byte);
  to_set0.set_value(t, 0, 8);

  x >>= 8;

  t = static_cast<uint8_t>(x & 0xFF);
  Byte to_set1(data + idx_high_byte);
  to_set1.set_value(t, 0, 8);

  /* AERROR << "send=" << value
    << " b0=" << static_cast<int32_t>(*(data + idx_low_byte))
    << " b1=" << static_cast<int32_t>(*(data + idx_high_byte)); */
}


void Aduautosteering98ffefe8::set_p_ad_desiredoperatingmode(uint8_t* data, Adu_autosteering_98ffefe8::Ad_desiredoperatingmode ad_desiredoperatingmode){
  int32_t x = ad_desiredoperatingmode;
  Byte to_set(data + 0);
  to_set.set_value(x, 0, 8);
}


void Aduautosteering98ffefe8::set_p_ad_command_from_external_controller(uint8_t* data, double  ad_command_from_external_controller){
  asc_set_sign_value_2bytes(data, 
                            ad_command_from_external_controller, 
                            -1224.0, 
                            1224.0,
                            1,
                            2,
                            0.0625);
}


void Aduautosteering98ffefe8::set_p_ad_message_counter(uint8_t* data, int ad_message_counter){ 
  ad_message_counter_ = (ad_message_counter_ + 1) & 0x0F;
  int32_t x = ad_message_counter_;

  Byte to_set(data + 7);
  to_set.set_value(x, 0, 4);
}


void Aduautosteering98ffefe8::set_p_ad_message_checksum(uint8_t* data, int ad_message_checksum){
  uint32_t id = ID - 0x80000000;
  ad_message_checksum_ = 0;
  for(int i = 0; i < 7; i++){
    ad_message_checksum_ += *(data + i);
  }
  ad_message_checksum_ += (ad_message_counter_ & 0x0F);
  ad_message_checksum_ += ((id & 0xFF) + ((id >> 8) & 0xFF) + ((id >> 16) & 0xFF) + ((id >> 24) & 0xFF));
  ad_message_checksum_ = ((ad_message_checksum_ >> 4) + ad_message_checksum_) & 0x0F;

  int32_t x = ad_message_checksum_;
  Byte to_set(data + 7);
  to_set.set_value(x, 4, 4);
}

}  // namespace ch
}  // namespace canbus
}  // namespace apollo
