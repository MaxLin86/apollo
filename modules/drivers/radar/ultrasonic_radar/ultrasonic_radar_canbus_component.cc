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

#include "modules/drivers/radar/ultrasonic_radar/ultrasonic_radar_canbus_component.h"
#include "modules/common/adapters/adapter_gflags.h"

namespace apollo {
namespace drivers {
namespace ultrasonic_radar {

bool UltrasonicRadarCanbusComponent::Init() {
  writer_ = node_->CreateWriter<Ultrasonic>(FLAGS_ultrasonic_radar_topic);
  ultra_radar_writer_ =  node_->CreateWriter<Ultra_Radar>(FLAGS_ultra_radar_topic);
  return utralsonic_radar_canbus_.Init(ConfigFilePath(), writer_,ultra_radar_writer_).ok() &&
         utralsonic_radar_canbus_.Start().ok();
}

bool UltrasonicRadarCanbusComponent::Proc() {
  Ultra431 tmpValue;
  SenderMessage<Ultrasonic> sender_message(Ultra431::ID,&tmpValue);
  sender_message.Update();
  utralsonic_radar_canbus_.can_client_->SendSingleFrame({sender_message.CanFrame()}); 
  return true; 
}

}  // namespace ultrasonic_radar
}  // namespace drivers
}  // namespace apollo
