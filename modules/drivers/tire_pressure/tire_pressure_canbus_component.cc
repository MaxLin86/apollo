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

#include "modules/drivers/tire_pressure/tire_pressure_canbus_component.h"
#include "modules/common/adapters/adapter_gflags.h"

namespace apollo {
namespace drivers {
namespace tire_pressure {

bool TirePressureCanbusComponent::Init() {
  tire_pressure_writer_ =  node_->CreateWriter<Tire_Pressure>(FLAGS_tire_pressure_topic);
  return tire_pressure_canbus_.Init(ConfigFilePath(), tire_pressure_writer_).ok() &&
         tire_pressure_canbus_.Start().ok();
}

bool TirePressureCanbusComponent::Proc() {
  return true; 
}

}  // namespace tire_pressure
}  // namespace drivers
}  // namespace apollo
