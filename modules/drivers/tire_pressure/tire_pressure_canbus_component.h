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

#pragma once

#include <memory>

#include "modules/drivers/tire_pressure/tire_pressure_canbus.h"
#include "cyber/cyber.h"
#include "cyber/component/timer_component.h"
#include "modules/drivers/proto/tire_pressure.pb.h"
/**
 * @namespace apollo::drivers
 * @brief apollo::drivers
 */

using apollo::cyber::Component;

namespace apollo {
namespace drivers {
namespace tire_pressure {

class TirePressureCanbusComponent : public apollo::cyber::TimerComponent {
 public:
  ~TirePressureCanbusComponent() = default;
  bool Init() override;
  bool Proc() override;
 private:
  TirePressureCanbus tire_pressure_canbus_;
  std::shared_ptr<::apollo::cyber::Writer<Tire_Pressure>> tire_pressure_writer_;
};

CYBER_REGISTER_COMPONENT(TirePressureCanbusComponent)

}  // namespace tire_pressure
}  // namespace drivers
}  // namespace apollo
