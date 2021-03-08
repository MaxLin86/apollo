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

/**
 * @file
 */

#pragma once

#include <memory>

#include "cyber/common/macros.h"
#include "cyber/component/timer_component.h"
#include "cyber/cyber.h"
#include "modules/common/time/time.h"
#include "modules/canbus/proto/chassis.pb.h"
#include "modules/control/proto/control_cmd.pb.h"
#include "modules/guardian/proto/guardian.pb.h"
#include "modules/guardian/proto/guardian_conf.pb.h"
#include "modules/monitor/proto/system_status.pb.h"
#include "modules/manuctrl/proto/manuctrl.pb.h"
#include "modules//tmc/proto/tmc.pb.h"
#include "modules/urgencystop/proto/aeb_cmd.pb.h"
/**
 * @namespace apollo::guardian
 * @brief apollo::guardian
 */
namespace apollo {
namespace guardian {

class GuardianComponent : public apollo::cyber::TimerComponent {
 public:
  bool Init() override;
  bool Proc() override;

 private:
  void PassThroughControlCommand();
  void TriggerSafetyMode();

  apollo::guardian::GuardianConf guardian_conf_;
  apollo::canbus::Chassis chassis_;
  apollo::monitor::SystemStatus system_status_;
  apollo::control::ControlCommand control_cmd_;
  apollo::guardian::GuardianCommand guardian_cmd_;
  apollo::urgency::AEBCommand aeb_cmd_;
  apollo::manuctrl::manuctrlMsg manu_ctrl_cmd_;
  apollo::tmc::tmcMsg remote_control_cmd_;

  std::shared_ptr<apollo::cyber::Reader<apollo::canbus::Chassis>>
      chassis_reader_;
  std::shared_ptr<apollo::cyber::Reader<apollo::control::ControlCommand>>
      control_cmd_reader_;
  std::shared_ptr<apollo::cyber::Reader<apollo::monitor::SystemStatus>>
      system_status_reader_;
  std::shared_ptr<apollo::cyber::Writer<apollo::guardian::GuardianCommand>>
      guardian_writer_;

  std::shared_ptr<apollo::cyber::Reader<apollo::urgency::AEBCommand>>
      aeb_cmd_reader_;
  std::shared_ptr<apollo::cyber::Reader<apollo::manuctrl::manuctrlMsg>>
      serial_cmd_reader_;
  
  std::shared_ptr<apollo::cyber::Reader<apollo::tmc::tmcMsg>>
      remote_cmd_reader_;

  std::mutex mutex_;

  double miss_message_time_;
  double miss_monitor_time_;
};

CYBER_REGISTER_COMPONENT(GuardianComponent)

}  // namespace guardian
}  // namespace apollo
