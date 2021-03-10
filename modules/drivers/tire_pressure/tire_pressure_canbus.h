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
#include <string>
#include <utility>
#include <vector>

#include "cyber/common/macros.h"

#include "modules/common/monitor_log/monitor_log_buffer.h"
#include "modules/common/status/status.h"
#include "modules/common/time/time.h"
#include "modules/drivers/canbus/can_client/can_client.h"
#include "modules/drivers/canbus/can_client/can_client_factory.h"
#include "modules/drivers/canbus/can_comm/can_receiver.h"
#include "modules/drivers/canbus/can_comm/can_sender.h"
#include "modules/drivers/canbus/can_comm/message_manager.h"
#include "modules/drivers/canbus/proto/can_card_parameter.pb.h"
#include "modules/drivers/canbus/proto/sensor_canbus_conf.pb.h"
#include "modules/drivers/canbus/sensor_gflags.h"
#include "modules/drivers/proto/tire_pressure.pb.h"
#include "modules/drivers/tire_pressure/proto/tire_pressure_conf.pb.h"
#include "modules/drivers/tire_pressure/tire_pressure_message_manager.h"

/**
 * @namespace apollo::drivers
 * @brief apollo::drivers
 */
namespace apollo {
namespace drivers {
namespace tire_pressure {

/**
 * @class TirePressureCanbus
 *
 * @brief template of canbus-based sensor module main class (e.g.,
 * ultrasonic_radar).
 */

using apollo::common::ErrorCode;
using apollo::common::Status;
using apollo::common::monitor::MonitorMessageItem;
using apollo::common::time::Clock;
using apollo::drivers::canbus::CanClient;
using apollo::drivers::canbus::CanClientFactory;
using apollo::drivers::canbus::CanReceiver;
using apollo::drivers::canbus::SenderMessage;
using apollo::drivers::canbus::SensorCanbusConf;

class TirePressureCanbus {
 public:
  TirePressureCanbus();
  ~TirePressureCanbus();

  /**
   * @brief obtain module name
   * @return module name
   */
  std::string Name() const;

  /**
   * @brief module initialization function
   * @return initialization status
   */
  apollo::common::Status Init(
      const std::string& config_path,
      const std::shared_ptr<::apollo::cyber::Writer<Tire_Pressure>>& tire_pressure_writer);

  /**
   * @brief module start function
   * @return start status
   */
  apollo::common::Status Start();
  std::shared_ptr<CanClient> can_client_;

 private:
  Status OnError(const std::string& error_msg);
  void RegisterCanClients();

  TirePressureConf tire_pressure_conf_;

  CanReceiver<Tire_Pressure> can_receiver_;
  std::unique_ptr<TirePressureMessageManager> sensor_message_manager_;

  int64_t last_timestamp_ = 0;
  apollo::common::monitor::MonitorLogBuffer monitor_logger_buffer_;
};

}  // namespace tire_pressure
}  // namespace drivers
}  // namespace apollo