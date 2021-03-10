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

#include "modules/drivers/tire_pressure/tire_pressure_canbus.h"

#include "cyber/common/file.h"
#include "modules/common/util/util.h"
#include "modules/drivers/proto/tire_pressure.pb.h"
#include "modules/drivers/tire_pressure/tire_pressure_message_manager.h"

/**
 * @namespace apollo::drivers::tire_pressure
 * @brief apollo::drivers
 */
namespace apollo {
namespace drivers {
namespace tire_pressure {

TirePressureCanbus::TirePressureCanbus():
monitor_logger_buffer_(common::monitor::MonitorMessageItem::TIRE_PRESSURE){

}//

TirePressureCanbus::~TirePressureCanbus() {
  can_receiver_.Stop();
  can_client_->Stop();
}

std::string TirePressureCanbus::Name() const { return "tire_pressure"; }

apollo::common::Status TirePressureCanbus::Init(
    const std::string& config_path,
    const std::shared_ptr<::apollo::cyber::Writer<Tire_Pressure>>& tire_pressure_writer) {
  if (!cyber::common::GetProtoFromFile(config_path, &tire_pressure_conf_)) {
    return OnError("Unable to load canbus conf file: " + config_path);
  }

  AINFO << "The canbus conf file is loaded: " << config_path;
  ADEBUG << "Canbus_conf:" << tire_pressure_conf_.ShortDebugString();

  // Init can client
  auto can_factory = CanClientFactory::Instance();
  can_factory->RegisterCanClients();
  can_client_ = can_factory->CreateCANClient(
      tire_pressure_conf_.can_conf().can_card_parameter());
  if (!can_client_) {
    AERROR << "Failed to start can client";
    return OnError("Failed to create can client.");
  }
  AINFO << "Can client is successfully created.";

  sensor_message_manager_ = std::unique_ptr<TirePressureMessageManager>(
      new TirePressureMessageManager(tire_pressure_writer));
  if (sensor_message_manager_ == nullptr) {
    return OnError("Failed to create message manager.");
  }
  sensor_message_manager_->set_can_client(can_client_);
  AINFO << "Sensor message manager is successfully created.";

  bool enable_receiver_log =
      tire_pressure_conf_.can_conf().enable_receiver_log();
  if (can_receiver_.Init(can_client_.get(), sensor_message_manager_.get(),
                         enable_receiver_log) != ErrorCode::OK) {
    return OnError("Failed to init can receiver.");
  }
  AINFO << "The can receiver is successfully initialized.";

  return Status::OK();
}

apollo::common::Status TirePressureCanbus::Start() {
  // 1. init and start the can card hardware
  if (can_client_->Start() != ErrorCode::OK) {
    return OnError("Failed to start can client");
  }
  AINFO << "Can client is started.";

  // 2. start receive first then send
  if (can_receiver_.Start() != ErrorCode::OK) {
    return OnError("Failed to start can receiver.");
  }
  AINFO << "Can receiver is started.";

  // last step: publish monitor messages
  monitor_logger_buffer_.INFO("Canbus is started.");

  return Status::OK();
}

// Send the error to monitor and return it
Status TirePressureCanbus::OnError(const std::string& error_msg) {
  monitor_logger_buffer_.ERROR(error_msg);
  return Status(ErrorCode::CANBUS_ERROR, error_msg);
}

}  // namespace tire_pressure
}  // namespace drivers
}  // namespace apollo
