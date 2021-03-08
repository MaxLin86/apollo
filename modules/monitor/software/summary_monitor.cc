/*
 * @Author: your name
 * @Date: 2020-04-17 14:07:43
 * @LastEditTime: 2020-05-13 16:02:48
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: /apollo-5.5.0/modules/monitor/software/summary_monitor.cc
 */
/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
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

#include "modules/monitor/software/summary_monitor.h"

#include "cyber/common/log.h"
#include "modules/common/adapters/adapter_gflags.h"
#include "modules/common/util/string_util.h"
#include "modules/monitor/common/monitor_manager.h"

#include "modules/monitor/common/monitor_can.h" 

DEFINE_string(summary_monitor_name, "SummaryMonitor",
              "Name of the summary monitor.");

DEFINE_double(system_status_publish_interval, 10,
              "SystemStatus publish interval.");

namespace apollo {
namespace monitor {

void SummaryMonitor::EscalateStatus(const ComponentStatus::Status new_status,
                                    const ErrorCode new_error_code,
                                    const std::string& message,
                                    ComponentStatus* current_status) { 
  if (new_status > current_status->status()) {
    current_status->set_status(new_status);
    current_status->set_error_code(new_error_code);
    if (!message.empty()) {
      current_status->set_message(message);
    } else {
      current_status->clear_message();
    }
  }
}

void SummaryMonitor::EscalateStatus(const ComponentStatus::Status new_status,
                                    const std::string& message,
                                    ComponentStatus* current_status) {
  // Overwrite priority: FATAL > ERROR > WARN > OK > UNKNOWN.
/***********************************************************/
  // apollo::canbus::canbus_api can_api;
  // int err_code = 3;
  // if (new_status >= ComponentStatus_Status::ComponentStatus_Status_ERROR) {
  //   can_api.hurry_full_brake_req();
  //   can_api.send_ipc_dtc_req(monitor::common::FaultLevel::TWO, err_code);
  //   AERROR << "send fault level:" << monitor::common::FaultLevel::TWO
  //          << ";error code:" << err_code;
  //   // monitor::common::monitor_can::mc()->SendFault(monitor::common::FaultLevel::TWO);
  // } else if (new_status == ComponentStatus_Status_WARN) {
  //   can_api.hurry_full_brake_req();
  //   can_api.send_ipc_dtc_req(monitor::common::FaultLevel::ONE, err_code);
  //   AERROR << "send fault level:" << monitor::common::FaultLevel::ONE
  //          << ";error code:" << err_code;
  //   // monitor::common::monitor_can::mc()->SendFault(monitor::common::FaultLevel::ONE);
  // }
  // AERROR << "err message:" << message;
/***********************************************************/
  if (new_status > current_status->status()) {
    current_status->set_status(new_status);
    if (!message.empty()) {
      current_status->set_message(message);
    } else {
      current_status->clear_message();
    }
  }
}

// Set interval to 0, so it runs every time when ticking.
SummaryMonitor::SummaryMonitor()
    : RecurrentRunner(FLAGS_summary_monitor_name, 0) {}

void SummaryMonitor::RunOnce(const double current_time) {
  auto manager = MonitorManager::Instance();
  auto* status = manager->GetStatus();
  status->mutable_summary_status()->clear_status();
  // Escalate the summary status to the most severe one.
  for (auto& component : *status->mutable_components()) {
    auto* summary = component.second.mutable_summary();
    const auto& process_status = component.second.process_status();
    EscalateStatus(process_status.status(),process_status.error_code(), process_status.message(), summary);
    const auto& channel_status = component.second.channel_status();
    EscalateStatus(channel_status.status(),channel_status.error_code(), channel_status.message(), summary);
    const auto& resource_status = component.second.resource_status();
    EscalateStatus(resource_status.status(),resource_status.error_code(), resource_status.message(),
                   summary);
    const auto& other_status = component.second.other_status();
    EscalateStatus(other_status.status(),other_status.error_code(), other_status.message(), summary);
    EscalateStatus(summary->status(),summary->error_code(), summary->message(), status->mutable_summary_status());
  }

  // Get fingerprint of current status.
  // Don't use DebugString() which has known bug on Map field. The string
  // doesn't change though the value has changed.
  static std::hash<std::string> hash_fn;
  std::string proto_bytes;
  status->SerializeToString(&proto_bytes);
  const size_t new_fp = hash_fn(proto_bytes);

  if (system_status_fp_ != new_fp ||
      current_time - last_broadcast_ > FLAGS_system_status_publish_interval) {
    static auto writer =
        manager->CreateWriter<SystemStatus>(FLAGS_system_status_topic);

    apollo::common::util::FillHeader("SystemMonitor", status);
    writer->Write(*status);
    status->clear_header();
    system_status_fp_ = new_fp;
    last_broadcast_ = current_time;
  }
}

}  // namespace monitor
}  // namespace apollo
