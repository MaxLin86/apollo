/*
 * @Author: your name
 * @Date: 2020-04-17 14:07:43
 * @LastEditTime: 2020-04-20 10:31:32
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: /apollo-5.5.0/modules/monitor/software/channel_monitor.h
 */
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
#include <string>
#include <unordered_map>

#include "modules/dreamview/proto/hmi_mode.pb.h"
#include "modules/monitor/common/recurrent_runner.h"
#include "modules/monitor/proto/system_status.pb.h"
#include "modules/monitor/software/latency_monitor.h"

#include "modules/monitor/common/monitor_can.h"
namespace apollo {
namespace monitor {

using  monitor::common::FaultLevel;
using  monitor::common::fault_into_t;
class ChannelMonitor : public RecurrentRunner {
 public:
  explicit ChannelMonitor(
      const std::shared_ptr<LatencyMonitor>& latency_monitor);
  void RunOnce(const double current_time) override;
  void init();
  void CheckChannelStatus();
  fault_into_t GetFaultInfo(std::shared_ptr<google::protobuf::Message> message,const std::string& channel);
  fault_into_t GetFaultInfo(const std::string& channel);
  int GetFaultCode(const std::string& channel);
 private:
  static void UpdateStatus(
      const apollo::dreamview::ChannelMonitorConfig& config,
      ComponentStatus* status, const bool update_freq, const double freq);
  std::shared_ptr<LatencyMonitor> latency_monitor_;
  std::unordered_map<std::string,fault_into_t> channel_fl_;
};

}  // namespace monitor
}  // namespace apollo
