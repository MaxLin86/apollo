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


#include "cyber/cyber.h"
#include "cyber/component/component.h"
#include "modules/drivers/radar/quill_radar/proto/quill.pb.h"
#include "modules/drivers/proto/conti_radar.pb.h"
#include "modules/drivers/proto/ultra_radar.pb.h"
#include "modules/common/time/time.h"

#include "modules/drivers/radar/quill_radar/driver/system-radar-software/engine/scp-src/eng-api/uhdp-msg-structs.h"
#include "modules/drivers/radar/quill_radar/driver/system-radar-software/engine/scp-src/eng-api/rdc-scanctrl.h"
#include "modules/localization/proto/localization.pb.h"

#include "modules/drivers/radar/uhnder_radar/objProcess.h"

#include "modules/common/monitor_log/monitor_log_buffer.h"

using apollo::cyber::Component;
using apollo::cyber::ComponentBase;
using apollo::drivers::uhnder_radar::RadarTrackProcess;

namespace apollo {
namespace drivers {
namespace uhnder_radar {

class QuillRadarComponent : public Component<> {
 public:
  ~QuillRadarComponent() {}
  bool Init() override;

 private:

  bool poll();

  QuillRadarConf  uhnder_radar_config_;

  std::thread device_thread_;
  std::string radar_id_;
  const char* radar_ip_;
  float install_agle_;
  float install_pos_x_;
  float install_pos_y_;
  float install_pos_z_;

  std::shared_ptr<apollo::cyber::Writer<ContiRadar>> detection_radar_writer_;
  std::shared_ptr<apollo::cyber::Writer<ContiRadar>> uhnder_radar_writer_;
  std::shared_ptr<apollo::cyber::Writer<Ultra_Radar>> stop_radar_writer_;
  std::shared_ptr<cyber::Writer<localization::LocalizationEstimate>> local_writer_;

  apollo::drivers::ContiRadar rawRadarMsg_;
  apollo::drivers::ContiRadar radarMsg_;
  apollo::drivers::Ultra_Radar tmpradarMsg_;
  float positive_;

  RadarTrackProcess*  track_process_;
  std::vector<track>  trackObjects_;
  uint32_t frame_count;

  // common::monitor::MonitorLogBuffer monitor_logger_buffer_;
};

CYBER_REGISTER_COMPONENT(QuillRadarComponent)

}  // namespace uhnder_radar
}  // namespace drivers
}  // namespace apollo
