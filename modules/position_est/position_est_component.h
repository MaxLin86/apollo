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
#include <vector>

#include "cyber/cyber.h"
#include "cyber/component/component.h"
#include "modules/drivers/proto/conti_radar.pb.h"
#include "modules/common/time/time.h"
#include "modules/control/proto/pad_msg.pb.h"
#include "modules/position_est/proto/position_est_conf.pb.h"
#include "modules/canbus/proto/chassis.pb.h"
#include "Eigen/Eigen"

namespace apollo {
namespace PositionEst {

using apollo::drivers::ContiRadar;
using apollo::control::PadMessage;

class PositionEst : public cyber::Component<ContiRadar> {
 public:
  bool Init() override;
  bool Proc(const std::shared_ptr<ContiRadar>&RadarMessage ) override;

 private:
  std::mutex mutex_;
  std::shared_ptr<cyber::Writer< PadMessage>> position_est_writer_;
  std::shared_ptr<apollo::cyber::Reader<apollo::canbus::Chassis>>
      chassis_reader_;
  float delta_height_;
  float dist2Radar_;
  float px_diff_;
  float py_diff_;
  float snr_min_;
  float sigma_;
  uint32_t count_;
  int validCount;
  float snr_diff_;

  Eigen::Matrix2d P;
  Eigen::Vector2d X;
  double radarTime;

  PositionConf position_est_conf_;
};
CYBER_REGISTER_COMPONENT(PositionEst)
}
}
