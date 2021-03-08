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
#include "modules/position_est/position_est_component.h"

#include <math.h>

#include <utility>

#include "modules/common/adapters/adapter_gflags.h"
#include "modules/common/util/message_util.h"
// #include "modules/drivers/canbus_api.h"
#include <fstream>  //文件流库函数
#include <iomanip>
#include <iostream>

namespace apollo {
namespace PositionEst {

bool PositionEst::Init() {

  if (!GetProtoConfig(&position_est_conf_)) {
    return false;
  }
  dist2Radar_ = position_est_conf_.dist2radar();
  float radar_height = position_est_conf_.radar_height();
  float target_height = position_est_conf_.target_height();
  delta_height_ = target_height - radar_height;
  px_diff_ = position_est_conf_.px_diff();
  py_diff_ = position_est_conf_.py_diff();
  snr_min_ = position_est_conf_.snr_min();
  snr_diff_ = position_est_conf_.snr_diff();
  sigma_ = position_est_conf_.sigma();

  position_est_writer_ =
      node_->CreateWriter<apollo::control::PadMessage>(FLAGS_pad_topic);

  cyber::ReaderConfig chassis_reader_config;
  chassis_reader_config.channel_name = FLAGS_chassis_topic;
  chassis_reader_config.pending_queue_size = 10;

  chassis_reader_ = node_->CreateReader<apollo::canbus::Chassis>(
      chassis_reader_config, nullptr);
  CHECK(chassis_reader_ != nullptr);

  count_ = 0;
  validCount = 0;
  return true;
}

bool PositionEst::Proc(const std::shared_ptr<ContiRadar>& RadarMessage) {
  std::lock_guard<std::mutex> lock(mutex_);

  float range = 0.0;
  auto obs = RadarMessage->contiobs();
  bool flag = false;

  int index_1 = 0;
  int index_2 = 0;

  std::vector<int> tmpIndex;
  for (int i = 0; i < obs.size(); i++) {
    auto obs_1 = obs[i];
    float x1 = obs_1.pos_x();
    float snr1 = obs_1.snr();
    if (snr1 < snr_min_ || x1 < delta_height_) {
      continue;
    }
    for (int j = i + 1; j < obs.size(); j++) {
      auto obs_2 = obs[j];
      float x2 = obs_2.pos_x();
      float snr2 = obs_2.snr();
      if (snr2 < snr_min_ ||  x2 < delta_height_ ) {
        continue;
      }
       
      if (fabs(obs_1.pos_x() - obs_2.pos_x()) < px_diff_ &&
          fabs(obs_1.pos_y()) < py_diff_ &&
          fabs(obs_2.pos_y() ) < py_diff_ &&
          fabs(obs_1.snr() - obs_2.snr()) < snr_diff_ &&
          obs_1.snr() > snr_min_ && obs_2.snr() > snr_min_) {
        index_1 = i;
        index_2 = j;
      }
    }
  }

  if (index_1 > 0 && index_2 > 0)
  {
    float sumPos = obs[index_1].pos_x() + obs[index_2].pos_x();
    float mean_pos_x = sumPos / 2;
    range = sqrt(mean_pos_x * mean_pos_x - delta_height_ * delta_height_) - dist2Radar_;
    if (range < 10 && range > position_est_conf_.min_range())
    {
      flag = true;
    }
  }

  PadMessage outputMessage;
  static float outrange = 0;
  if (flag) {
    fflush(NULL);
    outrange = 0.85 * range + 0.15 * outrange;
    range = range - position_est_conf_.offset();
    printf("range is : %.3f\n",range);
    outputMessage.set_moving_distance(range);
    common::util::FillHeader(node_->Name(), &outputMessage);
    position_est_writer_->Write(outputMessage);
  }

  count_++;
  radarTime = RadarMessage->header().timestamp_sec();
  return true;
}

}  // namespace PositionEst
}  // namespace apollo
