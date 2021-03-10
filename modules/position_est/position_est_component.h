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
#include "modules/rms/proto/rms.pb.h"
#include "modules/lidar/proto/lidar.pb.h"
#include "modules/localization/proto/localization.pb.h"
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
  std::shared_ptr<cyber::Writer<PadMessage>> position_est_writer_;
  std::shared_ptr<apollo::cyber::Reader<apollo::canbus::Chassis>>
      chassis_reader_;
  std::shared_ptr<apollo::cyber::Reader<apollo::rms::msgTaskCoord>>
      tmcTask_reader_;
  std::shared_ptr<apollo::cyber::Reader<apollo::lidar::BenewakeLidar>>
      boxPos_reader_;      
  std::shared_ptr<cyber::Reader<apollo::localization::LocalizationEstimate>>
      localization_reader_;
   
  int validCnt_side_;
  int validCnt_top_;
  float dist_last_;

  PositionConf position_est_conf_;

  int smoothEnable_;
  float sResolution_;
  float sMin_;
  int sNum_;
  float motionStd_;
  float measStd_;
  Eigen::VectorXf coeffFiler_;
  Eigen::VectorXf pdfData_;
  double ds_;
  float veh_speed_last_;
  double timestamp_sec_last_; 

  uint32_t task_type_;
  uint32_t destination_type_;
  uint32_t machine_num_;
  uint32_t inplace_status_;
  uint32_t box_type_;
  uint32_t box_position_;

  uint32_t destination_type_last_;
  uint32_t box_position_last_;

  float boxpos_lidar_;

  double destination_area_offset_x1_;
  double destination_area_offset_y1_;
  double destination_area_offset_x2_;
  double destination_area_offset_y2_;

  bool side_distance_est(const std::shared_ptr<ContiRadar> &RadarMessage,
                         const SideConf& config, float &dist_est);
  bool top_distance_est(const std::shared_ptr<ContiRadar> &RadarMessage,
                        const TopConf& config, float &dist_est);
  bool top_distance_est2(const std::shared_ptr<ContiRadar> &RadarMessage,
                        const TopConf2& config, float &dist_est);
   void dist_compensation(const BoxRelPosConf& config, float &dist_est);
   void motion_update(float veh_speed,double timestamp_sec);
   void observation_update(float &dist_est);
   void map_shift(int sShift);
   void gaussian_filter_1D();
   float calc_norm_pdf(float d,float std);
   void normalize_probability(int &indexMaxPdf,float &maxPdf);
};
CYBER_REGISTER_COMPONENT(PositionEst)
}
}
