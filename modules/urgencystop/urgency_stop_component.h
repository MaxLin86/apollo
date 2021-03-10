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
#include "modules/drivers/proto/ultra_radar.pb.h"
#include "modules/drivers/proto/conti_radar.pb.h"
#include "modules/urgencystop/proto/aeb_cmd.pb.h"
#include "modules/common/time/time.h"
#include "modules/urgencystop/proto/urgency.pb.h"
#include "modules/canbus/proto/chassis.pb.h"

#include "modules/localization/proto/localization.pb.h"
#include "modules/drivers/proto/ultrasonic_radar.pb.h"

namespace apollo {
namespace urgency {

using apollo::drivers::Ultra_Radar;
using apollo::drivers::Ultrasonic;
using apollo::drivers::ContiRadar;


class UrgencyStop : public cyber::Component<Ultra_Radar> {
 public:
  bool Init() override;
  bool Proc(const std::shared_ptr<Ultra_Radar>& ultraRadarMessage) override;

 private:
  void radarObjectFuse(std::vector<drivers::ContiRadarObs>& Obsvector,
                       drivers::ContiRadar& tmp_message);
  void messageCollection(drivers::ContiRadar& sub_message,
                         std::vector<drivers::ContiRadarObs>* Obsvector);
  bool judgeStopRules(drivers::ContiRadar& message);

  bool ultrasonicJudge(drivers::Ultrasonic& message);
  void outputFusedObj(drivers::ContiRadar& message);
  
  bool polyfitLine(drivers::ContiRadar& message);

  safeStopConf urgenStopConf_;
  std::vector<drivers::ContiRadarObs> leftObs_;
  std::vector<drivers::ContiRadarObs> rightObs_;
  std::vector<drivers::ContiRadarObs> leftCornerObs_;
  std::vector<drivers::ContiRadarObs> rightCornerObs_;

  std::shared_ptr<cyber::Writer<AEBCommand>> control_guardian_writer_;
  std::shared_ptr<apollo::cyber::Reader<apollo::canbus::Chassis>>
      chassis_reader_;
  std::shared_ptr<cyber::Writer<drivers::ContiRadar>> radar_writer_;

  std::shared_ptr<
      apollo::cyber::Reader<apollo::localization::LocalizationEstimate>>
      localization_reader_;

  float origin_dist_;
  float safety_zone_left_;
  float safety_zone_right_;

  uint32_t check_frame_;

  uint32_t check_count_;
  float match_size_;
  bool brakeFlag_;
  std::vector<double> veh_speed_;
  std::vector<double> stop_dis_;
  std::mutex mutex_;

  float begin_stop_dis_;
  int16_t stop_count_;
  double time_;

  bool radarBrakeFlag_;
  bool ultraBrakeFlag_;

  float steer_ratio_;
  float wheel_base_;
  float max_steer_angle_;

  float stop_px_ ;
  float stop_py_ ;
  float stop_pz_ ;
  float ttc_;
  float thw_;
  float ratio_;
  float dy_;

  float fence_ratio_;
  float fence_dist_;
  float dis2fence_;
  float fence_begin_stop_dis_;
  bool fence_flag_;
  bool steer_flag_;
  bool CorD_; // C:true; D:false;

//   int16_t idx ;

};
CYBER_REGISTER_COMPONENT(UrgencyStop)
}
}