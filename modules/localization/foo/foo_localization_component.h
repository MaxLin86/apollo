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

#pragma once

#include <memory>
#include <string>

#include "cyber/class_loader/class_loader.h"
#include "cyber/component/component.h"
#include "cyber/cyber.h"
#include "cyber/message/raw_message.h"

#include "modules/drivers/gnss/proto/imu.pb.h"

#include "modules/drivers/gnss/proto/ins.pb.h"
#include "modules/localization/proto/gps.pb.h"

#include "modules/localization/proto/imu.pb.h"
#include "modules/localization/proto/localization.pb.h"
#include "modules/localization/proto/foo_config.pb.h"
#include "modules/localization/foo/foo_localization.h"
#include "modules/transform/transform_broadcaster.h"

namespace apollo {
namespace localization {

class FooLocalizationComponent final
    : public cyber::Component<drivers::gnss::Imu> {
 public:
    FooLocalizationComponent();
  ~FooLocalizationComponent() = default;

  bool Init() override;

  bool Proc(const std::shared_ptr<drivers::gnss::Imu>& imu_msg) override;
  
 private:
  bool InitConfig();
  bool InitIO();

  void PublishPoseBroadcastTF(const LocalizationEstimate &localization);
  void PublishPoseBroadcastTopic(const LocalizationEstimate &localization);
  void PublishLocalizationStatus(const LocalizationStatus &localization_status);

 private:
  std::shared_ptr<cyber::Reader<drivers::gnss::Imu>>
      imu_listener_ = nullptr;
  std::shared_ptr<cyber::Reader<drivers::gnss::InsStat>> gps_status_listener_ =
      nullptr;
  std::shared_ptr<cyber::Reader<drivers::gnss::GnssBestPose>>
      bestgnsspos_listener_ = nullptr;

  std::shared_ptr<cyber::Reader<drivers::gnss::Heading>>
      gnss_heading_listener_ = nullptr;

  std::shared_ptr<cyber::Reader<canbus::Chassis>> chassis_listener_ = nullptr;

    std::shared_ptr<cyber::Reader<LocalizationEstimate>>msf_localization_listener_= nullptr;

  std::shared_ptr<cyber::Reader<routing::RoutingResponse>> routing_reader_ = 
      nullptr;

  std::shared_ptr<cyber::Writer<LocalizationEstimate>> localization_talker_ =
      nullptr;
  std::shared_ptr<cyber::Writer<LocalizationStatus>>
      localization_status_talker_ = nullptr;

  std::string localization_topic_ = "";
  std::string localization_status_topic_ = "";
  std::string gps_topic_ = "";
  std::string gnss_heading_topic_="";
  std::string gps_status_topic_ = "";
  std::string imu_topic_ = "";
  std::string best_gnsspos_topic_ = "";
  std::string chassis_topic_ = "";
  std::string routing_response_topic_ = "";

  std::string broadcast_tf_frame_id_ = "";
  std::string broadcast_tf_child_frame_id_ = "";
  std::unique_ptr<apollo::transform::TransformBroadcaster> tf2_broadcaster_;

  int localization_mode_ = 1;

  std::unique_ptr<FooLocalization> localization_;
};

CYBER_REGISTER_COMPONENT(FooLocalizationComponent);

}  // namespace localization
}  // namespace apollo
