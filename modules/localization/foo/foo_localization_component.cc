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

#include "modules/localization/foo/foo_localization_component.h"
#include "modules/common/time/time.h"

namespace apollo {
namespace localization {

using apollo::common::time::Clock;

FooLocalizationComponent::FooLocalizationComponent()
    : localization_(new FooLocalization()) {}

bool FooLocalizationComponent::Init() {
  
  tf2_broadcaster_.reset(new apollo::transform::TransformBroadcaster(node_));
  if (!InitConfig()) {
    AERROR << "Init Config falseed.";
    return false;
  }

  if (!InitIO()) {
    AERROR << "Init Interval falseed.";
    return false;
  }

  return true;
}

bool FooLocalizationComponent::InitConfig() {
  foo_config::Config foo_config;
  if (!apollo::cyber::common::GetProtoFromFile(config_file_path_,
                                               &foo_config)) {
    return false;
  }
  AINFO << "Foo localization config: " << foo_config.DebugString();

  localization_topic_ = foo_config.localization_topic();
  localization_status_topic_ = foo_config.localization_status_topic();
  imu_topic_ = foo_config.imu_topic();
  gps_topic_ = foo_config.gps_topic();
  gnss_heading_topic_ = foo_config.gnss_heading_topic();
  chassis_topic_ = foo_config.chassis_topic();
  routing_response_topic_ = foo_config.routing_response_topic();
  gps_status_topic_ = foo_config.gps_status_topic(); //--------------------
  broadcast_tf_frame_id_ = foo_config.broadcast_tf_frame_id();
  broadcast_tf_child_frame_id_ = foo_config.broadcast_tf_child_frame_id();

  localization_mode_= foo_config.localization_mode();

  localization_->InitConfig(foo_config);

  return true;
}

bool FooLocalizationComponent::InitIO() {
  /*
  imu_listener_ = node_->CreateReader<drivers::gnss::Imu>(
      imu_topic_, std::bind(&FooLocalization::ImuCallback, localization_.get(),
                            std::placeholders::_1));
  CHECK(imu_listener_);

  gps_status_listener_ = node_->CreateReader<drivers::gnss::InsStat>(
      gps_status_topic_, std::bind(&FooLocalization::GpsStatusCallback,
                                   localization_.get(), std::placeholders::_1));
  CHECK(gps_status_listener_);

  bestgnsspos_listener_ = node_->CreateReader<drivers::gnss::GnssBestPose>(
      gps_topic_, std::bind(&FooLocalization::BestGnssPosCallback,
                                   localization_.get(), std::placeholders::_1));
  CHECK(bestgnsspos_listener_);

  */ 
  /*
  imu_listener_ = node_->CreateReader<drivers::gnss::Imu>(   
    imu_topic_,
    [this](const std::shared_ptr<drivers::gnss::Imu>& imu) {
      AINFO << "Received imu data: run imu callback.";
      localization_->OnImu(*imu.get());
    });
    */

  bestgnsspos_listener_ = node_->CreateReader<drivers::gnss::GnssBestPose>(
    gps_topic_,
    [this](const std::shared_ptr<drivers::gnss::GnssBestPose>& best_gnss_pos) {
      AINFO << "Received gnss data: run gnss callback.";
      localization_->OnBestGnssPos(*best_gnss_pos.get());
    });

    gnss_heading_listener_ = node_->CreateReader<drivers::gnss::Heading>(
    gnss_heading_topic_,
    [this](const std::shared_ptr<drivers::gnss::Heading>& gnss_heading) {
      AINFO << "Received gnss heading data: run gnss callback.";
      localization_->OnGnssHeading(*gnss_heading.get());
    });

  chassis_listener_ = node_->CreateReader<canbus::Chassis>(
    chassis_topic_,
    [this](const std::shared_ptr<canbus::Chassis>& chassis) {
      AINFO << "Received chassis data: run chassis callback.";
      localization_->OnChassis(*chassis.get());
    });

  msf_localization_listener_ = node_->CreateReader<LocalizationEstimate>(
    "/apollo/localization/msf_pose",
    [this](const std::shared_ptr<LocalizationEstimate>& msf_localization) {
      AINFO << "Received chassis data: run chassis callback.";
      localization_->OnMsfLocalization(*msf_localization.get());
    });

  // add 20200708
  routing_reader_ = node_->CreateReader<routing::RoutingResponse>(
     routing_response_topic_,
      [this](const std::shared_ptr<routing::RoutingResponse>& routing) {
        ADEBUG << "Received routing data: run routing callback.";
       localization_->OnRouting(*routing.get());
      });      
      //----------------------------
     /**/


  localization_talker_ =
      node_->CreateWriter<LocalizationEstimate>(localization_topic_);
  CHECK(localization_talker_);

  localization_status_talker_ =
      node_->CreateWriter<LocalizationStatus>(localization_status_topic_);
  CHECK(localization_status_talker_);
  return true;
}

bool FooLocalizationComponent::Proc(
  const std::shared_ptr<drivers::gnss::Imu>& imu_msg) {
     localization_->OnImu(*imu_msg.get());

    LocalizationEstimate localization;
    LocalizationStatus localization_status; 

  if (localization_mode_ == 0) {
    localization_->GPSLocalizationFilterProc();
    localization_->FillGpsLocalizationFilterMsg(&localization, &localization_status);


  } else {
    localization_->ImuGpsLocalization();
    // if (localization_->IsServiceStarted()) {

      
    localization_->FillLocalizationMsg(&localization, &localization_status);
  

    localization_->LocalizationDRCorrect(localization);

    localization_->ReFillLocalizationMsg(&localization, &localization_status);

    if (0) {
    localization_->GuidepostCorrectKF(localization);
    localization_->ReFillLocalizationMsg2(&localization, &localization_status);
    }
  }

    //localization_->GetLocalization(&localization);

    //localization_->GetLocalizationStatus(&localization_status);

    // publish localization messages
    PublishPoseBroadcastTopic(localization);
    //PublishPoseBroadcastTF(localization);
    //PublishLocalizationStatus(localization_status);
    //ADEBUG << "[OnTimer]: Localization message publish success!";
  /*
  localization_->GpsCallback(gps_msg);
  if (localization_->IsServiceStarted()) {
    LocalizationEstimate localization;
    localization_->GetLocalization(&localization);
    LocalizationStatus localization_status;
    localization_->GetLocalizationStatus(&localization_status);

    // publish localization messages
    PublishPoseBroadcastTopic(localization);
    PublishPoseBroadcastTF(localization);
    PublishLocalizationStatus(localization_status);
    ADEBUG << "[OnTimer]: Localization message publish success!";
   
  }
 */
  return true;
}

void FooLocalizationComponent::PublishPoseBroadcastTF(
    const LocalizationEstimate& localization) {
  // broadcast tf message
  apollo::transform::TransformStamped tf2_msg;

  auto mutable_head = tf2_msg.mutable_header();
  mutable_head->set_timestamp_sec(localization.measurement_time());
  mutable_head->set_frame_id(broadcast_tf_frame_id_);
  tf2_msg.set_child_frame_id(broadcast_tf_child_frame_id_);

  auto mutable_translation = tf2_msg.mutable_transform()->mutable_translation();
  mutable_translation->set_x(localization.pose().position().x());
  mutable_translation->set_y(localization.pose().position().y());
  mutable_translation->set_z(localization.pose().position().z());

  auto mutable_rotation = tf2_msg.mutable_transform()->mutable_rotation();
  mutable_rotation->set_qx(localization.pose().orientation().qx());
  mutable_rotation->set_qy(localization.pose().orientation().qy());
  mutable_rotation->set_qz(localization.pose().orientation().qz());
  mutable_rotation->set_qw(localization.pose().orientation().qw());

  tf2_broadcaster_->SendTransform(tf2_msg);
}

void FooLocalizationComponent::PublishPoseBroadcastTopic(
    const LocalizationEstimate& localization) {
  localization_talker_->Write(localization);
}

void FooLocalizationComponent::PublishLocalizationStatus(
    const LocalizationStatus& localization_status) {
  localization_status_talker_->Write(localization_status);
}

}  // namespace localization
}  // namespace apollo
