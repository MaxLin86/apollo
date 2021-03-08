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

#include <list>
#include <memory>
#include <string>
#include <vector>

#include "gtest/gtest_prod.h"

#include "modules/common/monitor_log/monitor_log_buffer.h"
#include "modules/drivers/gnss/proto/ins.pb.h"

#include "modules/drivers/gnss/proto/imu.pb.h"

#include "modules/localization/proto/gps.pb.h"
#include "modules/localization/proto/imu.pb.h"
#include "modules/localization/proto/localization.pb.h"
#include "modules/localization/proto/foo_config.pb.h"
#include "modules/drivers/gnss/proto/gnss_best_pose.pb.h"
#include "modules/drivers/gnss/proto/heading.pb.h"

#include "modules/localization/foo/imu_gps_localizer/imu_gps_localizer.h"
#include "modules/localization/foo/self_car_tracking.h"

#include "modules/localization/foo/GeographicLib/LocalCartesian.h"
#include "modules/routing/proto/routing.pb.h"//add 20200708
#include "modules/localization/foo/tracking_object.h" //20200708

namespace apollo {
namespace localization {

class FooLocalization {
 public:
  FooLocalization();
  ~FooLocalization() = default;

  void InitConfig(const foo_config::Config &config);
  bool DR_ekf_init;
  void OnImu(const drivers::gnss::Imu& imu);
  void OnBestGnssPos(
    const drivers::gnss::GnssBestPose& best_gnss_pos);

  void OnGnssHeading(
    const drivers::gnss::Heading& gnss_heading);

void OnChassis(const canbus::Chassis& chassis);


  void OnMsfLocalization(const LocalizationEstimate& msf_localization);
  
  //add 20200708
  void OnRouting(const routing::RoutingResponse& routing);
  bool LoadMap();
  //---------------

  bool GPSLocalizationFilterProc();
  bool gps_localization_filter_init;

  void FillLocalizationMsg(LocalizationEstimate *localization,
    LocalizationStatus *localization_status);
      void ReFillLocalizationMsg(LocalizationEstimate *localization,
    LocalizationStatus *localization_status);
    bool guidepost_ekf_init;
      void ReFillLocalizationMsg2(LocalizationEstimate *localization,
    LocalizationStatus *localization_status);
    void FillGpsLocalizationFilterMsg(LocalizationEstimate *localization,
    LocalizationStatus *localization_status);

  void ImuGpsLocalization();
  void LocalizationDRCorrect(LocalizationEstimate &localization);
  bool GuidepostCorrectKF(LocalizationEstimate &localization);

  void GpsCallback(const std::shared_ptr<localization::Gps> &gps_msg);
  void GpsStatusCallback(
      const std::shared_ptr<drivers::gnss::InsStat> &status_msg);
  void ImuCallback(const std::shared_ptr<drivers::gnss::Imu> &imu_msg);

  bool IsServiceStarted();
  void GetLocalization(LocalizationEstimate *localization);
  void GetLocalizationStatus(LocalizationStatus *localization_status);

 private:
  void RunWatchDog(double gps_timestamp);

  void PrepareLocalizationMsg(const localization::Gps &gps_msg,
                              LocalizationEstimate *localization,
                              LocalizationStatus *localization_status);
  void ComposeLocalizationMsg(const drivers::gnss::GnssBestPose& best_gnss_pos,
                              const drivers::gnss::Imu &imu,
                              LocalizationEstimate *localization);
  void FillLocalizationMsgHeader(LocalizationEstimate *localization);
  void FillLocalizationStatusMsg(const drivers::gnss::InsStat &status,
                                 LocalizationStatus *localization_status);

  bool FindMatchingIMU(const double gps_timestamp_sec, drivers::gnss::Imu *imu_msg);
  bool InterpolateIMU(const drivers::gnss::Imu &imu1, const drivers::gnss::Imu &imu2,
                      const double timestamp_sec, drivers::gnss::Imu *imu_msg);
  template <class T>
  T InterpolateXYZ(const T &p1, const T &p2, const double frac1);

  bool FindNearestGpsStatus(const double gps_timestamp_sec,
                            drivers::gnss::InsStat *status);

 private:
  std::string module_name_ = "localization";


  drivers::gnss::Imu imu_;
  std::mutex imu_mutex_;
  drivers::gnss::GnssBestPose best_gnss_pose_;
  std::mutex best_gnsspos_mutex_;

  drivers::gnss::Heading gnss_heading_;
  std::mutex gnss_heading_mutex_;

  canbus::Chassis chassis_;

  std::mutex chassis_mutex_;

  routing::RoutingResponse routing_;
  std::mutex routing_mutex_;

  LocalizationEstimate msf_localization_;
  std::mutex msf_localization_mutex_;
  
  std::unique_ptr<TrackingSelfCarObject> tracking_selfcar_;

  std::shared_ptr<drivers::gnss::GnssBestPose> localview_gps_;
  
  std::unique_ptr<GPSLocalizationFilter> gps_localization_filter_;
  std::unique_ptr<TrailerLocalizationFilter> trailer_localization_filter_;

  std::unique_ptr<ImuGpsLocalizer> imu_gps_localizer_;
   std::unique_ptr<SelfCarLocalizationTracking> DR_correct_localizer_;

  LocalizationEstimate init_localization_;
  LocalizationEstimate start_localization_;

    std::list<drivers::gnss::Imu> imu_list_;
  size_t imu_list_max_size_ = 50;
  std::mutex imu_list_mutex_;

  std::list<drivers::gnss::GnssBestPose> best_gnsspos_list_;
  size_t best_gnsspos_list_max_size_ = 10;
  std::mutex best_gnsspos_list_mutex_;

  std::list<drivers::gnss::InsStat> gps_status_list_;
  size_t gps_status_list_max_size_ = 10;
  std::mutex gps_status_list_mutex_;

  std::vector<double> map_offset_;

  bool ins_localization_valid_ = true;
  bool guidepost_localization_valid_ = true;

  double gps_time_delay_tolerance_ = 1.0;
  double gps_imu_time_diff_threshold_ = 0.02;
  double gps_status_time_diff_threshold_ = 1.0;

  double last_received_timestamp_sec_ = 0.0;
  double last_reported_timestamp_sec_ = 0.0;

  bool enable_watch_dog_ = true;
  bool service_started_ = false;
  double service_started_time = 0.0;

  int64_t localization_seq_num_ = 0;
  LocalizationEstimate last_localization_result_;
  LocalizationStatus last_localization_status_result_;

  int localization_publish_freq_ = 100;
  int report_threshold_err_num_ = 10;
  int service_delay_threshold = 1;
  apollo::common::monitor::MonitorLogBuffer monitor_logger_;



  FRIEND_TEST(FooLocalizationTest, InterpolateIMU);
  FRIEND_TEST(FooLocalizationTest, ComposeLocalizationMsg);
};

}  // namespace localization
}  // namespace apollo
