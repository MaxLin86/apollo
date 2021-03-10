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

#include "modules/localization/foo/foo_localization.h"

#include <limits>

#include "modules/common/math/quaternion.h"
#include "modules/common/time/time.h"
#include "modules/drivers/gnss/proto/gnss_best_pose.pb.h"

#include "modules/common/util/util.h"

//add 20200708
#include "modules/map/pnc_map/pnc_map.h"
#include "modules/map/hdmap/hdmap_util.h"
#include "modules/localization/msf/common/util/frame_transform.h"
//------------------------

#define save_debug_info true

namespace apollo {
namespace localization {

using apollo::common::time::Clock;
using ::Eigen::Vector3d;

void split(const std::string& s, std::vector<std::string>& sv, const char flag = ' ') {
    sv.clear();
    std::istringstream iss(s);
    std::string temp;
    while (std::getline(iss, temp, flag)) {
        sv.push_back(temp);
    }
    return;
}

bool FooLocalization::LoadMap() {
  //to get guidepost world coordinate
  std::string mapPath = "/apollo/modules/map/data/demo/signpost_shzhw.txt";
  std::ifstream mapFile;
  mapFile.open(mapPath, std::ios::in);
  if (!mapFile.is_open()) {
    AERROR << "load guidepsot map file failed";
    return false;
  }
  std::string strLine;
  while (getline(mapFile, strLine))
  {
    if (strLine.empty())
      continue;
    std::vector<std::string> sv;
    split(strLine, sv, ',');
    if (sv.size() == 3)
    {
        NewGuidepostGroup guidepostGroup;
        guidepostGroup.id = sv[0];
        guidepostGroup.x = atof(sv[1].c_str());
        guidepostGroup.y = atof(sv[2].c_str());
        tracking_selfcar_->guidepost_groups_.push_back(guidepostGroup);
    }
  }
  mapFile.close();

  if ( tracking_selfcar_->guidepost_groups_.size() > 0) {
  } else
  {
    AERROR << "guidepos group num: 0";
  }
  
  AINFO << "guidepos group num: " <<  tracking_selfcar_->guidepost_groups_.size();
  std::cout << "guidepos group num: " <<  tracking_selfcar_->guidepost_groups_.size() << std::endl;
  return true;
}


FooLocalization::FooLocalization()
    : map_offset_{0.0, 0.0, 0.0},
      monitor_logger_(
          apollo::common::monitor::MonitorMessageItem::LOCALIZATION) {}

void FooLocalization::InitConfig(const foo_config::Config &config) {
  imu_list_max_size_ = config.imu_list_max_size();
  gps_imu_time_diff_threshold_ = config.gps_imu_time_diff_threshold();
  map_offset_[0] = config.map_offset_x();
  map_offset_[1] = config.map_offset_y();
  map_offset_[2] = config.map_offset_z();
//  imu_gps_localizer_ = std::unique_ptr<ImuGpsLocalizer>(new ImuGpsLocalizer(1e-1, 1e-2, 
//                                                    1e-1, 1e-4, Eigen::Vector3d(0.0, 0.0, 0.0))); // TODO 20200902
    imu_gps_localizer_ = std::unique_ptr<ImuGpsLocalizer>(new ImuGpsLocalizer(1e-6, 1e-6, 
                                                    1e-6, 1e-6, Eigen::Vector3d(0.150, 0.150, 0.0))); // TODO 20200902
  DR_correct_localizer_ = std::unique_ptr<SelfCarLocalizationTracking>(new SelfCarLocalizationTracking());
  int kf_coor_sys = 1; 
  DR_correct_localizer_->CreateKalmanFilter(kf_coor_sys);   
  DR_ekf_init=false;

  if (config.localization_mode()==0) {
    gps_localization_filter_ = std::unique_ptr<GPSLocalizationFilter>(new GPSLocalizationFilter());
  int kf_coor_sys = 1; 
  gps_localization_filter_->CreateKalmanFilter(kf_coor_sys);  
  gps_localization_filter_init = false;

  trailer_localization_filter_ = std::unique_ptr<TrailerLocalizationFilter>(new TrailerLocalizationFilter());
  trailer_localization_filter_->CreateKalmanFilter(kf_coor_sys);
  }

  //tracking_selfcar_ = std::make_unique<TrackingSelfCarObject>();
  tracking_selfcar_ =std::unique_ptr<TrackingSelfCarObject>(new TrackingSelfCarObject());
   LoadMap();
   kf_coor_sys = 0; 
   tracking_selfcar_->CreateINSreKalmanFilter(kf_coor_sys);   
   tracking_selfcar_->CreateGuidepostKalmanFilter(kf_coor_sys);   
  ins_localization_valid_ = config.ins_localization_valid();
  guidepost_localization_valid_ = config.guidepost_localization_valid();
  guidepost_ekf_init = false;
}

void FooLocalization::OnImu(const drivers::gnss::Imu& imu) {
  std::lock_guard<std::mutex> lock(imu_mutex_);
  imu_.CopyFrom(imu);
}
void FooLocalization::OnBestGnssPos(const drivers::gnss::GnssBestPose& best_gnss_pos) {
  std::lock_guard<std::mutex> lock(best_gnsspos_mutex_);
  best_gnss_pose_.CopyFrom(best_gnss_pos);
}

void FooLocalization::OnGnssHeading(const drivers::gnss::Heading& gnss_heading) {
  std::lock_guard<std::mutex> lock(gnss_heading_mutex_);
  gnss_heading_.CopyFrom(gnss_heading);
}

void FooLocalization::OnChassis(const canbus::Chassis& chassis) { 
    std::lock_guard<std::mutex> lock(chassis_mutex_);
    //    std::unique_lock<std::mutex> lock(chassis_mutex_);
    chassis_.CopyFrom(chassis);  

    //  lock.unlock();
}

void FooLocalization::OnMsfLocalization(const LocalizationEstimate& msf_localization) { 
    std::lock_guard<std::mutex> lock(msf_localization_mutex_);
    //    std::unique_lock<std::mutex> lock(chassis_mutex_);
    msf_localization_.CopyFrom(msf_localization);  

    //  lock.unlock();
}

//add 20200708
void FooLocalization::OnRouting(const routing::RoutingResponse& routing) {
  {
    std::lock_guard<std::mutex> lock(routing_mutex_);
    routing_.CopyFrom(routing);
  }
}
//------------------------------

void FooLocalization::FillLocalizationMsg(LocalizationEstimate *localization,
    LocalizationStatus *localization_status) {
      ComposeLocalizationMsg(best_gnss_pose_, imu_, localization);
}

void FooLocalization::ReFillLocalizationMsg(LocalizationEstimate *localization,
    LocalizationStatus *localization_status) {
  auto mutable_pose = localization->mutable_pose();
  auto mutable_uncertainty = localization->mutable_uncertainty();
 if (DR_ekf_init ) {
    const auto &DR_eskf_state = DR_correct_localizer_->kalman_filter_->state_post_;
    mutable_pose->mutable_position()->set_x(DR_eskf_state.at<double>(0,0));
    mutable_pose->mutable_position()->set_y(DR_eskf_state.at<double>(1,0));
    const auto &DR_eskf_error = DR_correct_localizer_->kalman_filter_->error_cov_post_;
    mutable_uncertainty->mutable_position_std_dev()->set_x(std::sqrt(DR_eskf_error.at<double>(0,0)));
    mutable_uncertainty->mutable_position_std_dev()->set_y(std::sqrt(DR_eskf_error.at<double>(1,1)));    

    mutable_pose->set_heading(common::math::NormalizeAngle(DR_eskf_state.at<double>(2,0)));
    mutable_uncertainty->set_heading_std_dev(std::sqrt(DR_eskf_error.at<double>(2,2)));
    
  }
}

void FooLocalization::ReFillLocalizationMsg2(LocalizationEstimate *localization,
    LocalizationStatus *localization_status) {
  auto mutable_pose = localization->mutable_pose();
  auto mutable_uncertainty = localization->mutable_uncertainty();
  std::string source_msg =  "";
  if (ins_localization_valid_ && guidepost_localization_valid_ && tracking_selfcar_ &&
        tracking_selfcar_->new_local_view_.localization_estimate &&
        tracking_selfcar_->new_local_view_.routing) {
    if (tracking_selfcar_->new_local_view_.localization_estimate->has_header() && 
        tracking_selfcar_->new_local_view_.routing->has_header()) {
      source_msg =  "gnss and guidepost";   
    }  else if (tracking_selfcar_->new_local_view_.localization_estimate->has_header()) {
      source_msg =  "gnss";   
    } else if (tracking_selfcar_->new_local_view_.routing->has_header()) {
      source_msg =  "guidepost";   
    } else {
      source_msg = "null"; 
    }
  } else if (guidepost_localization_valid_ && tracking_selfcar_ &&
        tracking_selfcar_->new_local_view_.routing) {
    if (tracking_selfcar_->new_local_view_.routing->has_header()) {
      source_msg =  "guidepost";   
    } else {
      source_msg = "null"; 
    }        
  } else if(ins_localization_valid_ && tracking_selfcar_ &&
        tracking_selfcar_->new_local_view_.localization_estimate) {
    if (tracking_selfcar_->new_local_view_.localization_estimate->has_header()) {
      source_msg =  "gnss";   
    } else {
      source_msg = "null"; 
    } 
  } else {
    source_msg = "null";
  }
  localization->set_localization_source(source_msg);
 if (guidepost_ekf_init ) {
    const auto &guidepost_ekf_state = tracking_selfcar_->guidepost_kalman_filter_->state_post_;
    mutable_pose->mutable_position()->set_x(guidepost_ekf_state.at<double>(0,0));
    mutable_pose->mutable_position()->set_y(guidepost_ekf_state.at<double>(1,0));
    const auto &guidepost_ekf_cov = tracking_selfcar_->guidepost_kalman_filter_->error_cov_post_;
    mutable_uncertainty->mutable_position_std_dev()->set_x(std::sqrt(guidepost_ekf_cov.at<double>(0,0)));
    mutable_uncertainty->mutable_position_std_dev()->set_y(std::sqrt(guidepost_ekf_cov.at<double>(1,1)));    

    mutable_pose->set_heading(common::math::NormalizeAngle(guidepost_ekf_state.at<double>(2,0)));
    mutable_uncertainty->set_heading_std_dev(std::sqrt(guidepost_ekf_cov.at<double>(2,2)));

    const auto cur_perception_guidepost_index = tracking_selfcar_->cur_perception_guidepost_index_;

    if (tracking_selfcar_->new_local_view_.routing->has_header()) {
      localization->set_current_perception_guidepost_index(cur_perception_guidepost_index);
    }
  }
}

void FooLocalization::FillGpsLocalizationFilterMsg(LocalizationEstimate *localization,
    LocalizationStatus *localization_status) {

  FillLocalizationMsgHeader(localization);

  auto mutable_pose = localization->mutable_pose();
  auto mutable_uncertainty = localization->mutable_uncertainty();
  auto mutable_trailer_pose = localization->mutable_trailer_pose();
  auto mutable_trailer_uncertainty = localization->mutable_trailer_uncertainty();
 if (gps_localization_filter_init) {
    const auto &gps_kf_state = gps_localization_filter_->kalman_filter_->state_post_;
    mutable_pose->mutable_position()->set_x(gps_kf_state.at<double>(0,0));
    mutable_pose->mutable_position()->set_y(gps_kf_state.at<double>(1,0));
    const auto &gps_kf_error = gps_localization_filter_->kalman_filter_->error_cov_post_;
    mutable_uncertainty->mutable_position_std_dev()->set_x(std::sqrt(gps_kf_error.at<double>(0,0)));
    mutable_uncertainty->mutable_position_std_dev()->set_y(std::sqrt(gps_kf_error.at<double>(1,1)));    

    mutable_pose->set_heading(common::math::NormalizeAngle(gps_kf_state.at<double>(2,0)));
    mutable_uncertainty->set_heading_std_dev(std::sqrt(gps_kf_error.at<double>(2,2)));

    // trailer localization info
    const auto &trailer_kf_state = trailer_localization_filter_->kalman_filter_->state_post_;
    mutable_trailer_pose->mutable_position()->set_x(trailer_kf_state.at<double>(0,0));
    mutable_trailer_pose->mutable_position()->set_y(trailer_kf_state.at<double>(1,0));
    const auto &trailer_kf_error = trailer_localization_filter_->kalman_filter_->error_cov_post_;
    mutable_trailer_uncertainty->mutable_position_std_dev()->set_x(std::sqrt(trailer_kf_error.at<double>(0,0)));
    mutable_trailer_uncertainty->mutable_position_std_dev()->set_y(std::sqrt(trailer_kf_error.at<double>(1,1)));    

    mutable_trailer_pose->set_heading(common::math::NormalizeAngle(trailer_kf_state.at<double>(2,0)));
    mutable_trailer_uncertainty->set_heading_std_dev(std::sqrt(trailer_kf_error.at<double>(2,2)));
    
  }
}

void FooLocalization::ComposeLocalizationMsg(
    const drivers::gnss::GnssBestPose& best_gnss_pos, const drivers::gnss::Imu &imu,
    LocalizationEstimate *localization) {
  localization->Clear();

  FillLocalizationMsgHeader(localization);

  localization->set_measurement_time(imu.header().timestamp_sec());
  
  // combine gps and imu
  
  auto mutable_pose = localization->mutable_pose();
  if(best_gnss_pose_.has_header()) {
    if(imu_gps_localizer_) {
      // position
      // world frame -> map frame
      const auto &eskf_state = imu_gps_localizer_->GetState();
      mutable_pose->mutable_position()->set_x(eskf_state.G_p_I.x() -
                                              map_offset_[0]);
      mutable_pose->mutable_position()->set_y(eskf_state.G_p_I.y() -
                                              map_offset_[1]);
      mutable_pose->mutable_position()->set_z(eskf_state.G_p_I.z() -
                                              map_offset_[2]);
      // linear velocity
      mutable_pose->mutable_linear_velocity()->set_x(eskf_state.G_v_I.x());
      mutable_pose->mutable_linear_velocity()->set_y(eskf_state.G_v_I.y());
      mutable_pose->mutable_linear_velocity()->set_z(eskf_state.G_v_I.z()); 

      //orientation
      //mutable_pose->mutable_orientation()->set_qx(eskf_state.G_R_I.x());
      //mutable_pose->mutable_orientation()->set_qy(eskf_state.G_R_I.y());
      //mutable_pose->mutable_orientation()->set_qz(eskf_state.G_R_I.z());

      // std_dev
      auto mutable_uncertainty = localization->mutable_uncertainty();
      mutable_uncertainty->mutable_position_std_dev()->set_x(std::sqrt(eskf_state.cov(0,0)));
      mutable_uncertainty->mutable_position_std_dev()->set_y(std::sqrt(eskf_state.cov(1,1)));
      mutable_uncertainty->mutable_position_std_dev()->set_z(std::sqrt(eskf_state.cov(2,2)));
      mutable_uncertainty->mutable_linear_velocity_std_dev()->set_x(std::sqrt(eskf_state.cov(3,3)));
      mutable_uncertainty->mutable_linear_velocity_std_dev()->set_y(std::sqrt(eskf_state.cov(4,4)));
      mutable_uncertainty->mutable_linear_velocity_std_dev()->set_z(std::sqrt(eskf_state.cov(5,5)));
      //mutable_uncertainty->mutable_orientation_std_dev()->set_x(std::sqrt(eskf_state.cov(6,6)));
      //mutable_uncertainty->mutable_orientation_std_dev()->set_y(std::sqrt(eskf_state.cov(7,7)));
      //mutable_uncertainty->mutable_orientation_std_dev()->set_z(std::sqrt(eskf_state.cov(8,8)));



    }
  }
  /*
  if (imu.has_header()) { 
    // linear acceleration
    if (imu.has_linear_acceleration()) {
      if (localization->pose().has_orientation()) {
        // linear_acceleration:
        // convert from vehicle reference to map reference
        Vector3d orig(imu.linear_acceleration().x(),
                      imu.linear_acceleration().y(),
                      imu.linear_acceleration().z());
        Vector3d vec = common::math::QuaternionRotate(
            localization->pose().orientation(), orig);
        mutable_pose->mutable_linear_acceleration()->set_x(vec[0]);
        mutable_pose->mutable_linear_acceleration()->set_y(vec[1]);
        mutable_pose->mutable_linear_acceleration()->set_z(vec[2]);

        // linear_acceleration_vfr
        mutable_pose->mutable_linear_acceleration_vrf()->CopyFrom(
            imu.linear_acceleration());
      } else {
        AERROR << "[PrepareLocalizationMsg]: "
               << "fail to convert linear_acceleration";
      }
    }

    // angular velocity
    if (imu.has_angular_velocity()) {
      if (localization->pose().has_orientation()) {
        // angular_velocity:
        // convert from vehicle reference to map reference
        Vector3d orig(imu.angular_velocity().x(), imu.angular_velocity().y(),
                      imu.angular_velocity().z());
        Vector3d vec = common::math::QuaternionRotate(
            localization->pose().orientation(), orig);
        mutable_pose->mutable_angular_velocity()->set_x(vec[0]);
        mutable_pose->mutable_angular_velocity()->set_y(vec[1]);
        mutable_pose->mutable_angular_velocity()->set_z(vec[2]);

        // angular_velocity_vf
        mutable_pose->mutable_angular_velocity_vrf()->CopyFrom(
            imu.angular_velocity());
      } else {
        AERROR << "[PrepareLocalizationMsg]: fail to convert angular_velocity";
      }
    }

    // euler angle
   // if (imu.has_euler_angles()) {
   //   mutable_pose->mutable_euler_angles()->CopyFrom(imu.euler_angles());
   // }
  }
  */
}
/*
void FooLocalization::GpsCallback(
    const std::shared_ptr<localization::Gps> &gps_msg) {
  double time_delay =
      last_received_timestamp_sec_
          ? common::time::Clock::NowInSeconds() - last_received_timestamp_sec_
          : last_received_timestamp_sec_;
  if (time_delay > gps_time_delay_tolerance_) {
    std::stringstream ss;
    ss << "GPS message time interval: " << time_delay;
    monitor_logger_.WARN(ss.str());
  }

  {
    std::unique_lock<std::mutex> lock(imu_list_mutex_);

    if (imu_list_.empty()) {
      AERROR << "IMU message buffer is empty.";
      if (service_started_) {
        monitor_logger_.ERROR("IMU message buffer is empty.");
      }
      return;
    }
  }

  {
    std::unique_lock<std::mutex> lock(gps_status_list_mutex_);

    if (gps_status_list_.empty()) {
      AERROR << "Gps status message buffer is empty.";
      if (service_started_) {
        monitor_logger_.ERROR("Gps status message buffer is empty.");
      }
      return;
    }
  }

  // publish localization messages
  PrepareLocalizationMsg(*gps_msg, &last_localization_result_,
                         &last_localization_status_result_);
  service_started_ = true;
  if (service_started_time == 0.0) {
    service_started_time = common::time::Clock::NowInSeconds();
  }

  // watch dog
  RunWatchDog(gps_msg->header().timestamp_sec());

  last_received_timestamp_sec_ = common::time::Clock::NowInSeconds();
}
*/
/*
void FooLocalization::GpsStatusCallback(
    const std::shared_ptr<drivers::gnss::InsStat> &status_msg) {
  std::unique_lock<std::mutex> lock(gps_status_list_mutex_);
  if (gps_status_list_.size() < gps_status_list_max_size_) {
    gps_status_list_.push_back(*status_msg);
  } else {
    gps_status_list_.pop_front();
    gps_status_list_.push_back(*status_msg);
  }
}

void FooLocalization::GpsCallback(
    const std::shared_ptr<drivers::gnss::InsStat> &status_msg) {
  std::unique_lock<std::mutex> lock(gps_status_list_mutex_);
  if (gps_status_list_.size() < gps_status_list_max_size_) {
    gps_status_list_.push_back(*status_msg);
  } else {
    gps_status_list_.pop_front();
    gps_status_list_.push_back(*status_msg);
  }
}

void FooLocalization::ImuCallback(
    const std::shared_ptr<drivers::gnss::Imu> &imu_msg) {
  std::unique_lock<std::mutex> lock(imu_list_mutex_);
  if (imu_list_.size() < imu_list_max_size_) {
    imu_list_.push_back(*imu_msg);
  } else {
    imu_list_.pop_front();
    imu_list_.push_back(*imu_msg);
  }
}

void FooLocalization::BestGnssPosCallback(
    const std::shared_ptr<drivers::gnss::GnssBestPose> &best_gnss_pos_msg) {
  std::unique_lock<std::mutex> lock(best_gnsspos_list_mutex_);
  if (best_gnsspos_list_.size() < best_gnsspos_list_max_size_) {
    best_gnsspos_list_.push_back(*best_gnss_pos_msg);
  } else {
    best_gnsspos_list_.pop_front();
    best_gnsspos_list_.push_back(*best_gnss_pos_msg);
  }
}
*/

bool FooLocalization::IsServiceStarted() { return service_started_; }

void FooLocalization::GetLocalization(LocalizationEstimate *localization) {
  *localization = last_localization_result_;
  
}

void FooLocalization::GetLocalizationStatus(
    LocalizationStatus *localization_status) {
  *localization_status = last_localization_status_result_;
}

bool FooLocalization::GPSLocalizationFilterProc() {

  if(!best_gnss_pose_.has_header()) {return false;}
  if(!chassis_.has_speed_mps()) {return false;} // to avoid ctrl variable appears nan

  if(! gps_localization_filter_->local_view_.chassis || 
     (gps_localization_filter_->local_view_.chassis->header().timestamp_sec()!=  chassis_.header().timestamp_sec())) {

    std::unique_lock<std::mutex> lock(chassis_mutex_);    
    gps_localization_filter_->local_view_.chassis = std::make_shared<canbus::Chassis>(chassis_);
    trailer_localization_filter_->local_view_.chassis = std::make_shared<canbus::Chassis>(chassis_);
  }
  
  static bool update_gps = false;
  static int update_gps_count = 0;
  if((!localview_gps_ || 
    (localview_gps_->header().timestamp_sec() != best_gnss_pose_.header().timestamp_sec())) /*&&
    (best_gnss_pose_.num_sats_tracked() > 2)*/) {
    std::lock_guard<std::mutex> lock(best_gnsspos_mutex_);
    localview_gps_ = std::make_shared<drivers::gnss::GnssBestPose>(best_gnss_pose_); 
    update_gps = true;
    if (localview_gps_->sol_type() == drivers::gnss::NARROW_INT ||
        localview_gps_->sol_type() == drivers::gnss::NARROW_FLOAT) {
      update_gps_count++;
    }
  } else {
    update_gps = false;
  }

  static cv::Point2d start_utm_xy;
  static cv::Point2d init_utm_xy;
  if (!gps_localization_filter_init) {

    if (update_gps_count == 1) {
      msf::UTMCoor utm_xy;
      msf::FrameTransform::LatlonToUtmXY(localview_gps_->longitude()/180.0*M_PI,
                                  localview_gps_->latitude()/180.0*M_PI, &utm_xy);
        start_utm_xy.x = utm_xy.x;
        start_utm_xy.y = utm_xy.y;
    } else if (update_gps_count > 5) {
      msf::UTMCoor utm_xy;
      msf::FrameTransform::LatlonToUtmXY(localview_gps_->longitude()/180.0*M_PI,
                                  localview_gps_->latitude()/180.0*M_PI, &utm_xy);
      init_utm_xy.x = utm_xy.x;
      init_utm_xy.y = utm_xy.y;
    
      double dx = init_utm_xy.x - start_utm_xy.x;
      double dy = init_utm_xy.y - start_utm_xy.y; 
      const double gnss_heading = gnss_heading_.heading();//-180.0;//\\\\\\\\\\\\\\\\\\\\\\\\\\//

      bool gnss_heading_dependable = false;
      if (gnss_heading_.heading_std_dev() < 1.0 && gnss_heading_.heading_std_dev() > 1e-6) {
        gnss_heading_dependable = true;
      }
      bool gnss_pose_dependable = false;
      if ((localview_gps_->sol_type() == drivers::gnss::NARROW_INT ||
        localview_gps_->sol_type() == drivers::gnss::NARROW_FLOAT) &&
        (localview_gps_->latitude_std_dev() < 0.2 && localview_gps_->longitude_std_dev() < 0.2)) {
          gnss_pose_dependable = true;
        }

      if (gnss_heading_dependable && gnss_pose_dependable) {
        gps_localization_filter_init =  gps_localization_filter_->InitKalmanFilter0(gps_localization_filter_->local_view_,
            start_utm_xy, init_utm_xy, gnss_heading);
      }  else  if (dx*dx + dy*dy > 1.0) {
        gps_localization_filter_init =  gps_localization_filter_->InitKalmanFilter(gps_localization_filter_->local_view_,
            start_utm_xy, init_utm_xy, gnss_heading, gnss_heading_.heading_std_dev());     
      }
    }
  }

  if (gps_localization_filter_init ) {
        double gnss_heading_std_dev = gnss_heading_.heading_std_dev();
    gps_localization_filter_->KalmanFilterPredict(gps_localization_filter_->local_view_,gnss_heading_std_dev);
  }

  if (gps_localization_filter_init && update_gps) {
      cv::Point2d cur_utm_xy;
      msf::UTMCoor utm_xy;
      msf::FrameTransform::LatlonToUtmXY(localview_gps_->longitude()/180.0*M_PI,
                                  localview_gps_->latitude()/180.0*M_PI, &utm_xy);
      cur_utm_xy.x = utm_xy.x;
      cur_utm_xy.y = utm_xy.y; 
      double gnss_heading = gnss_heading_.heading(); 
      double gnss_heading_std_dev = gnss_heading_.heading_std_dev();
      /*if (std::fabs(chassis_.steering_percentage() > 30)) {
        gnss_heading_std_dev *=300.0;
      }*/

      double obs_dx = cur_utm_xy.x - gps_localization_filter_->kalman_filter_->state_post_.at<double>(0,0);
      double obs_dy = cur_utm_xy.y - gps_localization_filter_->kalman_filter_->state_post_.at<double>(1,0);
      cv::Point2d utm_xy_std_dev =   cv::Point2d(localview_gps_->longitude_std_dev(), localview_gps_->latitude_std_dev());
      
      bool is_msf_heading = false;
      if (msf_localization_.has_header()) {
        is_msf_heading = true;
        gnss_heading = msf_localization_.pose().heading();
        gnss_heading_std_dev = msf_localization_.uncertainty().orientation_std_dev().z() /2.0/M_PI*180.0;
      } else {
        is_msf_heading = false;
      }

      
      if(obs_dx*obs_dx + obs_dy*obs_dy < 200 /*&& (localview_gps_->sol_type() == drivers::gnss::NARROW_INT ||
        localview_gps_->sol_type() == drivers::gnss::NARROW_FLOAT)*/ ) {//add 20201019
        gps_localization_filter_->KalmanFilterCorrect(gps_localization_filter_->local_view_, cur_utm_xy,utm_xy_std_dev,
              gnss_heading,gnss_heading_std_dev, is_msf_heading);
      }
      
      #if save_debug_info
        std::ofstream obs_gnss_out;
        obs_gnss_out.open("obs_gnss_out.txt",std::ios::app);
        obs_gnss_out <<std::fixed << chassis_.header().timestamp_sec() << "\t" << cur_utm_xy.x << "\t" <<cur_utm_xy.y << "\t" << 0 << std::endl;  
    
      #endif
      

  }
  if (gps_localization_filter_init) {
    
    #if save_debug_info
      double selfcar_px2 = gps_localization_filter_->kalman_filter_->state_post_.at<double>(0,0);
      double selfcar_py2 = gps_localization_filter_->kalman_filter_->state_post_.at<double>(1,0);
      double selfcar_theta2 = gps_localization_filter_->kalman_filter_->state_post_.at<double>(2,0);
      {
          std::ofstream ekf_out;
          ekf_out.open("map_ekf_result_out3.txt", std::ios::out | std::ios::app);
          ekf_out << std::fixed << gps_localization_filter_->local_view_.chassis->header().timestamp_sec() << "\t" <<
          0 << "\t" <<
          0 << "\t" <<
          gnss_heading_.heading() << "\t" <<
            selfcar_px2 << "\t" << selfcar_py2 <<  "\t"<< selfcar_theta2  << "\t"<<  std::endl;
      
          ekf_out.close();
      }
    
    #endif 
  
  }

  // trailer localization estimate
  // todo
  if (gps_localization_filter_init) {
    static bool trailer_localization_filter_init = false;
    cv::Point2d vehicle_xy;
    vehicle_xy.x = gps_localization_filter_->kalman_filter_->state_post_.at<double>(0,0);
    vehicle_xy.y = gps_localization_filter_->kalman_filter_->state_post_.at<double>(1,0);
    double vehicle_heading =  gps_localization_filter_->kalman_filter_->state_post_.at<double>(2,0);    

    if (!trailer_localization_filter_init) {
      double init_trailer_heading = vehicle_heading;
      trailer_localization_filter_init = trailer_localization_filter_->InitKalmanFilter(vehicle_xy, vehicle_heading, init_trailer_heading);
    }

    if (trailer_localization_filter_init) {
      trailer_localization_filter_->KalmanFilterPredict(trailer_localization_filter_->local_view_);

      cv::Point2d vehicle_xy_std_dev;
      double vehicle_heading_std_dev;
      trailer_localization_filter_->KalmanFilterCorrect(vehicle_xy, vehicle_xy_std_dev,
        vehicle_heading, vehicle_heading_std_dev);

      #if save_debug_info
      std::ofstream trailter_localization_out;
      trailter_localization_out.open("trailter_localization.txt",std::ios::out | std::ios::app);

      trailter_localization_out << std::fixed << trailer_localization_filter_->kalman_filter_->state_post_.at<double>(0,0) << "\t" 
      <<  trailer_localization_filter_->kalman_filter_->state_post_.at<double>(1,0) << "\t" 
      <<  trailer_localization_filter_->kalman_filter_->state_post_.at<double>(2,0) << "\t" 
      <<  trailer_localization_filter_->kalman_filter_->state_post_.at<double>(3,0) << "\t" 
      <<  trailer_localization_filter_->kalman_filter_->state_post_.at<double>(4,0) << "\t" 
      << trailer_localization_filter_->kalman_filter_->state_post_.at<double>(5,0) << "\n";
      
      #endif

    }
  }




  return true;

}


void FooLocalization::ImuGpsLocalization() {
  static State fused_state;
  /*
 if(!localview_gps_ || 
      !common::util::IsProtoEqual(*localview_gps_, best_gnss_pose_)) {
        localview_gps_ = std::make_shared<drivers::gnss::GnssBestPose>(best_gnss_pose_);
  }
*/
  //imu_gps_localizer_->ProcessImuData();

  if (imu_.has_header()) {
  ImuData cur_imudata;
  cur_imudata.timestamp = imu_.header().timestamp_sec();
  cur_imudata.acc.x() = imu_.linear_acceleration().x()-(-0.227661133);// - (-0.36);//+0.15;
  cur_imudata.acc.y() = imu_.linear_acceleration().y()-0.298461914;// - (0.323);// - 0.39;
  cur_imudata.acc.z() = imu_.linear_acceleration().z();//0;//-
  cur_imudata.gyro.x() = imu_.angular_velocity().x()-(0.001443142);// - (0.00145);// - 0.0016;
  cur_imudata.gyro.y() = imu_.angular_velocity().y()-(0.001660156);//-0.003;
  cur_imudata.gyro.z() = imu_.angular_velocity().z()-(-0.001182726);//-(3.7760e-05);
  const ImuDataPtr cur_imu_data_ptr = std::make_shared<ImuData>(cur_imudata);
  imu_gps_localizer_->ProcessImuData(cur_imu_data_ptr, &fused_state);

  }

  //if(!localview_gps_ || 
  //   !common::util::IsProtoEqual(*localview_gps_, best_gnss_pose_)) {
  if((!localview_gps_ || 
     (localview_gps_->header().timestamp_sec() != best_gnss_pose_.header().timestamp_sec())) /*&&
     (best_gnss_pose_.num_sats_tracked() > 2)*/) {
      static int imu_pgs_correct_count =0;
    localview_gps_ = std::make_shared<drivers::gnss::GnssBestPose>(best_gnss_pose_); 
  GpsPositionData cur_gpsdata;
  cur_gpsdata.timestamp = best_gnss_pose_.header().timestamp_sec();
  cur_gpsdata.lla.x() = best_gnss_pose_.latitude();//+(rand()%2 -1) * rand()%100/20000.0;//path_c1 += (rand()%2 -1) * rand()%10/500.0;
  cur_gpsdata.lla.y() = best_gnss_pose_.longitude();//+(rand()%2 -1) * rand()%100/20000.0;
  cur_gpsdata.lla.z() = best_gnss_pose_.height_msl(); //---0.0;//----------------
  cur_gpsdata.cov(0,1) = 0.0;
  cur_gpsdata.cov(0,2) = 0.0;
  cur_gpsdata.cov(1,0) = 0.0;
  cur_gpsdata.cov(1,2) = 0.0;
  cur_gpsdata.cov(2,0) = 0.0;
  cur_gpsdata.cov(2,1) = 0.0;
  cur_gpsdata.cov(0,0) = best_gnss_pose_.latitude_std_dev();//* best_gnss_pose_.latitude_std_dev();//3.0;//20.0*best_gnss_pose_.latitude_std_dev();//best_gnss_pose_.latitude_std_dev()* best_gnss_pose_.latitude_std_dev();//1.0
  cur_gpsdata.cov(1,1) = best_gnss_pose_.longitude_std_dev();//* best_gnss_pose_.longitude_std_dev();// 3.0;//20.0*best_gnss_pose_.longitude_std_dev();//best_gnss_pose_.longitude_std_dev()* best_gnss_pose_.longitude_std_dev();/1.0
  cur_gpsdata.cov(2,2) = best_gnss_pose_.height_std_dev();// * best_gnss_pose_.height_std_dev();// 3.0;//20.0*best_gnss_pose_.height_std_dev();//best_gnss_pose_.height_std_dev()* best_gnss_pose_.height_std_dev();//1.0 //0.001,0.001,0.001,0.001
  //cur_gpsdata.cov <<
  const GpsPositionDataPtr cur_gps_data_ptr = std::make_shared<GpsPositionData>(cur_gpsdata);
  imu_gps_localizer_->ProcessGpsPositionData(cur_gps_data_ptr);

  imu_pgs_correct_count ++;

  if (imu_pgs_correct_count == 3) {   
    const auto &init_eskf_state = imu_gps_localizer_->GetState();
    apollo::common::Header header;
    start_localization_.set_allocated_header(&header);

    start_localization_.mutable_pose()->mutable_position()->set_x(init_eskf_state.G_p_I.x() -
                                            map_offset_[0]);
    start_localization_.mutable_pose()->mutable_position()->set_y(init_eskf_state.G_p_I.y() -
                                            map_offset_[1]);
    start_localization_.mutable_pose()->mutable_position()->set_z(init_eskf_state.G_p_I.z() -
                                            map_offset_[2]);
  } else if (!init_localization_.has_header() && imu_pgs_correct_count >= 4) { 
  //else if (imu_pgs_correct_count == 3) {   
    const auto &cur_eskf_state = imu_gps_localizer_->GetState();
    double dx = (cur_eskf_state.G_p_I.x() - map_offset_[0]) - start_localization_.pose().position().x();
    double dy = (cur_eskf_state.G_p_I.y() - map_offset_[1]) - start_localization_.pose().position().y();
    double dis2 = dx*dx + dy*dy;
    if (dis2 > 0.3) {
      apollo::common::Header header;
      init_localization_.set_allocated_header(&header);

      init_localization_.mutable_pose()->mutable_position()->set_x(cur_eskf_state.G_p_I.x() -
                                              map_offset_[0]);
      init_localization_.mutable_pose()->mutable_position()->set_y(cur_eskf_state.G_p_I.y() -
                                              map_offset_[1]);
      init_localization_.mutable_pose()->mutable_position()->set_z(cur_eskf_state.G_p_I.z() -
                                              map_offset_[2]);
    }
  }

  std::cout << "G_p_I: " << imu_gps_localizer_->GetState().G_p_I << std::endl;

  }
  
  #if save_debug_info
  std::ofstream local_out;
  local_out.open("imu_gps_local.txt",std::ios::app);

  local_out << "fused_State: " << fused_state.G_p_I.x() << ", " << fused_state.G_p_I.y() << ", " << fused_state.G_p_I.z() << ", lla: " << 
                    fused_state.lla.x() << ", " <<  fused_state.lla.y() << ", " <<  fused_state.lla.z() << ", " << std::endl;
  local_out << "final_ State: " << imu_gps_localizer_->GetState().lla.x() << ", " << imu_gps_localizer_->GetState().lla.y() << ", " << imu_gps_localizer_->GetState().lla.z() << std::endl;
  local_out.close();

    std::ofstream local_out2;
  local_out2.open("imu_gps_local2.txt",std::ios::app);


      Eigen::Vector3d euler_angles=imu_gps_localizer_->GetState().G_R_I.eulerAngles(2,1,0);
  
  //std::cout << "ekf pose: "<<  imu_gps_localizer_->GetState().G_p_I.x() <<  "\t" << imu_gps_localizer_->GetState().G_p_I.y() << std::endl;
  //local_out2<<setiosflags(std::ios::fixed)<<local_out2.precision(4);
  
  local_out2  << std::fixed <<imu_gps_localizer_->GetState().G_p_I.x() << "\t " << imu_gps_localizer_->GetState().G_p_I.y() << "\t " << imu_gps_localizer_->GetState().G_p_I.z() << "\t "
              << imu_gps_localizer_->GetState().G_v_I.x()<< "\t " << imu_gps_localizer_->GetState().G_v_I.y()<< "\t " << imu_gps_localizer_->GetState().G_v_I.z()<< "\t "
              << imu_gps_localizer_->GetState().G_R_I(0,0)<< "\t " << imu_gps_localizer_->GetState().G_R_I(0,1)<< "\t " << imu_gps_localizer_->GetState().G_R_I(0,2)<< "\t "
              << imu_gps_localizer_->GetState().G_R_I(1,0)<< "\t " << imu_gps_localizer_->GetState().G_R_I(1,1)<< "\t " << imu_gps_localizer_->GetState().G_R_I(1,2)<< "\t "
              << imu_gps_localizer_->GetState().G_R_I(2,0)<< "\t " << imu_gps_localizer_->GetState().G_R_I(2,1)<< "\t " << imu_gps_localizer_->GetState().G_R_I(2,2)<< "\t "
               << euler_angles.x() << "\t" << euler_angles.y() << "\t" << euler_angles.z() << "\t"
                << fused_state.lla.x() << "\t " <<  fused_state.lla.y() << "\t " <<  fused_state.lla.z() << "\t " << std::endl;

  local_out2.close();
  #endif
  

}

//FooLocalization::DR_ekf_init = false;       
void FooLocalization::LocalizationDRCorrect(LocalizationEstimate &localization) {
  // process fused input data
  //if(! DR_correct_localizer_->local_view_.chassis.has_header() || 
   //   !common::util::IsProtoEqual(DR_correct_localizer_->local_view_.chassis, chassis_)) {
    if(! DR_correct_localizer_->local_view_.chassis || 
     (DR_correct_localizer_->local_view_.chassis->header().timestamp_sec()!=  chassis_.header().timestamp_sec())) {

    std::unique_lock<std::mutex> lock(chassis_mutex_);
  DR_correct_localizer_->local_view_.chassis = std::make_shared<canbus::Chassis>(chassis_);
   //DR_correct_localizer_->local_view_.chassis.CopyFrom(chassis_);
   // lock.unlock();
  DR_correct_localizer_->local_view_.localization_estimate = std::make_shared<LocalizationEstimate>(localization);

  //const auto& cur_vehicle_state = VehicleStateProvider::Instance()->vehicle_state();
  //const double vehicle_speed = std::max(std::fabs(cur_vehicle_state.linear_velocity()),0.002);
  //static double pre_timestamp = apollo::common::time::Clock::NowInSeconds();
  //double cur_timestamp = apollo::common::time::Clock::NowInSeconds();
  //double time_diff = cur_timestamp - pre_timestamp;
  //pre_timestamp = cur_timestamp;

 
  
  if (!DR_ekf_init && init_localization_.has_header()) {
    static auto init_pos = DR_correct_localizer_->local_view_.localization_estimate->pose().position();
    auto cur_pos = DR_correct_localizer_->local_view_.localization_estimate->pose().position();
    /*
    LocalizationEstimate cur_localization;
    apollo::common::Header header;   
    cur_localization.set_allocated_header(&header);
    //cur_localization.mutable_header()->CopyFrom(DR_correct_localizer_->local_view_.localization_estimate->header());
    cur_localization.mutable_pose()->mutable_position()->set_x(cur_pos.x());
    cur_localization.mutable_pose()->mutable_position()->set_y(cur_pos.y());
    cur_localization.mutable_pose()->mutable_position()->set_z(cur_pos.z());
    
    double dis = (cur_pos.x() - init_pos.x()) * (cur_pos.x() - init_pos.x()) + (cur_pos.y() - init_pos.y()) * (cur_pos.y() - init_pos.y());
    AERROR << "DIS: " << dis << "\n";
    if (dis >= 0.1){
      //DR_ekf_init =  DR_correct_localizer_->InitKalmanFilter(DR_correct_localizer_->local_view_, start_localization_, init_localization_); 
      DR_ekf_init =  DR_correct_localizer_->InitKalmanFilter(DR_correct_localizer_->local_view_, start_localization_, cur_localization); 
    }
    */
   DR_ekf_init =  DR_correct_localizer_->InitKalmanFilter(DR_correct_localizer_->local_view_, start_localization_, init_localization_); 
  }
  if (DR_ekf_init ) {
    DR_correct_localizer_->KalmanFilterPredict(DR_correct_localizer_->local_view_);
    DR_correct_localizer_->KalmanFilterCorrect(DR_correct_localizer_->local_view_);
  
  #if save_debug_info
       std::ofstream correct_local_out;
  correct_local_out.open("correct_local_out.txt",std::ios::app);
  correct_local_out << std::fixed << DR_correct_localizer_->kalman_filter_->state_post_.at<double>(0,0) << "\t" << DR_correct_localizer_->kalman_filter_->state_post_.at<double>(1,0) << 
      "\t" << DR_correct_localizer_->kalman_filter_->state_post_.at<double>(2,0) << std::endl;
  correct_local_out.close();
  #endif
  
  }
  }

}

bool FooLocalization::GuidepostCorrectKF(LocalizationEstimate &localization) {
  // process fused input data
  //TODO-------------------------------
  if (ins_localization_valid_) {
    if(!localization.has_pose() || !localization.pose().has_heading()) {return false;}
  }
  //if(!localization.pose().has_heading()) {return false;}
  if(!chassis_.has_speed_mps()) {return false;} // to avoid ctrl variable appears nan


  if(! tracking_selfcar_->new_local_view_.chassis || 
     (tracking_selfcar_->new_local_view_.chassis->header().timestamp_sec()!=  chassis_.header().timestamp_sec())) {

    std::unique_lock<std::mutex> lock(chassis_mutex_);    
    tracking_selfcar_->new_local_view_.chassis = std::make_shared<canbus::Chassis>(chassis_);
  }
  
  static bool update_guidepost = false;
  {
    //std::lock_guard<std::mutex> lock(navigation_lane_mutex_);
    //if (!tracking_selfcar_->new_local_view_.routing ||
    //    hdmap::PncMap::IsNewRouting(*(tracking_selfcar_->new_local_view_.routing), routing_)) {
    if (!tracking_selfcar_->new_local_view_.routing ||
        tracking_selfcar_->new_local_view_.routing->header().timestamp_sec() != routing_.header().timestamp_sec()) {
      std::unique_lock<std::mutex> lock(routing_mutex_);  
      tracking_selfcar_->new_local_view_.routing =
          std::make_shared<routing::RoutingResponse>(routing_);  
      update_guidepost = true;
    } else {
      update_guidepost = false;
    }
  }

  static bool update_INSpost = false;
  {
    if (!tracking_selfcar_->new_local_view_.localization_estimate ||
        (tracking_selfcar_->new_local_view_.localization_estimate->header().timestamp_sec() != localization.header().timestamp_sec())) {
      tracking_selfcar_->new_local_view_.localization_estimate =
                            std::make_shared<LocalizationEstimate>(localization);
      update_INSpost = true;
    } else {  
      update_INSpost = false;
    }
  }

  //static bool ekf_init = false; 
  if (!guidepost_ekf_init) {
    guidepost_ekf_init =  tracking_selfcar_->InitKalmanFilter(ins_localization_valid_, 
                                    tracking_selfcar_->new_local_view_,tracking_selfcar_->guidepost_groups_);     
  }

  if (guidepost_ekf_init ) {
    tracking_selfcar_->KalmanFilterPredict(tracking_selfcar_->new_local_view_);
  }

  if (ins_localization_valid_ && 
            update_INSpost && guidepost_ekf_init) {
      tracking_selfcar_->KalmanFilterINSCorrect(tracking_selfcar_->new_local_view_);
      
  #if save_debug_info
    double selfcar_px2 = tracking_selfcar_->ins_kalman_filter_->state_post_.at<double>(0,0);
    double selfcar_py2 = tracking_selfcar_->ins_kalman_filter_->state_post_.at<double>(1,0);
    double selfcar_theta2 = tracking_selfcar_->ins_kalman_filter_->state_post_.at<double>(2,0);
    {
        std::ofstream ekf_out;
        ekf_out.open("map_ekf_result_out4.txt", std::ios::out | std::ios::app);
        ekf_out << std::fixed << tracking_selfcar_->new_local_view_.chassis->header().timestamp_sec() << "\t" <<
        tracking_selfcar_->new_local_view_.localization_estimate->pose().position().x() << "\t" <<
        tracking_selfcar_->new_local_view_.localization_estimate->pose().position().y() << "\t" <<
        tracking_selfcar_->new_local_view_.localization_estimate->pose().heading() << "\t" <<
          selfcar_px2 << "\t" << selfcar_py2 <<  "\t"<< selfcar_theta2  << "\t"<<  std::endl;
     
        ekf_out.close();
    }
  #endif
  }
  

  if (tracking_selfcar_->new_local_view_.routing && tracking_selfcar_->new_local_view_.routing->has_routing_request()) {
    const auto end_guidepost_id = tracking_selfcar_->new_local_view_.routing->routing_request().end_id();
    //to get current perception guidepsot coordinate
    const auto& perception_guidepost = *(tracking_selfcar_->new_local_view_.routing->routing_request().waypoint().rbegin());
    //const std::string startPostID = perception_guidepost.id();
    const auto perception_point = perception_guidepost.pose();

    if (guidepost_localization_valid_ && 
                         update_guidepost && guidepost_ekf_init) {
      tracking_selfcar_->KalmanFilterGuidepostCorrect(tracking_selfcar_->new_local_view_,tracking_selfcar_->guidepost_groups_);
    }

  }
  
  #if save_debug_info
  if (guidepost_localization_valid_ && guidepost_ekf_init) {

    const double selfcar_px = tracking_selfcar_->guidepost_kalman_filter_->state_post_.at<double>(0,0);
    const double selfcar_py = tracking_selfcar_->guidepost_kalman_filter_->state_post_.at<double>(1,0);
    const double selfcar_theta = tracking_selfcar_->guidepost_kalman_filter_->state_post_.at<double>(2,0);
    {
      std::ofstream ekf_out;
      ekf_out.open("map_ekf_result_out.txt", std::ios::out | std::ios::app);
      ekf_out << std::fixed << tracking_selfcar_->new_local_view_.chassis->header().timestamp_sec() << "\t" << selfcar_px << "\t" << selfcar_py <<  "\t"<< selfcar_theta  <<  std::endl;
  
      ekf_out.close();
    }
  }
  #endif
  
  return true;

}

void FooLocalization::RunWatchDog(double gps_timestamp) {
  if (!enable_watch_dog_) {
    return;
  }

  // check GPS time stamp against system time
  double gps_delay_sec = common::time::Clock::NowInSeconds() - gps_timestamp;
  double gps_service_delay =
      common::time::Clock::NowInSeconds() - service_started_time;
  int64_t gps_delay_cycle_cnt =
      static_cast<int64_t>(gps_delay_sec * localization_publish_freq_);

  bool msg_delay = false;
  if (gps_delay_cycle_cnt > report_threshold_err_num_ &&
      static_cast<int>(gps_service_delay) > service_delay_threshold) {
    msg_delay = true;
    std::stringstream ss;
    ss << "Raw GPS Message Delay. GPS message is " << gps_delay_cycle_cnt
       << " cycle " << gps_delay_sec << " sec behind current time.";
    monitor_logger_.ERROR(ss.str());
  }

  // check IMU time stamp against system time
  std::unique_lock<std::mutex> lock(imu_list_mutex_);
  auto imu_msg = imu_list_.back();
  lock.unlock();
  double imu_delay_sec =
      common::time::Clock::NowInSeconds() - imu_msg.header().timestamp_sec();
  int64_t imu_delay_cycle_cnt =
      static_cast<int64_t>(imu_delay_sec * localization_publish_freq_);
  if (imu_delay_cycle_cnt > report_threshold_err_num_ &&
      static_cast<int>(gps_service_delay) > service_delay_threshold) {
    msg_delay = true;
    std::stringstream ss;
    ss << "Raw IMU Message Delay. IMU message is " << imu_delay_cycle_cnt
       << " cycle " << imu_delay_sec << " sec behind current time.";
    monitor_logger_.ERROR(ss.str());
  }

  // to prevent it from beeping continuously
  if (msg_delay && (last_reported_timestamp_sec_ < 1. ||
                    common::time::Clock::NowInSeconds() >
                        last_reported_timestamp_sec_ + 1.)) {
    AERROR << "gps/imu frame Delay!";
    last_reported_timestamp_sec_ = common::time::Clock::NowInSeconds();
  }
}

/*
void FooLocalization::PrepareLocalizationMsg(
    const localization::Gps &gps_msg, LocalizationEstimate *localization,
    LocalizationStatus *localization_status) {
  // find the matching gps and imu message
  double gps_time_stamp = gps_msg.header().timestamp_sec();
  drivers::gnss::Imu imu_msg;
  FindMatchingIMU(gps_time_stamp, &imu_msg);
  ComposeLocalizationMsg(gps_msg, imu_msg, localization);

  drivers::gnss::InsStat gps_status;
  FindNearestGpsStatus(gps_time_stamp, &gps_status);
  FillLocalizationStatusMsg(gps_status, localization_status);
}
*/
void FooLocalization::FillLocalizationMsgHeader(
    LocalizationEstimate *localization) {
  auto *header = localization->mutable_header();
  double timestamp = apollo::common::time::Clock::NowInSeconds();
  header->set_module_name(module_name_);
  header->set_timestamp_sec(timestamp);
  header->set_sequence_num(static_cast<unsigned int>(++localization_seq_num_));
}

void FooLocalization::FillLocalizationStatusMsg(
    const drivers::gnss::InsStat &status,
    LocalizationStatus *localization_status) {
  apollo::common::Header *header = localization_status->mutable_header();
  double timestamp = apollo::common::time::Clock::NowInSeconds();
  header->set_timestamp_sec(timestamp);
  localization_status->set_measurement_time(status.header().timestamp_sec());

  if (!status.has_pos_type()) {
    localization_status->set_fusion_status(MeasureState::ERROR);
    localization_status->set_state_message(
        "Error: Current Localization Status Is Missing.");
    return;
  }

  auto pos_type = static_cast<drivers::gnss::SolutionType>(status.pos_type());
  switch (pos_type) {
    case drivers::gnss::SolutionType::INS_RTKFIXED:
      localization_status->set_fusion_status(MeasureState::OK);
      localization_status->set_state_message("");
      break;
    case drivers::gnss::SolutionType::INS_RTKFLOAT:
      localization_status->set_fusion_status(MeasureState::WARNNING);
      localization_status->set_state_message(
          "Warning: Current Localization Is Unstable.");
      break;
    default:
      localization_status->set_fusion_status(MeasureState::ERROR);
      localization_status->set_state_message(
          "Error: Current Localization Is Very Unstable.");
      break;
  }
}
/*
void FooLocalization::ComposeLocalizationMsg(
    const localization::Gps &gps_msg, const drivers::gnss::Imu &imu_msg,
    LocalizationEstimate *localization) {
  localization->Clear();

  FillLocalizationMsgHeader(localization);

  localization->set_measurement_time(gps_msg.header().timestamp_sec());

  // combine gps and imu
  auto mutable_pose = localization->mutable_pose();
  if (gps_msg.has_localization()) {
    const auto &pose = gps_msg.localization();

    if (pose.has_position()) {
      // position
      // world frame -> map frame
      mutable_pose->mutable_position()->set_x(pose.position().x() -
                                              map_offset_[0]);
      mutable_pose->mutable_position()->set_y(pose.position().y() -
                                              map_offset_[1]);
      mutable_pose->mutable_position()->set_z(pose.position().z() -
                                              map_offset_[2]);
    }

    // orientation
    if (pose.has_orientation()) {
      mutable_pose->mutable_orientation()->CopyFrom(pose.orientation());
      double heading = common::math::QuaternionToHeading(
          pose.orientation().qw(), pose.orientation().qx(),
          pose.orientation().qy(), pose.orientation().qz());
      mutable_pose->set_heading(heading);
    }
    // linear velocity
    if (pose.has_linear_velocity()) {
      mutable_pose->mutable_linear_velocity()->CopyFrom(pose.linear_velocity());
    }
  }

  if (imu_msg.has_imu()) {
    const auto &imu = imu_msg.imu();
    // linear acceleration
    if (imu.has_linear_acceleration()) {
      if (localization->pose().has_orientation()) {
        // linear_acceleration:
        // convert from vehicle reference to map reference
        Vector3d orig(imu.linear_acceleration().x(),
                      imu.linear_acceleration().y(),
                      imu.linear_acceleration().z());
        Vector3d vec = common::math::QuaternionRotate(
            localization->pose().orientation(), orig);
        mutable_pose->mutable_linear_acceleration()->set_x(vec[0]);
        mutable_pose->mutable_linear_acceleration()->set_y(vec[1]);
        mutable_pose->mutable_linear_acceleration()->set_z(vec[2]);

        // linear_acceleration_vfr
        mutable_pose->mutable_linear_acceleration_vrf()->CopyFrom(
            imu.linear_acceleration());
      } else {
        AERROR << "[PrepareLocalizationMsg]: "
               << "fail to convert linear_acceleration";
      }
    }

    // angular velocity
    if (imu.has_angular_velocity()) {
      if (localization->pose().has_orientation()) {
        // angular_velocity:
        // convert from vehicle reference to map reference
        Vector3d orig(imu.angular_velocity().x(), imu.angular_velocity().y(),
                      imu.angular_velocity().z());
        Vector3d vec = common::math::QuaternionRotate(
            localization->pose().orientation(), orig);
        mutable_pose->mutable_angular_velocity()->set_x(vec[0]);
        mutable_pose->mutable_angular_velocity()->set_y(vec[1]);
        mutable_pose->mutable_angular_velocity()->set_z(vec[2]);

        // angular_velocity_vf
        mutable_pose->mutable_angular_velocity_vrf()->CopyFrom(
            imu.angular_velocity());
      } else {
        AERROR << "[PrepareLocalizationMsg]: fail to convert angular_velocity";
      }
    }

    // euler angle
    if (imu.has_euler_angles()) {
      mutable_pose->mutable_euler_angles()->CopyFrom(imu.euler_angles());
    }
  }
}

bool FooLocalization::FindMatchingIMU(const double gps_timestamp_sec,
                                      drivers::gnss::Imu *imu_msg) {
  if (imu_msg == nullptr) {
    AERROR << "imu_msg should NOT be nullptr.";
    return false;
  }

  std::unique_lock<std::mutex> lock(imu_list_mutex_);
  auto imu_list = imu_list_;
  lock.unlock();

  if (imu_list.empty()) {
    AERROR << "Cannot find Matching IMU. "
           << "IMU message Queue is empty! GPS timestamp[" << gps_timestamp_sec
           << "]";
    return false;
  }

  // scan imu buffer, find first imu message that is newer than the given
  // timestamp
  auto imu_it = imu_list.begin();
  for (; imu_it != imu_list.end(); ++imu_it) {
    if ((*imu_it).header().timestamp_sec() - gps_timestamp_sec >
        std::numeric_limits<double>::min()) {
      break;
    }
  }

  if (imu_it != imu_list.end()) {  // found one
    if (imu_it == imu_list.begin()) {
      AERROR << "IMU queue too short or request too old. "
             << "Oldest timestamp[" << imu_list.front().header().timestamp_sec()
             << "], Newest timestamp["
             << imu_list.back().header().timestamp_sec() << "], GPS timestamp["
             << gps_timestamp_sec << "]";
      *imu_msg = imu_list.front();  // the oldest imu
    } else {
      // here is the normal case
      auto imu_it_1 = imu_it;
      imu_it_1--;
      if (!(*imu_it).has_header() || !(*imu_it_1).has_header()) {
        AERROR << "imu1 and imu_it_1 must both have header.";
        return false;
      }
      if (!InterpolateIMU(*imu_it_1, *imu_it, gps_timestamp_sec, imu_msg)) {
        AERROR << "failed to interpolate IMU";
        return false;
      }
    }
  } else {
    // give the newest imu, without extrapolation
    *imu_msg = imu_list.back();
    if (imu_msg == nullptr) {
      AERROR << "Fail to get latest observed imu_msg.";
      return false;
    }

    if (!imu_msg->has_header()) {
      AERROR << "imu_msg must have header.";
      return false;
    }

    if (std::fabs(imu_msg->header().timestamp_sec() - gps_timestamp_sec) >
        gps_imu_time_diff_threshold_) {
      // 20ms threshold to report error
      AERROR << "Cannot find Matching IMU. IMU messages too old. "
             << "Newest timestamp[" << imu_list.back().header().timestamp_sec()
             << "], GPS timestamp[" << gps_timestamp_sec << "]";
    }
  }

  return true;
}

bool FooLocalization::InterpolateIMU(const drivers::gnss::Imu &imu1,
                                     const drivers::gnss::Imu &imu2,
                                     const double timestamp_sec,
                                     drivers::gnss::Imu *imu_msg) {
  if (!(imu1.header().has_timestamp_sec() &&
        imu2.header().has_timestamp_sec())) {
    AERROR << "imu1 and imu2 has no header or no timestamp_sec in header";
    return false;
  }
  if (timestamp_sec - imu1.header().timestamp_sec() <
      std::numeric_limits<double>::min()) {
    AERROR << "[InterpolateIMU1]: the given time stamp[" << timestamp_sec
           << "] is older than the 1st message["
           << imu1.header().timestamp_sec() << "]";
    *imu_msg = imu1;
  } else if (timestamp_sec - imu2.header().timestamp_sec() >
             std::numeric_limits<double>::min()) {
    AERROR << "[InterpolateIMU2]: the given time stamp[" << timestamp_sec
           << "] is newer than the 2nd message["
           << imu2.header().timestamp_sec() << "]";
    *imu_msg = imu1;
  } else {
    *imu_msg = imu1;
    imu_msg->mutable_header()->set_timestamp_sec(timestamp_sec);

    double time_diff =
        imu2.header().timestamp_sec() - imu1.header().timestamp_sec();
    if (fabs(time_diff) >= 0.001) {
      double frac1 =
          (timestamp_sec - imu1.header().timestamp_sec()) / time_diff;

      if (imu1.imu().has_angular_velocity() &&
          imu2.imu().has_angular_velocity()) {
        auto val = InterpolateXYZ(imu1.imu().angular_velocity(),
                                  imu2.imu().angular_velocity(), frac1);
        imu_msg->mutable_imu()->mutable_angular_velocity()->CopyFrom(val);
      }

      if (imu1.imu().has_linear_acceleration() &&
          imu2.imu().has_linear_acceleration()) {
        auto val = InterpolateXYZ(imu1.imu().linear_acceleration(),
                                  imu2.imu().linear_acceleration(), frac1);
        imu_msg->mutable_imu()->mutable_linear_acceleration()->CopyFrom(val);
      }

      if (imu1.imu().has_euler_angles() && imu2.imu().has_euler_angles()) {
        auto val = InterpolateXYZ(imu1.imu().euler_angles(),
                                  imu2.imu().euler_angles(), frac1);
        imu_msg->mutable_imu()->mutable_euler_angles()->CopyFrom(val);
      }
    }
  }
  return true;
}
*/
template <class T>
T FooLocalization::InterpolateXYZ(const T &p1, const T &p2,
                                  const double frac1) {
  T p;
  double frac2 = 1.0 - frac1;
  if (p1.has_x() && !std::isnan(p1.x()) && p2.has_x() && !std::isnan(p2.x())) {
    p.set_x(p1.x() * frac2 + p2.x() * frac1);
  }
  if (p1.has_y() && !std::isnan(p1.y()) && p2.has_y() && !std::isnan(p2.y())) {
    p.set_y(p1.y() * frac2 + p2.y() * frac1);
  }
  if (p1.has_z() && !std::isnan(p1.z()) && p2.has_z() && !std::isnan(p2.z())) {
    p.set_z(p1.z() * frac2 + p2.z() * frac1);
  }
  return p;
}

bool FooLocalization::FindNearestGpsStatus(const double gps_timestamp_sec,
                                           drivers::gnss::InsStat *status) {
  CHECK_NOTNULL(status);

  std::unique_lock<std::mutex> lock(gps_status_list_mutex_);
  auto gps_status_list = gps_status_list_;
  lock.unlock();

  double timestamp_diff_sec = 1e8;
  auto nearest_itr = gps_status_list.end();
  for (auto itr = gps_status_list.begin(); itr != gps_status_list.end();
       ++itr) {
    double diff = std::abs(itr->header().timestamp_sec() - gps_timestamp_sec);
    if (diff < timestamp_diff_sec) {
      timestamp_diff_sec = diff;
      nearest_itr = itr;
    }
  }

  if (nearest_itr == gps_status_list.end()) {
    return false;
  }

  if (timestamp_diff_sec > gps_status_time_diff_threshold_) {
    return false;
  }

  *status = *nearest_itr;
  return true;
}

}  // namespace localization
}  // namespace apollo
