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

#include "modules/localization/msf/msf_localization.h"

#include "yaml-cpp/yaml.h"

#include "cyber/common/file.h"
#include "modules/common/math/euler_angles_zxy.h"
#include "modules/common/math/math_utils.h"
#include "modules/common/math/quaternion.h"
#include "modules/common/time/time.h"
#include "modules/drivers/gnss/proto/config.pb.h"
#include "modules/localization/common/localization_gflags.h"
#include "modules/localization/msf/msf_localization_component.h"

#define enable_DR_kf true

#define enable_guidepost_kf true

namespace apollo {
namespace localization {

using apollo::common::Status;

void split(const std::string& s, std::vector<std::string>& sv, const char flag = ' ') {
    sv.clear();
    std::istringstream iss(s);
    std::string temp;
    while (std::getline(iss, temp, flag)) {
        sv.push_back(temp);
    }
    return;
}

bool MSFLocalization::LoadMap() {
  //to get guidepost world coordinate
  //std::string mapPath = "/apollo/modules/map/data/demo/signpost_shzhw.txt";
  std::string mapPath = "/apollo/modules/map/data/demo/guidepost_utm_out.txt";
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
        GuidepostGroup guidepostGroup;
        guidepostGroup.id = sv[0];
        guidepostGroup.x = atof(sv[1].c_str());
        guidepostGroup.y = atof(sv[2].c_str());        
        msf_localization_DR_filter_->guidepost_groups_.push_back(guidepostGroup);
    }
  }
  mapFile.close();

  if ( msf_localization_DR_filter_->guidepost_groups_.size() > 0) {
  } else
  {
    AERROR << "guidepos group num: 0";
  }
  
  AINFO << "guidepos group num: " <<  msf_localization_DR_filter_->guidepost_groups_.size();
  std::cout << "guidepos group num: " <<  msf_localization_DR_filter_->guidepost_groups_.size() << std::endl;
  return true;
}



MSFLocalization::MSFLocalization()
    : monitor_logger_(
          apollo::common::monitor::MonitorMessageItem::LOCALIZATION),
      localization_state_(msf::LocalizationMeasureState::OK),
      pcd_msg_index_(-1) {}

Status MSFLocalization::Init() {
  InitParams();



  if (enable_DR_kf) {    
    int kf_coor_sys = 1; 

    msf_localization_DR_filter_ = std::unique_ptr<MSFLocalizationDRCorrectFilter>(new MSFLocalizationDRCorrectFilter());
    msf_localization_DR_filter_->CreateKalmanFilter(kf_coor_sys);  
    msf_localization_DR_filter_init = false;

    trailer_localization_filter_ = std::unique_ptr<TrailerLocalizationFilter>(new TrailerLocalizationFilter());
    trailer_localization_filter_->CreateKalmanFilter(kf_coor_sys);

  #if enable_guidepost_kf
    msf_localization_DR_filter_->CreateGuidepostKalmanFilter(kf_coor_sys);
    LoadMap();  
  #endif

  }

  return localization_integ_.Init(localization_param_);
}

void MSFLocalization::InitParams() {
  // integration module
  localization_param_.is_ins_can_self_align = FLAGS_integ_ins_can_self_align;
  localization_param_.is_sins_align_with_vel = FLAGS_integ_sins_align_with_vel;
  localization_param_.is_sins_state_check = FLAGS_integ_sins_state_check;
  localization_param_.sins_state_span_time = FLAGS_integ_sins_state_span_time;
  localization_param_.sins_state_pos_std = FLAGS_integ_sins_state_pos_std;
  localization_param_.vel_threshold_get_yaw = FLAGS_vel_threshold_get_yaw;
  localization_param_.is_trans_gpstime_to_utctime =
      FLAGS_trans_gpstime_to_utctime;
  localization_param_.gnss_mode = FLAGS_gnss_mode;
  localization_param_.is_using_raw_gnsspos = true;

  // gnss module
  localization_param_.enable_ins_aid_rtk = FLAGS_enable_ins_aid_rtk;

  // lidar module
  localization_param_.map_path = FLAGS_map_dir + "/" + FLAGS_local_map_name;
  localization_param_.lidar_extrinsic_file = FLAGS_lidar_extrinsics_file;
  localization_param_.lidar_height_file = FLAGS_lidar_height_file;
  localization_param_.lidar_height_default = FLAGS_lidar_height_default;
  localization_param_.localization_mode = FLAGS_lidar_localization_mode;
  localization_param_.lidar_yaw_align_mode = FLAGS_lidar_yaw_align_mode;
  localization_param_.lidar_filter_size = FLAGS_lidar_filter_size;
  localization_param_.map_coverage_theshold = FLAGS_lidar_map_coverage_theshold;
  localization_param_.imu_lidar_max_delay_time = FLAGS_lidar_imu_max_delay_time;
  localization_param_.if_use_avx = FLAGS_if_use_avx;

  AINFO << "map: " << localization_param_.map_path;
  AINFO << "lidar_extrin: " << localization_param_.lidar_extrinsic_file;
  AINFO << "lidar_height: " << localization_param_.lidar_height_file;

  localization_param_.utm_zone_id = FLAGS_local_utm_zone_id;
  // try load zone id from local_map folder
  if (FLAGS_if_utm_zone_id_from_folder) {
    bool success = LoadZoneIdFromFolder(localization_param_.map_path,
                                        &localization_param_.utm_zone_id);
    if (!success) {
      AWARN << "Can't load utm zone id from map folder, use default value.";
    }
  }
  AINFO << "utm zone id: " << localization_param_.utm_zone_id;

  // vehicle imu extrinsic
  imu_vehicle_quat_.x() = FLAGS_imu_vehicle_qx;
  imu_vehicle_quat_.y() = FLAGS_imu_vehicle_qy;
  imu_vehicle_quat_.z() = FLAGS_imu_vehicle_qz;
  imu_vehicle_quat_.w() = FLAGS_imu_vehicle_qw;
  // try to load imu vehicle quat from file
  if (FLAGS_if_vehicle_imu_from_file) {
    double qx = 0.0;
    double qy = 0.0;
    double qz = 0.0;
    double qw = 0.0;

    AINFO << "Vehile imu file: " << FLAGS_vehicle_imu_file;
    if (LoadImuVehicleExtrinsic(FLAGS_vehicle_imu_file, &qx, &qy, &qz, &qw)) {
      imu_vehicle_quat_.x() = qx;
      imu_vehicle_quat_.y() = qy;
      imu_vehicle_quat_.z() = qz;
      imu_vehicle_quat_.w() = qw;
    } else {
      AWARN << "Can't load imu vehicle quat from file, use default value.";
    }
  }
  AINFO << "imu_vehicle_quat: " << imu_vehicle_quat_.x() << " "
        << imu_vehicle_quat_.y() << " " << imu_vehicle_quat_.z() << " "
        << imu_vehicle_quat_.w();

  // common
  localization_param_.enable_lidar_localization =
      FLAGS_enable_lidar_localization;

  if (!FLAGS_if_imuant_from_file) {
    localization_param_.imu_to_ant_offset.offset_x = FLAGS_imu_to_ant_offset_x;
    localization_param_.imu_to_ant_offset.offset_y = FLAGS_imu_to_ant_offset_y;
    localization_param_.imu_to_ant_offset.offset_z = FLAGS_imu_to_ant_offset_z;
    localization_param_.imu_to_ant_offset.uncertainty_x =
        FLAGS_imu_to_ant_offset_ux;
    localization_param_.imu_to_ant_offset.uncertainty_y =
        FLAGS_imu_to_ant_offset_uy;
    localization_param_.imu_to_ant_offset.uncertainty_z =
        FLAGS_imu_to_ant_offset_uz;
  } else {
    double offset_x = 0.0;
    double offset_y = 0.0;
    double offset_z = 0.0;
    double uncertainty_x = 0.0;
    double uncertainty_y = 0.0;
    double uncertainty_z = 0.0;
    AINFO << "Ant imu lever arm file: " << FLAGS_ant_imu_leverarm_file;
    CHECK(LoadGnssAntennaExtrinsic(FLAGS_ant_imu_leverarm_file, &offset_x,
                                   &offset_y, &offset_z, &uncertainty_x,
                                   &uncertainty_y, &uncertainty_z));
    localization_param_.ant_imu_leverarm_file = FLAGS_ant_imu_leverarm_file;

    localization_param_.imu_to_ant_offset.offset_x = offset_x;
    localization_param_.imu_to_ant_offset.offset_y = offset_y;
    localization_param_.imu_to_ant_offset.offset_z = offset_z;
    localization_param_.imu_to_ant_offset.uncertainty_x = uncertainty_x;
    localization_param_.imu_to_ant_offset.uncertainty_y = uncertainty_y;
    localization_param_.imu_to_ant_offset.uncertainty_z = uncertainty_z;

    AINFO << localization_param_.imu_to_ant_offset.offset_x << " "
          << localization_param_.imu_to_ant_offset.offset_y << " "
          << localization_param_.imu_to_ant_offset.offset_z << " "
          << localization_param_.imu_to_ant_offset.uncertainty_x << " "
          << localization_param_.imu_to_ant_offset.uncertainty_y << " "
          << localization_param_.imu_to_ant_offset.uncertainty_z;
  }

  localization_param_.imu_delay_time_threshold_1 =
      FLAGS_imu_delay_time_threshold_1;
  localization_param_.imu_delay_time_threshold_2 =
      FLAGS_imu_delay_time_threshold_2;
  localization_param_.imu_delay_time_threshold_3 =
      FLAGS_imu_delay_time_threshold_3;

  localization_param_.imu_missing_time_threshold_1 =
      FLAGS_imu_missing_time_threshold_1;
  localization_param_.imu_missing_time_threshold_2 =
      FLAGS_imu_missing_time_threshold_2;
  localization_param_.imu_missing_time_threshold_3 =
      FLAGS_imu_missing_time_threshold_3;

  localization_param_.bestgnsspose_loss_time_threshold =
      FLAGS_bestgnsspose_loss_time_threshold;
  localization_param_.lidar_loss_time_threshold =
      FLAGS_lidar_loss_time_threshold;

  localization_param_.localization_std_x_threshold_1 =
      FLAGS_localization_std_x_threshold_1;
  localization_param_.localization_std_y_threshold_1 =
      FLAGS_localization_std_y_threshold_1;

  localization_param_.localization_std_x_threshold_2 =
      FLAGS_localization_std_x_threshold_2;
  localization_param_.localization_std_y_threshold_2 =
      FLAGS_localization_std_y_threshold_2;
}

void MSFLocalization::OnPointCloud(
    const std::shared_ptr<drivers::PointCloud> &message) {
  ++pcd_msg_index_;
  if (pcd_msg_index_ % FLAGS_point_cloud_step != 0) {
    return;
  }

  localization_integ_.PcdProcess(*message);

  const auto &result = localization_integ_.GetLastestLidarLocalization();

  if (result.state() == msf::LocalizationMeasureState::OK ||
      result.state() == msf::LocalizationMeasureState::VALID) {
    // publish lidar message to debug
    publisher_->PublishLocalizationMsfLidar(result.localization());
  }
}

void MSFLocalization::OnChassis(const canbus::Chassis& chassis) { 
    std::lock_guard<std::mutex> lock(chassis_mutex_);
    //    std::unique_lock<std::mutex> lock(chassis_mutex_);
    chassis_.CopyFrom(chassis);  
    /*
     if(!msf_localization_DR_filter_->local_view_.chassis || 
     (msf_localization_DR_filter_->local_view_.chassis->header().timestamp_sec()!=  chassis_.header().timestamp_sec())) {

   // std::unique_lock<std::mutex> lock(chassis_mutex_);
    msf_localization_DR_filter_->local_view_.chassis = std::make_shared<canbus::Chassis>(chassis_);
  }
  */
}


void MSFLocalization::OnRouting(const routing::RoutingResponse& routing) {
  {
    std::lock_guard<std::mutex> lock(routing_mutex_);
    routing_.CopyFrom(routing);
  }
}


void MSFLocalization::LocalizationDRCorrect(LocalizationEstimate &localization, LocalizationStatus &localization_status) {
  // process fused input data

 if ((!msf_localization_DR_filter_->local_view_.chassis || 
     (msf_localization_DR_filter_->local_view_.chassis->header().timestamp_sec()!=  chassis_.header().timestamp_sec()))
     && (chassis_.header().timestamp_sec() > 0.0)) {
  /**/
  /*
if ((!msf_localization_DR_filter_->local_view_.chassis || 
     (msf_localization_DR_filter_->local_view_.chassis->header().sequence_num()!=  chassis_.header().sequence_num()))
     && (chassis_.header().timestamp_sec() > 0.0)) {
  */
    std::unique_lock<std::mutex> lock(chassis_mutex_);
    msf_localization_DR_filter_->local_view_.chassis = std::make_shared<canbus::Chassis>(chassis_);
  }
  
  

  static bool update_msf_localization = false;
  //static int msf_localization_cnt = 0;
  if (!msf_localization_DR_filter_->local_view_.localization_estimate || 
     std::fabs(msf_localization_DR_filter_->local_view_.localization_estimate->header().timestamp_sec()-  localization.header().timestamp_sec()) > 1e-4) {

    msf_localization_DR_filter_->local_view_.localization_estimate = std::make_shared<LocalizationEstimate>(localization);
    //msf_localization_cnt ++;
   // if (msf_localization_cnt%100 ==0) {
    update_msf_localization = true;
    //AERROR << '..............: ' << msf_localization_cnt;
    //} else {
    //  update_msf_localization = false;
    //}
  } else {
    update_msf_localization = false;
  }
   
  static bool update_gps = false;
  if((!msf_localization_DR_filter_->local_view_.best_gnss_pos  || 
    std::fabs(msf_localization_DR_filter_->local_view_.best_gnss_pos ->header().timestamp_sec() - bestgnsspos_msg_.header().timestamp_sec())>1e-4) ) {
    std::unique_lock<std::mutex> lock(bestgnsspos_msg_mutex_);
    
    msf_localization_DR_filter_->local_view_.best_gnss_pos = std::make_shared<drivers::gnss::GnssBestPose>(bestgnsspos_msg_); 
    update_gps = true;
  } else {
    update_gps = false;
  }

  //static bool update_localization_status = false;
  if((!msf_localization_DR_filter_->local_view_.localization_status  || 
    std::fabs(msf_localization_DR_filter_->local_view_.localization_status ->header().timestamp_sec() - localization_status.header().timestamp_sec())>1e-4) ) {
    //std::unique_lock<std::mutex> lock(bestgnsspos_msg_mutex_);
    
    msf_localization_DR_filter_->local_view_.localization_status = std::make_shared<LocalizationStatus>(localization_status); 
    //update_localization_status = true;
  } else {
    //update_localization_status = false;
  }

    //static bool update_heading = false;
  if((!msf_localization_DR_filter_->local_view_.gnss_heading  || 
    std::fabs(msf_localization_DR_filter_->local_view_.gnss_heading ->measurement_time() - gnss_heading_msg_.measurement_time())>1e-4) ) {
    std::unique_lock<std::mutex> lock(gnssheading_msg_mutex_);
    
    msf_localization_DR_filter_->local_view_.gnss_heading = std::make_shared<drivers::gnss::Heading>(gnss_heading_msg_); 
    //update_heading = true;
  } else {
    //update_heading = false;
  }

  static bool update_guidepost = false;
  if (routing_.has_header()) {
    if (!msf_localization_DR_filter_->local_view_.routing ||
          std::fabs(msf_localization_DR_filter_->local_view_.routing->header().timestamp_sec() - routing_.header().timestamp_sec())>1e-4) {
        std::unique_lock<std::mutex> lock(routing_mutex_);  
        msf_localization_DR_filter_->local_view_.routing =
            std::make_shared<routing::RoutingResponse>(routing_);
        update_guidepost = true;
    } else {
        update_guidepost = false;
    }
  }
  
  if (std::fabs(msf_localization_DR_filter_->local_view_.chassis->header().timestamp_sec()-1607321842.01221) < 0.001) {
    AERROR <<std::fixed<< "pre_pre22:" << msf_localization_DR_filter_->kalman_filter_->state_pre_.at<double>(0,0) << "\t" <<
     msf_localization_DR_filter_->kalman_filter_->state_pre_.at<double>(1,0) << "\t" <<
      msf_localization_DR_filter_->kalman_filter_->state_pre_.at<double>(2,0);

    AERROR <<std::fixed<< "pre_post22:" << msf_localization_DR_filter_->kalman_filter_->state_post_.at<double>(0,0) << "\t" <<
     msf_localization_DR_filter_->kalman_filter_->state_post_.at<double>(1,0) << "\t" <<
      msf_localization_DR_filter_->kalman_filter_->state_post_.at<double>(2,0);
    AERROR <<"best pose type: " <<msf_localization_DR_filter_->local_view_.best_gnss_pos->sol_type();
  }

  if (!msf_localization_DR_filter_init ) {
    msf_localization_DR_filter_init =  msf_localization_DR_filter_->InitKalmanFilter(msf_localization_DR_filter_->local_view_); 
  }
  if (msf_localization_DR_filter_init ) {
    std::lock_guard<std::mutex> lock(chassis_mutex_);
    msf_localization_DR_filter_->KalmanFilterPredict(msf_localization_DR_filter_->local_view_);//-----------------------1118

      if (std::fabs(msf_localization_DR_filter_->local_view_.chassis->header().timestamp_sec()-1607321842.01221) < 0.001) {
         AERROR <<std::fixed<< "cur_pre22:" << msf_localization_DR_filter_->kalman_filter_->state_pre_.at<double>(0,0) << "\t" <<
     msf_localization_DR_filter_->kalman_filter_->state_pre_.at<double>(1,0) << "\t" <<
      msf_localization_DR_filter_->kalman_filter_->state_pre_.at<double>(2,0);
      }
    
    if (update_msf_localization) {
      msf_localization_DR_filter_->KalmanFilterCorrect(msf_localization_DR_filter_->local_view_, update_gps);
    }
  
    if (update_guidepost) {
      #if enable_guidepost_kf
      msf_localization_DR_filter_->KalmanFilterGuidepostCorrect(msf_localization_DR_filter_->local_view_,msf_localization_DR_filter_->guidepost_groups_);
      #endif
    }
  

  #if save_debug_info
      std::ofstream correct_local_out;
  correct_local_out.open("correct_local_out.txt",std::ios::app);
  correct_local_out << std::fixed << DR_correct_localizer_->kalman_filter_->state_post_.at<double>(0,0) << "\t" << DR_correct_localizer_->kalman_filter_->state_post_.at<double>(1,0) << 
      "\t" << DR_correct_localizer_->kalman_filter_->state_post_.at<double>(2,0) << std::endl;
  correct_local_out.close();
  #endif

  #if 1
      std::ofstream test_data;
  test_data.open("test_data.txt",std::ios::app);
  auto can_time_t = chassis_.header().timestamp_sec();
  auto time_t = msf_localization_DR_filter_->local_view_.chassis->header().timestamp_sec();
  auto speed_t = msf_localization_DR_filter_->local_view_.chassis->speed_mps();
  auto steer_per_t = msf_localization_DR_filter_->local_view_.chassis->steering_percentage();

  auto px_t  = msf_localization_DR_filter_->local_view_.localization_estimate->pose().position().x();
  auto py_t = msf_localization_DR_filter_->local_view_.localization_estimate->pose().position().y();
  auto theta_t = msf_localization_DR_filter_->local_view_.localization_estimate->pose().heading();
  auto px_std_t = msf_localization_DR_filter_->local_view_.localization_estimate->uncertainty().position_std_dev().x();
  auto py_std_t = msf_localization_DR_filter_->local_view_.localization_estimate->uncertainty().position_std_dev().y();
  auto theta_std_t = msf_localization_DR_filter_->local_view_.localization_estimate->uncertainty().orientation_std_dev().z();

  auto gnss_heading_t = msf_localization_DR_filter_->local_view_.gnss_heading->heading();
  auto gnss_heading_std_t = msf_localization_DR_filter_->local_view_.gnss_heading->heading_std_dev();

  
  test_data << std::fixed << can_time_t << "\t" << time_t << "\t" << speed_t << "\t" << steer_per_t << "\t" << 
      px_t << "\t" << py_t << "\t" << theta_t <<"\t" <<px_std_t << "\t" << py_std_t  << "\t" << theta_std_t<< 
        "\t" << msf_localization_DR_filter_->kalman_filter_->state_post_.at<double>(0,0) << "\t" <<msf_localization_DR_filter_->kalman_filter_->state_post_.at<double>(1,0) <<"\t" <<
      msf_localization_DR_filter_->kalman_filter_->state_post_.at<double>(2,0)  
      << "\t" << msf_localization_DR_filter_->kalman_filter_->state_pre_.at<double>(0,0) << "\t" <<msf_localization_DR_filter_->kalman_filter_->state_pre_.at<double>(1,0) <<"\t" <<
      msf_localization_DR_filter_->kalman_filter_->state_pre_.at<double>(2,0)  
      <<  "\t" << gnss_heading_t << "\t" << gnss_heading_std_t << std::endl;
  test_data.close();
  #endif
  
  
  }
 

}

void MSFLocalization::OnRawImu(
    const std::shared_ptr<drivers::gnss::Imu> &imu_msg) {
  if (FLAGS_imu_coord_rfu) {
    localization_integ_.RawImuProcessRfu(*imu_msg);
  } else {
    localization_integ_.RawImuProcessFlu(*imu_msg);
  }

  const auto &result = localization_integ_.GetLastestIntegLocalization();

  // compose localization status
  LocalizationStatus status;
  apollo::common::Header *status_headerpb = status.mutable_header();
  status_headerpb->set_timestamp_sec(
      result.localization().header().timestamp_sec());
  status.set_fusion_status(
      static_cast<MeasureState>(result.integ_status().integ_state));
  status.set_state_message(result.integ_status().state_message);
  status.set_measurement_time(result.localization().measurement_time());
  publisher_->PublishLocalizationStatus(status);

  if (result.state() == msf::LocalizationMeasureState::OK ||
      result.state() == msf::LocalizationMeasureState::VALID) {
    // calculate orientation_vehicle_world
    LocalizationEstimate local_result = result.localization();
    CompensateImuVehicleExtrinsic(&local_result);

    publisher_->PublishPoseBroadcastTF(local_result);


  #if 1
    // add chassis info DRekf
    LocalizationDRCorrect(local_result, status);
    const auto &DR_ekf_state = msf_localization_DR_filter_->kalman_filter_->state_post_;
    local_result.mutable_pose()->mutable_position()->set_x(DR_ekf_state.at<double>(0,0));
    local_result.mutable_pose()->mutable_position()->set_y(DR_ekf_state.at<double>(1,0));
    #if 0
    AERROR << "-----------------------------------------------------";
    AERROR << "111111111111111111111111111111111";

  std::ofstream msf_out;
  msf_out.open("msf_out.txt",std::ios::app);
  msf_out <<std::fixed << result.localization().header().timestamp_sec() << "\t" <<
         result.localization().pose().position().x() << "\t" << result.localization().pose().position().y() << "\t" <<  result.localization().pose().position().z() << "\t" <<
         result.localization().pose().heading() << "\t" << local_result.pose().heading()  << "\t"
          << local_result.pose().position().x() << "\t" << local_result.pose().position().y() << std::endl;
    #endif
#endif
    publisher_->PublishPoseBroadcastTopic(local_result);
  }

  localization_state_ = result.state();
}

void MSFLocalization::OnGnssBestPose(
    const std::shared_ptr<drivers::gnss::GnssBestPose> &bestgnsspos_msg) {
  if ((localization_state_ == msf::LocalizationMeasureState::OK ||
       localization_state_ == msf::LocalizationMeasureState::VALID) &&
      FLAGS_gnss_only_init) {
    return;
  }

  localization_integ_.GnssBestPoseProcess(*bestgnsspos_msg);
  bestgnsspos_msg_.CopyFrom(*bestgnsspos_msg);

  const auto &result = localization_integ_.GetLastestGnssLocalization();

  if (result.state() == msf::LocalizationMeasureState::OK ||
      result.state() == msf::LocalizationMeasureState::VALID) {
    publisher_->PublishLocalizationMsfGnss(result.localization());
  }
}

void MSFLocalization::OnGnssRtkObs(
    const std::shared_ptr<drivers::gnss::EpochObservation> &raw_obs_msg) {
  if ((localization_state_ == msf::LocalizationMeasureState::OK ||
       localization_state_ == msf::LocalizationMeasureState::VALID) &&
      FLAGS_gnss_only_init) {
    return;
  }

  localization_integ_.RawObservationProcess(*raw_obs_msg);

  const auto &result = localization_integ_.GetLastestGnssLocalization();

  if (result.state() == msf::LocalizationMeasureState::OK ||
      result.state() == msf::LocalizationMeasureState::VALID) {
    publisher_->PublishLocalizationMsfGnss(result.localization());
  }
}

void MSFLocalization::OnGnssRtkEph(
    const std::shared_ptr<drivers::gnss::GnssEphemeris> &gnss_orbit_msg) {
  if ((localization_state_ == msf::LocalizationMeasureState::OK ||
       localization_state_ == msf::LocalizationMeasureState::VALID) &&
      FLAGS_gnss_only_init) {
    return;
  }

  localization_integ_.RawEphemerisProcess(*gnss_orbit_msg);
}

void MSFLocalization::OnGnssHeading(
    const std::shared_ptr<drivers::gnss::Heading> &gnss_heading_msg) {
  if ((localization_state_ == msf::LocalizationMeasureState::OK ||
       localization_state_ == msf::LocalizationMeasureState::VALID) &&
      FLAGS_gnss_only_init) {
    return;
  }
  #if 1
  localization_integ_.GnssHeadingProcess(*gnss_heading_msg);
  gnss_heading_msg_.CopyFrom(*gnss_heading_msg);
  #else
  //modified by shzhw, correct heading
  drivers::gnss::Heading correct_gnss_heading_msg;  
  correct_gnss_heading_msg.CopyFrom(*gnss_heading_msg);
    #if 0
    double correct_gnss_heading = correct_gnss_heading_msg.heading() - 90.0;
    if (correct_gnss_heading < 0.0) {
      correct_gnss_heading +=360.0;
    } else if (correct_gnss_heading > 360.0) {
      correct_gnss_heading -= 360.0;
    }
    #else
      double correct_gnss_heading = correct_gnss_heading_msg.heading()-90;
              if (correct_gnss_heading < 0.0) {
      correct_gnss_heading +=360.0;
    } else if (correct_gnss_heading > 360.0) {
      correct_gnss_heading -= 360.0;
    }
    if (correct_gnss_heading < 180.0) {
      correct_gnss_heading = -correct_gnss_heading;
    } else if (correct_gnss_heading >= 180.0) {
      correct_gnss_heading = 360.0 - correct_gnss_heading;
    }
        if (correct_gnss_heading < 0.0) {
      correct_gnss_heading +=360.0;
    } else if (correct_gnss_heading > 360.0) {
      correct_gnss_heading -= 360.0;
    }
    #endif

  correct_gnss_heading_msg.set_heading(correct_gnss_heading);
  localization_integ_.GnssHeadingProcess(correct_gnss_heading_msg);
  #endif
}

void MSFLocalization::SetPublisher(
    const std::shared_ptr<LocalizationMsgPublisher> &publisher) {
  publisher_ = publisher;
}

void MSFLocalization::CompensateImuVehicleExtrinsic(
    LocalizationEstimate *local_result) {
  CHECK_NOTNULL(local_result);
  // calculate orientation_vehicle_world
  apollo::localization::Pose *posepb_loc = local_result->mutable_pose();
  const apollo::common::Quaternion &orientation = posepb_loc->orientation();
  const Eigen::Quaternion<double> quaternion(
      orientation.qw(), orientation.qx(), orientation.qy(), orientation.qz());
  Eigen::Quaternion<double> quat_vehicle_world = quaternion * imu_vehicle_quat_;

  // set heading according to rotation of vehicle
  posepb_loc->set_heading(common::math::QuaternionToHeading(
      quat_vehicle_world.w(), quat_vehicle_world.x(), quat_vehicle_world.y(),
      quat_vehicle_world.z()));

  // set euler angles according to rotation of vehicle
  apollo::common::Point3D *eulerangles = posepb_loc->mutable_euler_angles();
  common::math::EulerAnglesZXYd euler_angle(
      quat_vehicle_world.w(), quat_vehicle_world.x(), quat_vehicle_world.y(),
      quat_vehicle_world.z());
  eulerangles->set_x(euler_angle.pitch());
  eulerangles->set_y(euler_angle.roll());
  eulerangles->set_z(euler_angle.yaw());
}

bool MSFLocalization::LoadGnssAntennaExtrinsic(
    const std::string &file_path, double *offset_x, double *offset_y,
    double *offset_z, double *uncertainty_x, double *uncertainty_y,
    double *uncertainty_z) {
  YAML::Node config = YAML::LoadFile(file_path);
  if (config["leverarm"]) {
    if (config["leverarm"]["primary"]["offset"]) {
      *offset_x = config["leverarm"]["primary"]["offset"]["x"].as<double>();
      *offset_y = config["leverarm"]["primary"]["offset"]["y"].as<double>();
      *offset_z = config["leverarm"]["primary"]["offset"]["z"].as<double>();

      if (config["leverarm"]["primary"]["uncertainty"]) {
        *uncertainty_x =
            config["leverarm"]["primary"]["uncertainty"]["x"].as<double>();
        *uncertainty_y =
            config["leverarm"]["primary"]["uncertainty"]["y"].as<double>();
        *uncertainty_z =
            config["leverarm"]["primary"]["uncertainty"]["z"].as<double>();
      }
      return true;
    }
  }
  return false;
}

bool MSFLocalization::LoadImuVehicleExtrinsic(const std::string &file_path,
                                              double *quat_qx, double *quat_qy,
                                              double *quat_qz,
                                              double *quat_qw) {
  if (!cyber::common::PathExists(file_path)) {
    return false;
  }
  YAML::Node config = YAML::LoadFile(file_path);
  if (config["transform"]) {
    if (config["transform"]["translation"]) {
      if (config["transform"]["rotation"]) {
        *quat_qx = config["transform"]["rotation"]["x"].as<double>();
        *quat_qy = config["transform"]["rotation"]["y"].as<double>();
        *quat_qz = config["transform"]["rotation"]["z"].as<double>();
        *quat_qw = config["transform"]["rotation"]["w"].as<double>();
        return true;
      }
    }
  }
  return false;
}

bool MSFLocalization::LoadZoneIdFromFolder(const std::string &folder_path,
                                           int *zone_id) {
  std::string map_zone_id_folder;
  if (cyber::common::DirectoryExists(folder_path + "/map/000/north")) {
    map_zone_id_folder = folder_path + "/map/000/north";
  } else if (cyber::common::DirectoryExists(folder_path + "/map/000/south")) {
    map_zone_id_folder = folder_path + "/map/000/south";
  } else {
    return false;
  }

  auto folder_list = cyber::common::ListSubPaths(map_zone_id_folder);
  for (auto itr = folder_list.begin(); itr != folder_list.end(); ++itr) {
    *zone_id = std::stoi(*itr);
    return true;
  }
  return false;
}

}  // namespace localization
}  // namespace apollo
