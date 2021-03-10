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
#include "modules/localization/msf/common/util/frame_transform.h"
// #include "modules/drivers/canbus_api.h"
#include <fstream>  //文件流库函数
#include <iomanip>
#include <iostream>

#define  float_eps 0.000001

namespace apollo {
namespace PositionEst {

bool PositionEst::Init() {

  if (!GetProtoConfig(&position_est_conf_)) {
    return false;
  }

  smoothEnable_ = position_est_conf_.positionpara_smooth().smooth_enable();
  sResolution_ = position_est_conf_.positionpara_smooth().s_resolution();
  sMin_ = position_est_conf_.positionpara_smooth().s_min();
  sNum_ = position_est_conf_.positionpara_smooth().s_num();
  motionStd_ = position_est_conf_.positionpara_smooth().motion_std();
  measStd_ = position_est_conf_.positionpara_smooth().meas_std();

  pdfData_.resize(sNum_);
  for (int i = 0; i < sNum_; i++){
    pdfData_(i) = 1.0/sNum_;
  }
  coeffFiler_.resize(2*sNum_ - 1);
  for (int i = 0; i < (2*sNum_ -1);i++){
    if (i < sNum_){
      coeffFiler_(i) =  calc_norm_pdf((sNum_ -1 -i),motionStd_/sResolution_);
    }else{
      coeffFiler_(i) =  coeffFiler_(2*sNum_ -2 - i);
    }    
  }

  ds_ = 0.0;
  veh_speed_last_ = 0.0;
  timestamp_sec_last_ = 0.0;
  validCnt_side_ = 0;
  validCnt_top_ = 0;
  dist_last_ = 0.0;

  task_type_ = 0;  
  machine_num_ = 0;
  inplace_status_ = 0;
  destination_type_ = 0;
  box_type_ = 0;
  box_position_ = 0;

  destination_type_last_ = 0;
  box_position_last_ = 0;

  boxpos_lidar_ = 0.0;

  destination_area_offset_x1_ = 0.0;
  destination_area_offset_y1_ = 0.0;
  destination_area_offset_x2_ = 0.0;
  destination_area_offset_y2_ = 0.0;

  position_est_writer_ =
      node_->CreateWriter<apollo::control::PadMessage>(FLAGS_pad_topic);

  cyber::ReaderConfig chassis_reader_config;
  chassis_reader_config.channel_name = FLAGS_chassis_topic;
  chassis_reader_config.pending_queue_size = 10;

  chassis_reader_ = node_->CreateReader<apollo::canbus::Chassis>(
      chassis_reader_config, nullptr);
  CHECK(chassis_reader_ != nullptr);

  cyber::ReaderConfig tmcTask_reader_config;
  tmcTask_reader_config.channel_name = FLAGS_rms_task_topic;
  tmcTask_reader_config.pending_queue_size = 10;

  tmcTask_reader_ = node_->CreateReader<apollo::rms::msgTaskCoord>(
      tmcTask_reader_config, nullptr);
  CHECK(tmcTask_reader_ != nullptr);

  cyber::ReaderConfig  boxPos_reader_config;
  boxPos_reader_config.channel_name =  "apollo/lidar";
  boxPos_reader_config.pending_queue_size = 10;

  boxPos_reader_ = node_->CreateReader<apollo::lidar::BenewakeLidar>(
      boxPos_reader_config, nullptr);
  CHECK( boxPos_reader_ != nullptr);

  cyber::ReaderConfig localization_reader_config;
  localization_reader_config.channel_name = FLAGS_localization_topic;
  localization_reader_config.pending_queue_size = 10;

  localization_reader_ = node_->CreateReader<apollo::localization::LocalizationEstimate>(
      localization_reader_config, nullptr);
  CHECK(localization_reader_ != nullptr);

  return true;
}

bool PositionEst::Proc(const std::shared_ptr<ContiRadar>& RadarMessage) {  
  std::lock_guard<std::mutex> lock(mutex_);

  if ( smoothEnable_ == 1){
    chassis_reader_->Observe();
    const auto msgChassis = chassis_reader_->GetLatestObserved();     
    float veh_speed = 0.0;
    double timestamp_sec = 0.0;
    if (msgChassis != nullptr) {      
      if (msgChassis->gear_location() == canbus::Chassis::GEAR_REVERSE){
        veh_speed =  - msgChassis->speed_mps();
      }else if (msgChassis->gear_location() == canbus::Chassis::GEAR_DRIVE){
        veh_speed =  msgChassis->speed_mps();
      }else{
        veh_speed =  0.0;
      }    
      timestamp_sec = msgChassis->header().timestamp_sec();
      motion_update(veh_speed,timestamp_sec);      
    }
  }

  float range = 0.0f;
  bool flag = false;

  tmcTask_reader_->Observe();
  const auto msgTask = tmcTask_reader_->GetLatestObserved();  
     //traget position gps to UTM
  apollo::localization::msf::UTMCoor traget_utm_xy;      
   
  if (msgTask != nullptr){
    task_type_ = msgTask->task_type();
    destination_type_ = msgTask->machine_type();
    machine_num_ = msgTask->machine_num();
    inplace_status_ = msgTask->machine_status();
    box_type_ = msgTask->box_type();
    box_position_ = msgTask->box_position();
    /**/
    double lat = msgTask->dst_coord().y();
    double lon = msgTask->dst_coord().x();
    apollo::localization::msf::FrameTransform::LatlonToUtmXY(lon/180.0*M_PI,
                                    lat/180.0*M_PI, &traget_utm_xy);
  }  
  
  if ((destination_type_ != destination_type_last_) || (box_position_ != box_position_last_)){
    AERROR << "destination_type_last = " << destination_type_last_;
    AERROR << "box_position_last = " << box_position_last_;
    AERROR << "destination_type = " << destination_type_;
    AERROR << "box_position = " << box_position_;
    AERROR << "task_type = " << task_type_;
    AERROR << "box_type = " << box_type_;
    destination_type_last_ = destination_type_;
    box_position_last_ = box_position_;

    validCnt_side_ = 0;
    validCnt_top_ = 0;    
    dist_last_ = 0.0;
    for (int i = 0; i < sNum_; i++){
    pdfData_(i) = 1.0/sNum_;
   }

    uint32_t  key = destination_type_ * 10 + box_position_;
    PosLimitConf   config_posLimit = position_est_conf_.positionpara_poslimit();
    switch (key) {
      case 11: {
        destination_area_offset_x1_ =  (config_posLimit.dist_des_rear_yard_f()
                                                                - config_posLimit.dist_offset_yard_f()) *cos(-1.24) ;
        destination_area_offset_y1_ =  (config_posLimit.dist_des_rear_yard_f()
                                                                - config_posLimit.dist_offset_yard_f()) *sin(-1.24) ;
        destination_area_offset_x2_ =  (config_posLimit.dist_des_rear_yard_f()
                                                                + config_posLimit.dist_offset_yard_r()) *cos(-1.24) ;
        destination_area_offset_y2_ =  (config_posLimit.dist_des_rear_yard_f()
                                                                + config_posLimit.dist_offset_yard_r()) *sin(-1.24) ;
        break;
      }
      case 12: {
        destination_area_offset_x1_ =  (config_posLimit.dist_des_rear_yard_c()
                                                                - config_posLimit.dist_offset_yard_f()) *cos(-1.24) ;
        destination_area_offset_y1_ =  (config_posLimit.dist_des_rear_yard_c()
                                                                - config_posLimit.dist_offset_yard_f()) *sin(-1.24) ;
        destination_area_offset_x2_ =  (config_posLimit.dist_des_rear_yard_c()
                                                                + config_posLimit.dist_offset_yard_r()) *cos(-1.24) ;
        destination_area_offset_y2_ =  (config_posLimit.dist_des_rear_yard_c()
                                                                + config_posLimit.dist_offset_yard_r()) *sin(-1.24) ;
        break;
      }
      case 13: {
        destination_area_offset_x1_ =  (config_posLimit.dist_des_rear_yard_r()
                                                                - config_posLimit.dist_offset_yard_f()) *cos(-1.24) ;
        destination_area_offset_y1_ =  (config_posLimit.dist_des_rear_yard_r()
                                                                - config_posLimit.dist_offset_yard_f()) *sin(-1.24) ;
        destination_area_offset_x2_ =  (config_posLimit.dist_des_rear_yard_r()
                                                                + config_posLimit.dist_offset_yard_r()) *cos(-1.24) ;
        destination_area_offset_y2_ =  (config_posLimit.dist_des_rear_yard_r()
                                                                + config_posLimit.dist_offset_yard_r()) *sin(-1.24) ;
        break;
      }
      case 21: {
        destination_area_offset_x2_ =  (config_posLimit.dist_des_rear_bank_f()
                                                                - config_posLimit.dist_offset_bank_f()) *cos(1.9) ;
        destination_area_offset_y2_ =  (config_posLimit.dist_des_rear_bank_f()
                                                                - config_posLimit.dist_offset_bank_f()) *sin(1.9) ;
        destination_area_offset_x1_ =  (config_posLimit.dist_des_rear_bank_f()
                                                                + config_posLimit.dist_offset_bank_r()) *cos(1.9) ;
        destination_area_offset_y1_ =  (config_posLimit.dist_des_rear_bank_f()
                                                                + config_posLimit.dist_offset_bank_r()) *sin(1.9) ;
        break;
      }
      case 22: {
        destination_area_offset_x2_ =  (config_posLimit.dist_des_rear_bank_c()
                                                                - config_posLimit.dist_offset_bank_f()) *cos(1.9) ;
        destination_area_offset_y2_ =  (config_posLimit.dist_des_rear_bank_c()
                                                                - config_posLimit.dist_offset_bank_f()) *sin(1.9) ;
        destination_area_offset_x1_ =  (config_posLimit.dist_des_rear_bank_c()
                                                                + config_posLimit.dist_offset_bank_r()) *cos(1.9) ;
        destination_area_offset_y1_ =  (config_posLimit.dist_des_rear_bank_c()
                                                                + config_posLimit.dist_offset_bank_r()) *sin(1.9) ;
        break;
      }
      case 23: {
        destination_area_offset_x2_ =  (config_posLimit.dist_des_rear_bank_r()
                                                                - config_posLimit.dist_offset_bank_f()) *cos(1.9) ;
        destination_area_offset_y2_ =  (config_posLimit.dist_des_rear_bank_r()
                                                                - config_posLimit.dist_offset_bank_f()) *sin(1.9) ;
        destination_area_offset_x1_ =  (config_posLimit.dist_des_rear_bank_r()
                                                                + config_posLimit.dist_offset_bank_r()) *cos(1.9) ;
        destination_area_offset_y1_ =  (config_posLimit.dist_des_rear_bank_r()
                                                                + config_posLimit.dist_offset_bank_r()) *sin(1.9) ;
        break;
      }
      default: {
        destination_area_offset_x1_ = 0.0;
        destination_area_offset_y1_ = 0.0;
        destination_area_offset_x2_ = 0.0;
        destination_area_offset_y2_ = 0.0;
        break;
      }
    }    
  }

  double PositionEst_vaild_startx = traget_utm_xy.x  +  destination_area_offset_x1_; 
  double PositionEst_vaild_starty = traget_utm_xy.y + destination_area_offset_y1_;  
  double PositionEst_vaild_endx =  traget_utm_xy.x + destination_area_offset_x2_;
  double PositionEst_vaild_endy = traget_utm_xy.y  + destination_area_offset_y2_;

  localization_reader_->Observe();
  const auto localization_msg = localization_reader_->GetLatestObserved();
  if (localization_msg != nullptr) {  
    double heading = localization_msg->pose().heading() ;
    double cur_x = localization_msg->pose().position().x();
    double cur_y = localization_msg->pose().position().y();
    if  (  ( (destination_type_ == 1)  &&  (std::fabs(heading - (-1.24)) < 0.2))
        || ( (destination_type_ == 2)  &&  (std::fabs(heading - 1.9) < 0.2) ) ){
      if ((cur_x - PositionEst_vaild_startx < 0.0) ||
           (cur_y - PositionEst_vaild_starty > 0.0) ||
           (cur_x - PositionEst_vaild_endx > 0.0) ||
           (cur_y - PositionEst_vaild_endy < 0.0)) {        
        return false;   
      }
      AERROR << "PositionEst_vaild_startx = " << PositionEst_vaild_startx;
      AERROR << "PositionEst_vaild_starty = " << PositionEst_vaild_starty;
      AERROR << "PositionEst_vaild_endx = " << PositionEst_vaild_endx;
      AERROR << "PositionEst_vaild_endy = " << PositionEst_vaild_endy;
      AERROR << "cur_x = " << cur_x;
      AERROR << "cur_y = " << cur_y;
      AERROR << "Cur pos is vaild!";
   } 
 }

  boxPos_reader_->Observe();
  auto msgLidar = boxPos_reader_->GetLatestObserved();
  if(msgLidar != nullptr){
    boxpos_lidar_ = (msgLidar->distance())/100.0;
  }   

  if (RadarMessage->has_header()) {
    if ((RadarMessage->radar_id() == 5 ||
         RadarMessage->header().module_name() == "radar_top") &&
        (destination_type_ == 2)) {
      // flag = top_distance_est(RadarMessage,
      //                       position_est_conf_.positionpara_top(), range);
      flag = top_distance_est2(RadarMessage,
                               position_est_conf_.positionpara_top2(), range);
    } else if ((RadarMessage->radar_id() == -6 ||
                RadarMessage->header().module_name() == "radar_top_left") &&
               (destination_type_ == 1)) {
      flag = side_distance_est(RadarMessage, 
                               position_est_conf_.positionpara_topleft(), range);
    } else if ((RadarMessage->radar_id() == 6 ||
               RadarMessage->header().module_name() == "radar_top_right")
               && (destination_type_ == 1) ){
      flag = side_distance_est(RadarMessage, 
                               position_est_conf_.positionpara_topright(),range);
    }
  }

  PadMessage outputMessage;
  if (flag) {
    if ( smoothEnable_ == 1) {      
      AERROR << "veh_speed = " << veh_speed_last_;
      observation_update(range);
    }
   
    AERROR << "dist_est2 = " << range;
    outputMessage.set_moving_distance(range);
    common::util::FillHeader(node_->Name(), &outputMessage);
    position_est_writer_->Write(outputMessage);
  }

  return true;
}


void PositionEst::dist_compensation(const BoxRelPosConf& config, float &dist_est){
  float box20_yard_rel_f = config.box20_yard_rel_f();
  float box20_yard_rel_c = config.box20_yard_rel_c();
  float box20_yard_rel_r = config.box20_yard_rel_r();

  float box20d_yard_rel_c = config.box20d_yard_rel_c();
  float box40_yard_rel_c = config.box40_yard_rel_c();
  float box45_yard_rel_c = config.box45_yard_rel_c();

  float box20_bank_rel_f = config.box20_bank_rel_f();
  float box20_bank_rel_c = config.box20_bank_rel_c();
  float box20_bank_rel_r = config.box20_bank_rel_r();

  float box20d_bank_rel_c = config.box20d_bank_rel_c();
  float box40_bank_rel_c = config.box40_bank_rel_c();
  float box45_bank_rel_c = config.box45_bank_rel_c();

  uint32_t  key = destination_type_ * 100 + box_type_ *10 + box_position_;
  float dist_compensation_;
  switch (key) {
    case 111: {
      dist_compensation_ = (boxpos_lidar_ - box20_yard_rel_f); break;
    }
    case 112: {
      dist_compensation_ = (boxpos_lidar_ - box20_yard_rel_c); break;
    }
    case 113: {
      dist_compensation_ = (boxpos_lidar_ - box20_yard_rel_r); break;
    }
    case 122: {
      dist_compensation_ = (boxpos_lidar_ - box20d_yard_rel_c); break;
    }
    case 132: {
      dist_compensation_ = (boxpos_lidar_ - box40_yard_rel_c); break;
    }
    case 142: {
      dist_compensation_ = (boxpos_lidar_ - box45_yard_rel_c); break;
    }
    case 211: {
      dist_compensation_ = (boxpos_lidar_ - box20_bank_rel_f); break;
    }
    case 212: {
      dist_compensation_ = (boxpos_lidar_ - box20_bank_rel_c); break;
    }
    case 213: {
      dist_compensation_ = (boxpos_lidar_ - box20_bank_rel_r); break;
    }
    case 222: {
      dist_compensation_ = (boxpos_lidar_ - box20d_bank_rel_c); break;
    }
    case 232: {
      dist_compensation_ = (boxpos_lidar_ - box40_bank_rel_c); break;
    }
    case 242: {
      dist_compensation_ = (boxpos_lidar_ - box45_bank_rel_c); break;
    }
    default: { 
      dist_compensation_ = 0.0; break;
    }    
 }
  if (fabs(dist_compensation_) < 0.5)  {
    dist_est += dist_compensation_;
  }
}

bool PositionEst::side_distance_est(
    const std::shared_ptr<ContiRadar> &RadarMessage, const SideConf &config,
    float &dist_est) {
  
  float snr_min_ = config.snr_min();
  float delta_range_ = config.delta_range();
  float y_min_ = config.y_min();
  float y_max_ = config.y_max();  
  size_t num_min_ = config.num_min();
  size_t num_max_ = config.num_max();
  float err_weight_ = config.err_weight(); 
  float output_range_limit_ = config.output_range_limit();
  float output_range_limit_max = 2.0;
  int validCnt_side_max = 2;

  float interval_reflector_1,interval_reflector_2,range_min_,range_max_,distance_offset_;
  uint32_t  key = box_type_ *10 + box_position_;
  switch (key) {
    case 11: {
      interval_reflector_1 = config.interval_reflector_r1();
      interval_reflector_2 = config.interval_reflector_r2();
      range_min_ = config.range_min_f();
      range_max_ = config.range_max_f();
      distance_offset_ = config.box20_distance_offset_f();
      break;
    }
    case 12:{
      interval_reflector_1 = config.interval_reflector_f1();
      interval_reflector_2 = config.interval_reflector_f2();
      range_min_ = config.range_min_c();
      range_max_ = config.range_max_c();
      distance_offset_ = config.box20_distance_offset_c();
      break;
    }
    case 22: {
      interval_reflector_1 = config.interval_reflector_f1();
      interval_reflector_2 = config.interval_reflector_f2();
      range_min_ = config.range_min_c();
      range_max_ = config.range_max_c();
      distance_offset_ = config.box20d_distance_offset_c();
      break;
    }
    case 32: {
      interval_reflector_1 = config.interval_reflector_f1();
      interval_reflector_2 = config.interval_reflector_f2();
      range_min_ = config.range_min_c();
      range_max_ = config.range_max_c();
      distance_offset_ = config.box40_distance_offset_c();
      break;
    }
    case 42:{
      interval_reflector_1 = config.interval_reflector_f1();
      interval_reflector_2 = config.interval_reflector_f2();
      range_min_ = config.range_min_c();
      range_max_ = config.range_max_c();
      distance_offset_ = config.box45_distance_offset_c();
      break;
    }
    case 13: {
      interval_reflector_1 = config.interval_reflector_f1();
      interval_reflector_2 = config.interval_reflector_f2();
      range_min_ = config.range_min_r();
      range_max_ = config.range_max_r();
      distance_offset_ = config.box20_distance_offset_r();
      break;
    }
    default:{
      interval_reflector_1 = config.interval_reflector_f1();
      interval_reflector_2 = config.interval_reflector_f2();
      range_min_ = config.range_min_c();
      range_max_ = config.range_max_c();
      distance_offset_ = config.box40_distance_offset_c();
      break;
    }
  }

  float dist = 0.0;
  bool flag = false;

  float interval_reflector = interval_reflector_1 + interval_reflector_2;

  // step1
  auto radar_objs = RadarMessage->contiobs();
  std::vector<int> obj_index;
  for (int i = 0; i < radar_objs.size(); i++) {
    if ((radar_objs[i].snr() > snr_min_) &&
        (radar_objs[i].range() > range_min_) &&
        (radar_objs[i].range() < range_max_) &&
        (radar_objs[i].pos_y() > y_min_) && (radar_objs[i].pos_y() < y_max_)) {
      obj_index.push_back(i);
    }
  }

  // step2
  if ((obj_index.size() < num_min_) || (obj_index.size() > num_max_)) {
    if ((validCnt_side_ > 0) && (validCnt_side_ < validCnt_side_max)){
      validCnt_side_ -=  1;  
    }    
    if (obj_index.size() > num_max_){
      AERROR << "num_points = " << obj_index.size();  //test
    } 
    return flag;
  }


    // step3
  Eigen::MatrixXd distance = Eigen::MatrixXd(num_max_, num_max_);
  for (size_t row = 0; row < obj_index.size(); row++) {
    float x1 = radar_objs[obj_index[row]].pos_x();
    float y1 = radar_objs[obj_index[row]].pos_y();
    for (size_t col = row + 1; col < obj_index.size(); col++) {
      float x2 = radar_objs[obj_index[col]].pos_x();
      float y2 = radar_objs[obj_index[col]].pos_y();
      distance(row, col) = sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2));
      distance(col, row) = distance(row, col);
    }
  }

  // step4
  std::list<std::pair<size_t, size_t>> pair_list;
  std::pair<size_t, size_t> onepair;
  for (size_t row = 0; row < obj_index.size(); row++) {
    for (size_t col = row + 1; col < obj_index.size(); col++) {
      if (fabs(distance(row, col) - interval_reflector_1) < delta_range_) {
        onepair.first = row;
        onepair.second = col;
        pair_list.insert(pair_list.end(), onepair);
      }
    }
  }

  if (pair_list.size() < 1) {
    if ((validCnt_side_ > 0) && (validCnt_side_ < validCnt_side_max)){
      validCnt_side_ -=  1;  
    }
    return flag;
  }

  size_t index1, index2, index3;
  size_t index_p1, index_p2, index_p3;
  std::vector<size_t> point1_list, point2_list, point3_list;
  for (std::list<std::pair<size_t, size_t>>::iterator it = pair_list.begin();
       it != pair_list.end(); it++) {
    index1 = obj_index[it->first];
    index2 = obj_index[it->second];
    if (radar_objs[index1].pos_y() > radar_objs[index2].pos_y()) {
      index_p1 = (it->first);
      index_p2 = (it->second);
    } else {
      index_p1 = (it->second);
      index_p2 = (it->first);
    }
    index2 = obj_index[index_p2];
    for (index_p3 = 0; index_p3 < obj_index.size(); index_p3++) {
      if ((index_p3 == index_p1) || (index_p3 == index_p2)) {
        continue;
      }
      index3 = obj_index[index_p3];
      if ((fabs(distance(index_p2, index_p3) - interval_reflector_2) <
           delta_range_) &&
          (fabs(distance(index_p1, index_p3) - interval_reflector) <
           delta_range_) &&
          (radar_objs[index2].pos_y() > radar_objs[index3].pos_y())) {
        point1_list.push_back(index_p1);
        point2_list.push_back(index_p2);
        point3_list.push_back(index_p3);
      }
    }
  }
  
  // step5
  if (point1_list.size() < 1) {
    if ((validCnt_side_ > 0) && (validCnt_side_ < validCnt_side_max)){
      validCnt_side_ -=  1;  
    }
    return flag;
  }
  

  float err, err1, err2, err3, err4; 
  float min_err = delta_range_;
  size_t best_index = point1_list.size();
  for (size_t i = 0; i < point1_list.size(); i++) {
    index1 = obj_index[point1_list[i]];
    index2 = obj_index[point2_list[i]];
    index3 = obj_index[point3_list[i]];
    float deltax = radar_objs[index1].pos_x() - radar_objs[index3].pos_x();
    float deltay = radar_objs[index1].pos_y() - radar_objs[index3].pos_y();
    if ((fabs(deltax) < fabs(deltay)) || (deltax * deltay < 0.0)) {
      continue;
    }
    err1 = fabs(distance(point1_list[i], point3_list[i]) - interval_reflector);
    err2 =
        fabs(distance(point1_list[i], point2_list[i]) - interval_reflector_1);
    err3 =
        fabs(distance(point2_list[i], point3_list[i]) - interval_reflector_2);
    err4 = fabs(distance(point1_list[i], point2_list[i]) +
                distance(point2_list[i], point3_list[i]) -
                distance(point1_list[i], point3_list[i]));
    err = std::max(std::max(err1, err2), err3) * err_weight_ +
          (1 - err_weight_) * err4;
    if (err < min_err) {
      min_err = err;
      best_index = i;
    }
  }
  if (best_index == point1_list.size()) {
    if ((validCnt_side_ > 0) && (validCnt_side_ < validCnt_side_max)){
      validCnt_side_ -=  1;  
    }
    return flag;
  }

  // step6
  float range1, range2, range3;
  index1 = obj_index[point1_list[best_index]];
  index2 = obj_index[point2_list[best_index]];
  index3 = obj_index[point3_list[best_index]];

  //if (radar_objs[index1].pos_y() > radar_objs[index3].pos_y()) {
    range1 = radar_objs[index1].range();
    range3 = radar_objs[index3].range();
 // } else {
  //  range1 = radar_objs[index3].range();
  //  range3 = radar_objs[index1].range();
  //}
  range2 = radar_objs[index2].range();

  float L1 = distance(point1_list[best_index],point2_list[best_index]);  
  float L2 = distance(point2_list[best_index],point3_list[best_index]);
  float L =  distance(point1_list[best_index],point3_list[best_index]);

  // step7
  if ((range1 > range2) && (range2 > range3)) {
    // flag = true;

    // float dist1 = (interval_reflector_1 * interval_reflector_1 +
    //                range1 * range1 - range2 * range2) /
    //               (2.0 * interval_reflector_1);
    // float dist2 = (interval_reflector * interval_reflector + range1 * range1 -
    //                range3 * range3) /
    //               (2.0 * interval_reflector);
    // float dist3 = (interval_reflector_2 * interval_reflector_2 +
    //                range2 * range2 - range3 * range3) /
    //                   (2.0 * interval_reflector_2) +
    //               interval_reflector_1;

    float dist1 = (L1 * L1 +  range1 * range1 - range2 * range2) / (2.0 * L1);
    float dist2 = (L * L + range1 * range1 -  range3 * range3) / (2.0 * L);
    float dist3 = (L2 * L2 + range2 * range2 - range3 * range3) / (2.0 * L2) + L1;

    dist = distance_offset_ - (dist1 + dist2 + dist3) / 3.0;
    AERROR << "dist_est1 = " << dist;
  
    if ((boxpos_lidar_ > 0.0) && (task_type_ == 2 )) {          /////////////////////
      dist_compensation(position_est_conf_.positionpara_boxrelpos(),dist);    
      AERROR << "boxpos_lidar = " << boxpos_lidar_;
    } 
   
    if ((dist_last_ < float_eps) && (dist < output_range_limit_max)){
      dist_last_ = dist;
    } 
    //AERROR << "dist_last_ = " << dist_last_;
  }
  
  float  delta_dist  = dist_last_ - dist;
  float  delta_dist_limit_max =  fabs(veh_speed_last_)  / 20.0 + 0.07;
  float  delta_dist_limit_min = -0.03;

  if ((dist < output_range_limit_max) && (dist > output_range_limit_) 
        && (delta_dist < delta_dist_limit_max)
        && (delta_dist > delta_dist_limit_min)
        && (validCnt_side_ < validCnt_side_max)) {
    validCnt_side_ += 1;  
  }  else  if  ((validCnt_side_ > 0) && (dist < output_range_limit_)){
    validCnt_side_ -= 1;  
  }
  //AERROR << "validCnt_side_ = " << validCnt_side_;//test
  
  if (dist < output_range_limit_max){
    dist_last_ = dist;
  }
  
  if ( validCnt_side_ == validCnt_side_max)
  {
    flag = true;
    dist_est = dist;
  }
  return flag;
}

bool PositionEst::top_distance_est(
    const std::shared_ptr<ContiRadar> &RadarMessage,  const TopConf& config,
    float &dist_est) {
  float dist2Radar = config.dist2radar();
  float radar_height = config.radar_height();
  float target_height = config.target_height();
  float delta_height = target_height - radar_height;
  float px_diff = config.px_diff();
  float py_diff = config.py_diff();
  float snr_min = config.snr_min();
  float snr_diff = config.snr_diff();

  float range = 0.0;
  auto obs = RadarMessage->contiobs();
  bool flag = false;

  int index_1 = -1;
  int index_2 = -1;

  std::vector<int> tmpIndex;
  for (int i = 0; i < obs.size(); i++) {
    auto obs_1 = obs[i];
    float x1 = obs_1.pos_x();
    float snr1 = obs_1.snr();
    if (snr1 < snr_min || x1 < delta_height) {
      continue;
    }
    for (int j = i + 1; j < obs.size(); j++) {
      auto obs_2 = obs[j];
      float x2 = obs_2.pos_x();
      float snr2 = obs_2.snr();
      if (snr2 < snr_min || x2 < delta_height) {
        continue;
      }

      if (fabs(obs_1.pos_x() - obs_2.pos_x()) < px_diff &&
          fabs(obs_1.pos_y()) < py_diff && fabs(obs_2.pos_y()) < py_diff &&
          fabs(obs_1.snr() - obs_2.snr()) < snr_diff &&
          obs_1.snr() > snr_min && obs_2.snr() > snr_min) {
        index_1 = i;
        index_2 = j;
      }
    }
  }

  if (index_1 > -1 && index_2 > -1) {
    float sumPos = obs[index_1].pos_x() + obs[index_2].pos_x();
    float mean_pos_x = sumPos / 2;
    range = sqrt(mean_pos_x * mean_pos_x - delta_height * delta_height) -
            dist2Radar;
    if (range < 10 && range > config.min_range()) {
      flag = true;
      dist_est = (float)(range - config.offset());
    }
  }

  return flag;
}

bool PositionEst::top_distance_est2(
    const std::shared_ptr<ContiRadar> &RadarMessage, const TopConf2 &config,
    float &dist_est) {
  float interval_reflector_1 = config.interval_reflector1();
  float interval_reflector_2 = config.interval_reflector2();
  float snr_min_ = config.snr_min();
  float delta_range_ = config.delta_range();
 
  size_t num_min_ = config.num_min();
  size_t num_max_ = config.num_max();
  float err_weight_ = config.err_weight();
  //float distance_offset_ = config.distance_offset();
  float output_range_limit_ = config.output_range_limit();
  float output_range_limit_max = 2.0;
  int validCnt_top_max = 2;

  float y_min_,y_max_,range_min_,range_max_,y_offset;
  uint32_t  key = box_type_ *10 + box_position_;
  switch (key) {
    case 11: {
      y_min_ = config.y_min_f();
      y_max_ = config.y_max_f();
      range_min_ = config.range_min_f();
      range_max_ = config.range_max_f();
      y_offset = config.box20_y_offset_f();    
      break;
    }
    case 12: {
      y_min_ = config.y_min_c();
      y_max_ = config.y_max_c();
      range_min_ = config.range_min_c();
      range_max_ = config.range_max_c();
      y_offset = config.box20_y_offset_c();
      break;
    }
    case 22: {
      y_min_ = config.y_min_c();
      y_max_ = config.y_max_c();
      range_min_ = config.range_min_c();
      range_max_ = config.range_max_c();
      y_offset = config.box20d_y_offset_c();
      break;
    }
    case 32: {
      y_min_ = config.y_min_c();
      y_max_ = config.y_max_c();
      range_min_ = config.range_min_c();
      range_max_ = config.range_max_c();
      y_offset = config.box40_y_offset_c();
      break;
    }
    case 42: {
      y_min_ = config.y_min_c();
      y_max_ = config.y_max_c();
      range_min_ = config.range_min_c();
      range_max_ = config.range_max_c();
      y_offset = config.box45_y_offset_c();
      break;
    }
    case 13: {
      y_min_ = config.y_min_r();
      y_max_ = config.y_max_r();
      range_min_ = config.range_min_r();
      range_max_ = config.range_max_r();
      y_offset = config.box20_y_offset_r();    
      output_range_limit_max = config.output_range_limit_max(); 
      break;
    }
    default:{
      y_min_ = config.y_min_c();
      y_max_ = config.y_max_c();
      range_min_ = config.range_min_c();
      range_max_ = config.range_max_c();
      y_offset = config.box40_y_offset_c();     
      break;
    }
  }

  float dist = 0.0;
  bool flag = false;

  float interval_reflector = interval_reflector_1 + interval_reflector_2;

  // step1
  auto radar_objs = RadarMessage->contiobs();
  std::vector<int> obj_index;
  for (int i = 0; i < radar_objs.size(); i++) {
    if ((radar_objs[i].snr() > snr_min_) &&
        (radar_objs[i].range() > range_min_) &&
        (radar_objs[i].range() < range_max_) &&
        (radar_objs[i].pos_x() < (range_min_ + 2.0)) && 
	      (radar_objs[i].pos_x() > range_min_) &&       
        (radar_objs[i].pos_y() > y_min_) &&
        (radar_objs[i].pos_y() < y_max_)) {
      obj_index.push_back(i);
    }
  }

  // step2
  if ((obj_index.size() < num_min_) || (obj_index.size() > num_max_)) {
    if ((validCnt_top_ > 0) && (validCnt_top_ < validCnt_top_max)){
      validCnt_top_ -=  1;  
    }
    if (obj_index.size() > num_max_){
      AERROR << "num_points = " << obj_index.size();  //test
    }    
    return flag;
  }


    // step3
  Eigen::MatrixXd distance = Eigen::MatrixXd(num_max_, num_max_);
  for (size_t row = 0; row < obj_index.size(); row++) {
    float x1 = radar_objs[obj_index[row]].pos_x();
    float y1 = radar_objs[obj_index[row]].pos_y();
    for (size_t col = row + 1; col < obj_index.size(); col++) {
      float x2 = radar_objs[obj_index[col]].pos_x();
      float y2 = radar_objs[obj_index[col]].pos_y();
      distance(row, col) = sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2));
      distance(col, row) = distance(row, col);
    }
  }

  // step4
  std::list<std::pair<size_t, size_t>> pair_list;
  std::pair<size_t, size_t> onepair;
  for (size_t row = 0; row < obj_index.size(); row++) {
    for (size_t col = row + 1; col < obj_index.size(); col++) {
      if (fabs(distance(row, col) - interval_reflector_1) < delta_range_) {
        onepair.first = row;
        onepair.second = col;
        pair_list.insert(pair_list.end(), onepair);
      }
    }
  }

  if (pair_list.size() < 1) {
    if ((validCnt_top_ > 0) && (validCnt_top_ < validCnt_top_max)){
      validCnt_top_ -=  1;  
    }
    return flag;
  }

  size_t index1, index2, index3;
  size_t index_p1, index_p2, index_p3;
  std::vector<size_t> point1_list, point2_list, point3_list;
  for (std::list<std::pair<size_t, size_t>>::iterator it = pair_list.begin();
       it != pair_list.end(); it++) {
    index1 = obj_index[it->first];
    index2 = obj_index[it->second];
    if (radar_objs[index1].pos_y() < radar_objs[index2].pos_y()) {
      index_p1 = (it->first);
      index_p2 = (it->second);
    } else {
      index_p1 = (it->second);
      index_p2 = (it->first);
    }
    index2 = obj_index[index_p2];
    for (index_p3 = 0; index_p3 < obj_index.size(); index_p3++) {
      if ((index_p3 == index_p1) || (index_p3 == index_p2)) {
        continue;
      }
      index3 = obj_index[index_p3];
      if ((fabs(distance(index_p2, index_p3) - interval_reflector_2) <
           delta_range_) &&
          (fabs(distance(index_p1, index_p3) - interval_reflector) <
           delta_range_) &&
          (radar_objs[index2].pos_y() < radar_objs[index3].pos_y())) {
        point1_list.push_back(index_p1);
        point2_list.push_back(index_p2);
        point3_list.push_back(index_p3);
      }
    }
  }
  
  // step5
  if (point1_list.size() < 1) {
    if ((validCnt_top_ > 0) && (validCnt_top_ < validCnt_top_max)){
      validCnt_top_ -=  1;  
    }
    return flag;
  }
  

  float err, err1, err2, err3, err4; 
  float min_err = delta_range_;
  size_t best_index = point1_list.size();
  for (size_t i = 0; i < point1_list.size(); i++) {
    index1 = obj_index[point1_list[i]];
    index2 = obj_index[point2_list[i]];
    index3 = obj_index[point3_list[i]];
    float deltax = radar_objs[index1].pos_x() - radar_objs[index3].pos_x();
    float deltay = radar_objs[index1].pos_y() - radar_objs[index3].pos_y();
    //if ((fabs(deltax) > fabs(deltay)) || (deltax * deltay > 0.0)) {
    if ((fabs(deltax) > fabs(deltay)) ) {
      continue;
    }
    err1 = fabs(distance(point1_list[i], point3_list[i]) - interval_reflector);
    err2 =
        fabs(distance(point1_list[i], point2_list[i]) - interval_reflector_1);
    err3 =
        fabs(distance(point2_list[i], point3_list[i]) - interval_reflector_2);
    err4 = fabs(distance(point1_list[i], point2_list[i]) +
                distance(point2_list[i], point3_list[i]) -
                distance(point1_list[i], point3_list[i]));
    err = std::max(std::max(err1, err2), err3) * err_weight_ +
          (1 - err_weight_) * err4;
    if (err < min_err) {
      min_err = err;
      best_index = i;
    }
  }
  if (best_index == point1_list.size()) {
    if ((validCnt_top_ > 0) && (validCnt_top_ < validCnt_top_max)){
      validCnt_top_ -=  1;  
    }
    return flag;
  }

  // step6
 // float range1, range2, range3;
  index1 = obj_index[point1_list[best_index]];
  index2 = obj_index[point2_list[best_index]];
  index3 = obj_index[point3_list[best_index]];

  
  //range1 = radar_objs[index1].range();
  //range3 = radar_objs[index3].range();
  //range2 = radar_objs[index2].range();
  
  float y1, y2, y3;
  y1 = radar_objs[index1].pos_y();
  y2 = radar_objs[index2].pos_y();
  y3 = radar_objs[index3].pos_y();  

  // step7
  //if ((range1 > range2) && (range2 > range3)) {
    // flag = true;

    //float dist1 = (interval_reflector_1 * interval_reflector_1 +
    //               range1 * range1 - range2 * range2) /
    //              (2.0 * interval_reflector_1);
    //float dist2 = (interval_reflector * interval_reflector + range1 * range1 -
    //               range3 * range3) /
    //              (2.0 * interval_reflector);
    //float dist3 = (interval_reflector_2 * interval_reflector_2 +
    //               range2 * range2 - range3 * range3) /
    //                  (2.0 * interval_reflector_2) +
    //              interval_reflector_1;

    //dist =  (dist1 + dist2 + dist3) / 3.0 - distance_offset_;
    
    dist = (y_offset - y1 - y2 - y3) / 3.0;    
    AERROR << "dist_est1 = " << dist;    
    
    if ((boxpos_lidar_ > 0.0) && (task_type_ == 2 )) {          /////////////////////
      dist_compensation(position_est_conf_.positionpara_boxrelpos(),dist);    
      AERROR << "boxpos_lidar = " << boxpos_lidar_;
    }  
  //}
 
  //AERROR << "dist_last_ = " << dist_last_;//test
  if ((dist_last_ < float_eps) && (dist < output_range_limit_max)){
      dist_last_ = dist;
  }
  float  delta_dist  = dist_last_ - dist;
  float  delta_dist_limit_max = fabs(veh_speed_last_)  / 20.0 + 0.07;
  float  delta_dist_limit_min = -0.03;

  //AERROR << "delta_dist = " << delta_dist;//test
  //AERROR << "delta_dist_limit_max = " << delta_dist_limit_max;//test

  if ((dist < output_range_limit_max) && (dist > output_range_limit_) 
        && (delta_dist < delta_dist_limit_max)
        && (delta_dist > delta_dist_limit_min)
        && (validCnt_top_ < validCnt_top_max)) {
    validCnt_top_ += 1;  
  }  else  if  ((validCnt_top_ > 0) && (dist < output_range_limit_)){
    validCnt_top_ -= 1;  
  }
  //AERROR << "validCnt_top_ = " << validCnt_top_;//test

  if (dist < output_range_limit_max){
    dist_last_ = dist;
  }
  if ( validCnt_top_ == validCnt_top_max)
  {
    flag = true;
    dist_est = dist;
  }
  return flag;

}

/// /////////////////////////////////////////////////////////////////////////////////////////

void PositionEst::motion_update(float veh_speed,double timestamp_sec){
  double dt = 0.0;
  dt = timestamp_sec - timestamp_sec_last_;
  // std::cout  << std::fixed << "timestamp_sec:" << timestamp_sec  << "       timestamp_sec_last_:" << timestamp_sec_last_  << "       dt: " << dt << std::endl;
  if (dt > 0.20){
    dt = 0.0;
  }
  ds_ = ds_ - veh_speed_last_ * dt ;
  timestamp_sec_last_ = timestamp_sec;
  veh_speed_last_ = veh_speed;

  int sShift = ds_ / sResolution_;    
  if (abs(sShift) >= 1 ){  //map should be shifted
    ds_ = ds_ - sShift * sResolution_;     
    map_shift(sShift);          
  }
  gaussian_filter_1D();
}

void PositionEst::map_shift(int sShift){
  Eigen::VectorXf pdfData_tmp = pdfData_;  
  for(int i = 0; i < sNum_; i++){       
    int iNew = i + sShift; 
    if ((0 <= iNew) && (iNew < sNum_) ){
      pdfData_(iNew) = pdfData_tmp(i);
    } 
  }
}

void PositionEst::gaussian_filter_1D(){
  Eigen::VectorXf pdfData_out(sNum_); 
  Eigen::VectorXf coeff(sNum_); 
  for(int i = 0; i < sNum_; i++){
    coeff = coeffFiler_.segment(sNum_ -1 - i, sNum_);
    Eigen::VectorXf m = pdfData_.cwiseProduct(coeff);
    pdfData_out(i) = m.sum()/coeff.sum();
  }    
 pdfData_ = pdfData_out; 
}


void PositionEst::observation_update(float &dist_est){
  for(int i = 0; i < sNum_; i++){
    float s = i * sResolution_ + sMin_; 
    float d = s - dist_est; 
    pdfData_(i) = pdfData_(i) * calc_norm_pdf(d,measStd_);
  }

  float maxPdf = 0.0;
  int indexMaxPdf = -1;      
  normalize_probability(indexMaxPdf,maxPdf);
         
  if ((indexMaxPdf >= 0) && (indexMaxPdf < sNum_)){
    dist_est = indexMaxPdf * sResolution_ + sMin_; 
  }   
  if (indexMaxPdf == -1) {
    AERROR << "indexMaxPdf = " << indexMaxPdf;  //test
  }
}

void PositionEst::normalize_probability(int &indexMaxPdf,float &maxPdf){
  float sump =  pdfData_.sum();   
  if (sump < float_eps){
    return;
  }
  for(int i = 0; i< sNum_; i++){ 
    pdfData_(i) = pdfData_(i) / sump;        
    if (pdfData_(i) > maxPdf) {
      maxPdf = pdfData_(i);
      indexMaxPdf = i; 
    }   
  }  
}


float PositionEst::calc_norm_pdf(float d,float std){
  float pdf = 0.0;
  if  (fabs(d) > 5 * std){
    pdf  = float_eps;
  }else{
    pdf = 1 / exp( d * d/ (2 * std * std));
  }
  return pdf;
}

}  // namespace PositionEst
}  // namespace apollo
