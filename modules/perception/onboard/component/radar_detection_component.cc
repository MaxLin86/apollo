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
#include "modules/perception/onboard/component/radar_detection_component.h"

#include "modules/common/time/time.h"
#include "modules/perception/common/sensor_manager/sensor_manager.h"
#include "modules/perception/lib/utils/perf.h"

#include <numeric>


namespace apollo {
namespace perception {
namespace onboard {

bool RadarDetectionComponent::Init() {
  RadarComponentConfig comp_config;
  if (!GetProtoConfig(&comp_config)) {
    return false;
  }
  AINFO << "Radar Component Configs: " << comp_config.DebugString();

  // To load component configs
  tf_child_frame_id_ = comp_config.tf_child_frame_id();
  radar_forward_distance_ = comp_config.radar_forward_distance();
  preprocessor_method_ = comp_config.radar_preprocessor_method();
  perception_method_ = comp_config.radar_perception_method();
  pipeline_name_ = comp_config.radar_pipeline_name();
  odometry_channel_name_ = comp_config.odometry_channel_name();

  obj_match_size_ = comp_config.obj_match_size();
  obj_cluster_size_ = comp_config.obj_cluster_size();

  if (!common::SensorManager::Instance()->GetSensorInfo(
          comp_config.radar_name(), &radar_info_)) {
    AERROR << "Failed to get sensor info, sensor name: "
           << comp_config.radar_name();
    return false;
  }

  writer_ = node_->CreateWriter<SensorFrameMessage>(
      comp_config.output_channel_name());

  // Init algorithm plugin
  ACHECK(InitAlgorithmPlugin()) << "Failed to init algorithm plugin.";
  return true;
}

bool RadarDetectionComponent::Proc(const std::shared_ptr<ContiRadar>& message) {
  double timestamp = apollo::common::time::Clock::NowInSeconds();
  static double leftCornerTime = 0.0;
  static double leftRadarTime = 0.0;
  static double rightCornerTime = 0.0;
  static double rightRadarTime = 0.0;

  if (message->radar_id() == 1 ||
      message->header().module_name() == "radar_left") {
    messageCollection(message, &leftObs_);
    leftRadarTime = timestamp;
  } else if (message->radar_id() == 2 ||
             message->header().module_name() == "radar_left_corner") {
    messageCollection(message, &leftCornerObs_);
    leftCornerTime = timestamp;
  } else if (message->radar_id() == -1 ||
             message->header().module_name() == "radar_right") {
    messageCollection(message, &rightObs_);
    rightRadarTime = timestamp;
  } else if (message->radar_id() == -2 ||
             message->header().module_name() == "radar_right_corner") {
    messageCollection(message, &rightCornerObs_);
    rightCornerTime = timestamp;
  } else if (message->radar_id() == 0 ||
             message->header().module_name() == "radar_front") {
    fflush(NULL);
    // // fuse object list
    if (std::fabs(timestamp - leftRadarTime) < 1.0) {
      radarObjectFuse(leftObs_, *message);
    }
    if (std::fabs(timestamp - leftCornerTime) < 1.0) {
      radarObjectFuse(leftCornerObs_, *message);
    }
    if (std::fabs(timestamp - rightRadarTime) < 1.0) {
      radarObjectFuse(rightObs_, *message);
    }
    if (std::fabs(timestamp - rightCornerTime) < 1.0) {
      radarObjectFuse(rightCornerObs_, *message);
    }
    FuseMainObj(*message);
    // //middle_radar_message

    std::shared_ptr<SensorFrameMessage> out_message(new (std::nothrow)
                                                        SensorFrameMessage);
    if (!InternalProc(message, out_message)) {
      return false;
    }
    writer_->Write(out_message);

    AINFO << "Send radar processing output message.";
  }
  return true;
}

bool RadarDetectionComponent::InitAlgorithmPlugin() {
  AINFO << "onboard radar_preprocessor: " << preprocessor_method_;

  radar::BasePreprocessor* preprocessor =
      radar::BasePreprocessorRegisterer::GetInstanceByName(
          preprocessor_method_);
  CHECK_NOTNULL(preprocessor);
  radar_preprocessor_.reset(preprocessor);
  ACHECK(radar_preprocessor_->Init()) << "Failed to init radar preprocessor.";
  radar::BaseRadarObstaclePerception* radar_perception =
      radar::BaseRadarObstaclePerceptionRegisterer::GetInstanceByName(
          perception_method_);
  ACHECK(radar_perception != nullptr)
      << "No radar obstacle perception named: " << perception_method_;
  radar_perception_.reset(radar_perception);
  ACHECK(radar_perception_->Init(pipeline_name_))
      << "Failed to init radar perception.";
  AINFO << "Init algorithm plugin successfully.";
  return true;
}

bool RadarDetectionComponent::InternalProc(
    const std::shared_ptr<ContiRadar>& in_message,
    std::shared_ptr<SensorFrameMessage> out_message) {
  PERCEPTION_PERF_FUNCTION_WITH_INDICATOR(radar_info_.name);
  ContiRadar raw_obstacles = *in_message;
  {
    std::unique_lock<std::mutex> lock(_mutex);
    ++seq_num_;
  }

  double timestamp = in_message->header().timestamp_sec();
  const double cur_time = apollo::common::time::Clock::NowInSeconds();
  const double start_latency = (cur_time - timestamp) * 1e3;
  AINFO << "FRAME_STATISTICS:Radar:Start:msg_time[" << timestamp
        << "]:cur_time[" << cur_time << "]:cur_latency[" << start_latency
        << "]";
  PERCEPTION_PERF_BLOCK_START();
  // Init preprocessor_options
  radar::PreprocessorOptions preprocessor_options;
  ContiRadar corrected_obstacles;
  radar_preprocessor_->Preprocess(raw_obstacles, preprocessor_options,
                                  &corrected_obstacles);
  PERCEPTION_PERF_BLOCK_END_WITH_INDICATOR(radar_info_.name,
                                           "radar_preprocessor");
  timestamp = corrected_obstacles.header().timestamp_sec();

  out_message->timestamp_ = timestamp;
  out_message->seq_num_ = seq_num_;
  out_message->process_stage_ = ProcessStage::LONG_RANGE_RADAR_DETECTION;
  out_message->sensor_id_ = radar_info_.name;

  // Init radar perception options
  radar::RadarPerceptionOptions options;
  options.sensor_name = radar_info_.name;

  std::vector<base::ObjectPtr> radar_objects;
  if (!radar_perception_->Perceive(corrected_obstacles, options,
                                    &radar_objects)) {
      out_message->error_code_ =
        apollo::common::ErrorCode::PERCEPTION_ERROR_PROCESS;
    AERROR << "RadarDetector Proc failed.";
    return true;
  }
  out_message->frame_.reset(new base::Frame());
  out_message->frame_->sensor_info = radar_info_;
  out_message->frame_->timestamp = timestamp;
 // out_message->frame_->sensor2world_pose = radar_trans;
  out_message->frame_->objects = radar_objects;

  const double end_timestamp = apollo::common::time::Clock::NowInSeconds();
  const double end_latency =
      (end_timestamp - in_message->header().timestamp_sec()) * 1e3;
  PERCEPTION_PERF_BLOCK_END_WITH_INDICATOR(radar_info_.name,
                                           "radar_perception");
  AINFO << "FRAME_STATISTICS:Radar:End:msg_time["
        << in_message->header().timestamp_sec() << "]:cur_time["
        << end_timestamp << "]:cur_latency[" << end_latency << "]";

  return true;
}



void RadarDetectionComponent::radarObjectFuse(
    const std::vector<drivers::ContiRadarObs>& Obsvector,
    drivers::ContiRadar& tmp_message) {
  // multi radar object list fuse
  uint32_t sub_size = Obsvector.size();
  std::vector<int> sub_match;
  sub_match.resize(sub_size);

  std::vector<std::vector<float>> tmp_error;
  tmp_error.resize(tmp_message.contiobs_size());
  int k = 0;
  for (auto& main_obs : tmp_message.contiobs()) {
    for (uint32_t i = 0; i < sub_size; i++) {
      // if (sub_match[i] == 1) {
      //   continue;
      // }
      drivers::ContiRadarObs sub_obs = Obsvector[i];
      float px_m = main_obs.pos_x();
      float py_m = main_obs.pos_y();
      float px_s = sub_obs.pos_x();
      float py_s = sub_obs.pos_y();
      float d_error =
          sqrt((px_m - px_s) * (px_m - px_s) + (py_m - py_s) * (py_m - py_s));
      if (d_error < obj_match_size_) {
        sub_match[i] = 1;
        tmp_message.mutable_contiobs(k)->set_flags(true);
      }
      tmp_error[k].push_back(d_error);
    }
    k++;
  }

  for (uint32_t i = 0; i < sub_size; i++) {
    if (sub_match[i] == 0) {
      drivers::ContiRadarObs* obs = tmp_message.add_contiobs();
      drivers::ContiRadarObs sub_obs = Obsvector[i];
      obs->set_range(sub_obs.range());
      obs->set_azimuth(sub_obs.azimuth());
      obs->set_elevation(sub_obs.elevation());
      obs->set_doppler(sub_obs.doppler());
      obs->set_magnitude(sub_obs.magnitude());
      obs->set_snr(sub_obs.snr());
      obs->set_uhn_rcs(sub_obs.rcs());
      obs->set_pos_x(sub_obs.pos_x());
      obs->set_pos_y(sub_obs.pos_y());
      obs->set_pos_z(sub_obs.pos_z());
      obs->set_flags(false);
    }
  }
}

void RadarDetectionComponent::messageCollection(
    const std::shared_ptr<ContiRadar>& sub_message,
    std::vector<drivers::ContiRadarObs>* Obsvector) {
  // collect radar detection object list 
  Obsvector->clear();
  for (int i = 0; i < sub_message->contiobs().size(); i++) {
    drivers::ContiRadarObs obs;
    const drivers::ContiRadarObs sub_obs = sub_message->contiobs(i);
    obs.set_range(sub_obs.range());
    obs.set_azimuth(sub_obs.azimuth());
    obs.set_elevation(sub_obs.elevation());
    obs.set_doppler(sub_obs.doppler());
    obs.set_magnitude(sub_obs.magnitude());
    obs.set_snr(sub_obs.snr());
    obs.set_uhn_rcs(sub_obs.rcs());
    obs.set_pos_x(sub_obs.pos_x());
    obs.set_pos_y(sub_obs.pos_y());
    obs.set_pos_z(sub_obs.pos_z());
    obs.set_flags(false);
    Obsvector->push_back(obs);
  }
}


void RadarDetectionComponent::FuseMainObj(drivers::ContiRadar& message) {
  uint32_t obj_size = message.contiobs_size();
  std::vector<drivers::ContiRadarObs> tmp_obs;
  std::vector<std::vector<int>> tmp_match;
  tmp_match.resize(obj_size);
  for (uint32_t i = 0; i < obj_size; i++) {
    const drivers::ContiRadarObs obs_i = message.contiobs(i);
    tmp_obs.push_back(obs_i);
  }

  message.clear_contiobs();
  for (uint32_t i = 0; i < obj_size; i++) {
    if (tmp_obs[i].flags() ||
        (tmp_obs[i].pos_x() < 2.0 && tmp_obs[i].pos_x() > 0.5 &&
         std::fabs(tmp_obs[i].pos_y()) < 1.5)) {
      drivers::ContiRadarObs* obs = message.add_contiobs();
      drivers::ContiRadarObs sub_obs = tmp_obs[i];
      obs->set_range(sub_obs.range());
      obs->set_azimuth(sub_obs.azimuth());
      obs->set_elevation(sub_obs.elevation());
      obs->set_doppler(sub_obs.doppler());
      obs->set_magnitude(sub_obs.magnitude());
      obs->set_snr(sub_obs.snr());
      obs->set_uhn_rcs(sub_obs.rcs());
      obs->set_pos_x(sub_obs.pos_x());
      obs->set_pos_y(sub_obs.pos_y());
      obs->set_pos_z(sub_obs.pos_z());
      obs->set_longitude_vel(sub_obs.longitude_vel());
      obs->set_lateral_vel(sub_obs.lateral_vel());
      obs->set_longitude_accel(sub_obs.longitude_accel());
      obs->set_lateral_accel(sub_obs.lateral_accel());
      obs->set_flags(sub_obs.flags());
    }
  }

  // printf("before fuse obj size:   %d   ", obj_size);
  // printf("after fuse obj size:   %d\n", message.contiobs_size());
}

}  // namespace onboard
}  // namespace perception
}  // namespace apollo
