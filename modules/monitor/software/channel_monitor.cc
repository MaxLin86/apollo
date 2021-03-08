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

#include "modules/monitor/software/channel_monitor.h"

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "absl/strings/str_cat.h"
#include "absl/strings/str_split.h"
#include "google/protobuf/compiler/parser.h"
#include "google/protobuf/descriptor.h"
#include "google/protobuf/dynamic_message.h"

#include "cyber/common/log.h"
#include "cyber/cyber.h"
#include "modules/common/adapters/adapter_gflags.h"
#include "modules/common/latency_recorder/proto/latency_record.pb.h"
#include "modules/common/util/map_util.h"
#include "modules/control/proto/control_cmd.pb.h"
#include "modules/drivers/proto/conti_radar.pb.h"
#include "modules/drivers/proto/pointcloud.pb.h"
#include "modules/drivers/proto/sensor_image.pb.h"
#include "modules/localization/proto/pose.pb.h"
#include "modules/map/relative_map/proto/navigation.pb.h"
#include "modules/monitor/common/monitor_manager.h"
#include "modules/monitor/software/summary_monitor.h"
#include "modules/perception/proto/perception_obstacle.pb.h"
#include "modules/planning/proto/planning.pb.h"
#include "modules/prediction/proto/prediction_obstacle.pb.h"


DEFINE_string(channel_monitor_name, "ChannelMonitor",
              "Name of the channel monitor.");

DEFINE_double(channel_monitor_interval, 5,
              "Channel monitor checking interval in seconds.");

namespace apollo {
namespace monitor {
namespace {

// We have to specify exact type of each channel. This function is a wrapper for
// those only need a ReaderBase.
std::pair<std::shared_ptr<cyber::ReaderBase>,
          std::shared_ptr<google::protobuf::Message>>
GetReaderAndLatestMessage(const std::string& channel) {
  auto manager = MonitorManager::Instance();
  if(channel == FLAGS_hmi_status_topic){
    
    const auto reader = manager->CreateReader<dreamview::HMIStatus>(channel);
    reader->Observe();
    const auto message = reader->GetLatestObserved();
    return std::pair<std::shared_ptr<cyber::ReaderBase>,
                     std::shared_ptr<google::protobuf::Message>>(reader,
                                                                 message);
  }
  if(channel == FLAGS_image_front_topic 
   || channel == FLAGS_image_short_topic
   || channel == FLAGS_image_long_topic
   || channel == FLAGS_camera_front_6mm_compressed_topic){
    
    const auto reader = manager->CreateReader<apollo::drivers::Image>(channel);
    // const auto reader = manager->CreateReader<dreamview::HMIStatus>(channel);
    reader->Observe();
    const auto message = reader->GetLatestObserved();
    return std::pair<std::shared_ptr<cyber::ReaderBase>,
                     std::shared_ptr<google::protobuf::Message>>(reader,
                                                                 message);
  }
  if (channel == FLAGS_control_command_topic) {
    const auto reader = manager->CreateReader<control::ControlCommand>(channel);
    reader->Observe();
    const auto message = reader->GetLatestObserved();
    return std::pair<std::shared_ptr<cyber::ReaderBase>,
                     std::shared_ptr<google::protobuf::Message>>(reader,
                                                                 message);
  } else if (channel == FLAGS_localization_topic) {
    const auto reader =
        manager->CreateReader<localization::LocalizationEstimate>(channel);
    reader->Observe();
    const auto message = reader->GetLatestObserved();
    return std::pair<std::shared_ptr<cyber::ReaderBase>,
                     std::shared_ptr<google::protobuf::Message>>(reader,
                                                                 message);
  } else if (channel == FLAGS_perception_obstacle_topic) {
    const auto reader =
        manager->CreateReader<perception::PerceptionObstacles>(channel);
    reader->Observe();
    const auto message = reader->GetLatestObserved();
    return std::pair<std::shared_ptr<cyber::ReaderBase>,
                     std::shared_ptr<google::protobuf::Message>>(reader,
                                                                 message);
  } else if (channel == FLAGS_prediction_topic) {
    const auto reader =
        manager->CreateReader<prediction::PredictionObstacles>(channel);
    reader->Observe();
    const auto message = reader->GetLatestObserved();
    return std::pair<std::shared_ptr<cyber::ReaderBase>,
                     std::shared_ptr<google::protobuf::Message>>(reader,
                                                                 message);
  } else if (channel == FLAGS_planning_trajectory_topic) {
    const auto reader = manager->CreateReader<planning::ADCTrajectory>(channel);
    reader->Observe();
    const auto message = reader->GetLatestObserved();
    return std::pair<std::shared_ptr<cyber::ReaderBase>,
                     std::shared_ptr<google::protobuf::Message>>(reader,
                                                                 message);
  } else if (channel == FLAGS_conti_radar_topic) {
    const auto reader = manager->CreateReader<drivers::ContiRadar>(channel);
    reader->Observe();
    const auto message = reader->GetLatestObserved();
    return std::pair<std::shared_ptr<cyber::ReaderBase>,
                     std::shared_ptr<google::protobuf::Message>>(reader,
                                                                 message);
  } else if (channel == FLAGS_relative_map_topic) {
    const auto reader = manager->CreateReader<relative_map::MapMsg>(channel);
    reader->Observe();
    const auto message = reader->GetLatestObserved();
    return std::pair<std::shared_ptr<cyber::ReaderBase>,
                     std::shared_ptr<google::protobuf::Message>>(reader,
                                                                 message);
  } else if (channel == FLAGS_pointcloud_topic ||
             channel == FLAGS_pointcloud_128_topic ||
             channel == FLAGS_pointcloud_16_front_up_topic) {
    const auto reader = manager->CreateReader<drivers::PointCloud>(channel);
    reader->Observe();
    const auto message = reader->GetLatestObserved();
    return std::pair<std::shared_ptr<cyber::ReaderBase>,
                     std::shared_ptr<google::protobuf::Message>>(reader,
                                                                 message);
  }
  // Add more channels here if you want to monitor.
  AERROR << "Channel is not handled by ChannelMonitor: " << channel;
  return std::pair<std::shared_ptr<cyber::ReaderBase>,
                   std::shared_ptr<google::protobuf::Message>>(nullptr,
                                                               nullptr);
}

bool ValidateFields(const google::protobuf::Message& message,
                    const std::vector<std::string>& fields,
                    const size_t field_step) {
  if (field_step >= fields.size()) {
    return true;
  }
  const auto* desc = message.GetDescriptor();
  const auto* refl = message.GetReflection();
  const auto field_count = desc->field_count();
  for (int field_idx = 0; field_idx < field_count; ++field_idx) {
    const auto* field_desc = desc->field(field_idx);
    if (field_desc->name() == fields[field_step]) {
      if (field_desc->is_repeated()) {
        // For repeated field, we do not expect it has deeper level validation
        const auto size = refl->FieldSize(message, field_desc);
        return size > 0 && field_step == fields.size() - 1;
      }
      if (field_desc->type() !=
          google::protobuf::FieldDescriptor::TYPE_MESSAGE) {
        return refl->HasField(message, field_desc) &&
               field_step == fields.size() - 1;
      }
      return ValidateFields(refl->GetMessage(message, field_desc), fields,
                            field_step + 1);
    }
  }
  return false;
}

}  // namespace

ChannelMonitor::ChannelMonitor(
    const std::shared_ptr<LatencyMonitor>& latency_monitor)
    : RecurrentRunner(FLAGS_channel_monitor_name,
                      FLAGS_channel_monitor_interval),
      latency_monitor_(latency_monitor) {
  AERROR << "bkx-ChannelMonitor::ChannelMonitor";
  init();
}

void ChannelMonitor::init(){

  AERROR << "bkx-ChannelMonitor::init";
    //device
    channel_fl_.emplace(FLAGS_image_front_topic, fault_into_t{FaultLevel::TWO,FaultLevel::TWO});//6mm camera

    //apollo moudule
    channel_fl_.emplace(FLAGS_control_command_topic, fault_into_t{FaultLevel::TWO,FaultLevel::TWO});
    channel_fl_.emplace(FLAGS_localization_topic, fault_into_t{FaultLevel::TWO,FaultLevel::TWO});
    channel_fl_.emplace(FLAGS_perception_obstacle_topic, fault_into_t{FaultLevel::TWO,FaultLevel::TWO});
    channel_fl_.emplace(FLAGS_prediction_topic, fault_into_t{FaultLevel::TWO,FaultLevel::TWO});
    channel_fl_.emplace(FLAGS_planning_trajectory_topic, fault_into_t{FaultLevel::TWO,FaultLevel::TWO});
    channel_fl_.emplace(FLAGS_conti_radar_topic, fault_into_t{FaultLevel::TWO,FaultLevel::TWO});
    channel_fl_.emplace(FLAGS_relative_map_topic, fault_into_t{FaultLevel::TWO,FaultLevel::TWO});

  // else  if (channel == FLAGS_control_command_topic) {
  //   return FaultLevel::ONE;
  // } else if (channel == FLAGS_localization_topic) {
  //   return FaultLevel::ONE;
  // } else if (channel == FLAGS_perception_obstacle_topic) {
  //   return FaultLevel::ONE;
  // } else if (channel == FLAGS_prediction_topic) {
  //   return FaultLevel::ONE;
  // } else if (channel == FLAGS_planning_trajectory_topic) {
  //   return FaultLevel::ONE;
  // } else if (channel == FLAGS_conti_radar_topic) {
  //   return FaultLevel::ONE;
  // } else if (channel == FLAGS_relative_map_topic) {
  //   return FaultLevel::ONE;
  // } else if (channel == FLAGS_pointcloud_topic ||
  //            channel == FLAGS_pointcloud_128_topic ||
  //            channel == FLAGS_pointcloud_16_front_up_topic) {
  //   return FaultLevel::ONE;
  // }  
}

void ChannelMonitor::RunOnce(const double current_time) {
  AERROR<<"bkx-ChannelMonitor::RunOnce";
  // CheckChannelStatus(); 
  auto manager = MonitorManager::Instance();
  //***********************************************************************************/
  // const auto reader = manager->CreateReader<apollo::common::ErrorInformation>("apollo/error");
  // // const auto reader = manager->CreateReader<apollo::common::StatusPb>("apollo/error");
  // if (nullptr == reader) {
  //   AERROR << "bkx-ChannelMonitor::RunOnce-reader";
  // }

  //   reader->Observe(); 
  // std::shared_ptr<apollo::common::ErrorInformation> message = reader->GetLatestObserved();
  // // const auto message = reader->GetLatestObserved();
  // if (nullptr == message)
  // {
  //   AERROR << "bkx-ChannelMonitor::RunOnce-message";
  //   /* code */
  // }else{
  //   // AERROR << "bkx-ChannelMonitor::RunOnce-message:succes";
    
  //   AERROR << "read error_info:seq " << message->seq()
  //                << ";error_code=" << message->error_code()
  //                << ";error_msg=" << message->error_msg()
  //                << ";level=" << message->error_level();
  //   // AERROR << "read error_code = " << message->error_code()
  //   //              << ";error_msg=" << message->msg()
  //   //              << ";level=" << message->error_level();
  // }
  
  //***********************************************************************************/
  const auto& mode = manager->GetHMIMode();
  auto* components = manager->GetStatus()->mutable_components();
  for (const auto& iter : mode.monitored_components()) { 
    const std::string& name = iter.first;
    const auto& config = iter.second;
    if (config.has_channel()) {
    AERROR<<"ChannelMonitor::RunOncehas_channel:"<<name;
      double freq;
      const auto update_freq =
          latency_monitor_->GetFrequency(config.channel().name(), &freq);
      UpdateStatus(config.channel(),
                   components->at(name).mutable_channel_status(), update_freq,
                   freq);
    }
  }
}

// ******************************************************************************************
////预留后续有根据频道信息得出故障
fault_into_t ChannelMonitor::GetFaultInfo(std::shared_ptr<google::protobuf::Message> manager,const std::string& channel)
{  
    return  {FaultLevel::ONE,FaultLevel::ONE};
}
fault_into_t ChannelMonitor::GetFaultInfo(const std::string& channel)
{  
  const auto it = channel_fl_.find(channel);
  if(channel_fl_.end() == it){ 
    return {FaultLevel::ZERO,FaultLevel::ZERO};
  }
  return it->second; 
    // return {FaultLevel::ZERO,FaultLevel::ZERO};
  
}

int ChannelMonitor::GetFaultCode(const std::string& channel)
{    
  if (channel == FLAGS_control_command_topic) {
    return 33;
  } else if (channel == FLAGS_localization_topic) {
    return 33;
  } else if (channel == FLAGS_perception_obstacle_topic) {
    return 33;
  } else if (channel == FLAGS_prediction_topic) {
    return 33;
  } else if (channel == FLAGS_planning_trajectory_topic) {
    return 33;
  } else if (channel == FLAGS_conti_radar_topic) {
    return 33;
  } else if (channel == FLAGS_relative_map_topic) {
    return 33;
  } else if (channel == FLAGS_pointcloud_topic ||
             channel == FLAGS_pointcloud_128_topic ||
             channel == FLAGS_pointcloud_16_front_up_topic) {
    return 33;
  } else{
    return 0;
  }
}

void ChannelMonitor::CheckChannelStatus()
{
  std::string channel_name=FLAGS_planning_trajectory_topic;
  // const auto reader_message_pair = GetReaderAndLatestMessage(FLAGS_hmi_status_topic);
  const auto reader_message_pair = GetReaderAndLatestMessage(channel_name);
  
  const auto reader = reader_message_pair.first;
  const auto message = reader_message_pair.second;

  // static auto chassis_reader = CreateReader<Chassis>(FLAGS_chassis_topic);
  // AERROR << "bkxline:"<<__LINE__<<";chassis_reader";
  // chassis_reader->Observe();
  // const auto chassis = chassis_reader->GetLatestObserved();
  // if (chassis == nullptr) {
  //   return false;
  // }

  if (reader == nullptr) {
    AERROR<<"reader nullptr";
        //send error fault level to vcu 
    return;
  }
  // const auto message = reader->GetLatestObserved();
  if (message == nullptr) { 
    // monitor::common::monitor_can mc(0);
    // mc.init();
    AERROR<<"message nullptr";
    fault_into_t fi = GetFaultInfo(message,channel_name);//0:no fault ;1 or 2 or 3:it's fault
    if( fi.ADU != FaultLevel::ZERO){
          //send error fault level to vcu  
    }
    monitor::common::monitor_can::mc()->SendFault(GetFaultInfo(channel_name).VCU);
    AERROR << GetFaultCode(channel_name);//no need send to vcu,just log now .maybe send to tmc.
    return ;
  }
  // AERROR<<"channel="<< FLAGS_hmi_status_topic<<" success get massge:";

}

// ******************************************************************************************
void ChannelMonitor::UpdateStatus(
    const apollo::dreamview::ChannelMonitorConfig& config,
    ComponentStatus* status, const bool update_freq, const double freq) {
  status->clear_status();

  const auto reader_message_pair = GetReaderAndLatestMessage(config.name());
  const auto reader = reader_message_pair.first;
  const auto message = reader_message_pair.second;

  if (reader == nullptr) {
    SummaryMonitor::EscalateStatus(
        ComponentStatus::UNKNOWN,
        absl::StrCat(config.name(), " is not registered in ChannelMonitor."),
        status);
    return;
  }

  // Check channel delay
  const double delay = reader->GetDelaySec();
  if (delay < 0 || delay > config.delay_fatal()) {
    SummaryMonitor::EscalateStatus(
        ComponentStatus::FATAL,ErrorCode::MONITOR_CHANNEL_FATAL, 
        absl::StrCat(config.name(), " delayed for ", delay, " seconds."),
        status);
  }

  // Check channel fields
  const std::string field_sepr = ".";
  if (message != nullptr) {
    for (const auto& field : config.mandatory_fields()) {
      if (!ValidateFields(*message, absl::StrSplit(field, field_sepr), 0)) {
        SummaryMonitor::EscalateStatus(
            ComponentStatus::ERROR, ErrorCode::MONITOR_CHANNEL_FATAL,
            absl::StrCat(config.name(), " missing field ", field), status);
      }
    }
  }

  // Check channel frequency
  if (update_freq) {
    if (freq > config.max_frequency_allowed()) {
      SummaryMonitor::EscalateStatus(
          ComponentStatus::WARN, ErrorCode::MONITOR_CHANNEL_WARN,
          absl::StrCat(config.name(), " has frequency ", freq,
                       " > max allowed ", config.max_frequency_allowed()),
          status);
    }
    if (freq < config.min_frequency_allowed()) {
      SummaryMonitor::EscalateStatus(
          ComponentStatus::WARN, ErrorCode::MONITOR_CHANNEL_WARN,
          absl::StrCat(config.name(), " has frequency ", freq,
                       " < min allowed ", config.max_frequency_allowed()),
          status);
    }
  }

  SummaryMonitor::EscalateStatus(ComponentStatus::OK, "", status);
}

}  // namespace monitor
}  // namespace apollo
