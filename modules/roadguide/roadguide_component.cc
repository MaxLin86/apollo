#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <stdbool.h>
#include <string.h>
#include <cstdint>
#include <sys/types.h>

#include "modules/roadguide/roadguide_component.h"
#include "modules/roadguide/common/roadguide_gflags.h"
#include "modules/common/adapters/adapter_gflags.h"
#include "modules/common/util/message_util.h"
#include "modules/roadguide/proto/roadguide_conf.pb.h"
#include "modules/common/time/time.h"
#include "modules/roadguide/rxmodule.h"

namespace apollo {
namespace roadguide {

using apollo::common::time::Clock;

bool RoadguideComponent::CreateTransfer(const RoadguideConf &conf)
{
  if (!conf.serial_port().has_device()) {
    AERROR << "serial_port def has no device field.";
    return false;
  }

  if (!conf.serial_port().has_baud_rate()) {
    AERROR << "serial_port def has no baud_rate field. Use default baud rate "
           << conf.serial_port().baud_rate();
    return false;
  }

  transfer_ptr_.reset(new Transfer(conf.serial_port().device().c_str(), conf.serial_port().baud_rate()));

  return transfer_ptr_->CreateStream();
}

void RoadguideComponent::PollingRadar()
{
  while (true) {
    rxmodule_ptr_->Proc();
  }
}

void RoadguideComponent::StartPollingRadar()
{
  data_thread_ptr_.reset(new std::thread(&RoadguideComponent::PollingRadar, this));
}

bool RoadguideComponent::Init()
{
  if (!GetProtoConfig(&roadguide_conf_)) {
    AERROR << "Unable to load roadguide conf file: " << ConfigFilePath();
    return false;
  }

  AINFO << "The roadguide conf file is loaded: " << FLAGS_roadguide_conf_file;
  ADEBUG << "roadguide_conf:" << roadguide_conf_.ShortDebugString();

  roadguide_writer_ = node_->CreateWriter<RoadguideMsg>(FLAGS_roadguide_topic);
  
  if (!CreateTransfer(roadguide_conf_)) {
    AERROR << "Create transfer failed";
    return false;
  }

  roadguide_msg_ = std::make_shared<RoadguideMsg>();

  rxmodule_ptr_.reset(new Rxmodule(roadguide_msg_, transfer_ptr_));

  StartPollingRadar();
  return true;
}

void RoadguideComponent::PublishMsg()
{
  std::lock_guard<std::mutex> lock(roadguide_mutex_);
  common::util::FillHeader(node_->Name(), roadguide_msg_.get());
  rxmodule_ptr_->UpdateMeasMsg();
  roadguide_writer_->Write(roadguide_msg_);
}

bool RoadguideComponent::Proc()
{
  PublishMsg();
  return true;
}

}  // namespace roadguide
}  // namespace apollo
