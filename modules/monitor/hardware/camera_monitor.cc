/*
 * @Author: your name
 * @Date: 2020-04-21 14:19:21
 * @LastEditTime: 2020-05-07 11:48:26
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: /apollo-5.5.0/modules/monitor/hardware/camera_monitor.cc
 */
 

#include "modules/monitor/hardware/camera_monitor.h"

#include "cyber/common/log.h"
#include "modules/common/adapters/adapter_gflags.h"
#include "modules/common/util/map_util.h"
// #include "modules/drivers/gnss/proto/gnss_status.pb.h"
// #include "modules/drivers/gnss/proto/ins.pb.h"
#include "modules/monitor/common/monitor_manager.h"
#include "modules/monitor/software/summary_monitor.h"

DEFINE_string(camera_monitor_name, "CameraMonitor", "Name of the camera monitor.");
DEFINE_double(camera_monitor_interval, 3, "camera status checking interval (s).");
DEFINE_string(camera_component_name, "camera", "camera component name.");

namespace apollo {
namespace monitor {

using apollo::drivers::gnss::GnssStatus;
using apollo::drivers::gnss::InsStatus;

CameraMonitor::CameraMonitor()
    : RecurrentRunner(FLAGS_camera_monitor_name, FLAGS_camera_monitor_interval) {}

void CameraMonitor::RunOnce(const double current_time) {
  auto manager = MonitorManager::Instance();
  Component* component = apollo::common::util::FindOrNull(
      *manager->GetStatus()->mutable_components(), FLAGS_camera_component_name);
  if (component == nullptr) {
    // camera is not monitored in current mode, skip.
    return;
  }
  ComponentStatus* component_status = component->mutable_other_status();
  component_status->clear_status();

  // Check Gnss status.
  static auto gnss_status_reader =
      manager->CreateReader<GnssStatus>(FLAGS_gnss_status_topic);
  gnss_status_reader->Observe();
  const auto gnss_status = gnss_status_reader->GetLatestObserved();
  if (gnss_status == nullptr) {
    SummaryMonitor::EscalateStatus(ComponentStatus::ERROR,ErrorCode::MONITOR_OTHER_ERROR,
                                   "No GNSS status message", component_status);
    return;
  }
  if (!gnss_status->solution_completed()) {
    SummaryMonitor::EscalateStatus(
        ComponentStatus::WARN,ErrorCode::MONITOR_OTHER_WARN, "GNSS solution uncompleted", component_status);
    return;
  }

  // Check Ins status.
  static auto ins_status_reader =
      manager->CreateReader<InsStatus>(FLAGS_ins_status_topic);
  ins_status_reader->Observe();
  const auto ins_status = ins_status_reader->GetLatestObserved();
  if (ins_status == nullptr) {
    SummaryMonitor::EscalateStatus(ComponentStatus::ERROR,ErrorCode::MONITOR_OTHER_ERROR,
                                   "No INS status message", component_status);
    return;
  }
  switch (ins_status->type()) {
    case InsStatus::CONVERGING:
      SummaryMonitor::EscalateStatus(
          ComponentStatus::WARN, ErrorCode::MONITOR_OTHER_WARN,
          "INS not ready, converging", component_status);
      break;
    case InsStatus::GOOD:
      SummaryMonitor::EscalateStatus(ComponentStatus::OK, "", component_status);
      break;
    case InsStatus::INVALID:
      SummaryMonitor::EscalateStatus(ComponentStatus::ERROR,
                                     ErrorCode::MONITOR_OTHER_ERROR,
                                     "INS status invalid", component_status);
      break;
    default:
      SummaryMonitor::EscalateStatus(ComponentStatus::ERROR,
                                     ErrorCode::MONITOR_OTHER_ERROR,
                                     "INS status unknown", component_status);
      break;
  }
}

}  // namespace monitor
}  // namespace apollo
