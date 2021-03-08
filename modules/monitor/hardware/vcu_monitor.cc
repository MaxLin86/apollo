/*
 * @Author: your name
 * @Date: 2020-04-20 12:08:02
 * @LastEditTime: 2020-04-24 22:16:45
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: /apollo-5.5.0/modules/monitor/hardware/vcu_monitor.cc
 */ 

#include "modules/monitor/hardware/vcu_monitor.h"

// #include "cyber/common/log.h"
// #include "modules/common/adapters/adapter_gflags.h"
// #include "modules/common/util/map_util.h"
// #include "modules/drivers/gnss/proto/gnss_status.pb.h"
// #include "modules/drivers/gnss/proto/ins.pb.h"
#include "modules/monitor/common/monitor_manager.h"
#include "modules/monitor/software/summary_monitor.h"
// #include "modules/canbus/canbus_api.h"

DEFINE_string(vcu_monitor_name, "VcuMonitor", "Name of the VCU monitor.");
DEFINE_double(vcu_monitor_interval, 1, "VCU status checking interval (s).");
DEFINE_string(vcu_component_name, "VCU", "VCU component name.");

namespace apollo {
namespace monitor {

// using apollo::drivers::gnss::GnssStatus;
// using apollo::drivers::gnss::InsStatus;

VcuMonitor::VcuMonitor()
    : RecurrentRunner(FLAGS_vcu_monitor_name, FLAGS_vcu_monitor_interval) {}

void VcuMonitor::RunOnce(const double current_time) {
  // AERROR << "bkx:VcuMonitor::RunOnce";
  // apollo::canbus::canbus_api can_api;
  // int vcu_faultcode = can_api.get_vcu_faultcode_req();
  // if (3 == vcu_faultcode || 2 == vcu_faultcode) {
  //   can_api.hurry_full_brake_req();
  //   int err_code = can_api.get_vcu_dtc_req();
  //   AERROR << "get vcu fault code=" << err_code;
  //   // can_api.send_ipc_dtc_req(3, err_code);
  // }else if(1 == vcu_faultcode ){ 
  //   // can_api.hurry_full_brake_req();
  //   int err_code = can_api.get_vcu_dtc_req();
  //   AERROR << "get vcu fault code=" << err_code;
  //   // can_api.send_ipc_dtc_req(3, err_code);
  // }
  // auto manager = MonitorManager::Instance();
  // Component* component = apollo::common::util::FindOrNull(
  //     *manager->GetStatus()->mutable_components(), FLAGS_gps_component_name);
  // if (component == nullptr) {
  //   // GPS is not monitored in current mode, skip.
  //   return;
  // }
  // ComponentStatus* component_status = component->mutable_other_status();
  // component_status->clear_status();

  // // Check Gnss status.
  // static auto gnss_status_reader =
  //     manager->CreateReader<GnssStatus>(FLAGS_gnss_status_topic);
  // gnss_status_reader->Observe();
  // const auto gnss_status = gnss_status_reader->GetLatestObserved();
  // if (gnss_status == nullptr) {
  //   SummaryMonitor::EscalateStatus(ComponentStatus::ERROR,
  //                                  "No GNSS status message", component_status);
  //   return;
  // }
  // if (!gnss_status->solution_completed()) {
  //   SummaryMonitor::EscalateStatus(
  //       ComponentStatus::WARN, "GNSS solution uncompleted", component_status);
  //   return;
  // }

  // // Check Ins status.
  // static auto ins_status_reader =
  //     manager->CreateReader<InsStatus>(FLAGS_ins_status_topic);
  // ins_status_reader->Observe();
  // const auto ins_status = ins_status_reader->GetLatestObserved();
  // if (ins_status == nullptr) {
  //   SummaryMonitor::EscalateStatus(ComponentStatus::ERROR,
  //                                  "No INS status message", component_status);
  //   return;
  // }
  // switch (ins_status->type()) {
  //   case InsStatus::CONVERGING:
  //     SummaryMonitor::EscalateStatus(
  //         ComponentStatus::WARN, "INS not ready, converging", component_status);
  //     break;
  //   case InsStatus::GOOD:
  //     SummaryMonitor::EscalateStatus(ComponentStatus::OK, "", component_status);
  //     break;
  //   case InsStatus::INVALID:
  //     SummaryMonitor::EscalateStatus(ComponentStatus::ERROR,
  //                                    "INS status invalid", component_status);
  //     break;
  //   default:
  //     SummaryMonitor::EscalateStatus(ComponentStatus::ERROR,
  //                                    "INS status unknown", component_status);
  //     break;
  // }
}

}  // namespace monitor
}  // namespace apollo
