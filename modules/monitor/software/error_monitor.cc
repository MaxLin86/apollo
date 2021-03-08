
#include "modules/monitor/software/error_monitor.h" 
// #include "cyber/common/log.h"
// #include "modules/common/adapters/adapter_gflags.h"
// #include "modules/common/util/map_util.h"
// #include "modules/drivers/gnss/proto/gnss_status.pb.h"
// #include "modules/drivers/gnss/proto/ins.pb.h"
#include "modules/monitor/common/monitor_manager.h"
#include "modules/monitor/software/summary_monitor.h"
// #include "modules/canbus/canbus_api.h"

DEFINE_string(Error_monitor_name, "ErrorMonitor", "Name of the Error monitor.");
DEFINE_double(Error_monitor_interval, 0.3, "Error status checking interval (s).");
DEFINE_double(heartbeat_interval, 1, "check module hearbeat interval(s)");
// DEFINE_string(Error_component_name, "Error", "Error component name.");

namespace apollo {
namespace monitor {
 

ErrorMonitor::ErrorMonitor()
    : RecurrentRunner(FLAGS_Error_monitor_name, FLAGS_Error_monitor_interval) {
  InitCodeLevel();
    // AERROR << "bkx-ErrorMonitor::ErrorMonitor";
}

void ErrorMonitor::InitCodeLevel(){
  code_level_.insert({{ErrorCode::OK, ErrorLevel::LEVEL1},
                      {ErrorCode::CONTROL_INIT_ERROR, ErrorLevel::LEVEL1}});

  const double current_time = apollo::common::time::Clock::NowInSeconds();
  heartbeat_.insert({ErrorCode::CONTROL_OK,current_time});
  //   code_level_.insert({ErrorCode::OK, ErrorLevel::LEVEL1});
  //  static std::once_flag flag;
  //   std::call_once(flag, [&] {
  //     // code_level_.emplace({ErrorCode::OK, ErrorLevel::LEVEL1});

  //   });
}

void ErrorMonitor::checkHearbeat(const double current_time) { 
    for(const auto hb : heartbeat_){
        if(current_time - hb.second  > FLAGS_heartbeat_interval){
            // ErrorInformation err_info;
            // err_info.set_error_code((ErrorCode)hb.first);
            // err_info.set_error_msg("over heartbeat interval");
            // // err_info.set_error_msg(message->msg());
            // err_info.set_error_level(ErrorLevel::LEVEL3);
            // WriteErrorInformation(message->error_code(),message->msg(),itcl->second);
            WriteErrorInformation((ErrorCode)hb.first,"over heartbeat interval",ErrorLevel::LEVEL3);
        }
    } 
}
 
void ErrorMonitor::RunOnce(const double current_time) {
    AERROR << "bkx:ErrorMonitor::RunOnce";

    auto manager = MonitorManager::Instance();
    const auto reader = manager->CreateReader<StatusPb>("apollo/error");
    // const auto reader = manager->CreateReader<apollo::common::StatusPb>("apollo/error");
    if (nullptr == reader) {
        AERROR << "bkx-ChannelMonitor::RunOnce-reader";
        return;
    }

    reader->Observe(); 
    std::shared_ptr<StatusPb> message = reader->GetLatestObserved(); 
    if (nullptr == message)
    {
        AERROR << "bkx-ChannelMonitor::RunOnce-message";
        return; 
    } 
    auto it = heartbeat_.find(message->error_code());
        if (heartbeat_.end() != it){ 
        it->second = current_time;
    }
    else{
        auto itcl = code_level_.find(message->error_code());
        if (code_level_.end() == itcl) {
            AERROR << "read not exist error code:" << message->error_code();
            return;
        }
        if(itcl->second > ErrorLevel::LEVEL0){ 
            WriteErrorInformation(message->error_code(),message->msg(),itcl->second);
            // ErrorInformation err_info;
            // err_info.set_error_code(message->error_code());
            // err_info.set_error_msg(message->msg());
            // err_info.set_error_level(itcl->second);
            // WriteErrorInformation(err_info);
        } 
    }
   
  checkHearbeat(current_time);
  return; 
}

void ErrorMonitor::WriteErrorInformation(const ErrorCode& err_code,const std::string & err_msg,const ErrorLevel& err_level) { 
  AERROR << "happen error"
         << ";error_code:" <<err_code
         << ";error_msg:" << err_msg
         << ";error_level:" << err_level;
  auto manager = MonitorManager::Instance();
  static auto writer = manager->CreateWriter<ErrorInformation>(
      "apollo/monitor/ErrorInformation");

  // apollo::common::util::FillHeader("SystemMonitor", status);
  ErrorInformation err_info;
  err_info.set_error_code(err_code);
  err_info.set_error_msg(err_msg);
  err_info.set_error_level(err_level);
  writer->Write(err_info);
}

// void WriteErrorInformation(const ErrorInformation& err_info) {

//   auto manager = MonitorManager::Instance();
//   static auto writer = manager->CreateWriter<ErrorInformation>(
//       "apollo/monitor/ErrorInformation");

//   // apollo::common::util::FillHeader("SystemMonitor", status);
//   writer->Write(err_info);
//   AERROR << "happen error"
//          << ";error_code:" << err_info.error_code()
//          << ";error_msg:" << err_info.error_msg()
//          << ";error_level:" << err_info.error_level();
// }
}  // namespace monitor
}  // namespace apollo
