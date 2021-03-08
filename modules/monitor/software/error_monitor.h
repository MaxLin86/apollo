/*
 * @Author: your name
 * @Date: 2020-04-20 12:08:15
 * @LastEditTime: 2020-05-05 12:07:56
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: /apollo-5.5.0/modules/monitor/hardware/Error_monitor.h
 */
#pragma once

#include "modules/monitor/common/recurrent_runner.h"
#include "modules/common/proto/error_code.pb.h"
#include <unordered_map>
#include <thread>

namespace apollo {
namespace monitor {

using apollo::common::ErrorCode;
using apollo::common::StatusPb;
using apollo::common::ErrorInformation;
using apollo::common::ErrorLevel;
class ErrorMonitor : public RecurrentRunner {

 public:
  ErrorMonitor();
  void RunOnce(const double current_time) override;
private:
  void InitCodeLevel();
//   void WriteErrorInformation(const ErrorInformation& err_info);
  void WriteErrorInformation(const ErrorCode& err_code,
                             const std::string& err_msg,
                             const ErrorLevel& err_level);
  void checkHearbeat(const double current_time);
  //   bool checkErrorLever(const ErrorCode& err_level);
  std::unordered_map<uint32_t, ErrorLevel> code_level_;  // key:ErrorCode
  std::unordered_map<uint32_t, double> heartbeat_; //key:module heartbeat ErrorCode
//  std::unordered_map<ErrorCode, ErrorLevel> code_level_;
};

}  // namespace monitor
}  // namespace apollo