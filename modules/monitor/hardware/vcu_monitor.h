/*
 * @Author: your name
 * @Date: 2020-04-20 12:08:15
 * @LastEditTime: 2020-04-20 13:12:39
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: /apollo-5.5.0/modules/monitor/hardware/vcu_monitor.h
 */
#pragma once

#include "modules/monitor/common/recurrent_runner.h"

namespace apollo {
namespace monitor {

class VcuMonitor : public RecurrentRunner {
 public:
  VcuMonitor();
  void RunOnce(const double current_time) override;
};

}  // namespace monitor
}  // namespace apollo