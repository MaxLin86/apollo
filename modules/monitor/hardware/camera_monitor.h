/*
 * @Author: your name
 * @Date: 2020-04-21 14:19:21
 * @LastEditTime: 2020-04-21 14:22:42
 * @LastEditors: your name
 * @Description: In User Settings Edit
 * @FilePath: /apollo-5.5.0/modules/monitor/hardware/camera_monitor.h
 */
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
#pragma once

#include "modules/monitor/common/recurrent_runner.h"

namespace apollo {
namespace monitor {

class CameraMonitor : public RecurrentRunner {
 public:
  CameraMonitor();
  void RunOnce(const double current_time) override;
};

}  // namespace monitor
}  // namespace apollo
