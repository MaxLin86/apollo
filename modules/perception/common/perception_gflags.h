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

#pragma once

#include "gflags/gflags.h"

namespace apollo {
namespace perception {

// sensor_manager
DECLARE_string(obs_sensor_intrinsic_path);
DECLARE_string(obs_sensor_meta_path);

DECLARE_bool(enable_base_object_pool);
DECLARE_bool(fusion_debug_display);
DECLARE_bool(lane_debug_display);
DECLARE_bool(guide_post_debug_display);
DECLARE_string(guide_post_start_id);
DECLARE_string(guide_post_end_id);
DECLARE_bool(guide_post_direction);
// config_manager
DECLARE_string(config_manager_path);
DECLARE_string(work_root);

}  // namespace perception
}  // namespace apollo
