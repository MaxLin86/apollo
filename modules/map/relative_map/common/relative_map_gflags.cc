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

#include "modules/map/relative_map/common/relative_map_gflags.h"

DEFINE_string(
    relative_map_config_filename,
    "/apollo/modules/map/relative_map/conf/relative_map_config.pb.txt",
    "Relative map configuration file");

DEFINE_string(navigator_config_filename,
              "/apollo/modules/map/relative_map/conf/navigator_config.pb.txt",
              "navigator config file name.");

DEFINE_int32(relative_map_loop_rate, 10, "Loop rate for relative_map node");

DEFINE_bool(enable_cyclic_rerouting, false,
            "Enable auto rerouting in a in a cyclic/circular navigaton line.");

DEFINE_bool(relative_map_generate_left_boundray, true,
            "Generate left boundary for detected lanes.");

//add by shzhw, define extract variables
DEFINE_bool(positive_order, true, "Drive on the positive order of guidepost mark.");

DEFINE_bool(use_virtual_lane_marker_from_flie, false,
            "Load virtual lane marker from file.");

DEFINE_string(virtual_lane_marker_fliename, 
    "/apollo/modules/map/relative_map/testdata/multi_lane_map/0_perception_obstacles.pb.txt", 
    "virtual lane marker file.");

DEFINE_int32(virtual_lane_marker_running_cycs, 100,
    "Setting the virtual lane marker running cycles to avoid keep on running.");

DEFINE_bool(tmc_navigation_points, true,
    "Receive tmc publish navigation gps points.");

DEFINE_double(path_smooth_param_alpha, 0.02,
"path smoothing parameter alpha");

DEFINE_double(path_smooth_param_beta, 0.4,
"path smoothing parameter alpha");
//--------------------------------------------------------