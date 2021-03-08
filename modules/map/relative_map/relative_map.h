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

#include <string>

#include <vector>
#include <list>
#include <memory>
#include <utility>
#include <algorithm>

#include "modules/map/relative_map/proto/navigation.pb.h"
#include "modules/map/relative_map/proto/relative_map_config.pb.h"
#include "modules/perception/proto/perception_obstacle.pb.h"

#include "modules/remoteManage/proto/remoteManage.pb.h"

#include "modules/common/monitor_log/monitor_log_buffer.h"
#include "modules/common/status/status.h"
#include "modules/map/relative_map/navigation_lane.h"

#include "opencv2/opencv.hpp"
#include "modules/planning/open_space/coarse_trajectory_generator/hybrid_a_star.h"//20200911


namespace apollo {
namespace relative_map {

  //路标组构造函数（自定义坐标系下） add by shzhw
struct GuidepostGroup
{
public:
  std::string id;//路标组路标1 id
	double x;//路标组路标1 x坐标
  double y;//路标组路标1 y坐标
};

class RelativeMap {
 public:
  RelativeMap();

  /**
   * @brief module name
   */
  std::string Name() const { return "RelativeMap"; }

  /**
   * @brief module initialization function
   * @return initialization status
   */
  apollo::common::Status Init();

  /**
   * @brief module start function
   * @return start status
   */
  apollo::common::Status Start();

  /**
   * @brief module stop function
   */
  void Stop();

  /**
   * @brief destructor
   */
  virtual ~RelativeMap() = default;

  /**
   * @brief main logic of the relative_map module, runs periodically triggered
   * by timer.
   */
  bool Process(MapMsg* const map_msg);

  void OnPerception(
      const perception::PerceptionObstacles& perception_obstacles);
  void OnChassis(const canbus::Chassis& chassis);
  void OnLocalization(const localization::LocalizationEstimate& localization);
  void OnNavigationInfo(const NavigationInfo& navigation_info);
  void OnTmcNavigationInfo(const remoteManage::actTask& act_task) ;
  std::shared_ptr<NavigationInfo> GetTcsNavigation(){return tcs_navigation_msg_;}//33333333333
  bool GetTcsNavigationStatus(){return tcs_navigator_status_;}
  bool LoadMap();
 private:
  bool CreateMapFromNavigationLane(MapMsg* map_msg);
  RelativeMapConfig config_;
  apollo::common::monitor::MonitorLogBuffer monitor_logger_buffer_;

  NavigationLane navigation_lane_;
  perception::PerceptionObstacles perception_obstacles_;
  canbus::Chassis chassis_;
  localization::LocalizationEstimate localization_;
  std::vector<GuidepostGroup> guidepost_group_vector_; 
  std::mutex navigation_lane_mutex_;

  remoteManage::actTask act_task_;
  bool reset_act_task_;

  //-----------------//
  bool GenerateNavigatorPath(const std::vector<cv::Point2d>& gps_points);
  static bool Smooth(std::vector<Eigen::Vector2d> raw_points,
                     std::vector<common::PathPoint>* ptr_smooth_points);
  std::unique_ptr<planning::HybridAStar> hybrid_test;
  planning::PlannerOpenSpaceConfig planner_open_space_config_;//20200911
  std::shared_ptr<NavigationInfo> tcs_navigation_msg_;//333333333333
  NavigationInfo tmc_navigation_info;
  bool tcs_navigator_status_;
};

}  // namespace relative_map
}  // namespace apollo
