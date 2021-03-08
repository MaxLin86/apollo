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

#include "modules/map/relative_map/relative_map.h"

#include "cyber/common/file.h"
#include "modules/common/math/vec2d.h"
#include "modules/common/util/util.h"
#include "modules/common/vehicle_state/vehicle_state_provider.h"
#include "modules/map/relative_map/common/relative_map_gflags.h"


#include "modules/localization/msf/common/util/frame_transform.h"
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
//#include "modules/planning/reference_line/spiral_reference_line_smoother.h"
//#include "modules/planning/reference_line/spiral_smoother_util.cc"

#define save_debug_info true

namespace apollo {
namespace relative_map {

using apollo::canbus::Chassis;
using apollo::common::ErrorCode;
using apollo::common::Status;
using apollo::common::VehicleStateProvider;
using apollo::common::monitor::MonitorMessageItem;
using apollo::localization::LocalizationEstimate;
using apollo::perception::PerceptionObstacles;


//add by shzhw
  template <class Type>
  Type stringToNum(const std::string& str)
  {
  std::istringstream iss(str);
  Type num;
  iss >> num;
  return num;
  }
  //删除字符串中空格，制表符tab等无效字符
  std::string Trim(std::string& str)
  {
    //str.find_first_not_of(" \t\r\n"),在字符串str中从索引0开始，返回首次不匹配"\t\r\n"的位置
    str.erase(0,str.find_first_not_of(" \t\r\n"));
    str.erase(str.find_last_not_of(" \t\r\n") + 1);
    return str;
  }



// 求拐角theta
// Arguments    : const double P0[2]
//                const double P1[2]
//                const double P2[2]
//                double r
//                double t
//                double Pt1[2]
//                double Pt2[2]
//                double *d1
//                double *d2
//                double C[2]
//                double *theta
// Return Type  : void
//
void PathSegmentTrans2D(const double P0[2], const double P1[2], const double P2
  [2], double r, double Pt1[2], double Pt2[2], double *d1, double *d2,
  double C[2], double *theta)
{
  double a_tmp;
  double b_a_tmp;
  double P1P0;
  double c_a_tmp;
  double d_a_tmp;
  double P1P2;
  double e_a_tmp;
  double a;
  a_tmp = P0[0] - P1[0];
  b_a_tmp = P0[1] - P1[1];
  P1P0 = std::sqrt(a_tmp * a_tmp + b_a_tmp * b_a_tmp);
  c_a_tmp = P2[0] - P1[0];
  d_a_tmp = P2[1] - P1[1];
  P1P2 = std::sqrt(c_a_tmp * c_a_tmp + d_a_tmp * d_a_tmp);
  *theta = std::acos((a_tmp * c_a_tmp + b_a_tmp * d_a_tmp) / (P1P0 * P1P2));

  //  求转接点Pt1、Pt2
  e_a_tmp = r / std::tan(*theta / 2.0);
  a = e_a_tmp / P1P0;
  e_a_tmp /= P1P2;

  //  求路径长度d1、弧长d2
  P1P2 = P1[0] + a * a_tmp;
  P1P0 = P1[0] + e_a_tmp * c_a_tmp;
  C[0] = P1P2 + 0.5 * (P1P0 - P1P2);
  Pt1[0] = P1P2;
  Pt2[0] = P1P0;
  P1P2 = P1[1] + a * b_a_tmp;
  P1P0 = P1[1] + e_a_tmp * d_a_tmp;
  C[1] = P1P2 + 0.5 * (P1P0 - P1P2);
  Pt1[1] = P1P2;
  Pt2[1] = P1P0;
  a = Pt1[0] - P0[0];
  e_a_tmp = P1P2 - P0[1];
  *d1 = std::sqrt(a * a + e_a_tmp * e_a_tmp);
  *d2 = (3.1415926535897931 - *theta) * r;

  //  % 求转接速度vt
  //  这是考虑机械系统动力学匀速因素得到的转接速度
  //  a = sqrt(Amax * r);
  //  b = d2 / t;
  //  if (a > b)
  //      vt  = b;
  //  else
  //      vt = a;
  //  end
  //  求圆心C
  a_tmp = C[0] - P1[0];
  a = C[1] - P1[1];
  a = r / std::sin(*theta / 2.0) / std::sqrt(a_tmp * a_tmp + a * a);
  C[0] = P1[0] + a * a_tmp;
  C[1] = P1[1] + a * (C[1] - P1[1]);
}


cv::Mat polyfit(std::vector<cv::Point2d>& in_point, int n)
{
	int size = in_point.size();
	//所求未知数个数
	int x_num = n + 1;
	//构造矩阵U和Y
	cv::Mat mat_u(size, x_num, CV_64F);
	cv::Mat mat_y(size, 1, CV_64F);
 
	for (int i = 0; i < mat_u.rows; ++i)
		for (int j = 0; j < mat_u.cols; ++j)
		{
			mat_u.at<double>(i, j) = pow(in_point[i].x, j);
		}
 
	for (int i = 0; i < mat_y.rows; ++i)
	{
		mat_y.at<double>(i, 0) = in_point[i].y;
	}
 
	//矩阵运算，获得系数矩阵K
	cv::Mat mat_k(x_num, 1, CV_64F);
	mat_k = (mat_u.t()*mat_u).inv()*mat_u.t()*mat_y;
	std::cout << mat_k << std::endl;
	return mat_k;
}

void split(const std::string& s, std::vector<std::string>& sv, const char flag = ' ') {
    sv.clear();
    std::istringstream iss(s);
    std::string temp;
    while (std::getline(iss, temp, flag)) {
        sv.push_back(temp);
    }
    return;
}
bool RelativeMap::LoadMap() {
  //to get guidepost world coordinate
  std::string mapPath = "/apollo/modules/map/data/demo/signpost_shzhw.txt";
  std::ifstream mapFile;
  mapFile.open(mapPath, std::ios::in);
  if (!mapFile.is_open()) {
    AERROR << "load guidepsot map file failed";
    return false;
  }
  std::string strLine;
  while (getline(mapFile, strLine))
  {
    if (strLine.empty())
      continue;
    std::vector<std::string> sv;
    split(strLine, sv, ',');
    if (sv.size() == 3)
    {
        GuidepostGroup guidepostGroup;
        guidepostGroup.id = sv[0];
        guidepostGroup.x = atof(sv[1].c_str());
        guidepostGroup.y = atof(sv[2].c_str());
        guidepost_group_vector_.push_back(guidepostGroup);
    }
  }
  mapFile.close();

  if ( guidepost_group_vector_.size() > 0) {
  } else
  {
    AERROR << "guidepos group num: 0";
  }
  
  AINFO << "guidepos group num: " <<  guidepost_group_vector_.size();
 
  return true;
}



RelativeMap::RelativeMap()
    : monitor_logger_buffer_(MonitorMessageItem::RELATIVE_MAP) {}

Status RelativeMap::Init() {
  if (!FLAGS_use_navigation_mode) {
    AERROR << "FLAGS_use_navigation_mode is false, system is not configured "
              "for relative map mode";
    return Status(ErrorCode::RELATIVE_MAP_ERROR,
                  "FLAGS_use_navigation_mode is not true.");
  }
  config_.Clear();
  if (!cyber::common::GetProtoFromFile(FLAGS_relative_map_config_filename,
                                       &config_)) {
    return Status(ErrorCode::RELATIVE_MAP_ERROR,
                  "Unable to load relative map conf file: " +
                      FLAGS_relative_map_config_filename);
  }

  navigation_lane_.SetConfig(config_.navigation_lane());
  const auto& map_param = config_.map_param();
  navigation_lane_.SetDefaultWidth(map_param.default_left_width(),
                                   map_param.default_right_width());


    LoadMap();
    reset_act_task_ = false;
    tcs_navigator_status_ = false;

  return Status::OK();
}

void LogErrorStatus(MapMsg* map_msg, const std::string& error_msg) {
  auto* status = map_msg->mutable_header()->mutable_status();
  status->set_msg(error_msg);
  status->set_error_code(ErrorCode::RELATIVE_MAP_ERROR);
}

apollo::common::Status RelativeMap::Start() {
  monitor_logger_buffer_.INFO("RelativeMap started");
  return Status::OK();
}

bool RelativeMap::Process(MapMsg* const map_msg) {
  {
    std::lock_guard<std::mutex> lock(navigation_lane_mutex_);
    CreateMapFromNavigationLane(map_msg);
  }
  return true;
}

void RelativeMap::OnNavigationInfo(const NavigationInfo& navigation_info) {
  {
    std::lock_guard<std::mutex> lock(navigation_lane_mutex_);
    //if (FLAGS_tmc_navigation_points) {
    //  return;
    //}//add by shzhw 0917
    navigation_lane_.UpdateNavigationInfo(navigation_info);
  }
}

void RelativeMap::OnTmcNavigationInfo(const remoteManage::actTask& act_task) {
  {
    std::lock_guard<std::mutex> lock(navigation_lane_mutex_);
    //if (FLAGS_tmc_navigation_points) {
    //  return;
    //}//add by shzhw 0917
    //navigation_lane_.UpdateTmcNavigationInfo(act_task);    
  act_task_.CopyFrom(act_task);

  static double act_task_timestamp = 0.0;
  if (act_task_.has_header()) {
      if (((act_task_timestamp == 0.0) && (act_task_.task_coord_size()> 0)) || 
            (act_task_timestamp != act_task_.header().timestamp_sec())) {
              act_task_timestamp = act_task_.header().timestamp_sec();
              reset_act_task_ = true;
      } else {
        reset_act_task_ = false;
      }
  }
        
  //tmc_gps_points_;


  }
}

void RelativeMap::OnPerception(
    const PerceptionObstacles& perception_obstacles) {
  {
    std::lock_guard<std::mutex> lock(navigation_lane_mutex_);
    perception_obstacles_.CopyFrom(perception_obstacles);
  }
}

void RelativeMap::OnChassis(const Chassis& chassis) {
  {
    std::lock_guard<std::mutex> lock(navigation_lane_mutex_);
    chassis_.CopyFrom(chassis);
  }
}

void RelativeMap::OnLocalization(const LocalizationEstimate& localization) {
  {
    std::lock_guard<std::mutex> lock(navigation_lane_mutex_);
    localization_.CopyFrom(localization);
  }
}

bool RelativeMap::CreateMapFromNavigationLane(MapMsg* map_msg) {
  CHECK_NOTNULL(map_msg);

  // update vehicle state from localization and chassis
  
  if(!localization_.has_pose() || !localization_.pose().has_heading()) {return false;}
  //if(!chassis_.has_speed_mps()) {return false;} // to avoid ctrl variable appears nan

  LocalizationEstimate const& localization = localization_;
  Chassis const& chassis = chassis_;
  VehicleStateProvider::Instance()->Update(localization, chassis);
  map_msg->mutable_localization()->CopyFrom(localization_);

  //------add 20200708-----------------------------
  static int nIndex = 0;
  //nIndex++;
  AERROR <<"nIndex: " << nIndex;
  //if ((nIndex == 1 /*||nIndex==100*/)  && FLAGS_tmc_navigation_points) {
   
  if ( FLAGS_tmc_navigation_points && (act_task_.task_coord_size()> 0)) {

    AERROR << "points size: " << act_task_.task_coord_size();
    nIndex++;
    if (nIndex == 1) {
    AERROR << "nIndex: " << nIndex;
  
    //if ( FLAGS_tmc_navigation_points && reset_act_task_) { 
      
  // generate navigation from tmc
      std::vector<cv::Point2d>  tmc_gps_points;

    /* */
    #if 0
      //----read gps points from csv file----
      cv::Point2d tmc_gps_point;
      int row = 0;
      std::ifstream fin("/apollo/data/new_csv_090914_test4.csv"); //打开文件流操作
      std::string line; 
      while (getline(fin, line))   //整行读取，换行符“\n”区分，遇到文件尾标志eof终止读取
      {
        row ++;
        if (row <=1) {continue;}
        //std::cout <<"原始字符串："<< line << std::endl; //整行输出      
        std::istringstream sin(line); //将整行字符串line读入到字符串流istringstream中
        std::vector<std::string> fields; //声明一个字符串向量
        std::string field;
        while (getline(sin, field, ',')) //将字符串流sin中的字符读入到field字符串中，以逗号为分隔符
        {
          fields.push_back(field); //将刚刚读取的字符串添加到向量fields中
        }
        std::string seq_number = Trim(fields[0]); //清除掉向量fields中第一个元素的无效字符，并赋值给变量seq
        std::string str_lat =Trim(fields[5]); //清除掉向量fields中第二个元素的无效字符，并赋值给变量lat
        std::string str_lon =Trim(fields[6]); //清除掉向量fields中第三个元素的无效字符，并赋值给变量lon
        //std::cout <<"处理之后的字符串："<< seq_number << "\t" << lat << "\t" << lon << std::endl; 
        double lat = stringToNum<double>(str_lat);
        double lon = stringToNum<double>(str_lon);
        tmc_gps_point.x = lat;
        tmc_gps_point.y = lon;
        tmc_gps_points.push_back(tmc_gps_point);
      }
      /////////////////////////////////////
    
    #else
      
      /**/
      cv::Point2d tmc_gps_point;
      
      for (int n = 0; n < act_task_.task_coord_size();n++) {
        tmc_gps_point.x = act_task_.task_coord(n).y();
        tmc_gps_point.y = act_task_.task_coord(n).x();
        AERROR << "X:" <<tmc_gps_point.x << ", Y: " << tmc_gps_point.y;
        tmc_gps_points.push_back(tmc_gps_point);
      }

    #endif 

      AERROR << "size:" << tmc_gps_points.size();

      bool staus = GenerateNavigatorPath(tmc_gps_points);
      if (staus) {
        tcs_navigator_status_ = true;
      } else {
        tcs_navigator_status_ = false;
        AERROR << "generate navigator path from tmc gps points failed.";
      }

    }

  }//add by shzhw 0917

  // process fused input dta


   // to generate lane marker using guidepost
   const auto guidepost_localization_valid = localization_.localization_source();
   static uint cur_guidepost_group_mark = 0;
   
  if (guidepost_localization_valid == "guidepost" ||
       guidepost_localization_valid == "gnss and guidepost") {           
   
    const double selfcar_px = localization_.pose().position().x();
    const double selfcar_py = localization_.pose().position().y();
    const double selfcar_theta = localization_.pose().heading();
    {
      std::ofstream ekf_out;
      ekf_out.open("map_ekf_result_out.txt", std::ios::out | std::ios::app);
      ekf_out << std::fixed << chassis.header().timestamp_sec() << "\t" << selfcar_px << "\t" << selfcar_py <<  "\t"<< selfcar_theta  <<  std::endl;
      //ekf_out<< perception_guidepost_target_px << "\t" << perception_guidepost_target_py<< std::endl;
      ekf_out.close();
    }

    if (localization_.has_current_perception_guidepost_index()) {
      cur_guidepost_group_mark = localization_.current_perception_guidepost_index();
      double offset =(guidepost_group_vector_[cur_guidepost_group_mark].x -selfcar_px)
                                       * (guidepost_group_vector_[cur_guidepost_group_mark].x - selfcar_px) + 
                                      (guidepost_group_vector_[cur_guidepost_group_mark].y - selfcar_py) 
                                        * (guidepost_group_vector_[cur_guidepost_group_mark].y - selfcar_py);

      if (offset > 10.0) {
        LogErrorStatus(map_msg, "no matched the approprite guidepost mark!");  
        //TODO
      }

      std::ofstream ekf_out_2;
      ekf_out_2.open("curve_line.txt", std::ios::out | std::ios::app);
      ekf_out_2 << std::fixed << chassis.header().timestamp_sec() << "\t" ;//////////////////

      std::vector<cv::Point2d> in_path_points;
      cv::Point2d in_point = cv::Point2d(0.0, 0.0);
      for (int j = 0; j < 10; j++) {
        uint next_guidepost_group_mark = 0;
        if (FLAGS_positive_order) {
          next_guidepost_group_mark = (cur_guidepost_group_mark - 1 + j + guidepost_group_vector_.size()) %
                                                                              (guidepost_group_vector_.size());
        } else {
          next_guidepost_group_mark = (cur_guidepost_group_mark + 1 - j + guidepost_group_vector_.size()) % 
                                                                              (guidepost_group_vector_.size());
        }
        const double next_world_guidepost_px = guidepost_group_vector_[next_guidepost_group_mark].x;
        const double next_world_guidepost_py = guidepost_group_vector_[next_guidepost_group_mark].y;       
        in_point.x = (next_world_guidepost_px - selfcar_px)*(cos(selfcar_theta)) + (next_world_guidepost_py - selfcar_py)*(sin(selfcar_theta));
        in_point.y = (next_world_guidepost_px - selfcar_px)*(-sin(selfcar_theta)) + (next_world_guidepost_py - selfcar_py)*(cos(selfcar_theta));
        in_path_points.push_back(in_point); 
        ekf_out_2 <<  in_point.x << "\t" <<  in_point.y << "\t";///////////////////// 
      }
      
      cv::Mat mat_k(4, 1, CV_64F);
      mat_k = polyfit(in_path_points, 3);

      perception::LaneMarker guidepost_marker;
      guidepost_marker.set_c0_position(mat_k.at<double>(0,0));
      guidepost_marker.set_c1_heading_angle(mat_k.at<double>(1,0));
      guidepost_marker.set_c2_curvature(mat_k.at<double>(2,0));
      guidepost_marker.set_c3_curvature_derivative(mat_k.at<double>(3,0));
      guidepost_marker.set_quality(1.0);
      guidepost_marker.set_view_range(18.0);
      map_msg->mutable_guidepost_marker()->CopyFrom(guidepost_marker);

      AERROR << "coeff: " << mat_k << "\n";
      {
          ekf_out_2 << mat_k.at<double>(0,0) << "\t" << mat_k.at<double>(1,0) << "\t" << mat_k.at<double>(2,0) << "\t" << mat_k.at<double>(3,0) << "\t" 
            <<  std::endl;
        ekf_out_2.close();
      }
      navigation_lane_.UpdateGuidepostMarker(guidepost_marker);
    }
  }
 //------------------------------------------------

  //add by shzhw, load virtual lane marker from file
  static int virtual_lane_marker_cycs = 0;
  if (FLAGS_use_virtual_lane_marker_from_flie) {    
    PerceptionObstacles perception_obstacles_from_file;
    if (cyber::common::GetProtoFromFile(FLAGS_virtual_lane_marker_fliename,
            &perception_obstacles_from_file)){
      navigation_lane_.UpdatePerception(perception_obstacles_from_file);
      map_msg->mutable_lane_marker()->CopyFrom(perception_obstacles_from_file.lane_marker());
      perception_obstacles_.mutable_lane_marker()->CopyFrom(perception_obstacles_from_file.lane_marker());
    } else {
      AERROR << "Failed to load virtual lane marker from file:" + FLAGS_virtual_lane_marker_fliename;
    }
    // 
    /*to change code by shzhw 20200502
      static double pre_timestamp =
          VehicleStateProvider::Instance()->vehicle_state().timestamp(); 
      const double current_timestamp = 
          VehicleStateProvider::Instance()->vehicle_state().timestamp();
      const double current_speed =
          VehicleStateProvider::Instance()->vehicle_state().linear_velocity();
      static double driving_path_length = 0.0;
      double time_diff = current_timestamp - pre_timestamp;
      double driving_path_length += current_speed * time_diff;
      pre_timestamp = current_timestamp; 
      if(driving_path_length >= FLAGS_max_driving_path_length) {}     
    */
    if(virtual_lane_marker_cycs >= FLAGS_virtual_lane_marker_running_cycs) {
      return false; // to avoid keep on running
    }
    virtual_lane_marker_cycs ++;
  } else {
    // update navigation_lane from perception_obstacles (lane marker)
    PerceptionObstacles const& perception = perception_obstacles_;
    navigation_lane_.UpdatePerception(perception);
    map_msg->mutable_lane_marker()->CopyFrom(perception_obstacles_.lane_marker());
  // add by shzhw, to check lane marker missing
  /*
  // method 1, error, to do
   static auto pre_lane_marker = perception_obstacles_.mutable_lane_marker();
  if (perception_obstacles_.has_lane_marker() &&
           perception_obstacles_.mutable_lane_marker()->has_left_lane_marker()) {
    pre_lane_marker = perception_obstacles_.mutable_lane_marker();
  } else {
    perception_obstacles_.mutable_lane_marker()->CopyFrom(*pre_lane_marker);
  }
  */

  // method 2, to do
  static auto pre_lane_marker = perception_obstacles_.mutable_lane_marker();
  static double left_lane_marker_c0 = 0.0;
  static double left_lane_marker_c1 = 0.0;
  static double left_lane_marker_view_range = 0.0;
  static double right_lane_marker_c0 = 0.0;
  static double right_lane_marker_c1 = 0.0;
  static double right_lane_marker_view_range = 0.0;

  static int null_lane_marker_count = 0;
  if(perception_obstacles_.has_header() && perception_obstacles_.has_lane_marker() &&
           perception_obstacles_.lane_marker().has_left_lane_marker() &&
           perception_obstacles_.lane_marker().left_lane_marker().has_c0_position() &&
           perception_obstacles_.lane_marker().has_right_lane_marker()) {
    pre_lane_marker = perception_obstacles_.mutable_lane_marker();
    std::cout << "22222222222222222" << std::endl;
    // AERROR << pre_lane_marker->left_lane_marker().c0_position();
    left_lane_marker_c0 = pre_lane_marker->left_lane_marker().c0_position();
    left_lane_marker_c1 = pre_lane_marker->left_lane_marker().c1_heading_angle();
    left_lane_marker_view_range = pre_lane_marker->left_lane_marker().view_range();
    right_lane_marker_c0 = pre_lane_marker->right_lane_marker().c0_position();
    right_lane_marker_c1 = pre_lane_marker->right_lane_marker().c1_heading_angle();
    right_lane_marker_view_range = pre_lane_marker->right_lane_marker().view_range();
    std::cout << left_lane_marker_c0 << "\t" << left_lane_marker_c1 << std::endl;
    null_lane_marker_count = 0;
  } else {    
    perception_obstacles_.mutable_lane_marker()->mutable_left_lane_marker()->set_c0_position(left_lane_marker_c0);
    perception_obstacles_.mutable_lane_marker()->mutable_left_lane_marker()->set_c1_heading_angle(left_lane_marker_c1);
    perception_obstacles_.mutable_lane_marker()->mutable_left_lane_marker()->set_c2_curvature(0.0);
    perception_obstacles_.mutable_lane_marker()->mutable_left_lane_marker()->set_c3_curvature_derivative(0.0);
    perception_obstacles_.mutable_lane_marker()->mutable_left_lane_marker()->set_quality(0.5);
    perception_obstacles_.mutable_lane_marker()->mutable_left_lane_marker()->set_view_range(left_lane_marker_view_range);

    perception_obstacles_.mutable_lane_marker()->mutable_right_lane_marker()->set_c0_position(right_lane_marker_c0);
    perception_obstacles_.mutable_lane_marker()->mutable_right_lane_marker()->set_c1_heading_angle(right_lane_marker_c1);
    perception_obstacles_.mutable_lane_marker()->mutable_right_lane_marker()->set_c2_curvature(0.0);
    perception_obstacles_.mutable_lane_marker()->mutable_right_lane_marker()->set_c3_curvature_derivative(0.0);
    perception_obstacles_.mutable_lane_marker()->mutable_right_lane_marker()->set_quality(0.5);
    perception_obstacles_.mutable_lane_marker()->mutable_right_lane_marker()->set_view_range(right_lane_marker_view_range);  
   // perception_obstacles_.mutable_lane_marker()->CopyFrom(*pre_lane_marker); //no do
   null_lane_marker_count ++;
  }
  AERROR << "NULL LANE MARKER COUNT NUM: " << null_lane_marker_count;
  if (null_lane_marker_count > 6) {
    LogErrorStatus(map_msg, "no perception lane marker!"); 
    //return false;
  }
  //-----------------------------
    navigation_lane_.SetLaneMarkerMissCount(null_lane_marker_count);
    map_msg->mutable_lane_marker()->CopyFrom(perception_obstacles_.lane_marker());
  }
  //---------------------------------------------------------------------------------

  //20201020
  auto cur_theta = localization_.pose().heading();
  if ( (std::fabs(cur_theta- 1.9) < 0.2) || (std::fabs(cur_theta -(-1.2)) < 0.2)) {
    navigation_lane_.SetLaneMarkerValid(true);
  } else {
    navigation_lane_.SetLaneMarkerValid(false);
  }


  if (!navigation_lane_.GeneratePath()) {
    LogErrorStatus(map_msg, "Failed to generate a navigation path.");
    return false;
  }

  if (navigation_lane_.Path().path().path_point().empty()) {
    LogErrorStatus(map_msg,
                   "There is no path point in currnet navigation path.");
    return false;
  }

  // create map proto from navigation_path
  if (!navigation_lane_.CreateMap(config_.map_param(), map_msg)) {
    LogErrorStatus(map_msg,
                   "Failed to create map from current navigation path.");
    AERROR << "Failed to create map from navigation path.";
    return false;
  }

  ADEBUG << "There is/are " << map_msg->navigation_path().size()
         << " navigation path(s) in the current reltative map.";
  return true;
}

void RelativeMap::Stop() { monitor_logger_buffer_.INFO("RelativeMap stopped"); }

//20200911
/**/
bool RelativeMap::GenerateNavigatorPath(const std::vector<cv::Point2d>& gps_points) {
  std::vector<Eigen::Vector2d> raw_points;
  Eigen::Vector2d raw_point;
  std::vector<size_t> inflection_point_indexs;

      std::ofstream gps_in_data;
    gps_in_data.open("gps_in_data.txt",std::ios::app);

  for (size_t i =0; i < gps_points.size(); i++) {

    if (save_debug_info) {
    gps_in_data<<std::fixed << gps_points[i].x << "\t" << gps_points[i].y << "\n";//test-----------------
    }
    // gps to UTM
    localization::msf::UTMCoor utm_xy;
    double lat = gps_points[i].x;
    double lon = gps_points[i].y;
    localization::msf::FrameTransform::LatlonToUtmXY(lon/180.0*M_PI,
                                  lat/180.0*M_PI, &utm_xy);

    raw_point.x() = utm_xy.x;
    raw_point.y() = utm_xy.y;
    raw_points.push_back(raw_point);
    //
    //hybrid_test = std::unique_ptr<planning::HybridAStar>(
    //      new planning::HybridAStar(planner_open_space_config_));

    // search  inflection point    
    double abs_dtheta = 0.0;
    if (i > 0 && i < gps_points.size()-2) {
      double theta1 = atan2(lat -gps_points[i-1].x,  lon-gps_points[i-1].y);
      double theta2 = atan2(gps_points[i+1].x-lat,  gps_points[i+1].y-lon);
      abs_dtheta = std::fabs(theta2 - theta1);
    }
    if (abs_dtheta > 1.0 && abs_dtheta < 6.0) {
      inflection_point_indexs.push_back(i);
    }

  }


  AERROR << "SIZE:" << inflection_point_indexs.size();
    std::vector<size_t> inflection_point_indexs_t;
    inflection_point_indexs_t.insert(inflection_point_indexs_t.end(),inflection_point_indexs.begin(),inflection_point_indexs.end());
  for (size_t k=0;k < inflection_point_indexs_t.size()-1; k++) {
      AERROR << "inflection_point_indexs:" <<inflection_point_indexs[k];
    if (inflection_point_indexs_t[k+1] - inflection_point_indexs_t[k] < 3 ) {
      AERROR << "k:" << k;
     
      for(std::vector<size_t>::iterator iter=inflection_point_indexs.begin();iter!=inflection_point_indexs.end();iter++){        //从vector中删除指定的某一个元素 
        if(*iter== inflection_point_indexs_t[k] || *iter== inflection_point_indexs_t[k+1]){
          inflection_point_indexs.erase(iter);
        //break;
        }   
      }

    }
  }
  AERROR << "SIZE2:" << inflection_point_indexs.size();

  //process infletion path segment
  std::vector<Eigen::Vector2d> new_raw_points;

  static size_t start_index = 0;
  static size_t end_index = std::max(static_cast<size_t>(0), inflection_point_indexs[0]-static_cast<size_t>(10));
  new_raw_points.insert(new_raw_points.end(),raw_points.begin()+start_index,raw_points.begin()+end_index);

  for (size_t k=0;k < inflection_point_indexs.size(); k++) {
    double P0[2];
    double P1[2];
    double P2[2];

    double Pt1[2];
    double Pt2[2];
    double d1;
    double d2;
    double C[2];
    double theta;
    const double r = 6.0;

    size_t P0_index = std::max(static_cast<size_t>(0), inflection_point_indexs[k]-static_cast<size_t>(10));
    size_t P2_index = std::min(gps_points.size()-static_cast<size_t>(2), inflection_point_indexs[k]+static_cast<size_t>(10));

    P0[0] = raw_points[P0_index].x();P0[1] = raw_points[P0_index].y();
    P1[0] = raw_points[inflection_point_indexs[k]].x();P1[1] = raw_points[inflection_point_indexs[k]].y();
    P2[0] = raw_points[P2_index].x();P2[1] = raw_points[P2_index].y();

    PathSegmentTrans2D(P0, P1,  P2, r, Pt1, Pt2, &d1, &d2,
      C, &theta);

    const double delta_phi = 0.1;    
    const double phi1 = atan2(Pt1[1]-C[1], Pt1[0]-C[0]);
    const double phi2 = atan2(Pt2[1]-C[1], Pt2[0]-C[0]); 
    
    std::vector<Eigen::Vector2d> new_inflection_points;
    Eigen::Vector2d  new_inflection_point;
    const double dfai=M_PI - theta;
    for (int k = 0;k<std::floor(std::fabs(dfai)/delta_phi);k++) {
        double phi_t = phi1 + k*(phi2-phi1)/std::fabs(phi2-phi1)*delta_phi;
        if(abs(phi2-phi1)> M_PI) {
            phi_t = phi1- k*(phi2-phi1)/std::fabs(phi2-phi1)*delta_phi;
        }
       new_inflection_point.x() = C[0] + r*cos(phi_t);
       new_inflection_point.y() = C[1] + r*sin(phi_t);
       new_inflection_points.push_back(new_inflection_point);
    }

    new_raw_points.insert(new_raw_points.end(),new_inflection_points.begin(),new_inflection_points.end());
    
    if (k < inflection_point_indexs.size()-static_cast<size_t>(1)) {
    start_index = std::min(gps_points.size()-static_cast<size_t>(2), inflection_point_indexs[k]+static_cast<size_t>(10));
    end_index = std::max(static_cast<size_t>(0), inflection_point_indexs[k+1]-static_cast<size_t>(10)); 
    new_raw_points.insert(new_raw_points.end(),raw_points.begin()+start_index,raw_points.begin()+end_index);
  
    }    
  }
  start_index = std::min(gps_points.size()-static_cast<size_t>(2), inflection_point_indexs[inflection_point_indexs.size()-1]+static_cast<size_t>(10));
  new_raw_points.insert(new_raw_points.end(),raw_points.begin()+start_index,raw_points.end());

  std::vector<apollo::common::PathPoint> smooth_points;
  auto res = Smooth(new_raw_points, &smooth_points);//--------------------
    //auto res = Smooth(raw_points, &smooth_points);//--------------------
  if (!res) {
    AERROR << "Failed to smooth a the line";
    return false;
  } else {
    AERROR   << "smooth the line success";
    //NavigationInfo tmc_navigation_info;
    auto tmc_navigation_path = tmc_navigation_info.mutable_navigation_path()->Add();
    tmc_navigation_path->set_path_priority(0);
    tmc_navigation_path->mutable_path()->set_name("tmc_navigation");

    std::ofstream smooth_out_data;
    smooth_out_data.open("smooth_out_data.txt",std::ios::app);    
    

    for (size_t ii =0;ii < smooth_points.size();ii++) {
      auto point = tmc_navigation_path->mutable_path()->mutable_path_point()->Add();
      point->set_x(smooth_points[ii].x());
      point->set_y(smooth_points[ii].y());
      point->set_s(smooth_points[ii].s());
      point->set_theta(smooth_points[ii].theta());
      point->set_kappa(smooth_points[ii].kappa());
      point->set_dkappa(smooth_points[ii].dkappa());

      if (save_debug_info) {
      smooth_out_data <<std::fixed << smooth_points[ii].x() << "\t" << smooth_points[ii].y() << "\t" <<
        smooth_points[ii].s()<< "\n";
      }

    }
    //navigation_lane_.UpdateNavigationInfo(tmc_navigation_info);//tttttttt


    //3333333333333
    //std::unique_lock<std::mutex> lock(navigation_lane_mutex_); 
    tcs_navigation_msg_ = std::make_shared<NavigationInfo>(tmc_navigation_info);
    return true;
  }

}

 bool RelativeMap::Smooth(std::vector<Eigen::Vector2d> raw_points,
                     std::vector<common::PathPoint>* ptr_smooth_points) {
    const double minimum_point_spacing = 5.0;
    if (raw_points.size() <= 2) {
      AERROR << "the original point size is " << raw_points.size();
      return false;
    }

    std::vector<Eigen::Vector2d> processed_points;
    processed_points.push_back(raw_points.front());

    for (const Eigen::Vector2d& p : raw_points) {
      Eigen::Vector2d d = p - processed_points.back();
      if (d.norm() < minimum_point_spacing) {
        continue;
      }
      processed_points.push_back(p);
    }

    if (processed_points.size() < 2) {
      processed_points.push_back(raw_points.back());
    }

    Eigen::Vector2d start_point = processed_points.front();
    std::for_each(processed_points.begin(), processed_points.end(),
                  [&start_point](Eigen::Vector2d& p) { p = p - start_point; });

    planning::ReferenceLineSmootherConfig config;
    //CHECK(cyber::common::GetProtoFromFile(
      //  "modules/planning/conf/spiral_smoother_config.pb.txt", &config));
        cyber::common::GetProtoFromFile(
        "modules/planning/conf/spiral_smoother_config.pb.txt", &config);

    std::vector<double> opt_theta;
    std::vector<double> opt_kappa;
    std::vector<double> opt_dkappa;
    std::vector<double> opt_s;
    std::vector<double> opt_x;
    std::vector<double> opt_y;

    planning::SpiralReferenceLineSmoother spiral_smoother(config);
    auto res = spiral_smoother.SmoothStandAlone(processed_points, &opt_theta,
                                                &opt_kappa, &opt_dkappa, &opt_s,
                                                &opt_x, &opt_y);

    if (!res) {
      AWARN << "Optimization failed; the result may not be smooth";
    } else {
      AINFO << "Optimal solution found";
    }

      AERROR << "size()" <<processed_points.size() <<"," <<opt_theta.size() << "," <<
          opt_kappa.size() << "," << opt_dkappa.size() << "," << opt_s.size() << "," << 
          opt_x.size() << "," << opt_y.size();


    std::for_each(opt_x.begin(), opt_x.end(),
                  [&start_point](double& x) { x += start_point.x(); });
    std::for_each(opt_y.begin(), opt_y.end(),
                  [&start_point](double& y) { y += start_point.y(); });

    *ptr_smooth_points =
        spiral_smoother.Interpolate(opt_theta, opt_kappa, opt_dkappa, opt_s,
                                    opt_x, opt_y, config.resolution());

    return true;
  }
//---------------------------

}  // namespace relative_map
}  // namespace apollo
