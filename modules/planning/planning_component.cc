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
#include "modules/planning/planning_component.h"

#include "cyber/common/file.h"
#include "modules/common/adapters/adapter_gflags.h"
#include "modules/common/configs/config_gflags.h"
#include "modules/common/util/message_util.h"
#include "modules/common/util/util.h"
#include "modules/map/hdmap/hdmap_util.h"
#include "modules/map/pnc_map/pnc_map.h"
#include "modules/planning/common/history.h"
#include "modules/planning/common/planning_context.h"
#include "modules/planning/navi_planning.h"
#include "modules/planning/on_lane_planning.h"

#define ekf_per_chassis 1 //add by shzhw 20200628

namespace apollo {
namespace planning {

using apollo::hdmap::HDMapUtil;
using apollo::perception::TrafficLightDetection;
using apollo::relative_map::MapMsg;
using apollo::routing::RoutingRequest;
using apollo::routing::RoutingResponse;
using apollo::common::math::Vec2d;


//add by shzhw
void split(const std::string& s, std::vector<std::string>& sv, const char flag = ' ') {
    sv.clear();
    std::istringstream iss(s);
    std::string temp;
    while (std::getline(iss, temp, flag)) {
        sv.push_back(temp);
    }
    return;
}
bool PlanningComponent::LoadMap() {
  //读取坐标地图文件
  std::string mapPath = "/apollo/modules/map/data/demo/signpost_shzhw.txt";
  std::ifstream mapFile;
  mapFile.open(mapPath, std::ios::in);
  if (!mapFile.is_open()) {
    AERROR << "load guidepsot map file failed";
    //is_guidepost_loaded = false;
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
        /*guidepostGroup.P2_id = sv[3];
        guidepostGroup.P2_x = atof(sv[4].c_str());
        guidepostGroup.P2_y = atof(sv[5].c_str());*/
        guidepostGroupVector.push_back(guidepostGroup);
    }
  }
  mapFile.close();

  if (guidepostGroupVector.size() > 0) {
    //is_guidepost_loaded = true;
  } else
  {
    AERROR << "guidepos group num: 0";
    //is_guidepost_loaded = false;
  }
  
  AINFO << "guidepos group num: " << guidepostGroupVector.size();
  std::cout << "guidepos group num: " << guidepostGroupVector.size() << std::endl;
  return true;
}


static double rt_atan2d_snf(double u0, double u1)
{
  double y;
  int b_u0;
  int b_u1;
  if (std::isnan(u0) || std::isnan(u1)) {
    y = std::nan("NAN");
  } else if (std::isinf(u0) && std::isinf(u1)) {
    if (u0 > 0.0) {
      b_u0 = 1;
    } else {
      b_u0 = -1;
    }

    if (u1 > 0.0) {
      b_u1 = 1;
    } else {
      b_u1 = -1;
    }

    y = atan2((double)b_u0, (double)b_u1);
  } else if (u1 == 0.0) {
    if (u0 > 0.0) {
      y = M_PI_2;
    } else if (u0 < 0.0) {
      y = -M_PI_2;
    } else {
      y = 0.0;
    }
  } else {
    y = atan2(u0, u1);
  }

  return y;
}

static double rt_hypotd_snf(double u0, double u1)
{
  double y;
  double a;
  double b;
  a = std::abs(u0);
  b = std::abs(u1);
  if (a < b) {
    a /= b;
    y = b * std::sqrt(a * a + 1.0);
  } else if (a > b) {
    b /= a;
    y = a * std::sqrt(b * b + 1.0);
  } else if (std::isnan(b)) {
    y = b;
  } else {
    y = a * 1.4142135623730951;
  }

  return y;
}
void rotate_shift(Eigen::Vector2d& coord1_p1, Eigen::Vector2d& coord1_p2, 
                  Eigen::Vector2d& coord2_P1, Eigen::Vector2d& coord2_P2,
                   Eigen::Matrix2d& R, Eigen::Vector2d& S, double *K) {
  double x_re_tmp;
  double x_im_tmp;
  double b_x_re_tmp;
  double b_x_im_tmp;
  double A;
  double R_tmp;
  const double x1 = coord1_p1(0);
  const double y1 = coord1_p1(1);
  const double x2 = coord1_p2(0);
  const double y2 = coord1_p2(1);

  const double X1 = coord2_P1(0);
  const double Y1 = coord2_P1(1);
  const double X2 = coord2_P2(0);
  const double Y2 = coord2_P2(1);
  x_re_tmp = X1 - X2;
  x_im_tmp = Y1 - Y2;
  b_x_re_tmp = x1 - x2;
  b_x_im_tmp = y1 - y2;
  A = rt_atan2d_snf(x_im_tmp, x_re_tmp) - rt_atan2d_snf(b_x_im_tmp, b_x_re_tmp);
  R_tmp = std::cos(A);
  R.setZero();
  R(0,0) = R_tmp;
  A = std::sin(A);
  R(1,0) = A;
  R(0,1) = -A;
  R(1,1) = R_tmp;
  *K = rt_hypotd_snf(x_re_tmp, x_im_tmp) / rt_hypotd_snf(b_x_re_tmp, b_x_im_tmp);
  S.setZero();
  S(0) = X1 - (R_tmp * x1 + -A * y1) * *K;
  S(1) = Y1 - (A * x1 + R_tmp * y1) * *K;
}
//-----------------------------------------------------------
//-----------multiple step turning------------------
bool CollideCheck(common::VehicleParam &vehicle_param, const std::vector<double>& XYbounds, 
                                std::vector<std::vector<common::math::LineSegment2d>> &obstacles_linesegments_vec, 
                                double vehilce_orient, cv::Point2d vehicle_center, double trailer_orient, cv::Point2d  trailer_center) {

  if (obstacles_linesegments_vec.empty()) {
    return true;
  }

  if (vehicle_center.x > XYbounds[1] || vehicle_center.x < XYbounds[0] ||
      vehicle_center.y > XYbounds[3] ||vehicle_center.y < XYbounds[2]) {
        return false;
  }
  if (trailer_center.x > XYbounds[1] || trailer_center.x < XYbounds[0] ||
      trailer_center.y > XYbounds[3] ||trailer_center.y < XYbounds[2]) {
        return false;
  }

  common::math::Box2d vehilce_bounding_box = Node3d::GetBoundingBox(
      vehicle_param,vehicle_center.x, vehicle_center.y, vehilce_orient);

  common::math::Box2d trailer_bounding_box = Node3d::GetTrailerBoundingBox(
      vehicle_param, trailer_center.x, trailer_center.y, trailer_orient);

  for (const auto& obstacle_linesegments : obstacles_linesegments_vec) {
    for (const common::math::LineSegment2d& linesegment :
          obstacle_linesegments) {
      if (vehilce_bounding_box.HasOverlap(linesegment)/* || trailer_bounding_box.HasOverlap(linesegment)*/) {
        AERROR <<std::fixed << "collision start at x: " << linesegment.start().x();
        AERROR <<std::fixed << "collision start at y: " << linesegment.start().y();
        AERROR <<std::fixed << "collision end at x: " << linesegment.end().x();
        AERROR <<std::fixed << "collision end at y: " << linesegment.end().y();
        AERROR << "vehilce_bounding_box: " << vehilce_bounding_box.min_x() << ", " << vehilce_bounding_box.max_x() << "\n";
        AERROR << "trailer_bounding_box: " << trailer_bounding_box.min_x() << ", " << trailer_bounding_box.max_x() << "\n";
        AERROR << "vehilce_bounding_box2: " << vehilce_bounding_box.center_x() << ", " << vehilce_bounding_box.center_y() << ", " << vehilce_bounding_box.heading() << ", " << vehilce_orient << "\n";
        AERROR << "trailer_bounding_box2: " << trailer_bounding_box.center_x() << ", " << trailer_bounding_box.center_y() << ", " << trailer_bounding_box.heading() << ", " << trailer_orient << "\n";
        return false;
      }
    }
  }
  return true;
}


void CalVehiclePosition(const double trailer_wheelbase, double dfai,double radius, cv::Point2d &vehicle_center_last, double &vehilce_orient_last,
                  double &vehicle_and_trailer_angle_last, double &trailer_orient_last, cv::Point2d &trailer_center_last, const bool is_straight) {
  double vehicle_orient_cur_time = dfai + vehilce_orient_last;
  dfai = std::fabs(dfai);
  double ds = dfai*radius;
  if (is_straight) {
    vehicle_orient_cur_time = vehilce_orient_last;
    dfai = 0.0;
    ds = 0.15;
  }
  double vehicle_center_pos_base_last_time_x = ds*sin(dfai);
	double vehicle_center_pos_base_last_time_y = ds * cos(dfai);
  double vehicle_center_x_cur = vehicle_center_last.x + (vehicle_center_pos_base_last_time_x * cos(M_PI_2-vehilce_orient_last) + vehicle_center_pos_base_last_time_y * sin(M_PI_2-vehilce_orient_last));
	double vehicle_center_y_cur = vehicle_center_last.y + (vehicle_center_pos_base_last_time_x * (-sin(M_PI_2-vehilce_orient_last)) + vehicle_center_pos_base_last_time_y * cos(M_PI_2-vehilce_orient_last));
	
  vehilce_orient_last = vehicle_orient_cur_time;
  vehicle_center_last.x = vehicle_center_x_cur;
  vehicle_center_last.y = vehicle_center_y_cur;

  double vehicle_and_trailer_angle_temp = vehicle_orient_cur_time - trailer_orient_last;
  double vehicle_and_trailer_angle_new = (vehicle_and_trailer_angle_last + vehicle_and_trailer_angle_temp) / 2.0;
  double trailer_junction_radius = trailer_wheelbase / sin(vehicle_and_trailer_angle_new);
  //拖车旋转角度
  double trailer_longitudinal_base_last_time = ds / trailer_junction_radius;
  //拖车当前朝向角度
  double trailer_orient_cur_time = trailer_orient_last + trailer_longitudinal_base_last_time;
  //车辆和拖车夹角
  vehicle_and_trailer_angle_new = vehicle_orient_cur_time - trailer_orient_cur_time;

  //计算拖车位置
  const double junction_and_vehicle_rear_dis = 0.4;
  double trailer_junction_px = junction_and_vehicle_rear_dis*cos(-vehicle_orient_cur_time) + vehicle_center_x_cur;
  double trailer_junction_py = junction_and_vehicle_rear_dis*(-sin(-vehicle_orient_cur_time)) + vehicle_center_y_cur;

  trailer_center_last.x = (-trailer_wheelbase)*cos(-trailer_orient_cur_time) + trailer_junction_px;
  trailer_center_last.y =  (-trailer_wheelbase)*(-sin(-trailer_orient_cur_time)) + trailer_junction_py;

  trailer_orient_last = trailer_orient_cur_time;
	vehicle_and_trailer_angle_last = vehicle_and_trailer_angle_new;
  /*
  std::ofstream position;
  position.open("vehicle_trailer_position.txt",std::ios::app);
  position << dfai << "\t" << radius << "\t" << vehicle_center_last.x << "\t" << vehicle_center_last.y << "\t" <<
          vehilce_orient_last << "\t" << trailer_junction_px << "\t" << trailer_junction_py << "\t" << 
          trailer_center_last.x << "\t" << trailer_center_last.y << "\t" << trailer_orient_last << "\n";
  position.close();
  */

}


bool MultipleStepTurningPlan(double sx, double sy, double sphi0, double ex, double ey, double ephi0, 
                                const std::vector<double>& XYbounds, 
                                const std::vector<std::vector<common::math::Vec2d>>& obstacles_vertices_vec,
                                bool *is_turn_left, double *realsx, double *realsy, double *realsphi, 
                                double *first_r, double *first_theta, double *second_r, double *second_theta) {
  std::vector<std::vector<common::math::LineSegment2d>> obstacles_linesegments_vec;
  for (const auto& obstacle_vertices : obstacles_vertices_vec) {
    size_t vertices_num = obstacle_vertices.size();
    std::vector<common::math::LineSegment2d> obstacle_linesegments;
    for (size_t i = 0; i < vertices_num - 1; ++i) {
      common::math::LineSegment2d line_segment = common::math::LineSegment2d(
          obstacle_vertices[i], obstacle_vertices[i + 1]);
      obstacle_linesegments.emplace_back(line_segment);
    }
    obstacles_linesegments_vec.emplace_back(obstacle_linesegments);
  }

  const double sphi = common::math::NormalizeAngle(
        sphi0); 
  const double ephi = common::math::NormalizeAngle(
        ephi0); 
  static bool turn_left = false;
  // transform endpoint to car sys
  double ey_in_carsys = -sin(sphi)*(ex-sx) + cos(sphi)*(ey-sy);
  double ex_in_carsys = cos(sphi)*(ex-sx) + sin(sphi)*(ey-sy);
  if (ey_in_carsys > 0.0) {
    turn_left = true;
  } else {
    turn_left = false;
  }

 // double junction_and_vehicle_rear_dis = 0.4;//连接处距离后轴中心距离
  double trailer_wheelbase = 9.50;//挂车轴距
  //double trailer_rear_overhang = 0.0;//挂车后悬
  //double trailer_length = trailer_wheelbase + trailer_rear_overhang;//挂车长度

  common::VehicleParam vehicle_param_t =
    common::VehicleConfigHelper::GetConfig().vehicle_param();
  const double steer_ratio = vehicle_param_t.steer_ratio();
  const double steer_single_direction_max_degree =
    vehicle_param_t.max_steer_angle() ;/// M_PI * 180;
  const double wheel_base = vehicle_param_t.wheel_base();
  const double min_radius = wheel_base/tan(steer_single_direction_max_degree/steer_ratio);//???????????

  for (int kk =0; kk < 100; kk++) {
    double sx2 = sx + 0.5*kk * cos(sphi);
    double sy2 = sy + 0.5 *kk* sin(sphi); // TODO

    for (int jj = 0; jj <= 50 ; jj++) {
      double r = (min_radius+1.5) + jj * 1.0;//avoid min radius
      double vehilce_orient_last_time = sphi;
      double trailer_orient_last_time = vehilce_orient_last_time;//初始化挂车朝向
      double vehicle_and_trailer_angle_last_time = vehilce_orient_last_time - trailer_orient_last_time;//??????????

      // 原方法中的2个参数值
      double rr1 = 0.0;
      double t3 = 0.0;
      double t4 = 0.0;
      if (turn_left) {
        double a_t = ex+r*sin(ephi); 
        double b_t = ey-r*cos(ephi);
        // 第一阶段转弯半径
        rr1 = ((sx2-a_t)*(sx2-a_t)+(sy2-b_t)*(sy2-b_t)-r*r)/(2*(r+(sx2-a_t)*sin(sphi)+(b_t-sy2)*cos(sphi)));//x1>x2
        if(rr1<min_radius) {
          //fprintf('轨迹无解，不等半径方法无法拐弯\n');
          break;
        }
        //第二阶段转弯角度
        t3 = acos((ey-r*cos(ephi)-sy2-rr1*cos(sphi))/(r+rr1))+(M_PI-ephi);
        //第一阶段转弯角度
        t4 = acos((ey-r*cos(ephi)-sy2-rr1*cos(sphi))/(r+rr1)) +(M_PI-sphi);
      } else {
        double a_t = ex-r*sin(ephi);
        double b_t = ey+r*cos(ephi);
        //第一阶段转弯半径
        rr1 = ((sx2-a_t)*(sx2-a_t)+(sy2-b_t)*(sy2-b_t)-r*r)/(2*(r-(sx2-a_t)*sin(sphi)-(b_t-sy2)*cos(sphi)));//x1>x2
       if(rr1<min_radius) {
         // fprintf('轨迹无解，不等半径方法无法拐弯\n');
          break;
        }      
        // 第二阶段转弯角度
        t3 = acos((ey+r*cos(ephi)-sy2+rr1*cos(sphi))/(r+rr1))+ephi;
        // 第一阶段转弯角度
        t4 = acos((ey+r*cos(ephi)-sy2+rr1*cos(sphi))/(r+rr1)) +sphi;
      }
      
      t3 = common::math::NormalizeAngle(t3); 
      t4 = common::math::NormalizeAngle(t4);

      if(ex_in_carsys*ex_in_carsys +ey_in_carsys*ey_in_carsys > (2*(r+rr1))*(2*(r+rr1)) ||
                ex_in_carsys*ex_in_carsys +ey_in_carsys*ey_in_carsys < (2*std::max(r,rr1))*(2*std::max(r,rr1))) {
                  //AERROR << "JJ:" << jj << "    kk: " << kk << "    r: " << r << "  rr1: " << rr1 << "   sx2:" << sx2 << "   sy2: "  << sy2 << "\n";
        continue;
      }

      bool no_collide_first_stage = true;
      bool no_collide_second_stage = true;
      bool no_collide_third_stage = true;
      bool no_collide = true;

      double dfai0 = 0.01;
      if (!turn_left) {
        dfai0 = -0.01;
      }
      cv::Point2d vehicle_center_last = cv::Point2d(sx2, sy2);
      double vehilce_orient_last = vehilce_orient_last_time;
      double vehicle_and_trailer_angle_last = vehicle_and_trailer_angle_last_time;
      double trailer_orient_last = trailer_orient_last_time;
      cv::Point2d  trailer_center_last;
      //第一阶段转弯
      double dfai = dfai0;
      int count_num1 = floor(t4/std::fabs(dfai));
      for (int ii =0;ii<count_num1; ii++) {
        CalVehiclePosition(trailer_wheelbase, dfai,rr1, vehicle_center_last, vehilce_orient_last,
                  vehicle_and_trailer_angle_last, trailer_orient_last, trailer_center_last, 0);
        /*
        common::math::Box2d vehilce_bounding_box = Node3d::GetBoundingBox(
              vehicle_param,vehicle_center.x, vehicle_center.y, vehilce_orient);
        common::math::Box2d trailer_bounding_box = Node3d::GetTrailerBoundingBox(
              vehicle_param, trailer_center.x, trailer_center.y, trailer_orient);
        //if ()      
        */
        no_collide_first_stage = CollideCheck(vehicle_param_t, XYbounds, obstacles_linesegments_vec, 
                  vehilce_orient_last, vehicle_center_last, trailer_orient_last, trailer_center_last);
        no_collide &=  no_collide_first_stage;
        if (!no_collide) {
          break;
        }
          std::ofstream position2;
  position2.open("vehicle_trailer_position2.txt",std::ios::app);
  position2 << dfai << "\t" << rr1 << "\t" << vehicle_center_last.x << "\t" << vehicle_center_last.y << "\t" <<
          vehilce_orient_last << "\t" << 0 << "\t" << 0 << "\t" << 
          trailer_center_last.x << "\t" << trailer_center_last.y << "\t" << trailer_orient_last << "\t" << no_collide <<"\n";
  position2.close();
      }
      CalVehiclePosition(trailer_wheelbase,(t4-count_num1*std::fabs(dfai))*(std::fabs(dfai)/dfai),rr1, vehicle_center_last, vehilce_orient_last,
                  vehicle_and_trailer_angle_last, trailer_orient_last, trailer_center_last, 0);
      no_collide_first_stage = CollideCheck(vehicle_param_t, XYbounds, obstacles_linesegments_vec, 
                  vehilce_orient_last, vehicle_center_last, trailer_orient_last, trailer_center_last);
      no_collide &=  no_collide_first_stage;
      //第二阶段转弯
      if (no_collide) {
        dfai = -dfai0;
        int count_num2 = floor(t3/std::fabs(dfai));
        for (int ii =0;ii<count_num2; ii++) {
          CalVehiclePosition(trailer_wheelbase, dfai,r, vehicle_center_last, vehilce_orient_last,
                    vehicle_and_trailer_angle_last, trailer_orient_last, trailer_center_last, 0);
          no_collide_second_stage = CollideCheck(vehicle_param_t, XYbounds, obstacles_linesegments_vec, 
                    vehilce_orient_last, vehicle_center_last, trailer_orient_last, trailer_center_last);
          no_collide &=  no_collide_second_stage;
          if (!no_collide) {
            break;
          }
                    std::ofstream position2;
  position2.open("vehicle_trailer_position2.txt",std::ios::app);
  position2 << dfai << "\t" << r << "\t" << vehicle_center_last.x << "\t" << vehicle_center_last.y << "\t" <<
          vehilce_orient_last << "\t" << 1 << "\t" << 1 << "\t" << 
          trailer_center_last.x << "\t" << trailer_center_last.y << "\t" << trailer_orient_last << "\t" << no_collide <<"\n";
  position2.close();
        }
        CalVehiclePosition(trailer_wheelbase,(t3-count_num2*std::fabs(dfai))*(std::fabs(dfai)/dfai),r, vehicle_center_last, vehilce_orient_last,
                    vehicle_and_trailer_angle_last, trailer_orient_last, trailer_center_last, 0);
        no_collide_second_stage = CollideCheck(vehicle_param_t, XYbounds, obstacles_linesegments_vec, 
                    vehilce_orient_last, vehicle_center_last, trailer_orient_last, trailer_center_last);
        no_collide &=  no_collide_second_stage;
      }
      
      //第三阶段，直行
      if (no_collide) {   
        for (int ii =0;ii<100; ii++) {
          CalVehiclePosition(trailer_wheelbase, dfai,r, vehicle_center_last, vehilce_orient_last,
                    vehicle_and_trailer_angle_last, trailer_orient_last, trailer_center_last, 1);
          no_collide_third_stage = CollideCheck(vehicle_param_t, XYbounds, obstacles_linesegments_vec, 
                    vehilce_orient_last, vehicle_center_last, trailer_orient_last, trailer_center_last);
          no_collide &=  no_collide_third_stage;
          if (!no_collide) {
            break;
          }
                    std::ofstream position2;
  position2.open("vehicle_trailer_position2.txt",std::ios::app);
  position2 << dfai << "\t" << r << "\t" << vehicle_center_last.x << "\t" << vehicle_center_last.y << "\t" <<
          vehilce_orient_last << "\t" << 1 << "\t" << 1 << "\t" << 
          trailer_center_last.x << "\t" << trailer_center_last.y << "\t" << trailer_orient_last << "\t" << no_collide <<"\n";
  position2.close();
        }
        
      }

      if (no_collide) {
        AERROR << "-------------------------" << "sucess" << std::endl;
        AERROR << "start x : " << sx2 << "   start y: " << sy2 << " , rr1: " << rr1 <<" r: " << r << std::endl;
        
        *is_turn_left =  turn_left;
        *realsx = sx2;
        *realsy = sy2;
        *realsphi = sphi;
        *first_r = rr1;
        *first_theta = t4;
        *second_r = r;
        *second_theta = t3;

        return true; 
      }
    }
  }
  return false; 
}

//--------------------------------------------------------

//--------Pure Pursuit to calculate steerangle-------------
bool CalSteerAngleByPurePursuit(const LocalView &local_view, 
      const std::vector<std::vector<Vec2d>> &obstacles_list, double *steer_angle) {
  auto vehicle_pose = local_view.localization_estimate->pose().position();
  cv::Point2d vehicle_center = cv::Point2d(vehicle_pose.x(), vehicle_pose.y());
  auto vehicle_heading = local_view.localization_estimate->pose().heading();
  auto trailer_pose = local_view.localization_estimate->trailer_pose().position();
   cv::Point2d  trailer_center = cv::Point2d(trailer_pose.x(), trailer_pose.y());
  auto trailer_heading = local_view.localization_estimate->trailer_pose().heading();

  std::vector<double> XYbounds;
  XYbounds.push_back(vehicle_center.x-20.0);
  XYbounds.push_back(vehicle_center.x+20.0);
  XYbounds.push_back(vehicle_center.y-20.0);
  XYbounds.push_back(vehicle_center.y+20.0); //TODO

    std::vector<std::vector<common::math::LineSegment2d>>
      obstacles_linesegments_vec;
  for (const auto& obstacle_vertices : obstacles_list) {
    size_t vertices_num = obstacle_vertices.size();
    std::vector<common::math::LineSegment2d> obstacle_linesegments;
    for (size_t i = 0; i < vertices_num - 1; ++i) {
      common::math::LineSegment2d line_segment = common::math::LineSegment2d(
          obstacle_vertices[i], obstacle_vertices[i + 1]);
      obstacle_linesegments.emplace_back(line_segment);
    }
    obstacles_linesegments_vec.emplace_back(obstacle_linesegments);
  }

  common::VehicleParam vehicle_param =
  common::VehicleConfigHelper::GetConfig().vehicle_param();


  bool is_valid = CollideCheck(vehicle_param, XYbounds, obstacles_linesegments_vec, 
                                vehicle_heading, vehicle_center, trailer_heading, trailer_center);
  if (!is_valid) {
    AERROR << "Current vehicle and trailer close to obstacles, stop.";
    return false;
  } else {
    
    if (!local_view.relative_map->navigation_path().empty()) {        
      const auto path_points = local_view.relative_map->navigation_path().begin()->
                                  second.path().path_point();
      for (auto iter = path_points.begin();
            iter != path_points.end(); ++iter) { //todo, search step too small...
          const auto& px = iter->x();
          const auto& py = iter->y();
          if (px < FLAGS_lanepoint_tartget_px) {       
            continue;
          } else if (px > 15.0) {
            return false;
          } else {
            // TODO
            const double wheel_base = vehicle_param.wheel_base();
            double sin_steering_angle = 2.0*wheel_base*py / (px*px+py*py);       
            if(std::isnan(sin_steering_angle)) {
              sin_steering_angle = 0.0;
            }
            sin_steering_angle = common::math::Clamp(sin_steering_angle, -1.0, 1.0);
            double steering_angle = asin(sin_steering_angle);
            

            // predict 2 step
            double speed_ms = local_view.chassis->speed_mps();
            if (local_view.chassis->gear_location() == canbus::Chassis::GEAR_REVERSE) {
              speed_ms = -1.0*speed_ms;
            }
            for (int predict_count = 1; predict_count < 3; predict_count++) {              
              double ds = speed_ms * (predict_count * 0.1);
              double steering_radius = wheel_base / tan(steering_angle);
              double dtheta = ds / steering_radius;
              cv::Point2d predict_vehicle_pose;
              double predict_vheicle_heading;             
              predict_vehicle_pose.x = vehicle_center.x + ds*cos(dtheta/2.0 + vehicle_heading);
              predict_vehicle_pose.y = vehicle_center.y + ds*sin(dtheta/2.0 + vehicle_heading);
              predict_vheicle_heading = vehicle_heading + dtheta;

              is_valid = CollideCheck(vehicle_param, XYbounds, obstacles_linesegments_vec, 
                                predict_vheicle_heading, predict_vehicle_pose, trailer_heading, trailer_center); //TODO, consider trailer pose
              if (!is_valid) {
                break;
              }
            }
            //
            if (is_valid) {          

              *steer_angle = steering_angle;
              return true;
            }
          }
      }
    }
    return false;   
  }
   
}
//------------------------------------------------------------------

bool PlanningComponent::Init() {
  if (FLAGS_use_navigation_mode) {
    planning_base_ = std::make_unique<NaviPlanning>();
    tracking_selfcar_ = std::make_unique<TrackingSelfCarObject>();
  } else {
    planning_base_ = std::make_unique<OnLanePlanning>();
  }

  CHECK(apollo::cyber::common::GetProtoFromFile(FLAGS_planning_config_file,
                                                &config_))
      << "failed to load planning config file " << FLAGS_planning_config_file;
  planning_base_->Init(config_);

  routing_reader_ = node_->CreateReader<RoutingResponse>(
      FLAGS_routing_response_topic,
      [this](const std::shared_ptr<RoutingResponse>& routing) {
        AINFO << "Received routing data: run routing callback."
              << routing->header().DebugString();
        std::lock_guard<std::mutex> lock(mutex_);
        routing_.CopyFrom(*routing);
      });
  traffic_light_reader_ = node_->CreateReader<TrafficLightDetection>(
      FLAGS_traffic_light_detection_topic,
      [this](const std::shared_ptr<TrafficLightDetection>& traffic_light) {
        ADEBUG << "Received traffic light data: run traffic light callback.";
        std::lock_guard<std::mutex> lock(mutex_);
        traffic_light_.CopyFrom(*traffic_light);
      });

  pad_msg_reader_ = node_->CreateReader<PadMessage>(
      FLAGS_planning_pad_topic,
      [this](const std::shared_ptr<PadMessage>& pad_msg) {
        ADEBUG << "Received pad data: run pad callback.";
        std::lock_guard<std::mutex> lock(mutex_);
        pad_msg_.CopyFrom(*pad_msg);
      });

  if (FLAGS_use_navigation_mode) {
    relative_map_reader_ = node_->CreateReader<MapMsg>(
        FLAGS_relative_map_topic,
        [this](const std::shared_ptr<MapMsg>& map_message) {
          ADEBUG << "Received relative map data: run relative map callback.";
          std::lock_guard<std::mutex> lock(mutex_);
          relative_map_.CopyFrom(*map_message);
        });
    LoadMap();//add by shzhw
    int kf_coor_sys = 1;
    tracking_selfcar_->CreateKalmanFilter(kf_coor_sys);     

        // obstalces 
      Vec2d obstacle_vertice_a1(794604.142851,2490921.821931);//1
      Vec2d obstacle_vertice_a2(794676.293472,2490946.563392);//2
      //Vec2d obstacle_vertice_a3(794784.298158,2490983.693279);//3
      Vec2d obstacle_vertice_a4(794824.007530,2490997.300234);//4
      std::vector<Vec2d> obstacle_a = {obstacle_vertice_a1, obstacle_vertice_a2, obstacle_vertice_a4};

      Vec2d obstacle_vertice_b1(794661.783616,2490926.056318);//11
      Vec2d obstacle_vertice_b2(794639.392561,2490918.323852);//13
      Vec2d obstacle_vertice_b3(794631.956553,2490909.219212);//14
      Vec2d obstacle_vertice_b4(794630.181916,2490901.650215);//15
      Vec2d obstacle_vertice_b5(794671.851551,2490780.138772);//17
      Vec2d obstacle_vertice_b6(794752.130746,2490547.847225);//18
      Vec2d obstacle_vertice_b7(794759.991728,2490539.515713);//19
      Vec2d obstacle_vertice_b8(794768.703049,2490538.573887);//20
      Vec2d obstacle_vertice_b9(794792.155141,2490546.365583);//21
      Vec2d obstacle_vertice_b10(794736.914858,2490707.184026);//33
      Vec2d obstacle_vertice_b11(794661.783616,2490926.056318);//11
      std::vector<Vec2d> obstacle_b = {obstacle_vertice_b1, obstacle_vertice_b2,obstacle_vertice_b3, obstacle_vertice_b4,
            obstacle_vertice_b5, obstacle_vertice_b6, obstacle_vertice_b7,obstacle_vertice_b8, obstacle_vertice_b9, obstacle_vertice_b10, obstacle_vertice_b11};

      Vec2d obstacle_vertice_c1(794746.479248,2490955.138771);//8
     // Vec2d obstacle_vertice_c2(794710.902617,2490942.992875);//9
      Vec2d obstacle_vertice_c3(794678.619405,2490931.895296);//10
      Vec2d obstacle_vertice_c4(794752.708018,2490715.718782);//34
      Vec2d obstacle_vertice_c5(794809.871141,2490553.811640);//30
      //Vec2d obstacle_vertice_c6(794840.434975,2490563.519082);//22
      Vec2d obstacle_vertice_c7(794876.704141,2490575.981682);//24
      Vec2d obstacle_vertice_c8(794818.945753,2490744.212851);//35
      Vec2d obstacle_vertice_c9(794746.479248,2490955.138771);//8
      std::vector<Vec2d> obstacle_c = {obstacle_vertice_c1, obstacle_vertice_c3,obstacle_vertice_c4, obstacle_vertice_c5, 
            obstacle_vertice_c7,obstacle_vertice_c8,obstacle_vertice_c9};

      Vec2d obstacle_vertice_d1(794965.419488,2490594.165741);//27
      //Vec2d obstacle_vertice_d2(794895.502999,2490570.205082);//28
      Vec2d obstacle_vertice_d3(794812.727016,2490541.847528);//29
      Vec2d obstacle_vertice_d4(794770.431159,2490527.133717);//31
      Vec2d obstacle_vertice_d5(794764.689948,2490529.700376);//32
      std::vector<Vec2d> obstacle_d = {obstacle_vertice_d1, obstacle_vertice_d3, obstacle_vertice_d4,obstacle_vertice_d5};


      Vec2d obstacle_vertice_e1(794831.207959,2490984.258077);//5
      //Vec2d obstacle_vertice_e2(794797.069154,2490972.593304);//6
      Vec2d obstacle_vertice_e3(794763.338538,2490960.972758);//7
      Vec2d obstacle_vertice_e4(794836.128645,2490748.681849);//36
      Vec2d obstacle_vertice_e5(794895.013619,2490584.312325);//25
      Vec2d obstacle_vertice_e6(794961.419345,2490605.031919);//26
      std::vector<Vec2d> obstacle_e = {obstacle_vertice_e1, obstacle_vertice_e3, obstacle_vertice_e4,obstacle_vertice_e5,
              obstacle_vertice_e6};


     yard_obstacles_list_.emplace_back(obstacle_a);
     yard_obstacles_list_.emplace_back(obstacle_b);
     yard_obstacles_list_.emplace_back(obstacle_c);
     yard_obstacles_list_.emplace_back(obstacle_d);
     yard_obstacles_list_.emplace_back(obstacle_e);     
  }
  planning_writer_ =
      node_->CreateWriter<ADCTrajectory>(FLAGS_planning_trajectory_topic);

  rerouting_writer_ =
      node_->CreateWriter<RoutingRequest>(FLAGS_routing_request_topic);

  return true;
}

bool PlanningComponent::Proc(
    const std::shared_ptr<prediction::PredictionObstacles>&
        prediction_obstacles,
    const std::shared_ptr<canbus::Chassis>& chassis,
    const std::shared_ptr<localization::LocalizationEstimate>&
        localization_estimate) {
  CHECK(prediction_obstacles != nullptr);

  // check and process possible rerouting request
  CheckRerouting();

  //fflush(NULL);
  //printf("planning\n");

  // process fused input data
  local_view_.prediction_obstacles = prediction_obstacles;
  local_view_.chassis = chassis;
  local_view_.localization_estimate = localization_estimate;

  //cyber::common::SetProtoToASCIIFile(*chassis,"chassis_data_save_tmp.pb.txt");//add by shzhw to save adc trajectory
  //cyber::common::SetProtoToASCIIFile(*prediction_obstacles,"predition_obstacle_data_save_tmp.pb.txt");//add by shzhw to save adc trajectory
  //add by shzhw, to delete and test
  /*
  local_view_.localization_estimate->mutable_pose()->mutable_position()->set_x(0.0);
  local_view_.localization_estimate->mutable_pose()->mutable_position()->set_y(0.0);
  local_view_.localization_estimate->mutable_pose()->mutable_orientation()->set_qx(0.0);
  local_view_.localization_estimate->mutable_pose()->mutable_orientation()->set_qy(0.0);
  local_view_.localization_estimate->mutable_pose()->mutable_orientation()->set_qz(0.0);
  local_view_.localization_estimate->mutable_pose()->mutable_orientation()->set_qw(0.0);
  local_view_.localization_estimate->mutable_pose()->set_heading(0.0);

  local_view_.localization_estimate->mutable_pose()->mutable_linear_acceleration()->set_x(0.0);
  local_view_.localization_estimate->mutable_pose()->mutable_linear_acceleration()->set_y(0.0);
  local_view_.localization_estimate->mutable_pose()->mutable_linear_acceleration()->set_z(0.0);
  local_view_.localization_estimate->mutable_pose()->mutable_angular_velocity_vrf()->set_x(0.0);
  local_view_.localization_estimate->mutable_pose()->mutable_angular_velocity_vrf()->set_y(0.0);
  local_view_.localization_estimate->mutable_pose()->mutable_angular_velocity_vrf()->set_z(0.0);
  
  RoutingResponse routing_from_file;
  //const string full_routing_path = "/apollo/modules/routing/testdata/routing_tester/routing_test.pb.txt";
  bool load_success =
        cyber::common::GetProtoFromASCIIFile("/apollo/modules/routing/testdata/routing_tester/routing_test_shzhw.pb.txt", &routing_from_file);
  if (load_success) {
    routing_.CopyFrom(routing_from_file);
  }
  */
  //---------------------------------------
  
  const auto& cur_vehicle_state = common::VehicleStateProvider::Instance()->vehicle_state();
  const double vehicle_speed = cur_vehicle_state.linear_velocity();//std::max(std::fabs(cur_vehicle_state.linear_velocity()),0.002);
  // static double pre_vehicle_state_timestamp = cur_vehicle_state.timestamp();

  static double pre_timestamp = apollo::common::time::Clock::NowInSeconds();

  static double drive_length_guidepost = 0.0;
  static double pre_drive_length_guidepost = 0.0; //20200608
  static bool reset_flag = false;
  static bool is_stop = false;
  // double time_diff = cur_vehicle_state.timestamp() - pre_vehicle_state_timestamp;

  double cur_timestamp = apollo::common::time::Clock::NowInSeconds();
  double time_diff = cur_timestamp - pre_timestamp;

  // AERROR << "TIME DIFF: " << time_diff;
  // pre_vehicle_state_timestamp = cur_vehicle_state.timestamp();

  pre_timestamp = cur_timestamp;
  
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!local_view_.routing ||
        hdmap::PncMap::IsNewRouting(*local_view_.routing, routing_)) {
      local_view_.routing =
          std::make_shared<routing::RoutingResponse>(routing_);
      if (!is_stop) {
        pre_drive_length_guidepost = drive_length_guidepost; //20200608
      	// drive_length_guidepost = 0.0;
        drive_length_guidepost = time_diff * vehicle_speed; //20200608
      }
      reset_flag = true;
    } else { // add by shzhw      
      drive_length_guidepost += time_diff * vehicle_speed;
      reset_flag = false;
    }
  }

    //AERROR << "drive_length_guidepost: " << drive_length_guidepost << ", " << pre_drive_length_guidepost;
    //std::cout << "reset_flag: " << reset_flag << ", vehilce speed: " << std::max(std::fabs(cur_vehicle_state.linear_velocity()),0.2) << std::endl;
  //--------------------------------------------------------------------
/*
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!local_view_.routing ||
        hdmap::PncMap::IsNewRouting(*local_view_.routing, routing_)) {
      local_view_.routing =
          std::make_shared<routing::RoutingResponse>(routing_);
    }
  }
*/
  {
    std::lock_guard<std::mutex> lock(mutex_);
    local_view_.traffic_light =
        std::make_shared<TrafficLightDetection>(traffic_light_);
    local_view_.relative_map = std::make_shared<MapMsg>(relative_map_);
  }
  {
    std::lock_guard<std::mutex> lock(mutex_);
    local_view_.pad_msg = std::make_shared<PadMessage>(pad_msg_);
  }

  if (!CheckInput()) {
    AERROR << "Input check failed";
    return false;
  }

  ADCTrajectory adc_trajectory_pb;
  planning_base_->RunOnce(local_view_, &adc_trajectory_pb);
  auto start_time = adc_trajectory_pb.header().timestamp_sec();

  //add by shzhw, method2, ouput steering and throttle to control 
  if (FLAGS_enable_turn_plan) {

    if (FLAGS_multiple_step_planner) {
      //publish  stop cmd
      adc_trajectory_pb.set_is_use_routing(true);
      adc_trajectory_pb.mutable_simple_control_cmd()->set_throttle(0.0);
      adc_trajectory_pb.mutable_simple_control_cmd()->set_brake(55);
      adc_trajectory_pb.mutable_simple_control_cmd()->set_steering_target(0.0);

      double sx =FLAGS_vehicle_start_px;
      double sy = FLAGS_vehicle_start_py;
      double sphi = FLAGS_vehicle_start_phi;
      double ex = FLAGS_vehicle_end_px;//10.0;//50;//
      double ey = FLAGS_vehicle_end_py;//15.0;
      double ephi = FLAGS_vehicle_end_phi;//M_PI;//0;//
      std::vector<std::vector<Vec2d>> obstacles_list;
      //HybridAStartResult result;
      Vec2d obstacle_vertice_a(13.0,sy-50.0);
      Vec2d obstacle_vertice_b(13.0, 13.0);
      Vec2d obstacle_vertice_c(13.0, 17.0);
      Vec2d obstacle_vertice_d(13.0,sy+50.0);
      std::vector<Vec2d> obstacle = {obstacle_vertice_a, obstacle_vertice_b};
      std::vector<Vec2d> obstacle2 = {obstacle_vertice_c, obstacle_vertice_d};
      
      // load xy boundary into the Plan() from configuration(Independent from frame)
      std::vector<double> XYbounds_;
      XYbounds_.push_back(sx-80.0);
      XYbounds_.push_back(sx+80.0);
      XYbounds_.push_back(sy-80.0);
      XYbounds_.push_back(sy+80.0);

      obstacles_list.emplace_back(obstacle);
      obstacles_list.emplace_back(obstacle2);

      static bool is_turn_left = false;
      static double realsx = 0.0;
      static double realsy = 0.0;
      static double realsphi = sphi;
      static double first_r = 100.0;
      static double first_theta = 0.0;
      static double second_r = 100.0;
      static double second_theta = 0.0;
      static int kkk = 0;
      static bool turn_plan_vaild = false;

      static double turn_drive_length = 0.0;

      static double first_steering_target = 0.0; 
      static double second_steering_target = 0.0;

      if(kkk ==0) {
          turn_plan_vaild = MultipleStepTurningPlan( sx,  sy,  sphi,  ex,  ey,  ephi, 
                                    XYbounds_, obstacles_list ,&is_turn_left, &realsx, &realsy, &realsphi,
                                    &first_r, &first_theta, &second_r, &second_theta) ;
        }
      kkk++;
      if (turn_plan_vaild){
        turn_drive_length += time_diff * vehicle_speed;
        // cal steering
        common::VehicleParam vehicle_param =
        common::VehicleConfigHelper::GetConfig().vehicle_param();
        const double steer_ratio = vehicle_param.steer_ratio();
        const double steer_single_direction_max_degree =
        vehicle_param.max_steer_angle() ;/// M_PI * 180;
        const double wheelbase = vehicle_param.wheel_base();

        double first_steering_angle = asin(wheelbase/first_r);
        first_steering_angle = first_steering_angle  * steer_ratio / steer_single_direction_max_degree * 100.0; /// M_PI *180.0
        double second_steering_angle = asin(wheelbase/second_r);
        second_steering_angle = second_steering_angle  * steer_ratio / steer_single_direction_max_degree * 100.0; /// M_PI *180.0

        if (is_turn_left) {
          first_steering_target = common::math::Clamp(first_steering_angle, -100.0, 100.0);
          second_steering_target = common::math::Clamp(-1.0 * second_steering_angle, -100.0, 100.0);
        } else {
          first_steering_target = common::math::Clamp(-1.0 * first_steering_angle, -100.0, 100.0);
          second_steering_target = common::math::Clamp(second_steering_angle, -100.0, 100.0);
        }
        AERROR << "first steering target:" << first_steering_target << "second steering target:" << second_steering_target <<"\n";
        AERROR << "drive length:" << turn_drive_length << "\n";
        //publish cmd
        const double cur_steering_angle = cur_vehicle_state.steering_percentage();
        double distance_to_real_start = std::sqrt((realsx-sx)*(realsx-sx)+(realsy-sy)*(realsy-sy));
        if (turn_drive_length < distance_to_real_start ) {
          adc_trajectory_pb.set_is_use_routing(true);
          adc_trajectory_pb.mutable_simple_control_cmd()->set_brake(0);
          adc_trajectory_pb.mutable_simple_control_cmd()->set_throttle(FLAGS_gudiepost_curise_throttle);
          adc_trajectory_pb.mutable_simple_control_cmd()->set_steering_target(0.0);
        } else if(turn_drive_length < (distance_to_real_start +first_r* first_theta)) {
          if ((cur_steering_angle*first_steering_target > 0.0) && 
                (std::fabs(cur_steering_angle)-std::fabs(first_steering_target) < 1.5)) {
            adc_trajectory_pb.set_is_use_routing(true);
            adc_trajectory_pb.mutable_simple_control_cmd()->set_brake(0);
            adc_trajectory_pb.mutable_simple_control_cmd()->set_throttle(FLAGS_gudiepost_curise_throttle);
            adc_trajectory_pb.mutable_simple_control_cmd()->set_steering_target(first_steering_target);
          } else {
            adc_trajectory_pb.set_is_use_routing(true);
            adc_trajectory_pb.mutable_simple_control_cmd()->set_brake(45);
            adc_trajectory_pb.mutable_simple_control_cmd()->set_throttle(0.0);
            adc_trajectory_pb.mutable_simple_control_cmd()->set_steering_target(first_steering_target);
          }
        } else if (turn_drive_length < (distance_to_real_start +first_r* first_theta+second_r*second_theta)) {
          if ((cur_steering_angle*second_steering_target > 0.0) && 
                (std::fabs(cur_steering_angle)-std::fabs(second_steering_target) < 1.5)) {
            adc_trajectory_pb.set_is_use_routing(true);
            adc_trajectory_pb.mutable_simple_control_cmd()->set_brake(0);
            adc_trajectory_pb.mutable_simple_control_cmd()->set_throttle(FLAGS_gudiepost_curise_throttle);
            adc_trajectory_pb.mutable_simple_control_cmd()->set_steering_target(second_steering_target);
          } else {
            adc_trajectory_pb.set_is_use_routing(true);
            adc_trajectory_pb.mutable_simple_control_cmd()->set_brake(35);
            adc_trajectory_pb.mutable_simple_control_cmd()->set_throttle(0.0);
            adc_trajectory_pb.mutable_simple_control_cmd()->set_steering_target(second_steering_target);                   
          }
        } else if (turn_drive_length < (distance_to_real_start +first_r* first_theta+second_r*second_theta + 10.0)) {
          if (std::fabs(cur_steering_angle) < 0.5) {
            adc_trajectory_pb.set_is_use_routing(true);
            adc_trajectory_pb.mutable_simple_control_cmd()->set_brake(0);
            adc_trajectory_pb.mutable_simple_control_cmd()->set_throttle(FLAGS_gudiepost_curise_throttle);
            adc_trajectory_pb.mutable_simple_control_cmd()->set_steering_target(0.0);
          } else {
            adc_trajectory_pb.set_is_use_routing(true);
            adc_trajectory_pb.mutable_simple_control_cmd()->set_brake(35);
            adc_trajectory_pb.mutable_simple_control_cmd()->set_throttle(0.0);
            adc_trajectory_pb.mutable_simple_control_cmd()->set_steering_target(0.0);
          }
        } else {
          //publish  stop cmd
          adc_trajectory_pb.set_is_use_routing(true);
          adc_trajectory_pb.mutable_simple_control_cmd()->set_throttle(0.0);
          adc_trajectory_pb.mutable_simple_control_cmd()->set_brake(55);
          adc_trajectory_pb.mutable_simple_control_cmd()->set_steering_target(0.0);
        }
        AERROR << "drive length:" << turn_drive_length << ",distance_to_real_start:" <<distance_to_real_start << "\n";
        AERROR << "first path: " << first_r* first_theta << ", second path: " << second_r*second_theta << "\n";
      }
    } else {
      //publish  stop cmd
      adc_trajectory_pb.set_is_use_routing(true);
      adc_trajectory_pb.mutable_simple_control_cmd()->set_throttle(0.0);
      adc_trajectory_pb.mutable_simple_control_cmd()->set_brake(55);
      adc_trajectory_pb.mutable_simple_control_cmd()->set_steering_target(0.0);

      //hybrid A*
      apollo::cyber::common::GetProtoFromFile(
          "/apollo/modules/planning/testdata/conf/open_space_standard_parking_lot.pb.txt", &planner_open_space_config_);

      hybrid_test = std::unique_ptr<HybridAStar>(
          new HybridAStar(planner_open_space_config_));
      /*
      double sx = 0.0;
      double sy = -20.0;
      double sphi = M_PI/2;
      double ex = -30.0;
      double ey = -2.50;
      double ephi = -M_PI;
      */
      double sx =FLAGS_vehicle_start_px;
      double sy = FLAGS_vehicle_start_py;
      double sphi = FLAGS_vehicle_start_phi;
      double ex = FLAGS_vehicle_end_px;//10.0;//50;//
      double ey = FLAGS_vehicle_end_py;//15.0;
      double ephi = FLAGS_vehicle_end_phi;//M_PI;//0;//
      std::vector<std::vector<Vec2d>> obstacles_list;
      HybridAStartResult result;
      Vec2d obstacle_vertice_a(ex+15.0, sy-80.0);
      Vec2d obstacle_vertice_b(ex+15.0, ey-2.0);
      Vec2d obstacle_vertice_c(ex+15.0, ey+2.0);
      Vec2d obstacle_vertice_d(ex+15.0, sy+80.0);
      std::vector<Vec2d> obstacle = {obstacle_vertice_a, obstacle_vertice_b};
      std::vector<Vec2d> obstacle2 = {obstacle_vertice_c, obstacle_vertice_d};

      // load xy boundary into the Plan() from configuration(Independent from frame)
      std::vector<double> XYbounds_;
      XYbounds_.push_back(sx-80.0);
      XYbounds_.push_back(sx+80.0);
      XYbounds_.push_back(sy-80.0);
      XYbounds_.push_back(sy+80.0);

      obstacles_list.emplace_back(obstacle);
      obstacles_list.emplace_back(obstacle2);

      static double A_star_target_px = 1.0;
      static double A_star_target_py = 0.0;

      static int hybrid_A_planner_count = 0;
      static bool hybrid_test_valid = false;
      if (hybrid_A_planner_count == 0) {
        hybrid_test_valid = hybrid_test->Plan(sx, sy, sphi, ex, ey, ephi, XYbounds_,
                                    obstacles_list, &result);

        std::ofstream hy_coarse_traject_out;
        hy_coarse_traject_out.open("hy_coarse_traject_out.txt",std::ios::app);
        for (size_t i = 0;i < result.a.size();i++) {
          hy_coarse_traject_out << result.x[i] << "\t" <<result.y[i] << "\t" << result.phi[i] << std::endl;
        }
        hy_coarse_traject_out.close(); 
      }
      hybrid_A_planner_count++;

      if (hybrid_test_valid) {
        const auto &cur_pose = local_view_.localization_estimate->pose().position();
        double cur_heading = local_view_.localization_estimate->pose().heading();
        cur_heading = common::math::NormalizeAngle(cur_heading);
        // transform to car sys
        for (size_t i = 0;i < result.a.size();i++) {          
          double path_point_px = cos(cur_heading)*(result.x[i] - cur_pose.x()) + sin(cur_heading)*(result.y[i] - cur_pose.y());
          double path_point_py = -sin(cur_heading)*(result.x[i] - cur_pose.x()) + cos(cur_heading)*(result.y[i] - cur_pose.y());
          if (path_point_px >= FLAGS_lanepoint_tartget_px) {
            A_star_target_px = path_point_px;
            A_star_target_py = path_point_py;
            break;
          }
        }
        // TODO  
        double sin_steering_angle = 2*3.34*A_star_target_py / 
                        (A_star_target_px*A_star_target_px+A_star_target_py*A_star_target_py);       
        if(std::isnan(sin_steering_angle)){
          sin_steering_angle = 0.0;
        }
        sin_steering_angle = common::math::Clamp(sin_steering_angle, -1.0, 1.0);
        double steering_angle = asin(sin_steering_angle);

        common::VehicleParam vehicle_param_t =
        common::VehicleConfigHelper::GetConfig().vehicle_param();
        const double steer_ratio = vehicle_param_t.steer_ratio();
        const double steer_single_direction_max_degree =
        vehicle_param_t.max_steer_angle() ;/// M_PI * 180;

        steering_angle = steering_angle  * steer_ratio / steer_single_direction_max_degree * 100.0; /// M_PI *180.0


        double steering_target = common::math::Clamp(steering_angle, -100.0, 100.0);
        if (steering_angle)  {
          AERROR << steering_angle;
          AERROR << A_star_target_px << "," << A_star_target_py << "\n";
        }
        adc_trajectory_pb.set_is_use_routing(true);
        adc_trajectory_pb.mutable_simple_control_cmd()->set_throttle(FLAGS_gudiepost_curise_throttle);
        adc_trajectory_pb.mutable_simple_control_cmd()->set_steering_target(steering_target);
        
        double path_point_end_px = cos(cur_heading)*(result.x[result.y.size()-1] - cur_pose.x()) + sin(cur_heading)*(result.y[result.y.size()-1] - cur_pose.y());
        if (path_point_end_px < 1.0) {
          adc_trajectory_pb.set_is_use_routing(true);
          adc_trajectory_pb.mutable_simple_control_cmd()->set_throttle(0.0);
          adc_trajectory_pb.mutable_simple_control_cmd()->set_brake(56);
          adc_trajectory_pb.mutable_simple_control_cmd()->set_steering_target(0.0);        
        }
      }

    }
    // test drive length , add by shzhw, 20200807
    if (FLAGS_enable_drive_len_test) {
      static double vehicle_drive_length = 0.0; 
      static double pre_timestamp_for_test = apollo::common::time::Clock::NowInSeconds();
      double cur_timestamp_for_test = apollo::common::time::Clock::NowInSeconds();
      double time_diff_for_test = cur_timestamp_for_test - pre_timestamp_for_test;
      pre_timestamp_for_test = cur_timestamp_for_test;
   
      vehicle_drive_length += time_diff_for_test * vehicle_speed;
      if(vehicle_drive_length <= FLAGS_test_drive_length) {
        adc_trajectory_pb.set_is_use_routing(true);
        adc_trajectory_pb.mutable_simple_control_cmd()->set_throttle(FLAGS_gudiepost_curise_throttle);
        adc_trajectory_pb.mutable_simple_control_cmd()->set_brake(0);
        adc_trajectory_pb.mutable_simple_control_cmd()->set_steering_target(0.0);
      } else {
        adc_trajectory_pb.set_is_use_routing(true);
        adc_trajectory_pb.mutable_simple_control_cmd()->set_throttle(0.0);
        adc_trajectory_pb.mutable_simple_control_cmd()->set_brake(50);
        adc_trajectory_pb.mutable_simple_control_cmd()->set_steering_target(0.0);
      }
    }

  }
   //----------------------------------------------------
  /**/
  if (FLAGS_enable_guidepost_cruise && !FLAGS_enable_turn_plan) { //互斥关系----------------------------------
    //const apollo::common::PointENU target_point;
    //static double turn_angle = 0.0;
    auto cur_vehicle_pose = local_view_.localization_estimate->pose().position();
    double rote_zone_px = cur_vehicle_pose.x()*cos(-1.24) + cur_vehicle_pose.y()*sin(-1.24);  
    double cur_vehicle_heading = local_view_.localization_estimate->pose().heading();
    static double perception_guidepost_turn_angle = 0.0;
    static double perception_guidepost_target_px = 1.0;
    static double perception_guidepost_target_py = 0.0;
    
    double target_s = 1.0;
    if (FLAGS_using_lanepoint_guide) {      
      if (!local_view_.relative_map->navigation_path().empty()) {
        #if 1
        double steer_angle_res = 0.0;
        bool res_vaild = CalSteerAngleByPurePursuit(local_view_, yard_obstacles_list_, &steer_angle_res);
        if (res_vaild) {
          common::VehicleParam vehicle_param_ =
          common::VehicleConfigHelper::GetConfig().vehicle_param();
          const double steer_ratio = vehicle_param_.steer_ratio();
          const double steer_single_direction_max_degree =
          vehicle_param_.max_steer_angle() ;/// M_PI * 180;

          double steering_angle = steer_angle_res  * steer_ratio / steer_single_direction_max_degree * 100.0; /// M_PI *180.0
          if (rote_zone_px > -2097764.86738379 && rote_zone_px < -2097384.65183531 &&
              (std::fabs(cur_vehicle_heading-(-1.24))<0.12 ||std::fabs(cur_vehicle_heading-1.9)<0.12) ) {
            steering_angle = common::math::Clamp(steering_angle, -5.5, 5.5);
            AERROR << "CHAO";
          }
          steering_angle -=2.0;//add 20201020 test
		      double steering_target = common::math::Clamp(steering_angle, -100.0, 100.0);
          adc_trajectory_pb.set_is_use_routing(true);
          adc_trajectory_pb.mutable_simple_control_cmd()->set_throttle(FLAGS_gudiepost_curise_throttle);
          if (std::fabs(local_view_.chassis->steering_percentage()) > 8.0) {
	          if (std::fabs(local_view_.chassis->speed_mps()) < 2.0/3.6) {
	            adc_trajectory_pb.mutable_simple_control_cmd()->set_throttle(13);
	          } else if (std::fabs(local_view_.chassis->speed_mps()) > 3.6/3.6) {
	            adc_trajectory_pb.mutable_simple_control_cmd()->set_throttle(11);
	          }
	        } else {
	          if (std::fabs(local_view_.chassis->speed_mps()) < 2.0/3.6) {
	            adc_trajectory_pb.mutable_simple_control_cmd()->set_throttle(12);
	          } else if (std::fabs(local_view_.chassis->speed_mps()) > 3.6/3.6) {
	            adc_trajectory_pb.mutable_simple_control_cmd()->set_throttle(FLAGS_gudiepost_curise_throttle);
	          }
	        }
		      adc_trajectory_pb.mutable_simple_control_cmd()->set_steering_target(steering_target);
        } else {
          adc_trajectory_pb.set_is_use_routing(true);
          adc_trajectory_pb.mutable_simple_control_cmd()->set_throttle(0.0);
          adc_trajectory_pb.mutable_simple_control_cmd()->set_brake(80.0);
          adc_trajectory_pb.mutable_simple_control_cmd()->set_steering_target(0.0);
          AERROR << "close to obstacles";
        }

        #else
        static double target_px = 1.0;
        static double target_py = 0.0;
        const auto path_points = local_view_.relative_map->navigation_path().begin()->
                                    second.path().path_point();//.begin()->x();
        for (auto iter = path_points.begin();
              iter != path_points.end(); ++iter) {
            const auto& point_x = iter->x();
            if (point_x >= FLAGS_lanepoint_tartget_px) {
              target_px = point_x;
              target_py = iter->y();
              target_s = iter->s();
              break;
            }
        }
        // TODO  
        double sin_steering_angle = 2*3.34*target_py / 
                        (target_px*target_px+target_py*target_py);       
        if(std::isnan(sin_steering_angle)){
          sin_steering_angle = 0.0;
        }
        if (sin_steering_angle < -1.0) {
          sin_steering_angle = -1.0;
        } else if(sin_steering_angle > 1.0) {
          sin_steering_angle =1.0;
        }  

        double steering_angle = asin(sin_steering_angle);

        common::VehicleParam vehicle_param_ =
        common::VehicleConfigHelper::GetConfig().vehicle_param();
        const double steer_ratio = vehicle_param_.steer_ratio();
        const double steer_single_direction_max_degree =
        vehicle_param_.max_steer_angle() ;/// M_PI * 180;

        steering_angle = steering_angle  * steer_ratio / steer_single_direction_max_degree * 100.0; /// M_PI *180.0

        steering_angle -=2.0;//add 20201020 test
        double steering_target = common::math::Clamp(steering_angle, -100.0, 100.0);
        if (steering_angle)  {
          AERROR << steering_angle;
          AERROR << target_px << "," << target_py << "," << target_s;
        }

        adc_trajectory_pb.set_is_use_routing(true);
        adc_trajectory_pb.mutable_simple_control_cmd()->set_throttle(FLAGS_gudiepost_curise_throttle);
        if (std::fabs(local_view_.chassis->steering_percentage()) > 12.0) {
          if (std::fabs(local_view_.chassis->speed_mps()) < 2.0/3.6) {
            adc_trajectory_pb.mutable_simple_control_cmd()->set_throttle(13);
          } else if (std::fabs(local_view_.chassis->speed_mps()) > 3.6/3.6) {
            adc_trajectory_pb.mutable_simple_control_cmd()->set_throttle(11);
          }
        } else {
          if (std::fabs(local_view_.chassis->speed_mps()) < 2.0/3.6) {
            adc_trajectory_pb.mutable_simple_control_cmd()->set_throttle(12);
          } else if (std::fabs(local_view_.chassis->speed_mps()) > 3.6/3.6) {
            adc_trajectory_pb.mutable_simple_control_cmd()->set_throttle(FLAGS_gudiepost_curise_throttle);
          }
        }
        adc_trajectory_pb.mutable_simple_control_cmd()->set_steering_target(steering_target);

        if (adc_trajectory_pb.header().sequence_num() < 100 && drive_length_guidepost > 2e8) {
          drive_length_guidepost = 0.0;
        } 
        #endif
      }
    } else {
      adc_trajectory_pb.set_is_use_routing(false);
    }

    if (local_view_.routing->has_routing_request() && 0) {// delete 20201016
      const auto end_guidepost_id = local_view_.routing->routing_request().end_id();
      //从通道获取起始坐标ID、摄像头检测到的一组路标坐标
      const auto& perception_guidepost = *(local_view_.routing->routing_request().waypoint().rbegin());
      const std::string startPostID = perception_guidepost.id();
      const auto perception_point = perception_guidepost.pose();
      static double perception_guidepost_steering_target = 0.0;

      static std::string cur_guidepost = "";
      // cur_guidepost = startPostID; // 
      //std::string startPostID = std::to_string(std::stoi(startPostID0)%38); //test //////////////////////////

              //起始路标ID不为空时，从该路标起步；否则默认从路标地图第一组坐标起步
              uint guidepostGroupMark = 0;
              if (!startPostID.empty()) {
                for(uint i = 0; i < guidepostGroupVector.size() - 1; i++)
                {
                  if (guidepostGroupVector[i].id == startPostID) {
                    guidepostGroupMark = i;
                    break;
                  }
                }
              }
              //取当前坐标组(绝对坐标系下),存储前后两个点
              GuidepostGroup currentGuidepostGroup = guidepostGroupVector[guidepostGroupMark];
              static Eigen::Vector2d perception_guidepost_points[2];
              static Eigen::Vector2d world_guidepost_points[2];

              if (!local_view_.routing->routing_request().is_use_guidepost()) {
                if (reset_flag) {
                  world_guidepost_points[1] = {currentGuidepostGroup.x,currentGuidepostGroup.y};
                  perception_guidepost_points[1] = {perception_point.x(),perception_point.y()};
                }
              }

      // add 20200628    
      static bool ekf_init = false; 
      static uint cur_guidepost_group_mark = 0;
      if (FLAGS_gudiepost_method == 3) {          
        if (reset_flag && !ekf_init) {
          ekf_init =  tracking_selfcar_->InitKalmanFilter(local_view_,guidepostGroupVector);     
        }
        #if ekf_per_chassis
        if (ekf_init ) {
          tracking_selfcar_->KalmanFilterPredict(local_view_);
        }
        #endif
        if (reset_flag && ekf_init) {
          #if !ekf_per_chassis
          tracking_selfcar_->KalmanFilterPredict(local_view_);
          #endif
          tracking_selfcar_->KalmanFilterCorrect(local_view_,guidepostGroupVector);

          float selfcar_px = tracking_selfcar_->kalman_filter_->state_post_.at<float>(0,0);
          float selfcar_py = tracking_selfcar_->kalman_filter_->state_post_.at<float>(1,0);
          float selfcar_theta = tracking_selfcar_->kalman_filter_->state_post_.at<float>(2,0);

            // to mathced routing map
          float m_x_1 = perception_point.x()*(cos(-selfcar_theta)) + perception_point.y()*(sin(-selfcar_theta));
          float m_y_1 = perception_point.x()*(-sin(-selfcar_theta)) + perception_point.y()*(cos(-selfcar_theta));

          float offset_min = 99999.0;
          for(uint i = 0; i < guidepostGroupVector.size() - 1; i++)
          {
            float offset = (guidepostGroupVector[i].x - m_x_1 - selfcar_px) * (guidepostGroupVector[i].x - m_x_1 - selfcar_px) + 
                        (guidepostGroupVector[i].y - m_y_1 - selfcar_py) * (guidepostGroupVector[i].y - m_y_1 - selfcar_py);
            if (offset <= offset_min) {
              offset_min = offset;
              cur_guidepost_group_mark = i;
            }
          }
          uint nex_guidepostGroupMark = (cur_guidepost_group_mark + 1) % (guidepostGroupVector.size()-0);
          const double next_world_guidepost_px = guidepostGroupVector[nex_guidepostGroupMark].x;
          const double next_world_guidepost_py = guidepostGroupVector[nex_guidepostGroupMark].y; 
            
          perception_guidepost_target_px = (next_world_guidepost_px - selfcar_px)*(cos(selfcar_theta)) + (next_world_guidepost_py - selfcar_py)*(sin(selfcar_theta));
          perception_guidepost_target_py = (next_world_guidepost_px - selfcar_px)*(-sin(selfcar_theta)) + (next_world_guidepost_py - selfcar_py)*(cos(selfcar_theta));
          target_s = perception_guidepost_target_px;
        }
        //float selfcar_px2 = tracking_selfcar_->kalman_filter_->state_post_.at<float>(0,0);
        //float selfcar_py2 = tracking_selfcar_->kalman_filter_->state_post_.at<float>(1,0);
        //float selfcar_theta2 = tracking_selfcar_->kalman_filter_->state_post_.at<float>(2,0);
        /*
          // to mathced routing map
        float m_x_1 = perception_point.x()*(cos(-selfcar_theta)) + perception_point.y()*(sin(-selfcar_theta));
        float m_y_1 = perception_point.x()*(-sin(-selfcar_theta)) + perception_point.y()*(cos(-selfcar_theta));

        float offset_min = 99999.0;
        for(uint i = 0; i < guidepostGroupVector.size() - 1; i++)
        {
          float offset = (guidepostGroupVector[i].x - m_x_1 - selfcar_px) * (guidepostGroupVector[i].x - m_x_1 - selfcar_px) + 
                      (guidepostGroupVector[i].y - m_y_1 - selfcar_py) * (guidepostGroupVector[i].y - m_y_1 - selfcar_py);
          if (offset <= offset_min) {
            offset_min = offset;
            cur_guidepost_group_mark = i;
          }
        }
        uint nex_guidepostGroupMark = (cur_guidepost_group_mark + 1) % (guidepostGroupVector.size()-0);
        const double next_world_guidepost_px = guidepostGroupVector[nex_guidepostGroupMark].x;
        const double next_world_guidepost_py = guidepostGroupVector[nex_guidepostGroupMark].y; 
          
        perception_guidepost_target_px = (next_world_guidepost_px - selfcar_px)*(cos(selfcar_theta)) + (next_world_guidepost_py - selfcar_py)*(sin(selfcar_theta));
        perception_guidepost_target_py = (next_world_guidepost_px - selfcar_px)*(-sin(selfcar_theta)) + (next_world_guidepost_py - selfcar_py)*(cos(selfcar_theta));
        target_s = perception_guidepost_target_px;
        */
        // add by shzhw
          /*           
        {
          std::ofstream ekf_out;
          ekf_out.open("ekf_result_out.txt", std::ios::out | std::ios::app);
          ekf_out << std::fixed << local_view_.chassis->header().timestamp_sec() << "\t" << selfcar_px2 << "\t" << selfcar_py2 <<  "\t"<< selfcar_theta2  << "\t" << local_view_.routing->routing_request().waypoint().rbegin()->id()<<  std::endl;
          //ekf_out<< perception_guidepost_target_px << "\t" << perception_guidepost_target_py<< std::endl;
          ekf_out.close();
          }
          */

      } 

      if (local_view_.routing->routing_request().is_use_guidepost()) { 
        static int guidepost_num = 0;

         if (FLAGS_gudiepost_method == 2) {
          perception_guidepost_target_px = perception_point.x();
          perception_guidepost_target_py = perception_point.y();        
          target_s = perception_guidepost_target_px;
        } else if (FLAGS_gudiepost_method == 1) {

          Eigen::Matrix2d rote_mat; // pre cor1 to cur cor2
          rote_mat << cos(perception_guidepost_turn_angle), sin(perception_guidepost_turn_angle), 
                      -sin(perception_guidepost_turn_angle), cos(perception_guidepost_turn_angle);

          //TODO, receive p1, p2, P1, P2

          //AERROR << "startPostID:" << startPostID 
          //  << ",pt1_x:" << perception_point.x() << ",pt1_y:" << perception_point.y();




          
          if(reset_flag) {
            
            // AERROR << "cur_guidepost: " << cur_guidepost << ", startPostID: " << startPostID;
            // AERROR << "guidepost_num: " << guidepost_num;
            if (cur_guidepost != startPostID) {
              guidepost_num = 0;
              const Eigen::Vector2d shift_mat = {-pre_drive_length_guidepost *cos(perception_guidepost_turn_angle), -pre_drive_length_guidepost *sin(perception_guidepost_turn_angle)};
              perception_guidepost_points[0] = rote_mat * (perception_guidepost_points[1] + shift_mat);
            // AERROR << "pre_drive_length_guidepost: " << pre_drive_length_guidepost;            
              world_guidepost_points[0] = world_guidepost_points[1];
              world_guidepost_points[1] = {currentGuidepostGroup.x,currentGuidepostGroup.y};
            } else {
              guidepost_num ++;
              const Eigen::Vector2d shift_mat = {-drive_length_guidepost *cos(perception_guidepost_turn_angle), -drive_length_guidepost *sin(perception_guidepost_turn_angle)};
              perception_guidepost_points[0] = rote_mat * (perception_guidepost_points[0] + shift_mat);      
            }
            perception_guidepost_points[1] = {perception_point.x(),perception_point.y()}; 
          //  const Eigen::Vector2d shift_mat = {-drive_length_guidepost *cos(perception_guidepost_turn_angle), -drive_length_guidepost *sin(perception_guidepost_turn_angle)};
          //  Eigen::Vector2d p1_cor2 = rote_mat * perception_guidepost_points[0] + shift_mat;
          
            Eigen::Matrix2d R;
            Eigen::Vector2d S;
            double K;
            rotate_shift(world_guidepost_points[0], world_guidepost_points[1], 
                        perception_guidepost_points[0], perception_guidepost_points[1], R, S, &K);
            uint next_guidepostGroupMark =  (guidepostGroupMark+1) % (guidepostGroupVector.size()-0);
                  
            Eigen::Vector2d world_target_point = 
                        {guidepostGroupVector[next_guidepostGroupMark].x, guidepostGroupVector[next_guidepostGroupMark].y};

            Eigen::Vector2d car_target_point = R*world_target_point*K +S;
            
            perception_guidepost_target_px = car_target_point(0);
            perception_guidepost_target_py = car_target_point(1);            
            target_s = perception_guidepost_target_px;

            // AERROR << "p1_cor2 : " << p1_cor2;
          }
          AERROR << "perception_guidepost_turn_angle: " << perception_guidepost_turn_angle;
          AERROR << "world_guidepost_points[0]:" <<world_guidepost_points[0];
          AERROR << "world_guidepost_points[1]:" <<world_guidepost_points[1];
          AERROR << "perception_guidepost_points[0]:" <<perception_guidepost_points[0];
          AERROR << "perception_guidepost_points[1]:" <<perception_guidepost_points[1];
         
        }
        cur_guidepost = startPostID; //to avoid reset_flag when same id
        AERROR << "startPostID:" << startPostID << ", car_target_point:" << perception_guidepost_target_px << "," << perception_guidepost_target_py;

        // TODO  
        double sin_steering_angle = 2*3.34*perception_guidepost_target_py / 
                        (perception_guidepost_target_px*perception_guidepost_target_px+perception_guidepost_target_py*perception_guidepost_target_py);
        if(std::isnan(sin_steering_angle)){
          sin_steering_angle = 0.0;
        }
        if (sin_steering_angle < -1.0) {
          sin_steering_angle = -1.0;
        } else if(sin_steering_angle > 1.0) {
          sin_steering_angle =1.0;
        }  

        if((FLAGS_gudiepost_method == 1 && guidepost_num <= 1) || (FLAGS_gudiepost_method >=2)) { //modified 20200617
        double steering_angle = asin(sin_steering_angle);      

        perception_guidepost_turn_angle = steering_angle; //pre cycle turn angle

        common::VehicleParam vehicle_param_ =
        common::VehicleConfigHelper::GetConfig().vehicle_param();
        const double steer_ratio = vehicle_param_.steer_ratio();
        const double steer_single_direction_max_degree =
        vehicle_param_.max_steer_angle() ;/// M_PI * 180;

        steering_angle = steering_angle  * steer_ratio / steer_single_direction_max_degree * 100.0; /// M_PI *180.0
        perception_guidepost_steering_target = common::math::Clamp(steering_angle, -100.0, 100.0);
        }

        if (FLAGS_gudiepost_method <= 3) {        
          adc_trajectory_pb.set_is_use_routing(true);
          adc_trajectory_pb.mutable_simple_control_cmd()->set_throttle(FLAGS_gudiepost_curise_throttle);
          adc_trajectory_pb.mutable_simple_control_cmd()->set_steering_target(perception_guidepost_steering_target);
        }

        if (adc_trajectory_pb.header().sequence_num() < 100 && drive_length_guidepost > 2e8) {
          drive_length_guidepost = 0.0;
        }   
        
      } else {
      //adc_trajectory_pb.set_is_use_routing(false);
      }
      // near end guidepost id, reduce speed
      if (std::abs(std::stoi(startPostID) - std::stoi(end_guidepost_id)) <= 1) {
        adc_trajectory_pb.mutable_simple_control_cmd()->set_throttle(FLAGS_gudiepost_slow_throttle);
      } 

      // near obstacles, reduce speed or stop ,TODO
      // int k=0;
      for (auto &prediction_ptr :
          Obstacle::CreateObstacles(*local_view_.prediction_obstacles)) {
        const auto posex = prediction_ptr->Perception().position().x();
        const auto posey = prediction_ptr->Perception().position().y();

        if (posex > -1.0 && posex < (10.0 + 5.0) && std::fabs(posey) < 1.5) {
          adc_trajectory_pb.mutable_simple_control_cmd()->set_throttle(FLAGS_gudiepost_slow_throttle);
          if (posex < (3.0 + 5.0)) {
            adc_trajectory_pb.mutable_simple_control_cmd()->set_throttle(0);
            adc_trajectory_pb.mutable_simple_control_cmd()->set_brake(60);
            break;
          } else {
          break;
          }
        }
      //  k++;
      }

      // arrive end guidepost id, stop   TODO (read front_edge_to_center from config)
      if (FLAGS_gudiepost_method == 1 || FLAGS_gudiepost_method == 2 || 
                  (FLAGS_gudiepost_method ==3 && !ekf_per_chassis)) {
        if (startPostID == end_guidepost_id && drive_length_guidepost >= (target_s - 5.0) ) {
          adc_trajectory_pb.mutable_simple_control_cmd()->set_throttle(0);
          adc_trajectory_pb.mutable_simple_control_cmd()->set_brake(61);
         is_stop = true;          
        } 
      } else if ((FLAGS_gudiepost_method ==3) && ekf_per_chassis) {
        if ((cur_guidepost_group_mark - std::stoi(end_guidepost_id)== 0) && drive_length_guidepost >= (target_s - 6.0)) {
          adc_trajectory_pb.mutable_simple_control_cmd()->set_throttle(0);
          adc_trajectory_pb.mutable_simple_control_cmd()->set_brake(61);
         is_stop = true;
        }
      }

      if (local_view_.routing->routing_request().is_use_guidepost() && drive_length_guidepost > 5.5) {
        adc_trajectory_pb.mutable_simple_control_cmd()->set_throttle(0);
        adc_trajectory_pb.mutable_simple_control_cmd()->set_brake(62);
        is_stop = true;
       
      } else if (drive_length_guidepost > 7.0) {
        adc_trajectory_pb.mutable_simple_control_cmd()->set_throttle(0);
        adc_trajectory_pb.mutable_simple_control_cmd()->set_brake(63);
        is_stop = true;
       
      }


      // add by shzhw
      /*     
      {
        std::ofstream planning_out;
        planning_out.open("palnning_result_out.txt", std::ios::out | std::ios::app);
        planning_out << std::fixed <<adc_trajectory_pb.header().timestamp_sec() << "\t" <<  perception_guidepost_target_px << "\t" << perception_guidepost_target_py<< "\t" 
            << adc_trajectory_pb.mutable_simple_control_cmd()->throttle() << "\t" << adc_trajectory_pb.mutable_simple_control_cmd()->brake() << "\t" << adc_trajectory_pb.mutable_simple_control_cmd()->steering_target()  
            << "\t" << drive_length_guidepost << "\t" << "cur_guidepost_group_mark:" <<cur_guidepost_group_mark
            << "\t" << std::stoi(end_guidepost_id) << std::endl;
        planning_out.close();
      }
      */
      

    }
  }
  
  
  //--------------------------------------------------

  common::util::FillHeader(node_->Name(), &adc_trajectory_pb);

  // modify trajectory relative time due to the timestamp change in header
  const double dt = start_time - adc_trajectory_pb.header().timestamp_sec();
  for (auto& p : *adc_trajectory_pb.mutable_trajectory_point()) {
    p.set_relative_time(p.relative_time() + dt);
  }
  planning_writer_->Write(adc_trajectory_pb);

  //cyber::common::SetProtoToASCIIFile(adc_trajectory_pb,"adc_traj_1.pb.txt");//add by shzhw to save adc trajectory

  // record in history
  auto* history = History::Instance();
  history->Add(adc_trajectory_pb);

  return true;
}

void PlanningComponent::CheckRerouting() {
  auto* rerouting = PlanningContext::Instance()
                        ->mutable_planning_status()
                        ->mutable_rerouting();
  if (!rerouting->need_rerouting()) {
    return;
  }
  common::util::FillHeader(node_->Name(), rerouting->mutable_routing_request());
  rerouting->set_need_rerouting(false);
  rerouting_writer_->Write(rerouting->routing_request());
}

bool PlanningComponent::CheckInput() {
  ADCTrajectory trajectory_pb;
  auto* not_ready = trajectory_pb.mutable_decision()
                        ->mutable_main_decision()
                        ->mutable_not_ready();

  if (local_view_.localization_estimate == nullptr) {
    not_ready->set_reason("localization not ready");
  } else if (local_view_.chassis == nullptr) {
    not_ready->set_reason("chassis not ready");
  } else if (HDMapUtil::BaseMapPtr() == nullptr) {
    // not_ready->set_reason("map not ready");// delete by shzhw
  } else {
    // nothing
  }

  if (FLAGS_use_navigation_mode) {
  //  if (!local_view_.routing->routing_request().is_use_guidepost()) { // add by shzhw
      if (!local_view_.relative_map->has_header()) {
        not_ready->set_reason("relative map not ready");
      }
  //  }
  } else {
    if (!local_view_.routing->has_header()) {
      not_ready->set_reason("routing not ready");
    }
  }

  if (not_ready->has_reason()) {
    AERROR << not_ready->reason() << "; skip the planning cycle.";
    common::util::FillHeader(node_->Name(), &trajectory_pb);
    planning_writer_->Write(trajectory_pb);
    return false;
  }
  return true;
}

}  // namespace planning
}  // namespace apollo
