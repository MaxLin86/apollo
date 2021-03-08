/******************************************************************************

 *****************************************************************************/


#include "modules/planning/tracking_object.h"
//#include "modules/common/filters/extended_kalman_filter.h"
//#include "modules/common/filters/kalman_filter.h"
#include "modules/common/filters/tracking_obj_ekf.h"
#include "modules/common/configs/vehicle_config_helper.h"
#include "cyber/common/file.h"
#include <vector>
#include <list>
#include <memory>
#include <utility>
#include <algorithm>

namespace apollo {
namespace planning {

//using apollo::common::Status;
using apollo::common::SelfCarEKF;

apollo::common::KalmanFilterInterface * TrackingSelfCarObject::CreateKalmanFilter(int kf_coor_sys){
  kalman_filter_ = new SelfCarEKF(kf_coor_sys);
  return kalman_filter_;
}
/**/
int TrackingSelfCarObject::kf_coor_sys_ = 1;
bool TrackingSelfCarObject::InitKalmanFilter(LocalView &local_view,
      const std::vector<GuidepostGroup> &guideposts) {
  // init state of ekf
  SelfCarEKF *ekf = dynamic_cast<SelfCarEKF *>(kalman_filter_);
  cv::Point2f init_pose;
  double init_theta = 0.0;
  if (kf_coor_sys_ == 0) {
    init_pose.x = local_view.localization_estimate->pose().position().x();
    init_pose.y = local_view.localization_estimate->pose().position().y();
    init_theta = local_view.localization_estimate->pose().heading();

  } else if (kf_coor_sys_ == 1) { // TODO
    double perception_px = local_view.routing->routing_request().waypoint().rbegin()->pose().x();
    double perception_py = local_view.routing->routing_request().waypoint().rbegin()->pose().y();
    const std::string start_guidepost_id = local_view.routing->routing_request().waypoint().rbegin()->id();
    
    uint guidepostGroupMark = 0;
    if (!start_guidepost_id.empty()) {
      for(uint i = 0; i < guideposts.size() - 1; i++)
      {
        if (guideposts[i].id == start_guidepost_id) {
          guidepostGroupMark = i;
          break;
        }
      }
      double start_guidepost_px = guideposts[guidepostGroupMark].x;
      double start_guidepost_py = guideposts[guidepostGroupMark].y;
      double next_guidepost_px = start_guidepost_px;
      double next_guidepost_py = start_guidepost_py;
      if (guidepostGroupMark +1 < guideposts.size() - 1) {
        next_guidepost_px = guideposts[guidepostGroupMark+1].x;
        next_guidepost_py = guideposts[guidepostGroupMark+1].y;
      }
      if (std::fabs(next_guidepost_px - start_guidepost_px) > 1e-6) {
        init_theta = atan((next_guidepost_py-start_guidepost_py)/(next_guidepost_px-start_guidepost_px));
        if(next_guidepost_px < start_guidepost_px) {
          if (next_guidepost_py >= start_guidepost_py) {
            init_theta = init_theta + M_PI;
          } else {
            init_theta = init_theta - M_PI;
          }
        }        
      } else {
        if (std::fabs(next_guidepost_py - start_guidepost_py) < 1e-6) {
          init_theta = 0.0;
        }else if (next_guidepost_py > start_guidepost_py) {
          init_theta = M_PI_2;
        } else {
          init_theta = -M_PI_2;
        }
      }
      double perception_px_t = perception_px*(cos(-init_theta)) + perception_py*(sin(-init_theta));
      double perception_py_t = perception_px*(-sin(-init_theta)) + perception_py*(cos(-init_theta));
      init_pose.x = start_guidepost_px - perception_px_t; // TODO
      init_pose.y = start_guidepost_py - perception_py_t;

      //AERROR << "init:perception pose: " << perception_px << ", " << perception_py <<  ", " << start_guidepost_id << ", ground pose: " <<
      //  start_guidepost_px << ", " << start_guidepost_py << ", " << ", init state: " << init_pose.x << ", " << init_pose.y << ", " << init_theta << "\t";
    } else {
      return false;
    } 
  }
  ekf->Init(init_pose, init_theta);

  /*
  float dt = 0.0f;
  cv::Mat ctrl(4, 1, CV_32FC1);
  ctrl.at<float>(0, 0) = dt / 1000.0f;
  ctrl.at<float>(1, 0) = 0.0f;
  ctrl.at<float>(2, 0) = 0.0f;
  const cv::Mat &predict = ekf->Predict(ctrl);
  */
  return true;
}

void TrackingSelfCarObject::KalmanFilterPredict(LocalView &local_view) {
  SelfCarEKF *ekf = dynamic_cast<SelfCarEKF *>(kalman_filter_);
  #if 1
  //if (local_view.chassis.has_header() && local_view.chassis.header().has_timestamp_sec()) {
    
    static double pre_timestamp = local_view.chassis->header().timestamp_sec();
    const double cur_timestamp = local_view.chassis->header().timestamp_sec();
  //}
  
#else
 // static double pre_timestamp = local_view.routing->routing_request().header().timestamp_sec();
  //const double cur_timestamp = local_view.routing->routing_request().header().timestamp_sec();
  #endif
  float dt = static_cast<float>(cur_timestamp - pre_timestamp);
  const float speed_ms = static_cast<float>(local_view.chassis->speed_mps());
  const float steering_percentage = static_cast<float>(local_view.chassis->steering_percentage());
  float ds = dt * speed_ms;
  common::VehicleParam vehicle_param_ =
  common::VehicleConfigHelper::GetConfig().vehicle_param();
  const double steer_ratio = vehicle_param_.steer_ratio();
  const double steer_single_direction_max_degree =
  vehicle_param_.max_steer_angle() ;/// M_PI * 180;
  float steer_theta = steering_percentage / 100.0 *steer_single_direction_max_degree / steer_ratio;

  float steer_radius = 3.34 / sin(steer_theta);
  float dtheta = ds / steer_radius;

  cv::Mat ctrl(4, 1, CV_32FC1);

  ctrl.at<float>(0, 0) = dt;
  ctrl.at<float>(1, 0) = ds;
  ctrl.at<float>(2, 0) = dtheta;
  ctrl.at<float>(3, 0) = steer_radius;

  const cv::Mat &predict = ekf->Predict(ctrl);
  pre_timestamp  = cur_timestamp;
  if (predict.at<float>(0,0)) {}
}


void TrackingSelfCarObject::KalmanFilterCorrect(LocalView &local_view,
         const std::vector<GuidepostGroup> &guideposts) {

  SelfCarEKF *ekf = dynamic_cast<SelfCarEKF *>(kalman_filter_);
  
  cv::Mat mesurement = cv::Mat(2, 1, CV_32FC1);
  mesurement.at<float>(0, 0) = local_view.routing->routing_request().waypoint().rbegin()->pose().x();
  mesurement.at<float>(1, 0) = local_view.routing->routing_request().waypoint().rbegin()->pose().y();

  // to mathced routing map
  const float cur_px =  ekf->state_pre_.at<float>(0,0);
  const float cur_py =  ekf->state_pre_.at<float>(1,0);
  const float cur_theta = ekf->state_pre_.at<float>(2,0);
  float m_x_1 = mesurement.at<float>(0, 0)*(cos(-cur_theta)) + mesurement.at<float>(1, 0)*(sin(-cur_theta));
  float m_y_1 = mesurement.at<float>(0, 0)*(-sin(-cur_theta)) + mesurement.at<float>(1, 0)*(cos(-cur_theta));
  uint cur_guidepostGroupMark = 0;
  float offset_min = 99999.0;
  for(uint i = 0; i < guideposts.size() - 1; i++)
  {
    float offset = (guideposts[i].x - m_x_1 - cur_px) * (guideposts[i].x - m_x_1 - cur_px) + 
                (guideposts[i].y - m_y_1 - cur_py) * (guideposts[i].y - m_y_1 - cur_py);
    if (offset <= offset_min) {
      offset_min = offset;
      cur_guidepostGroupMark = i;
    }
  }

  // if offset distance is too larget, virtual guidepsot, skill correct
  #if 0
  if (offset_min > 3.0) {
    return;
  }
  #endif

  cv::Mat extra_measure = cv::Mat(2, 1, CV_32FC1);
  extra_measure.at<float>(0,0) = guideposts[cur_guidepostGroupMark].x;
  extra_measure.at<float>(1,0) = guideposts[cur_guidepostGroupMark].y;

  const cv::Mat &correct = ekf->Correct(mesurement, extra_measure);

  // AERROR << "correct:pre state: " << cur_px << ", " << cur_py << ", " << cur_theta << 
  //   "extra_measure: " << cur_guidepostGroupMark << ", " << extra_measure.at<float>(0,0) << ", " <<extra_measure.at<float>(1,0) <<
  //     "correct state: "<<correct.at<float>(0,0) << ", " << correct.at<float>(1,0) << ", " << correct.at<float>(2,0) << "\t";


  if (correct.at<float>(0,0)) {}


  /*
  if (!StateToRect(cameras, correct, tf->rect_im_sta_, tf->rect_gnd_sta_)) {
    delete kalman_filter_;
    kalman_filter_ = NULL;
  }
  */
}




}  // namespace planning
}  // namespace apollo
