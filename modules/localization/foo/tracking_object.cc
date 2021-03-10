/******************************************************************************

 *****************************************************************************/


#include "modules/localization/foo/tracking_object.h"
#include "modules/common/configs/vehicle_config_helper.h"
#include "cyber/common/file.h"
#include <vector>
#include <list>
#include <memory>
#include <utility>
#include <algorithm>
#include "modules/common/math/quaternion.h"

namespace apollo {
namespace localization {

//using apollo::common::Status;
using apollo::common::INSreEKF;
using apollo::common::GuidepostEKF;

apollo::common::KalmanFilterInterface * TrackingSelfCarObject::CreateINSreKalmanFilter(int kf_coor_sys){
  ins_kalman_filter_ = new INSreEKF(kf_coor_sys);
  return ins_kalman_filter_;
}
apollo::common::KalmanFilterInterface * TrackingSelfCarObject::CreateGuidepostKalmanFilter(int kf_coor_sys){
  guidepost_kalman_filter_ = new GuidepostEKF(kf_coor_sys);
  return guidepost_kalman_filter_;
}
/**/
int TrackingSelfCarObject::kf_coor_sys_ = 0;

bool TrackingSelfCarObject::InitKalmanFilter(bool ins_localization_valid, NewLocalView &local_view,
      const std::vector<NewGuidepostGroup> &guideposts) {
  // init state of ekf
  cv::Point2d init_pose;
  double init_theta = 0.0;

  if (ins_localization_valid && 
            local_view.localization_estimate && local_view.localization_estimate->has_pose()) {
      init_pose.x = local_view.localization_estimate->pose().position().x();
      init_pose.y = local_view.localization_estimate->pose().position().y();
      init_theta = local_view.localization_estimate->pose().heading();   
  } else if(local_view.routing && local_view.routing->has_routing_request()) {
    if (guideposts.size() <=0 ) {
      return false;
    }
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
  } else {
    return false;
  }

  INSreEKF *ins_ekf = dynamic_cast<INSreEKF *>(ins_kalman_filter_);
  ins_ekf->Init(init_pose, init_theta);
  GuidepostEKF *guidepost_ekf = dynamic_cast<GuidepostEKF *>(guidepost_kalman_filter_);
  guidepost_ekf->Init(init_pose, init_theta);

  AERROR << "init_pose: " << init_pose.x << "\t" << init_pose.y << "\n";

  return true;
}

void TrackingSelfCarObject::KalmanFilterPredict(NewLocalView &local_view) {
  INSreEKF *ins_ekf = dynamic_cast<INSreEKF *>(ins_kalman_filter_);
  GuidepostEKF *guidepost_ekf = dynamic_cast<GuidepostEKF *>(guidepost_kalman_filter_);
  #if 1
  //if (local_view.chassis.has_header() && local_view.chassis.header().has_timestamp_sec()) {

    static double pre_timestamp = local_view.chassis->header().timestamp_sec();
    const double cur_timestamp = local_view.chassis->header().timestamp_sec();
  //}
  #else
 // static double pre_timestamp = local_view.routing->routing_request().header().timestamp_sec();
  //const double cur_timestamp = local_view.routing->routing_request().header().timestamp_sec();
  #endif
  double dt = static_cast<double>(cur_timestamp - pre_timestamp);
  const double speed_ms = static_cast<double>(local_view.chassis->speed_mps());
  const double steering_percentage = static_cast<double>(local_view.chassis->steering_percentage());
  double ds = dt * speed_ms;
  common::VehicleParam vehicle_param_ =
  common::VehicleConfigHelper::GetConfig().vehicle_param();
  const double steer_ratio = vehicle_param_.steer_ratio();
  const double steer_single_direction_max_degree =
  vehicle_param_.max_steer_angle() ;/// M_PI * 180;
  double steer_theta = steering_percentage / 100.0 *steer_single_direction_max_degree / steer_ratio;

  double steer_radius = vehicle_param_.wheel_base() / tan(steer_theta);
  double dtheta = ds / steer_radius;

  cv::Mat ctrl(4, 1, CV_64FC1);

  ctrl.at<double>(0, 0) = dt;
  ctrl.at<double>(1, 0) = ds;
  ctrl.at<double>(2, 0) = dtheta;
  ctrl.at<double>(3, 0) = steer_radius;

  const cv::Mat &predict = ins_ekf->Predict(ctrl);
    ins_ekf->state_pre_.at<double>(2,0) = common::math::NormalizeAngle(ins_ekf->state_pre_.at<double>(2,0));

  std::cout<< ins_ekf->state_pre_.at<double>(0,0) <<", " <<  ins_ekf->state_pre_.at<double>(1,0) <<", " <<  ins_ekf->state_pre_.at<double>(2,0) <<std::endl ;
  guidepost_ekf->state_pre_ = ins_ekf->state_pre_.clone();
  guidepost_ekf->state_post_ = ins_ekf->state_post_.clone();
  guidepost_ekf->error_cov_pre_ = ins_ekf->error_cov_pre_.clone();
  guidepost_ekf->error_cov_post_ = ins_ekf->error_cov_post_.clone();

  pre_timestamp  = cur_timestamp;
  if (predict.at<double>(0,0)) {}
}


void TrackingSelfCarObject::KalmanFilterGuidepostCorrect(NewLocalView &local_view,
         const std::vector<NewGuidepostGroup> &guideposts) {

  GuidepostEKF *guidepost_ekf = dynamic_cast<GuidepostEKF *>(guidepost_kalman_filter_);
  
  cv::Mat mesurement = cv::Mat(2, 1, CV_64FC1);
  mesurement.at<double>(0, 0) = local_view.routing->routing_request().waypoint().rbegin()->pose().x();
  mesurement.at<double>(1, 0) = local_view.routing->routing_request().waypoint().rbegin()->pose().y();

  // to mathced routing map
  const double cur_px =  guidepost_ekf->state_pre_.at<double>(0,0);
  const double cur_py =  guidepost_ekf->state_pre_.at<double>(1,0);
  const double cur_theta = guidepost_ekf->state_pre_.at<double>(2,0);
  double m_x_1 = mesurement.at<double>(0, 0)*(cos(-cur_theta)) + mesurement.at<double>(1, 0)*(sin(-cur_theta));
  double m_y_1 = mesurement.at<double>(0, 0)*(-sin(-cur_theta)) + mesurement.at<double>(1, 0)*(cos(-cur_theta));
  uint cur_guidepostGroupMark = 0;
  double offset_min = 99999.0;
  for(uint i = 0; i < guideposts.size() - 1; i++)
  {
    double offset = (guideposts[i].x - m_x_1 - cur_px) * (guideposts[i].x - m_x_1 - cur_px) + 
                (guideposts[i].y - m_y_1 - cur_py) * (guideposts[i].y - m_y_1 - cur_py);
    if (offset <= offset_min) {
      offset_min = offset;
      cur_guidepostGroupMark = i;
    }
  }

  // if offset distance is too larget, virtual guidepsot, skill correct
  #if 1
  if (offset_min > 10.0) {
     AERROR << "no matched the approprite guidepost mark!";    
     //return;
  }
  #endif
  //guidepost_ekf->SetGuidepostIndex(cur_guidepostGroupMark);
  cur_perception_guidepost_index_ = cur_guidepostGroupMark;
  cv::Mat extra_measure = cv::Mat(2, 1, CV_64FC1);
  extra_measure.at<double>(0,0) = guideposts[cur_guidepostGroupMark].x;
  extra_measure.at<double>(1,0) = guideposts[cur_guidepostGroupMark].y;

  const cv::Mat &correct = guidepost_ekf->Correct(mesurement, extra_measure);

  // AERROR << "correct:pre state: " << cur_px << ", " << cur_py << ", " << cur_theta << 
  //   "extra_measure: " << cur_guidepostGroupMark << ", " << extra_measure.at<double>(0,0) << ", " <<extra_measure.at<double>(1,0) <<
  //     "correct state: "<<correct.at<double>(0,0) << ", " << correct.at<double>(1,0) << ", " << correct.at<double>(2,0) << "\t";

  INSreEKF *ins_ekf = dynamic_cast<INSreEKF *>(ins_kalman_filter_);
  ins_ekf->state_pre_ = guidepost_ekf->state_pre_.clone();
  ins_ekf->state_post_ = guidepost_ekf->state_post_.clone();
  ins_ekf->error_cov_pre_ = guidepost_ekf->error_cov_pre_.clone();
  ins_ekf->error_cov_post_ = guidepost_ekf->error_cov_post_.clone();

  if (correct.at<double>(0,0)) {}


  /*
  if (!StateToRect(cameras, correct, tf->rect_im_sta_, tf->rect_gnd_sta_)) {
    delete kalman_filter_;
    kalman_filter_ = NULL;
  }
  */
}

void TrackingSelfCarObject::KalmanFilterINSCorrect(NewLocalView &local_view) {

  INSreEKF *ins_ekf = dynamic_cast<INSreEKF *>(ins_kalman_filter_);
  
  //cv::Mat mesurement = cv::Mat(3, 1, CV_64FC1);
  cv::Mat mesurement = cv::Mat(2, 1, CV_64FC1); // ingero heading, test
  mesurement.at<double>(0, 0) = local_view.localization_estimate->pose().position().x();
  mesurement.at<double>(1, 0) = local_view.localization_estimate->pose().position().y();
  //mesurement.at<double>(2, 0) = local_view.localization_estimate->pose().heading();

  cv::Mat extra_measure = cv::Mat(2, 1, CV_64FC1);

  double dev_x =  local_view.localization_estimate->uncertainty().position_std_dev().x();
  double dev_y =  local_view.localization_estimate->uncertainty().position_std_dev().y();
  double dev_heading =  local_view.localization_estimate->uncertainty().heading_std_dev();
  //if (local_view.chassis->steering_percentage() > 10.0) {
  //  ins_ekf->SetMeasureNoise(20*dev_x, 20*dev_y, 2*dev_heading);
 // } else {
    ins_ekf->SetMeasureNoise(100*dev_x, 100*dev_y, 2*dev_heading);
  //}
  
  const cv::Mat &correct = ins_ekf->Correct(mesurement, extra_measure);

  ins_ekf->state_post_.at<double>(2,0) = common::math::NormalizeAngle(ins_ekf->state_post_.at<double>(2,0));

  // AERROR << "correct:pre state: " << cur_px << ", " << cur_py << ", " << cur_theta << 
  //   "extra_measure: " << cur_guidepostGroupMark << ", " << extra_measure.at<double>(0,0) << ", " <<extra_measure.at<double>(1,0) <<
  //     "correct state: "<<correct.at<double>(0,0) << ", " << correct.at<double>(1,0) << ", " << correct.at<double>(2,0) << "\t";

  GuidepostEKF *guidepost_ekf = dynamic_cast<GuidepostEKF *>(guidepost_kalman_filter_);
  guidepost_ekf->state_pre_ = ins_ekf->state_pre_.clone();
  guidepost_ekf->state_post_ = ins_ekf->state_post_.clone();
  guidepost_ekf->error_cov_pre_ = ins_ekf->error_cov_pre_.clone();
  guidepost_ekf->error_cov_post_ = ins_ekf->error_cov_post_.clone();

  if (correct.at<double>(0,0)) {}

}


}  // namespace planning
}  // namespace apollo
