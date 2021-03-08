//
//

#include "modules/common/filters/ins_guidepost_fusion_ekf.h"
#include <vector>
#include <algorithm>

namespace apollo {
namespace common {


// measurement is valid in world coordinate, INSGuideEKF
// is calculated on ground.
INSreEKF::INSreEKF(int kf_coor_sys)
  :kf_coordinate_sys_(kf_coor_sys) {
  correct_count_ = 0;
  init_noise_ = 1.0;
  process_noise_ = 3.0;
  measure_noise_ = 0.08;
}

INSreEKF::~INSreEKF() {
}

void INSreEKF::Init(cv::Point2d init_pose, double init_theta) {
  predict_count_ = 0;
  correct_count_ = 0;
  re_init_ = false;
  // for ekf params
  // for ins measure, measure_params=3
  int dynam_params = 3;
  int measure_params = 2;//3-----------------------------
  int control_params = 1;
  int type = CV_64FC1;
  ExtendedKalmanFilter::Init(dynam_params, measure_params,
    control_params, type);

  // measurement_matrix_ will not use
  measurement_matrix_.release();

  // control_matrix_ will not use
  control_matrix_.release();

  // init process_noise_cov_
  cv::setIdentity(process_noise_cov_, cv::Scalar::all(1.0));
  cv::setIdentity(measurement_noise_cov_, cv::Scalar::all(1.0));

  if (kf_coordinate_sys_ == 0) {  // world gnd

  } else if (kf_coordinate_sys_ == 1) {  // car gnd    

  }
  state_post_.at<double>(0, 0) = init_pose.x;  // x, m
  state_post_.at<double>(1, 0) = init_pose.y;    // y, m
  state_post_.at<double>(2, 0) = init_theta;    // theta

  state_post_.copyTo(state_pre_);
  // init error_cov_post_, 3x3
  SetErrorCov();
  error_cov_post_.copyTo(error_cov_pre_);
}

void INSreEKF::SetErrorCov() {
  error_cov_post_.setTo(cv::Scalar::all(0.0));
  error_cov_post_.at<double>(0, 0) = 1.0;   // x, 0.1m
  error_cov_post_.at<double>(1, 1) = 1.0;   // y
  error_cov_post_.at<double>(2, 2) = 1.0;  // theta
}

void INSreEKF::SetMeasureNoise(double dev_posx, double dev_posy,double dev_heading) {
  measure_dev_posx_ = dev_posx;
  measure_dev_posy_ = dev_posy;
  measure_dev_heading_ = dev_heading;
}

const cv::Mat& INSreEKF::Predict(const cv::Mat& control) {
  //if (predict_count_ == 0) {
    ExtendedKalmanFilter::Predict(control);
  //}
  predict_count_++;
  return state_pre_;
}

const cv::Mat& INSreEKF::Correct(const cv::Mat& measurement,
  cv::Mat& extra_measurement) {
  // measure is (localization_px, localization_py, localization_theta)
  correct_count_++;
  const cv::Mat &mcorrect =
    ExtendedKalmanFilter::Correct(measurement, extra_measurement);
  predict_count_ = 0;

  return mcorrect;
}

void INSreEKF::Transition(const cv::Mat &state_post,
  cv::Mat &state_pre,
  const cv::Mat &control) {
  // x(k) = f( x(k-1), u(k-1) )
  //double dt = control.at<double>(0, 0);
  double ds = control.at<double>(1, 0);
  double dtheta = control.at<double>(2, 0);
  double steering_radius = control.at<double>(3, 0);

  state_pre.at<double>(0,0) = state_post.at<double>(0,0) + ds*cos(-dtheta/2 - state_post.at<double>(2,0));
  state_pre.at<double>(1,0) = state_post.at<double>(1,0) - ds*sin(-dtheta/2 - state_post.at<double>(2,0));
  state_pre.at<double>(2,0) = state_post.at<double>(2,0) + ds/steering_radius;
}

void INSreEKF::JacobianTrasition(cv::Mat &jacobian_trans,
  const cv::Mat &control, const cv::Mat &state_post) {
  //double dt = control.at<double>(0, 0);
  double ds = control.at<double>(1, 0);
  double dtheta = control.at<double>(2, 0);

  cv::setIdentity(jacobian_trans);

  jacobian_trans.at<double>(0, 2) =ds*sin(-dtheta/2 - state_post.at<double>(2,0));
  jacobian_trans.at<double>(1, 2) = ds*cos(-dtheta/2 - state_post.at<double>(2,0));
}

void INSreEKF::Measurement(const cv::Mat &state_pre,
  cv::Mat &measuremet, cv::Mat &extra_measuremet) {
  // ground coordinate
  double x = state_pre_.at<double>(0, 0);
  double y = state_pre_.at<double>(1, 0);
  //double theta = state_pre_.at<double>(2, 0);//------------------------------

  if (kf_coordinate_sys_ == 0) {

  } else if (kf_coordinate_sys_ == 1) {

  }
  measuremet.at<double>(0, 0) = x;
  measuremet.at<double>(1, 0) = y;
  //measuremet.at<double>(2,0) = theta; //-----------------
}


void INSreEKF::JacobianMeasurement(const cv::Mat &state_pre,
  cv::Mat &jacobian_measurement, cv::Mat &extra_measuremet) {

  if (kf_coordinate_sys_ == 0) {

  } else if (kf_coordinate_sys_ == 1) {

  }
  cv::setIdentity(jacobian_measurement);

}


// set noise positive
void INSreEKF::JacobianProcessNoise(cv::Mat &jacobian_process_noise,
  const cv::Mat &control) {
  double dt = control.at<double>(0, 0);
  
  if (kf_coordinate_sys_ == 0) {

    } else if (kf_coordinate_sys_ == 1) {

  }
  jacobian_process_noise.setTo(cv::Scalar::all(0.0));
  jacobian_process_noise.at<double>(0,0) = dt/5.0;
  jacobian_process_noise.at<double>(1,1) = dt/5.0;
  jacobian_process_noise.at<double>(2,2) = 1.0/2.0*dt;  
}

void INSreEKF::JacobianMeasurementNoise(cv::Mat &jacobian_measurement_noise) {
  jacobian_measurement_noise.setTo(cv::Scalar::all(0.0));
  jacobian_measurement_noise.at<double>(0,0) =measure_dev_posx_;// 0.3162;
  jacobian_measurement_noise.at<double>(1,1) = measure_dev_posy_;//0.3162;
 // jacobian_measurement_noise.at<double>(2,2) = measure_dev_heading_;//0.3162;============================
}


//class GuidepostEKF
GuidepostEKF::GuidepostEKF(int kf_coor_sys)
  :kf_coordinate_sys_(kf_coor_sys) {
  correct_count_ = 0;
  init_noise_ = 1.0;
  process_noise_ = 3.0;
  measure_noise_ = 0.08;
}

GuidepostEKF::~GuidepostEKF() {
}

void GuidepostEKF::Init(cv::Point2d init_pose, double init_theta) {
  predict_count_ = 0;
  correct_count_ = 0;
  re_init_ = false;
  // for ekf params
  // for guidepost measure, measure_params=2
  int dynam_params = 3;
  int measure_params = 2;
  int control_params = 1;
  int type = CV_64FC1;
  ExtendedKalmanFilter::Init(dynam_params, measure_params,
    control_params, type);

  // measurement_matrix_ will not use
  measurement_matrix_.release();

  // control_matrix_ will not use
  control_matrix_.release();

  // init process_noise_cov_
  cv::setIdentity(process_noise_cov_, cv::Scalar::all(1.0));
  cv::setIdentity(measurement_noise_cov_, cv::Scalar::all(1.0));

  if (kf_coordinate_sys_ == 0) {  // world gnd

  } else if (kf_coordinate_sys_ == 1) {  // car gnd    

  }
  state_post_.at<double>(0, 0) = init_pose.x;  // x, m
  state_post_.at<double>(1, 0) = init_pose.y;    // y, m
  state_post_.at<double>(2, 0) = init_theta;    // theta

  state_post_.copyTo(state_pre_);
  // init error_cov_post_, 3x3
  SetErrorCov();
  error_cov_post_.copyTo(error_cov_pre_);
}

const cv::Mat& GuidepostEKF::Correct(const cv::Mat& measurement,
  cv::Mat& extra_measurement) {
  // measure is (car_guidepost_px, car_guidepost_py)
  double perception_guidepost_px = measurement.at<double>(0, 0);
  double perception_guidepost_py = measurement.at<double>(1, 0);

  cv::Mat mesurement_pt = cv::Mat(2, 1, CV_64FC1);
  mesurement_pt.at<double>(0, 0) = perception_guidepost_px;
  mesurement_pt.at<double>(1, 0) = perception_guidepost_py;
  correct_count_++;

  const cv::Mat &mcorrect =
    ExtendedKalmanFilter::Correct(mesurement_pt, extra_measurement);
  predict_count_ = 0;

  return mcorrect;
}

void GuidepostEKF::Measurement(const cv::Mat &state_pre,
  cv::Mat &measuremet, cv::Mat &extra_measuremet) {
  // ground coordinate
  double x = state_pre_.at<double>(0, 0);
  double y = state_pre_.at<double>(1, 0);
  double theta = state_pre_.at<double>(2, 0);

  if (kf_coordinate_sys_ == 0) {

  } else if (kf_coordinate_sys_ == 1) {

  }

  double word_guidepost_px = extra_measuremet.at<double>(0,0);
  double word_guidepost_py = extra_measuremet.at<double>(1,0);

  measuremet.at<double>(0, 0) = cos(theta)*(word_guidepost_px - x) + sin(theta)*(word_guidepost_py - y);
  measuremet.at<double>(1, 0) = cos(theta)*(word_guidepost_py - y) - sin(theta)*(word_guidepost_px - x);
}


void GuidepostEKF::JacobianMeasurement(const cv::Mat &state_pre,
  cv::Mat &jacobian_measurement, cv::Mat &extra_measuremet) {
  double x = state_pre_.at<double>(0, 0);
  double y = state_pre_.at<double>(1, 0);
  double theta = state_pre_.at<double>(2, 0);

  double word_guidepost_px = extra_measuremet.at<double>(0,0);
  double word_guidepost_py = extra_measuremet.at<double>(1,0);

  if (kf_coordinate_sys_ == 0) {

  } else if (kf_coordinate_sys_ == 1) {

  }
  jacobian_measurement.setTo(cv::Scalar::all(0.0));
  jacobian_measurement.at<double>(0, 0) = -cos(theta);
  jacobian_measurement.at<double>(0, 1) = -sin(theta);
  jacobian_measurement.at<double>(0, 2) = cos(theta)*(word_guidepost_py - y) - sin(theta)*(word_guidepost_px - x);

  jacobian_measurement.at<double>(1, 0) = sin(theta);
  jacobian_measurement.at<double>(1, 1) = -cos(theta);
  jacobian_measurement.at<double>(1, 2) = -cos(theta)*(word_guidepost_px - x) - sin(theta)*(word_guidepost_py - y);
}

void GuidepostEKF::JacobianMeasurementNoise(cv::Mat &jacobian_measurement_noise) {
  jacobian_measurement_noise.setTo(cv::Scalar::all(0.0));
 jacobian_measurement_noise.at<double>(0,0) = 0.3162;
 jacobian_measurement_noise.at<double>(1,1) = 0.3162;
}

}  // namespace common
}  // namespace apollo
/*
#pragma GCC diagnostic pop
*/