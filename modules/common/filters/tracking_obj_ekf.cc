//
//

#include "modules/common/filters/tracking_obj_ekf.h"
#include <vector>
#include <algorithm>


/*
#pragma GCC diagnostic push
#pragma GCC diagnostic error "-Wall"
#pragma GCC diagnostic error "-Wextra"
#pragma GCC diagnostic ignored "-Wunused-parameter"
*/
namespace apollo {
namespace common {

//////////////////////////////////////////////////////////////////////////
static inline cv::Point2f Mat3x3Multiply(
  const cv::Mat &mat, const cv::Point2f &pt) {
  const float *p = reinterpret_cast<const float *>(mat.data);
  cv::Point2f pt_new;
  float t = p[6] * pt.x + p[7] * pt.y + p[8];
  pt_new.x = (p[0] * pt.x + p[1] * pt.y + p[2]) / t;
  pt_new.y = (p[3] * pt.x + p[4] * pt.y + p[5]) / t;
  return pt_new;
}

///////////////////////////////////////////////////
// measurement is valid in car coordinate, SelfCarEKF
// is calculated on ground.
SelfCarEKF::SelfCarEKF(int kf_coor_sys)
  :kf_coordinate_sys_(kf_coor_sys) {
  correct_count_ = 0;
  obs_dist_ = 0.0f;
  init_noise_ = 1.0f;
  process_noise_ = 3.0f;
  measure_noise_ = 0.08f;
  enable_filter_protect_ = false;
  distekf_with_state_ = false;
  obj_states_ = 0;
}

SelfCarEKF::~SelfCarEKF() {
}

void SelfCarEKF::Init(cv::Point2f init_pose, double init_theta) {
  predict_count_ = 0;
  correct_count_ = 0;
  re_init_ = false;
  // for ekf params
  // bottom center on camera ground (x', x, y', y)
  int dynam_params = 3;
  int measure_params = 2;
  int control_params = 1;
  int type = CV_32FC1;
  ExtendedKalmanFilter::Init(dynam_params, measure_params,
    control_params, type);


  // measurement_matrix_ will not use
  measurement_matrix_.release();

  // control_matrix_ will not use
  control_matrix_.release();

  // init process_noise_cov_
  cv::setIdentity(process_noise_cov_, cv::Scalar::all(1.0f));
  cv::setIdentity(measurement_noise_cov_, cv::Scalar::all(1.0f));

  if (kf_coordinate_sys_ == 0) {  // camera gnd

  } else if (kf_coordinate_sys_ == 1) {  // world gnd    

  }
  state_post_.at<float>(0, 0) = init_pose.x;  // x, m
  state_post_.at<float>(1, 0) = init_pose.y;    // y, m
  state_post_.at<float>(2, 0) = init_theta;    // theta
  //SetVel(ref_v);
  state_post_.copyTo(state_pre_);
  // init error_cov_post_, 3x3
  SetErrorCov();
  error_cov_post_.copyTo(error_cov_pre_);
}

void SelfCarEKF::SetVel(const cv::Point2f &ref_v) {
  if (kf_coordinate_sys_ == 0) {  // camera gnd

  } else if (kf_coordinate_sys_ == 1) {  // world gnd

  }
}

void SelfCarEKF::SetErrorCov() {
  error_cov_post_.setTo(cv::Scalar::all(0.0f));
  error_cov_post_.at<float>(0, 0) = 0.01f * 1000.0f;   // x, 0.1m
  error_cov_post_.at<float>(1, 1) = 0.01f * 1000.0f;   // y
  error_cov_post_.at<float>(2, 2) = 0.01f * 1000.0f;  // theta

}



const cv::Mat& SelfCarEKF::Predict(const cv::Mat& control) {
  //if (predict_count_ == 0) {
    ExtendedKalmanFilter::Predict(control);
  //}
  predict_count_++;

  return state_pre_;
}


const cv::Mat& SelfCarEKF::Correct(const cv::Mat& measurement,
  cv::Mat& extra_measurement) {
  // measure is (car_guidepost_px, car_guidepost_py)
  float perception_guidepost_px = measurement.at<float>(0, 0);
  float perception_guidepost_py = measurement.at<float>(1, 0);

  cv::Mat mesurement_pt = cv::Mat(2, 1, CV_32FC1);
  mesurement_pt.at<float>(0, 0) = perception_guidepost_px;
  mesurement_pt.at<float>(1, 0) = perception_guidepost_py;
  correct_count_++;

  const cv::Mat &mcorrect =
    ExtendedKalmanFilter::Correct(mesurement_pt, extra_measurement);
  predict_count_ = 0;

  return mcorrect;
}


void SelfCarEKF::Transition(const cv::Mat &state_post,
  cv::Mat &state_pre,
  const cv::Mat &control) {
  // x(k) = f( x(k-1), u(k-1) )
  //float dt = control.at<float>(0, 0);
  float ds = control.at<float>(1, 0);
  float dtheta = control.at<float>(2, 0);
  float steering_radius = control.at<float>(3, 0);
  /*
  cv::setIdentity(transition_matrix_);
  transition_matrix_.at<float>(1, 0) = dt;
  transition_matrix_.at<float>(3, 2) = dt;

  state_pre = transition_matrix_ * state_post;
  */
  state_pre.at<float>(0,0) = state_post.at<float>(0,0) + ds*cos(dtheta/2 - state_post.at<float>(2,0));
  state_pre.at<float>(1,0) = state_post.at<float>(1,0) - ds*sin(dtheta/2 - state_post.at<float>(2,0));
  state_pre.at<float>(2,0) = state_post.at<float>(2,0) + ds/steering_radius;

}

void SelfCarEKF::JacobianTrasition(cv::Mat &jacobian_trans,
  const cv::Mat &control, const cv::Mat &state_post) {
  //float dt = control.at<float>(0, 0);
  float ds = control.at<float>(1, 0);
  float dtheta = control.at<float>(2, 0);


  cv::setIdentity(jacobian_trans);

  jacobian_trans.at<float>(0, 2) =ds*sin(dtheta/2 - state_post.at<float>(2,0));
  jacobian_trans.at<float>(1, 2) = ds*cos(dtheta/2 - state_post.at<float>(2,0));

}


void SelfCarEKF::Measurement(const cv::Mat &state_pre,
  cv::Mat &measuremet, cv::Mat &extra_measuremet) {
  // ground coordinate
  float x = state_pre_.at<float>(0, 0);
  float y = state_pre_.at<float>(1, 0);
  float theta = state_pre_.at<float>(2, 0);

  if (kf_coordinate_sys_ == 0) {

  } else if (kf_coordinate_sys_ == 1) {

  }

  float word_guidepost_px = extra_measuremet.at<float>(0,0);
  float word_guidepost_py = extra_measuremet.at<float>(1,0);

  measuremet.at<float>(0, 0) = cos(theta)*(word_guidepost_px - x) + sin(theta)*(word_guidepost_py - y);
  measuremet.at<float>(1, 0) = cos(theta)*(word_guidepost_py - y) - sin(theta)*(word_guidepost_px - x);
}


void SelfCarEKF::JacobianMeasurement(const cv::Mat &state_pre,
  cv::Mat &jacobian_measurement, cv::Mat &extra_measuremet) {
  float x = state_pre_.at<float>(0, 0);
  float y = state_pre_.at<float>(1, 0);
  float theta = state_pre_.at<float>(2, 0);

  float word_guidepost_px = extra_measuremet.at<float>(0,0);
  float word_guidepost_py = extra_measuremet.at<float>(1,0);

  if (kf_coordinate_sys_ == 0) {

  } else if (kf_coordinate_sys_ == 1) {

  }
  jacobian_measurement.setTo(cv::Scalar::all(0.0f));
  jacobian_measurement.at<float>(0, 0) = -cos(theta);
  jacobian_measurement.at<float>(0, 1) = -sin(theta);
  jacobian_measurement.at<float>(0, 2) = cos(theta)*(word_guidepost_py - y) - sin(theta)*(word_guidepost_px - x);

  jacobian_measurement.at<float>(1, 0) = sin(theta);
  jacobian_measurement.at<float>(1, 1) = -cos(theta);
  jacobian_measurement.at<float>(1, 2) = -cos(theta)*(word_guidepost_px - x) - sin(theta)*(word_guidepost_py - y);
}


// set noise positive
void SelfCarEKF::JacobianProcessNoise(cv::Mat &jacobian_process_noise,
  const cv::Mat &control) {
  float dt = control.at<float>(0, 0);
  {
    if (kf_coordinate_sys_ == 0) {

    } else if (kf_coordinate_sys_ == 1) {

    }
  jacobian_process_noise.setTo(cv::Scalar::all(0.0f));
  jacobian_process_noise.at<float>(0,0) = dt/5.0f;
  jacobian_process_noise.at<float>(1,1) = dt/5.0f;
  jacobian_process_noise.at<float>(2,2) = 1.0f/2.0f*dt;
  }
}

void SelfCarEKF::JacobianMeasurementNoise(cv::Mat &jacobian_measurement_noise) {
  jacobian_measurement_noise.setTo(cv::Scalar::all(0.0f));
 jacobian_measurement_noise.at<float>(0,0) = 0.3162f;
 jacobian_measurement_noise.at<float>(1,1) = 0.3162f;
}


}  // namespace common
}  // namespace apollo
/*
#pragma GCC diagnostic pop
*/