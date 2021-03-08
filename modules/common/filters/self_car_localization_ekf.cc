//
//

#include "modules/common/filters/self_car_localization_ekf.h"
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

// measurement is valid in car coordinate, SelfCarLocalizationEKF
// is calculated on ground.
SelfCarLocalizationEKF::SelfCarLocalizationEKF(int kf_coor_sys)
  :kf_coordinate_sys_(kf_coor_sys) {
  correct_count_ = 0;
  init_noise_ = 1.0;
  process_noise_ = 3.0;
  measure_noise_ = 0.08;
  enable_filter_protect_ = false;
}

SelfCarLocalizationEKF::~SelfCarLocalizationEKF() {
}

void SelfCarLocalizationEKF::Init(cv::Point2d init_pose, double init_theta) {
  predict_count_ = 0;
  correct_count_ = 0;
  re_init_ = false;
  // for ekf params
  // bottom center on camera ground (x', x, y', y)
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

  if (kf_coordinate_sys_ == 0) {  // camera gnd

  } else if (kf_coordinate_sys_ == 1) {  // world gnd    

  }
  state_post_.at<double>(0, 0) = init_pose.x;  // x, m
  state_post_.at<double>(1, 0) = init_pose.y;    // y, m
  state_post_.at<double>(2, 0) = init_theta;    // theta
  //SetVel(ref_v);
  state_post_.copyTo(state_pre_);
  // init error_cov_post_, 3x3
  SetErrorCov();
  error_cov_post_.copyTo(error_cov_pre_);
}

void SelfCarLocalizationEKF::SetVel(const cv::Point2f &ref_v) {
  if (kf_coordinate_sys_ == 0) {  // camera gnd

  } else if (kf_coordinate_sys_ == 1) {  // world gnd

  }
}

void SelfCarLocalizationEKF::SetErrorCov() {
  error_cov_post_.setTo(cv::Scalar::all(0.0f));
  error_cov_post_.at<double>(0, 0) = 10.0;//0.01;   // x, 0.1m 1.0;//
  error_cov_post_.at<double>(1, 1) =10.0;//0.01;   // y1.0;//
  error_cov_post_.at<double>(2, 2) = 20.0;//0.01;  // theta4.0;//

}



const cv::Mat& SelfCarLocalizationEKF::Predict(const cv::Mat& control) {
  //if (predict_count_ == 0) {
    ExtendedKalmanFilter::Predict(control);
  //}
  predict_count_++;

  return state_pre_;
}


const cv::Mat& SelfCarLocalizationEKF::Correct(const cv::Mat& measurement,
  cv::Mat& extra_measurement) {
  // measure is (localization_px, localization_py)
  correct_count_++;

  const cv::Mat &mcorrect =
    ExtendedKalmanFilter::Correct(measurement, extra_measurement);
  predict_count_ = 0;

  return mcorrect;
}


void SelfCarLocalizationEKF::Transition(const cv::Mat &state_post,
  cv::Mat &state_pre,
  const cv::Mat &control) {
  // x(k) = f( x(k-1), u(k-1) )
  //double dt = control.at<double>(0, 0);
  double ds = control.at<double>(1, 0);
  double dtheta = control.at<double>(2, 0);
  double steering_radius = control.at<double>(3, 0);

  state_pre.at<double>(0,0) = state_post.at<double>(0,0) + ds*cos(dtheta/2.0- state_post.at<double>(2,0));
  state_pre.at<double>(1,0) = state_post.at<double>(1,0) - ds*sin(dtheta/2.0 - state_post.at<double>(2,0));
  state_pre.at<double>(2,0) = state_post.at<double>(2,0) + ds/steering_radius;

}

void SelfCarLocalizationEKF::JacobianTrasition(cv::Mat &jacobian_trans,
  const cv::Mat &control, const cv::Mat &state_post) {
  //double dt = control.at<double>(0, 0);
  double ds = control.at<double>(1, 0);
  double dtheta = control.at<double>(2, 0);


  cv::setIdentity(jacobian_trans);

  jacobian_trans.at<double>(0, 2) =ds*sin(dtheta/2.0 - state_post.at<double>(2,0));
  jacobian_trans.at<double>(1, 2) = ds*cos(dtheta/2.0 - state_post.at<double>(2,0));

}


void SelfCarLocalizationEKF::Measurement(const cv::Mat &state_pre,
  cv::Mat &measuremet, cv::Mat &extra_measuremet) {
  // ground coordinate
  double x = state_pre_.at<double>(0, 0);
  double y = state_pre_.at<double>(1, 0); 

  if (kf_coordinate_sys_ == 0) {

  } else if (kf_coordinate_sys_ == 1) {

  }
  measuremet.at<double>(0, 0) = x;
  measuremet.at<double>(1, 0) = y;
}


void SelfCarLocalizationEKF::JacobianMeasurement(const cv::Mat &state_pre,
  cv::Mat &jacobian_measurement, cv::Mat &extra_measuremet) {

  jacobian_measurement.setTo(cv::Scalar::all(0.0));
  jacobian_measurement.at<double>(0, 0) = 1.0;
  jacobian_measurement.at<double>(1, 1) = 1.0;

}


// set noise positive
void SelfCarLocalizationEKF::JacobianProcessNoise(cv::Mat &jacobian_process_noise,
  const cv::Mat &control) {
  double dt = control.at<double>(0, 0);
  double ds = control.at<double>(1, 0);

  jacobian_process_noise.setTo(cv::Scalar::all(0.0));
  jacobian_process_noise.at<double>(0,0) = dt/5.0;
  jacobian_process_noise.at<double>(1,1) = dt/5.0;
  jacobian_process_noise.at<double>(2,2) = 1.0/2.0*dt/2.0;

  if (ds < 3e-3) {
    jacobian_process_noise.at<double>(0,0) = dt/1000.0;
    jacobian_process_noise.at<double>(1,1) = dt/1000.0;
    jacobian_process_noise.at<double>(2,2) = 1.0/500.0*dt/2.0;
  }

}

void SelfCarLocalizationEKF::JacobianMeasurementNoise(cv::Mat &jacobian_measurement_noise) {
  jacobian_measurement_noise.setTo(cv::Scalar::all(0.0));
  
  if(correct_count_ < 500) {
    jacobian_measurement_noise.at<double>(0,0) =10.0;//0.3162/1.0;
    jacobian_measurement_noise.at<double>(1,1) = 10.0;//0.3162/1.0;
  } else {
    jacobian_measurement_noise.at<double>(0,0) = 1.0;//measurement_px_noise_;//0.31620;//0.3162/1.0;
    jacobian_measurement_noise.at<double>(1,1) = 1.0;//measurement_py_noise_;//0.31620;//0.3162/1.0;
  }
  /*
  double speed = extra_measuremet.at<double>(0,0)*extra_measuremet.at<double>(0,0) +  
                    extra_measuremet.at<double>(1,0)*extra_measuremet.at<double>(1,0);
  if (speed < 0.3) {
    jacobian_measurement_noise.at<double>(0,0) = 20.0;//0.3162/1.0;
    jacobian_measurement_noise.at<double>(1,1) = 20.0;//0.3162/1.0;
  } else {
    jacobian_measurement_noise.at<double>(0,0) = 1.0;//0.3162/1.0;
    jacobian_measurement_noise.at<double>(1,1) = 1.0;//0.3162/1.0;
  }
  */
  
 
}


}  // namespace common
}  // namespace apollo
/*
#pragma GCC diagnostic pop
*/