//
//

#include "modules/common/filters/gps_kalman_filter.h"
#include <vector>
#include <algorithm>

#define imu_px_offset 1.705//2.393
//#define imu_py_offset -0.535
#define imu_py_offset 0.8715//0.835

#define use_rear_wheel_center true

#define junction_to_vehicle 0.4
#define junction_to_trailer 12.0

/*
#pragma GCC diagnostic push
#pragma GCC diagnostic error "-Wall"
#pragma GCC diagnostic error "-Wextra"
#pragma GCC diagnostic ignored "-Wunused-parameter"
*/
namespace apollo {
namespace common {

// measurement is valid in car coordinate, GPSKF
// is calculated on ground.
GPSKF::GPSKF(int kf_coor_sys)
  :kf_coordinate_sys_(kf_coor_sys) {
  correct_count_ = 0;
  init_noise_ = 1.0;
  process_noise_ = 3.0;
  measure_noise_ = 0.08;
  enable_filter_protect_ = false;
}

GPSKF::~GPSKF() {
}

void GPSKF::Init(cv::Point2d init_pose, double init_theta) {
  predict_count_ = 0;
  correct_count_ = 0;
  re_init_ = false;
  // for ekf params
  // bottom center on camera ground (x', x, y', y)
  int dynam_params = 3;
  int measure_params = 3;//1===================
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

    if (use_rear_wheel_center) {
    state_post_.at<double>(0, 0) = init_pose.x - (imu_px_offset*cos(-init_theta) + imu_py_offset*sin(-init_theta));
    state_post_.at<double>(1, 0) =  init_pose.y -(- imu_px_offset*sin(-init_theta) + imu_py_offset*cos(-init_theta));
    state_post_.at<double>(2, 0) = init_theta; //1========================
  }
  //SetVel(ref_v);
  state_post_.copyTo(state_pre_);
  // init error_cov_post_, 3x3
  SetErrorCov();
  error_cov_post_.copyTo(error_cov_pre_);
}



void GPSKF::SetErrorCov() {
  error_cov_post_.setTo(cv::Scalar::all(0.0f));
  error_cov_post_.at<double>(0, 0) = 0.01;   // x, 0.1m 1.0;//
  error_cov_post_.at<double>(1, 1) = 0.01;   // y1.0;//
  error_cov_post_.at<double>(2, 2) = 0.01;  // theta4.0;//

}



const cv::Mat& GPSKF::Predict(const cv::Mat& control) {
  //if (predict_count_ == 0) {
    ExtendedKalmanFilter::Predict(control);
  //}
  predict_count_++;

  return state_pre_;
}


const cv::Mat& GPSKF::Correct(const cv::Mat& measurement,
  cv::Mat& extra_measurement) {
  // measure is (localization_px, localization_py)
  correct_count_++;

  const cv::Mat &mcorrect =
    ExtendedKalmanFilter::Correct(measurement, extra_measurement);
  predict_count_ = 0;

  return mcorrect;
}


void GPSKF::Transition(const cv::Mat &state_post,
  cv::Mat &state_pre,
  const cv::Mat &control) {
  // x(k) = f( x(k-1), u(k-1) )
  //double dt = control.at<double>(0, 0);
  double ds = control.at<double>(1, 0);
  double dtheta = control.at<double>(2, 0);
  double steering_radius = control.at<double>(3, 0);

  state_pre.at<double>(0,0) = state_post.at<double>(0,0) + ds*cos(dtheta/2.0 + state_post.at<double>(2,0));
  state_pre.at<double>(1,0) = state_post.at<double>(1,0) + ds*sin(dtheta/2.0 + state_post.at<double>(2,0));
  state_pre.at<double>(2,0) = state_post.at<double>(2,0) + ds/steering_radius;
  if (dtheta/2.0 + state_post.at<double>(2,0) < 0.0) {
     // state_pre.at<double>(1,0) = state_post.at<double>(1,0) - ds*sin(dtheta/2.0 + state_post.at<double>(2,0));
  }

}

void GPSKF::JacobianTrasition(cv::Mat &jacobian_trans,
  const cv::Mat &control, const cv::Mat &state_post) {
  //double dt = control.at<double>(0, 0);
  double ds = control.at<double>(1, 0);
  double dtheta = control.at<double>(2, 0);


  cv::setIdentity(jacobian_trans);

  jacobian_trans.at<double>(0, 2) = -ds*sin(dtheta/2.0 + state_post.at<double>(2,0));
  jacobian_trans.at<double>(1, 2) = ds*cos(dtheta/2.0 + state_post.at<double>(2,0));

}


void GPSKF::Measurement(const cv::Mat &state_pre,
  cv::Mat &measuremet, cv::Mat &extra_measuremet) {
  // ground coordinate
  double x = state_pre_.at<double>(0, 0);
  double y = state_pre_.at<double>(1, 0); 

  measuremet.at<double>(0, 0) = x;
  measuremet.at<double>(1, 0) = y;
    measuremet.at<double>(2, 0) = state_pre_.at<double>(2, 0); //1========================

  if (use_rear_wheel_center) {
    double theta =  state_pre_.at<double>(2, 0);
    measuremet.at<double>(0, 0) = x + imu_px_offset*cos(-theta) + imu_py_offset*sin(-theta);
    measuremet.at<double>(1, 0) = y - imu_px_offset*sin(-theta) + imu_py_offset*cos(-theta);
    measuremet.at<double>(2, 0) = state_pre_.at<double>(2, 0); //1========================
  }
}


void GPSKF::JacobianMeasurement(const cv::Mat &state_pre,
  cv::Mat &jacobian_measurement, cv::Mat &extra_measuremet) {

  jacobian_measurement.setTo(cv::Scalar::all(0.0));
  jacobian_measurement.at<double>(0, 0) = 1.0;
  jacobian_measurement.at<double>(1, 1) = 1.0;

    jacobian_measurement.at<double>(2, 2) = 1.0;//1===================

  if (use_rear_wheel_center) {
    double theta =  state_pre_.at<double>(2, 0);
    jacobian_measurement.at<double>(0, 0) = 1.0;
    jacobian_measurement.at<double>(0, 2) = -imu_py_offset*cos(-theta) - (-imu_px_offset*sin(-theta));
    jacobian_measurement.at<double>(1, 1) = 1.0;
    jacobian_measurement.at<double>(1, 2) = -(-imu_py_offset*sin(-theta)) - (-imu_px_offset*cos(-theta));
    jacobian_measurement.at<double>(2, 2) = 1.0;//1===================
  }

}


// set noise positive
void GPSKF::JacobianProcessNoise(cv::Mat &jacobian_process_noise,
  const cv::Mat &control) {
  double dt = control.at<double>(0, 0);
  //double ds = control.at<double>(1, 0);
  //double steering_radius = control.at<double>(3, 0);

  jacobian_process_noise.setTo(cv::Scalar::all(0.0));
  jacobian_process_noise.at<double>(0,0) = dt/1.0/1.0/30.0;//30
  jacobian_process_noise.at<double>(1,1) = dt/1.0/1.0/30.0;//30
  jacobian_process_noise.at<double>(2,2) = 1.0/1.0*dt/5.0/1.0;//10

  if (measurement_heading_noise_ >=1.0*M_PI) {
    //jacobian_process_noise.at<double>(0,0) = dt/1.0/1.0/10.0;
    //jacobian_process_noise.at<double>(1,1) = dt/1.0/1.0/10.0;
    //jacobian_process_noise.at<double>(2,2) = 1.0/1.0*dt/0.20/1.0;
  }

  //double p = std::max(1.0,std::fabs(asin(3.4/steering_radius)*1000.0));
  //jacobian_process_noise *= p;
/*
  if (ds/dt > 0.5 && std::fabs(asin(3.4/steering_radius)) > 0.1187) {
    jacobian_process_noise.at<double>(0,0) = dt/10.0*10.0;
    jacobian_process_noise.at<double>(1,1) =dt/10.0*10.0;
    jacobian_process_noise.at<double>(2,2) = dt/10.0*20.0;
  }
  */

}

void GPSKF::JacobianMeasurementNoise(cv::Mat &jacobian_measurement_noise) {
  jacobian_measurement_noise.setTo(cv::Scalar::all(0.0));  

    jacobian_measurement_noise.at<double>(0,0) = measurement_px_noise_;//5e-5;//;//0.31620;//0.3162/1.0;
    jacobian_measurement_noise.at<double>(1,1) = measurement_py_noise_;//5e-5;//;//0.31620;//0.3162/1.0;
 
   jacobian_measurement_noise.at<double>(2,2) = measurement_heading_noise_;//0.0005;//1=============================
}


/************************************
 * trailer CYRA ekf*
 * ***********************************/

TrailerKF::TrailerKF(int kf_coor_sys)
  :kf_coordinate_sys_(kf_coor_sys) {
  correct_count_ = 0;
  init_noise_ = 1.0;
  process_noise_ = 3.0;
  measure_noise_ = 0.08;
  enable_filter_protect_ = false;
}

TrailerKF::~TrailerKF() {
}

void TrailerKF::Init(cv::Point2d init_pose, double init_vel, double init_theta) {
  predict_count_ = 0;
  correct_count_ = 0;
  re_init_ = false;
  // for ekf params
  // bottom center on camera ground (x', x, y', y)
  int dynam_params = 6;
  int measure_params = 2;//1===================
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
  state_post_.at<double>(3, 0) = init_vel;    // v, m/s
  state_post_.at<double>(4, 0) = 0.0;    // a, m/s2
  state_post_.at<double>(5, 0) = 0.0;    // w, rad/s

  state_post_.copyTo(state_pre_);
  // init error_cov_post_, 6x6
  SetErrorCov();
  error_cov_post_.copyTo(error_cov_pre_);
}



void TrailerKF::SetErrorCov() {
  error_cov_post_.setTo(cv::Scalar::all(0.0));
  error_cov_post_.at<double>(0, 0) = 0.1;   // x, 0.1m 1.0;//
  error_cov_post_.at<double>(1, 1) = 0.1;   // y1.0;//
  error_cov_post_.at<double>(2, 2) = 0.1;  // theta4.0;//
  error_cov_post_.at<double>(3, 3) = 0.1;   // x, 0.1m 1.0;//
  error_cov_post_.at<double>(4, 4) = 0.1;   // y1.0;//
  error_cov_post_.at<double>(5, 5) = 0.1;  // theta4.0;//

}



const cv::Mat& TrailerKF::Predict(const cv::Mat& control) {
  //if (predict_count_ == 0) {
    ExtendedKalmanFilter::Predict(control);
  //}
  predict_count_++;

  return state_pre_;
}


const cv::Mat& TrailerKF::Correct(const cv::Mat& measurement,
  cv::Mat& extra_measurement) {
  // measure is (localization_px, localization_py)
  correct_count_++;

  const cv::Mat &mcorrect =
    ExtendedKalmanFilter::Correct(measurement, extra_measurement);
  predict_count_ = 0;

  return mcorrect;
}


void TrailerKF::Transition(const cv::Mat &state_post,
  cv::Mat &state_pre,
  const cv::Mat &control) {
  // x(k) = f( x(k-1), u(k-1) )
  const double dt = control.at<double>(0, 0);

  const double x = state_post.at<double>(0,0);
  const double y = state_post.at<double>(1,0);
  const double theta = state_post.at<double>(2,0);
  const double v = state_post.at<double>(3,0);
  const double a = state_post.at<double>(4,0);
  const double w = state_post.at<double>(5,0);

  //omega is 0, driving straight
  if (w < 0.0001) {
    state_pre.at<double>(0,0) = x+v*cos(theta)*dt+0.5*a*cos(theta)*dt*dt;
    state_pre.at<double>(1,0) = y+v*sin(theta)*dt+0.5*a*sin(theta)*dt*dt;
  } else {
    state_pre.at<double>(0,0) = x+a/(w*w)*(cos(theta+w*dt)-cos(theta))+((v+a*dt)*sin(theta+w*dt)-v*sin(theta))/w;
    state_pre.at<double>(1,0) = y+a/(w*w)*(sin(theta+w*dt)-sin(theta))-((v+a*dt)*cos(theta+w*dt)-v*cos(theta))/w;                
  }
  state_pre.at<double>(2,0) =  w*dt+theta;
  state_pre.at<double>(3,0) =  v+a*dt;
  state_pre.at<double>(4,0) =  a;
  state_pre.at<double>(5,0) = w;
}

void TrailerKF::JacobianTrasition(cv::Mat &jacobian_trans,
  const cv::Mat &control, const cv::Mat &state_post) {
  const double dt = control.at<double>(0, 0);

  const double theta = state_post.at<double>(2,0);
  const double v = state_post.at<double>(3,0);
  const double a = state_post.at<double>(4,0);
  const double w = state_post.at<double>(5,0);

  cv::setIdentity(jacobian_trans);

  if (w < 0.0001) {
    jacobian_trans.at<double>(0, 2) = - dt*v*sin(theta) - (a*dt*dt*sin(theta))/2.0;
    jacobian_trans.at<double>(0, 3) = dt*cos(theta);
    jacobian_trans.at<double>(0, 4) = (dt*dt*cos(theta))/2.0;
    jacobian_trans.at<double>(1, 2) = (a*cos(theta)*dt*dt)/2.0 + v*cos(theta)*dt;
    jacobian_trans.at<double>(1, 3) = dt*sin(theta);
    jacobian_trans.at<double>(1, 4) = (dt*dt*sin(theta))/2.0;
  } else {
    jacobian_trans.at<double>(0, 2) = - (v*cos(theta) - cos(theta + dt*w)*(v + a*dt))/w - (a*(sin(theta + dt*w) - sin(theta)))/(w*w);
    jacobian_trans.at<double>(0, 3) = (sin(theta + dt*w) - sin(theta))/w;
    jacobian_trans.at<double>(0, 4) = (cos(theta + dt*w) - cos(theta))/(w*w) + (dt*sin(theta + dt*w))/w;
    jacobian_trans.at<double>(0, 5) = (v*sin(theta) - sin(theta + dt*w)*(v + a*dt))/(w*w) - (2.0*a*(cos(theta + dt*w) - cos(theta)))/(w*w*w)
                                                                           + (dt*cos(theta + dt*w)*(v + a*dt))/w - (a*dt*sin(theta + dt*w))/(w*w);
    jacobian_trans.at<double>(1, 2) = (a*(cos(theta + dt*w) - cos(theta)))/(w*w) - (v*sin(theta) - sin(theta + dt*w)*(v + a*dt))/w;
    jacobian_trans.at<double>(1, 3) = -(cos(theta + dt*w) - cos(theta))/w;
    jacobian_trans.at<double>(1, 4) = (sin(theta + dt*w) - sin(theta))/(w*w) - (dt*cos(theta + dt*w))/w;
    jacobian_trans.at<double>(1, 5) = (dt*sin(theta + dt*w)*(v + a*dt))/w - (2.0*a*(sin(theta + dt*w) - sin(theta)))/(w*w*w)
                                                                           - (v*cos(theta) - cos(theta + dt*w)*(v + a*dt))/(w*w) + (a*dt*cos(theta + dt*w))/(w*w);
  }

  jacobian_trans.at<double>(2, 5) = dt;
  jacobian_trans.at<double>(3, 4) = dt;
}


void TrailerKF::Measurement(const cv::Mat &state_pre,
  cv::Mat &measuremet, cv::Mat &extra_measuremet) {
  // ground coordinate
  double x = state_pre_.at<double>(0, 0);
  double y = state_pre_.at<double>(1, 0); 
  double trailer_theta = state_pre_.at<double>(2, 0); 
  double junction_px = cos(-trailer_theta)*junction_to_trailer + x;
  double junction_py = -sin(-trailer_theta)*junction_to_trailer + y;

  const double vehile_theta = extra_measuremet.at<double>(0,0);
  measuremet.at<double>(0, 0) = cos(-vehile_theta)*(-junction_to_vehicle) + junction_px;
  measuremet.at<double>(1, 0) = -sin(-vehile_theta)*(-junction_to_vehicle) + junction_py;

  //measuremet.at<double>(0, 0) =x + 12*cos(trailer_theta) - (2*cos(vehile_theta))/5;
  //measuremet.at<double>(1, 0) = y + 12*sin(trailer_theta) - (2*sin(vehile_theta))/5;
}


void TrailerKF::JacobianMeasurement(const cv::Mat &state_pre,
  cv::Mat &jacobian_measurement, cv::Mat &extra_measuremet) {
  jacobian_measurement.setTo(cv::Scalar::all(0.0));
  jacobian_measurement.at<double>(0, 0) = 1.0;
  jacobian_measurement.at<double>(0, 2) = -junction_to_trailer*sin(state_pre.at<double>(2,0));
  jacobian_measurement.at<double>(1, 1) = 1.0;
  jacobian_measurement.at<double>(1, 2) = junction_to_trailer*cos(state_pre.at<double>(2,0));

}


// set noise positive
void TrailerKF::JacobianProcessNoise(cv::Mat &jacobian_process_noise,
  const cv::Mat &control) {
  double dt = control.at<double>(0, 0);
  double trailer_theta = state_pre_.at<double>(2,0);

  const double std_noise_a=2.0;//process noise standard deviation for a
  const double std_noise_yaw_dd=0.3;//process noise standard deviation for yaw acceleration

  const double G00 = 0.5*dt*dt*cos(trailer_theta);
  const double G10 = 0.5*dt*dt*sin(trailer_theta);
  //const double G21 = 0.5*dt*dt;
  const double G30 = 0.5*dt*dt;
  const double G40 = dt;
  //const double G51 = dt;

  jacobian_process_noise.setTo(cv::Scalar::all(0.0));
  jacobian_process_noise.at<double>(0,0) = G00 * std_noise_a;
  jacobian_process_noise.at<double>(1,0) = G10 * std_noise_a;
  jacobian_process_noise.at<double>(2,2) = G30 * std_noise_yaw_dd;
  jacobian_process_noise.at<double>(3,0) = G30 * std_noise_a;
  jacobian_process_noise.at<double>(4,0) = G40 * std_noise_a;
  jacobian_process_noise.at<double>(5,2) = G40 * std_noise_yaw_dd;
}

void TrailerKF::JacobianMeasurementNoise(cv::Mat &jacobian_measurement_noise) {
  jacobian_measurement_noise.setTo(cv::Scalar::all(0.0));  

  jacobian_measurement_noise.at<double>(0,0) = measurement_px_noise_;//5e-5;//;//0.31620;//0.3162/1.0;
  jacobian_measurement_noise.at<double>(1,1) = measurement_py_noise_;//5e-5;//;//0.31620;//0.3162/1.0; 

  #if 1
   jacobian_measurement_noise.at<double>(0,0) = 0.15;//5e-5;//;//0.31620;//0.3162/1.0;
   jacobian_measurement_noise.at<double>(1,1) = 0.15;
  #endif
}





/************************************
 * vehicle CYRA ekf*
 * ***********************************/

VehicleCYRAKF::VehicleCYRAKF(int kf_coor_sys)
  :kf_coordinate_sys_(kf_coor_sys) {
  correct_count_ = 0;
  init_noise_ = 1.0;
  process_noise_ = 3.0;
  measure_noise_ = 0.08;
  enable_filter_protect_ = false;
}

VehicleCYRAKF::~VehicleCYRAKF() {
}

void VehicleCYRAKF::Init(cv::Point2d init_pose, double init_vel, double init_theta) {
  predict_count_ = 0;
  correct_count_ = 0;
  re_init_ = false;
  // for ekf params
  // bottom center on camera ground (x', x, y', y)
  int dynam_params = 6;
  int measure_params = 3;//1===================
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
  state_post_.at<double>(3, 0) = init_vel;    // v, m/s
  state_post_.at<double>(4, 0) = 0.0;    // a, m/s2
  state_post_.at<double>(5, 0) = 0.0;    // w, rad/s

  state_post_.copyTo(state_pre_);
  // init error_cov_post_, 6x6
  SetErrorCov();
  error_cov_post_.copyTo(error_cov_pre_);
}



void VehicleCYRAKF::SetErrorCov() {
  error_cov_post_.setTo(cv::Scalar::all(0.0));
  error_cov_post_.at<double>(0, 0) = 0.1;   // x, 0.1m 1.0;//
  error_cov_post_.at<double>(1, 1) = 0.1;   // y1.0;//
  error_cov_post_.at<double>(2, 2) = 0.1;  // theta4.0;//
  error_cov_post_.at<double>(3, 3) = 0.1;   // x, 0.1m 1.0;//
  error_cov_post_.at<double>(4, 4) = 0.1;   // y1.0;//
  error_cov_post_.at<double>(5, 5) = 0.1;  // theta4.0;//

}



const cv::Mat& VehicleCYRAKF::Predict(const cv::Mat& control) {
  //if (predict_count_ == 0) {
    ExtendedKalmanFilter::Predict(control);
  //}
  predict_count_++;

  return state_pre_;
}


const cv::Mat& VehicleCYRAKF::Correct(const cv::Mat& measurement,
  cv::Mat& extra_measurement) {
  // measure is (localization_px, localization_py)
  correct_count_++;

  const cv::Mat &mcorrect =
    ExtendedKalmanFilter::Correct(measurement, extra_measurement);
  predict_count_ = 0;

  return mcorrect;
}


void VehicleCYRAKF::Transition(const cv::Mat &state_post,
  cv::Mat &state_pre,
  const cv::Mat &control) {
  // x(k) = f( x(k-1), u(k-1) )
  const double dt = control.at<double>(0, 0);

  const double x = state_post.at<double>(0,0);
  const double y = state_post.at<double>(1,0);
  const double theta = state_post.at<double>(2,0);
  const double v = state_post.at<double>(3,0);
  const double a = state_post.at<double>(4,0);
  const double w = state_post.at<double>(5,0);

  //omega is 0, driving straight
  if (w < 0.0001) {
    state_pre.at<double>(0,0) = x+v*cos(theta)*dt+0.5*a*cos(theta)*dt*dt;
    state_pre.at<double>(1,0) = y+v*sin(theta)*dt+0.5*a*sin(theta)*dt*dt;
  } else {
    state_pre.at<double>(0,0) = x+a/(w*w)*(cos(theta+w*dt)-cos(theta))+((v+a*dt)*sin(theta+w*dt)-v*sin(theta))/w;
    state_pre.at<double>(1,0) = y+a/(w*w)*(sin(theta+w*dt)-sin(theta))-((v+a*dt)*cos(theta+w*dt)-v*cos(theta))/w;                
  }
  state_pre.at<double>(2,0) =  w*dt+theta;
  state_pre.at<double>(3,0) =  v+a*dt;
  state_pre.at<double>(4,0) =  a;
  state_pre.at<double>(5,0) = w;
}

void VehicleCYRAKF::JacobianTrasition(cv::Mat &jacobian_trans,
  const cv::Mat &control, const cv::Mat &state_post) {
  const double dt = control.at<double>(0, 0);

  const double theta = state_post.at<double>(2,0);
  const double v = state_post.at<double>(3,0);
  const double a = state_post.at<double>(4,0);
  const double w = state_post.at<double>(5,0);

  cv::setIdentity(jacobian_trans);

  if (w < 0.0001) {
    jacobian_trans.at<double>(0, 2) = - dt*v*sin(theta) - (a*dt*dt*sin(theta))/2.0;
    jacobian_trans.at<double>(0, 3) = dt*cos(theta);
    jacobian_trans.at<double>(0, 4) = (dt*dt*cos(theta))/2.0;
    jacobian_trans.at<double>(1, 2) = (a*cos(theta)*dt*dt)/2.0 + v*cos(theta)*dt;
    jacobian_trans.at<double>(1, 3) = dt*sin(theta);
    jacobian_trans.at<double>(1, 4) = (dt*dt*sin(theta))/2.0;
  } else {
    jacobian_trans.at<double>(0, 2) = - (v*cos(theta) - cos(theta + dt*w)*(v + a*dt))/w - (a*(sin(theta + dt*w) - sin(theta)))/(w*w);
    jacobian_trans.at<double>(0, 3) = (sin(theta + dt*w) - sin(theta))/w;
    jacobian_trans.at<double>(0, 4) = (cos(theta + dt*w) - cos(theta))/(w*w) + (dt*sin(theta + dt*w))/w;
    jacobian_trans.at<double>(0, 5) = (v*sin(theta) - sin(theta + dt*w)*(v + a*dt))/(w*w) - (2.0*a*(cos(theta + dt*w) - cos(theta)))/(w*w*w)
                                                                           + (dt*cos(theta + dt*w)*(v + a*dt))/w - (a*dt*sin(theta + dt*w))/(w*w);
    jacobian_trans.at<double>(1, 2) = (a*(cos(theta + dt*w) - cos(theta)))/(w*w) - (v*sin(theta) - sin(theta + dt*w)*(v + a*dt))/w;
    jacobian_trans.at<double>(1, 3) = -(cos(theta + dt*w) - cos(theta))/w;
    jacobian_trans.at<double>(1, 4) = (sin(theta + dt*w) - sin(theta))/(w*w) - (dt*cos(theta + dt*w))/w;
    jacobian_trans.at<double>(1, 5) = (dt*sin(theta + dt*w)*(v + a*dt))/w - (2.0*a*(sin(theta + dt*w) - sin(theta)))/(w*w*w)
                                                                           - (v*cos(theta) - cos(theta + dt*w)*(v + a*dt))/(w*w) + (a*dt*cos(theta + dt*w))/(w*w);
  }

  jacobian_trans.at<double>(2, 5) = dt;
  jacobian_trans.at<double>(3, 4) = dt;
}


void VehicleCYRAKF::Measurement(const cv::Mat &state_pre,
  cv::Mat &measuremet, cv::Mat &extra_measuremet) {
  // ground coordinate
  double x = state_pre_.at<double>(0, 0);
  double y = state_pre_.at<double>(1, 0); 

  measuremet.at<double>(0, 0) = x;
  measuremet.at<double>(1, 0) = y;
    measuremet.at<double>(2, 0) = state_pre_.at<double>(2, 0); //1========================

  if (use_rear_wheel_center) {
    double theta =  state_pre_.at<double>(2, 0);
    measuremet.at<double>(0, 0) = x + imu_px_offset*cos(-theta) + imu_py_offset*sin(-theta);
    measuremet.at<double>(1, 0) = y - imu_px_offset*sin(-theta) + imu_py_offset*cos(-theta);
    //measuremet.at<double>(2, 0) = state_pre_.at<double>(2, 0); //1========================
  }
}


void VehicleCYRAKF::JacobianMeasurement(const cv::Mat &state_pre,
  cv::Mat &jacobian_measurement, cv::Mat &extra_measuremet) {
  jacobian_measurement.setTo(cv::Scalar::all(0.0));
    double theta =  state_pre_.at<double>(2, 0);
    jacobian_measurement.at<double>(0, 0) = 1.0;
    jacobian_measurement.at<double>(0, 2) = -imu_py_offset*cos(-theta) - (-imu_px_offset*sin(-theta));
    jacobian_measurement.at<double>(1, 1) = 1.0;
    jacobian_measurement.at<double>(1, 2) = -(-imu_py_offset*sin(-theta)) - (-imu_px_offset*cos(-theta));
    jacobian_measurement.at<double>(2, 2) = 1.0;//1===================

}


// set noise positive
void VehicleCYRAKF::JacobianProcessNoise(cv::Mat &jacobian_process_noise,
  const cv::Mat &control) {
  double dt = control.at<double>(0, 0);
  double trailer_theta = state_pre_.at<double>(2,0);

  const double std_noise_a=2.0;//process noise standard deviation for a
  const double std_noise_yaw_dd=0.3;//process noise standard deviation for yaw acceleration

  const double G00 = 0.5*dt*dt*cos(trailer_theta);
  const double G10 = 0.5*dt*dt*sin(trailer_theta);
  //const double G21 = 0.5*dt*dt;
  const double G30 = 0.5*dt*dt;
  const double G40 = dt;
  //const double G51 = dt;

  jacobian_process_noise.setTo(cv::Scalar::all(0.0));
  jacobian_process_noise.at<double>(0,0) = G00 * std_noise_a;
  jacobian_process_noise.at<double>(1,0) = G10 * std_noise_a;
  jacobian_process_noise.at<double>(2,2) = G30 * std_noise_yaw_dd;
  jacobian_process_noise.at<double>(3,0) = G30 * std_noise_a;
  jacobian_process_noise.at<double>(4,0) = G40 * std_noise_a;
  jacobian_process_noise.at<double>(5,2) = G40 * std_noise_yaw_dd;
}

void VehicleCYRAKF::JacobianMeasurementNoise(cv::Mat &jacobian_measurement_noise) {
  jacobian_measurement_noise.setTo(cv::Scalar::all(0.0));  

  jacobian_measurement_noise.at<double>(0,0) = measurement_px_noise_;//5e-5;//;//0.31620;//0.3162/1.0;
  jacobian_measurement_noise.at<double>(1,1) = measurement_py_noise_;//5e-5;//;//0.31620;//0.3162/1.0; 
  jacobian_measurement_noise.at<double>(2,2) = measurement_heading_noise_;//5e-5;//;//0.31620;//0.3162/1.0; 

  #if 1
   //jacobian_measurement_noise.at<double>(0,0) = 10.15;//5e-5;//;//0.31620;//0.3162/1.0;
   //jacobian_measurement_noise.at<double>(1,1) = 10.15;
  #endif
}


}  // namespace common
}  // namespace apollo
/*
#pragma GCC diagnostic pop
*/
