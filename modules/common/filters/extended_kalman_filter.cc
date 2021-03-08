//
//

#include "modules/common/filters/extended_kalman_filter.h"
//#include <hobot-adas/utils/camera/camera.h>
#include <vector>
#include <algorithm>

namespace apollo {
namespace common {
void ExtendedKalmanFilter::Init(int dynam_params, int measure_params,
                                int control_params, int type/* = CV_32F*/) {
  KalmanFilterInterface::Init(dynam_params, measure_params,
                              control_params, type);

  jacobian_trasition_.create(dynam_params, dynam_params, type);
  jacobian_measurement_.create(measure_params, dynam_params, type);

  temp_state_.create(dynam_params, 1, type);
  temp_measurement_.create(measure_params, 1, type);

  jacobian_process_noise_.create(dynam_params, dynam_params, type);
  jacobian_measurement_noise_.create(measure_params, measure_params,  type);

  process_noise_tmp_.create(dynam_params, dynam_params, type);
  measurement_noise_tmp_.create(measure_params, measure_params, type);

  temp1_.create(dynam_params, dynam_params, type);
  temp2_.create(measure_params, dynam_params, type);
  temp3_.create(measure_params, measure_params, type);
  temp4_.create(measure_params, dynam_params, type);
  temp5_.create(measure_params, 1, type);
}

const cv::Mat& ExtendedKalmanFilter::Predict(const cv::Mat& control) {
  // P'(k) = JF*P(k-1)*JFt + Q(k-1)

  JacobianTrasition(jacobian_trasition_, control, state_post_);

  // update error covariance matrices: temp1 = JF*P(k)
  temp1_ = jacobian_trasition_* error_cov_post_;

  // Q = W(k) * Q(k-1) * Wt(k)
  JacobianProcessNoise(jacobian_process_noise_, control);
//shzhw
cv::setIdentity(process_noise_cov_);

  cv::gemm(jacobian_process_noise_, process_noise_cov_, 1.0,
           cv::Mat(), 1.0, process_noise_tmp_);
  cv::gemm(process_noise_tmp_, jacobian_process_noise_, 1.0,
    cv::Mat(), 1.0, process_noise_tmp_, cv::GEMM_2_T);

  // P'(k) = temp1*JFt + Q
  cv::gemm(temp1_, jacobian_trasition_, 1.0,
           process_noise_tmp_, 1.0, error_cov_pre_, cv::GEMM_2_T);

  // calculate x'(k) = f( x(k-1), u(k-1) )
  Transition(state_post_, state_pre_, control);
  // x'(k) = x'(k) + B*u(k)
  if (!control.empty() && !control_matrix_.empty())
    state_pre_ += control_matrix_ * control;

  // handle the case when there will be measurement before the next predict.
  state_pre_.copyTo(state_post_);
  error_cov_pre_.copyTo(error_cov_post_); 
   
  return state_pre_;
}

// change temp variable calculate order
const cv::Mat& ExtendedKalmanFilter::Correct(const cv::Mat& measurement,
  cv::Mat& extra_measurement) {
  // K(k) = P'(k) * JHt * ( JH * P'(k) * JHt + R(k) )^–1
#if 0
  // R(k) = V(k) * R * Vt(k)
  // measurement_noise_tmp_ = jacobian_measurement_noise_
  //                        * measurement_noise_cov_
  //                        * jacobian_measurement_noise_.t();
  JacobianMeasurementNoise(jacobian_measurement_noise_);
  cv::gemm(jacobian_measurement_noise_, measurement_noise_cov_, 1.0f,
    cv::Mat(), 1.0f, measurement_noise_tmp_);
  // LOG(INFO) << "measurement_noise_tmp_ " << measurement_noise_tmp_;

  cv::gemm(measurement_noise_tmp_, jacobian_measurement_noise_, 1.0f,
    cv::Mat(), 1.0f, measurement_noise_tmp_, cv::GEMM_2_T);
  // LOG(INFO) << "measurement_noise_tmp_ " << measurement_noise_tmp_;
  // temp2 = JH * P'(k)
  JacobianMeasurement(state_pre_, jacobian_measurement_, extra_measurement);
  // LOG(INFO) << "jacobian_measurement_ " << jacobian_measurement_;
  temp2_ = jacobian_measurement_* error_cov_pre_;
  // LOG(INFO) << "error_cov_pre_=" << error_cov_pre_;
  // LOG(INFO) << "temp2_=JH * P'(k)= " << temp2_;
  // temp3 = temp2*Ht + R, <-- JH*P'(k)*JHt + R(k)
  cv::gemm(temp2_, jacobian_measurement_, 1.0f,
    measurement_noise_tmp_, 1.0f, temp3_, cv::GEMM_2_T);
  // LOG(INFO) << "temp3_=temp2*Ht + R= " << temp3_;
#elif 1
  // temp2 = JH * P'(k)
  JacobianMeasurement(state_pre_, jacobian_measurement_, extra_measurement);
  temp2_ = jacobian_measurement_* error_cov_pre_;
  // temp3 = temp2*Ht
  cv::gemm(temp2_, jacobian_measurement_, 1.0,
    cv::Mat(), 1.0, temp3_, cv::GEMM_2_T);
  // R(k) = V(k) * R * Vt(k)
  JacobianMeasurementNoise(jacobian_measurement_noise_);
  //shzhw
  cv::setIdentity(measurement_noise_cov_);

  cv::gemm(jacobian_measurement_noise_, measurement_noise_cov_, 1.0,
    cv::Mat(), 1.0, measurement_noise_tmp_);
  // temp3 = temp3 + R, <-- JH*P'(k)*JHt + R(k)
  cv::gemm(measurement_noise_tmp_, jacobian_measurement_noise_, 1.0,
    temp3_, 1.0, temp3_, cv::GEMM_2_T);
#endif


  // cv::Mat temp5;
  // cv::gemm(temp2_, jacobian_measurement_, 1.0f,
  //  cv::Mat(), 1.0f, temp5, cv::GEMM_2_T);

#if 1
  cv::gemm(jacobian_measurement_, error_cov_pre_, 1.0,
        cv::Mat(), 0.0, temp5_, cv::GEMM_2_T);
  cv::solve(temp3_, temp5_, temp4_, cv::DECOMP_SVD);
  // K(k)
  gain_ = temp4_.t();
#else
  // gain = K(k)
  gain_ = error_cov_pre_ * jacobian_measurement_.t()
    * temp3_.inv(cv::DECOMP_SVD);
#endif

  // 2. x(k) = x'(k) + K(k) * ( z(k) – h(x'(k), 0) )
  // temp5 = z(k) - h(x'(k))
  Measurement(state_pre_, temp_measurement_, extra_measurement);
  // LOG(INFO) << "temp_measurement_ = "
  //          << temp_measurement_ << " measurement = " << measurement;
  temp5_ = measurement - temp_measurement_;
  // LOG(INFO) << "temp5_ = " << temp5_;
  // x(k) = x'(k) + K(k) * temp5
  state_post_ = state_pre_ + gain_ * temp5_;
  // LOG(INFO) << "state_post_ = " << state_post_;

  // 3. P(k) = P'(k) - K(k) * JH * P'(k)
  error_cov_post_ = error_cov_pre_ - gain_ * temp2_;
  // LOG(INFO) << "error_cov_post_ = " << error_cov_post_;

  return state_post_;
}
/*
void ExtendedKalmanFilter::SetCvtMatrix(
  const std::vector<const CameraOdo *> &cams,
  const OdometryFrame* cur_odo) {
  cams_ = cams;
  // cur_odo_ = cams[0]->cur_odo_;
  cur_odo_ = *cur_odo;
}

cv::Point2f ExtendedKalmanFilter::cvtImageToVcsGnd(const cv::Point2f& pt,
  int idx) const {
  return cams_[idx]->CvtImageToVcsGnd(pt);
}

cv::Point2f ExtendedKalmanFilter::cvtVcsGndToImage(const cv::Point2f& pt,
  int idx) const {
  return cams_[idx]->CvtVcsGndToImage(pt);
}

cv::Point2f ExtendedKalmanFilter::cvtImageToWorldGnd(const cv::Point2f& pt,
  int idx) const {
  return cams_[idx]->CvtImageToWorldGnd(pt);
}

cv::Point2f ExtendedKalmanFilter::cvtWorldGndToImage(const cv::Point2f& pt,
  int idx) const {
  return cams_[idx]->CvtWorldGndToImage(pt);
}
cv::Point2f ExtendedKalmanFilter::cvtVcsGndToWorldGnd(
  const cv::Point2f& pt) const {
  return cams_[0]->CvtVcsGndToWorldGnd(pt);
}

cv::Point2f ExtendedKalmanFilter::cvtWorldGndToVcsGnd(
  const cv::Point2f& pt) const {
  return cams_[0]->CvtWorldGndToVcsGnd(pt);
}

cv::Point2f ExtendedKalmanFilter::cvtSpeedWorldGndToVcsGnd(
  const cv::Point2f &world_speed) const {
  cv::Point3f v_vcs = cvtSpeedWorldToVcs(cv::Point3f(world_speed.x,
    world_speed.y, 0.0f));
  return cv::Point2f(v_vcs.x, v_vcs.y);
}

cv::Point2f ExtendedKalmanFilter::cvtSpeedVcsGndToWorldGnd(
  const cv::Point2f &vcs_speed) const {
  cv::Point3f v_world = cvtSpeedVcsToWorld(cv::Point3f(vcs_speed.x,
    vcs_speed.y, 0));
  return cv::Point2f(v_world.x, v_world.y);
}

Point2DF ExtendedKalmanFilter::cvtSpeedWorldGndToVcsGnd(
  const Point2DF &world_speed) const {
  cv::Point3f v_vcs = cvtSpeedWorldToVcs(cv::Point3f(world_speed.x,
    world_speed.y, 0.0f));
  return Point2DF(v_vcs.x, v_vcs.y);
}

Point2DF ExtendedKalmanFilter::cvtSpeedVcsGndToWorldGnd(
  const Point2DF &vcs_speed) const {
  cv::Point3f v_world = cvtSpeedVcsToWorld(cv::Point3f(vcs_speed.x,
    vcs_speed.y, 0));
  return Point2DF(v_world.x, v_world.y);
}

cv::Point3f ExtendedKalmanFilter::cvtSpeedWorldToVcs(
  const cv::Point3f &world_speed) const {
  float host_vz = cur_odo_.speed * sinf(cur_odo_.pitch);
  float host_vg = cur_odo_.speed * cosf(cur_odo_.pitch);
  float host_vx = host_vg * cosf(cur_odo_.yaw);
  float host_vy = host_vg * sinf(cur_odo_.yaw);
  float relative_vx = world_speed.x - host_vx;
  float relative_vy = world_speed.y - host_vy;
  float relative_vz = world_speed.z - host_vz;

  return cams_[0]->RotWorldToVcs(cv::Point3f(relative_vx,
    relative_vy, relative_vz));
}

cv::Point3f ExtendedKalmanFilter::cvtSpeedVcsToWorld(
  const cv::Point3f &vcs_speed) const {
  float host_vz = cur_odo_.speed * sinf(cur_odo_.pitch);
  float host_vg = cur_odo_.speed * cosf(cur_odo_.pitch);
  float host_vx = host_vg * cosf(cur_odo_.yaw);
  float host_vy = host_vg * sinf(cur_odo_.yaw);

  cv::Point3f v_world = cams_[0]->RotVcsToWorld(vcs_speed);
  v_world.x += host_vx;
  v_world.y += host_vy;
  v_world.z += host_vz;
  return v_world;
}

cv::Point2f ExtendedKalmanFilter::cvtVcsGndToCamGnd(const cv::Point2f& pt,
  int idx) const {
  return cams_[idx]->CvtVcsGndToCamGnd(pt);
}

cv::Point2f ExtendedKalmanFilter::cvtCamGndToVcsGnd(const cv::Point2f& pt,
  int idx) const {
  return cams_[idx]->CvtCamGndToVcsGnd(pt);
}

cv::Point2f ExtendedKalmanFilter::cvtWorldGndToCamGnd(const cv::Point2f& pt,
  int idx) const {
  return cams_[idx]->CvtWorldGndToCamGnd(pt);
}

cv::Point2f ExtendedKalmanFilter::cvtCamGndToWorldGnd(const cv::Point2f& pt,
  int idx) const {
  return cams_[idx]->CvtCamGndToWorldGnd(pt);
}
*/
}  // namespace common
}  // namespace apollo
