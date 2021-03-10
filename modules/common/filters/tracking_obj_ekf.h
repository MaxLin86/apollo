//
//
#ifndef HOBOT_ADAS_UTILS_KALMAN_FILTER_TRACKING_OBJ_EKF_H_
#define HOBOT_ADAS_UTILS_KALMAN_FILTER_TRACKING_OBJ_EKF_H_
#include "modules/common/filters/extended_kalman_filter.h"
#include <vector>
#include <opencv2/opencv.hpp>




namespace apollo {
namespace common {


// for selfcar EKF
// input/output measurement is
//     (guidepost_x, guidepost_y)
// output state (x, y, theta)
// ekf state is (x, y, theta)
// ekf measurement is (guidepost_x, guidepost_y)
// control is dt, unit is second

class SelfCarEKF : public ExtendedKalmanFilter {
 public:
  explicit SelfCarEKF(int kf_coor_sys = 1);
  virtual ~SelfCarEKF();

  virtual void Init(cv::Point2f init_pose, double init_theta);

  const cv::Mat& Predict(const cv::Mat& control) override;
  const cv::Mat& Correct(const cv::Mat& measurement,
    cv::Mat& extra_measurement) override;

  void Transition(const cv::Mat &state_post,
    cv::Mat &state_pre,
    const cv::Mat &control) override;
  void Measurement(const cv::Mat &steate_pre,
    cv::Mat &measuremet,
    cv::Mat &extra_measuremet) override;
  void JacobianTrasition(cv::Mat &jacobian_trans,
    const cv::Mat &control, const cv::Mat &state_post) override;
  void JacobianMeasurement(const cv::Mat &steate_pre,
    cv::Mat &jacobian_measurement,
    cv::Mat &extra_measuremet) override;

  void JacobianProcessNoise(cv::Mat &jacobian_process_noise,
    const cv::Mat &control) override;
  void JacobianMeasurementNoise(cv::Mat &jacobian_measurement_noise) override;

  virtual void SetVel(const cv::Point2f &ref_v);
  virtual void SetErrorCov();
  void SetObsDist(float distance) {
    obs_dist_ = distance;
  }
  int GetCorrectCount() {
    return correct_count_;
  }
  void SetCorrectCount(int cor) {
    correct_count_ = cor;
  }
  void SetPredictCount(int pre_count) {
    predict_count_ = pre_count;
  }
  void SetReInit(bool reinit) {
    re_init_ = reinit;
  }
  bool GetReInit() {
    return re_init_;
  }

  void SetNoise(const float &init_noise,
    const float &process_noise,
    const float &measure_noise) {
    init_noise_ = init_noise;
    process_noise_ = process_noise;
    measure_noise_ = measure_noise;
  }
  void EnableSpeedLimit();
  void DisableSpeedLimit();
  // xzhj
  bool rect_yaw_en_;
  float rect_yaw_sta_;
  bool enable_filter_protect_;
  bool distekf_with_state_;
  int obj_states_;

 protected:
  // kf stage ,cnt less than 20, convergence stage
  int correct_count_;
  float obs_dist_;
  bool re_init_;
  int predict_count_;
  // width in image

  bool use_height_;

  float init_noise_;
  float process_noise_;
  float measure_noise_;

  // 0:vcs 1:world
  int kf_coordinate_sys_;  //
};

}  // namespace common
}  // namespace apollo

#endif  // HOBOT_ADAS_UTILS_KALMAN_FILTER_TRACKING_OBJ_EKF_H_
