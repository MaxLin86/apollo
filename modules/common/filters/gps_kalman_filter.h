//
//
#ifndef HOBOT_ADAS_UTILS_KALMAN_FILTER_GPS_KALMAN_FILTER_H_
#define HOBOT_ADAS_UTILS_KALMAN_FILTER_GPS_KALMAN_FILTER_H_
#include "modules/common/filters/extended_kalman_filter.h"
#include <vector>
#include <opencv2/opencv.hpp>

namespace apollo {
namespace common {


// for selfcar localization EKF/KF
// input/output measurement is
//     (localization_px, localization_py)
// output state (x, y, theta)
// ekf state is (x, y, theta)
// ekf measurement is (guidepost_x, guidepost_y)
// control is dt, unit is second

class GPSKF : public ExtendedKalmanFilter {
 public:
  explicit GPSKF(int kf_coor_sys = 1);
  virtual ~GPSKF();

  virtual void Init(cv::Point2d init_pose, double init_theta);

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

  virtual void SetErrorCov();
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

  void SetMeasureNoise(double measurement_px_noise,
    double measurement_py_noise, double measurement_heading_noise) {
      measurement_px_noise_ = measurement_px_noise;
      measurement_py_noise_ = measurement_py_noise;
      measurement_heading_noise_ = measurement_heading_noise;
    }

  void SetNoise(const double &init_noise,
    const double &process_noise,
    const double &measure_noise) {
    init_noise_ = init_noise;
    process_noise_ = process_noise;
    measure_noise_ = measure_noise;
  }
  void EnableSpeedLimit();
  void DisableSpeedLimit();
  // xzhj
  bool enable_filter_protect_;
 protected:
  // kf stage ,cnt less than 20, convergence stage
  int correct_count_;
  bool re_init_;
  int predict_count_;
  // width in image


  double init_noise_;
  double process_noise_;
  double measure_noise_;

  // 0:vcs 1:world
  int kf_coordinate_sys_;  //

  // measurement noise
  double measurement_px_noise_;
  double measurement_py_noise_;
  double measurement_heading_noise_;
};

class TrailerKF : public ExtendedKalmanFilter {
 public:
  explicit TrailerKF(int kf_coor_sys = 1);
  virtual ~TrailerKF();

  virtual void Init(cv::Point2d init_pose, double init_vel, double init_theta);

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

  virtual void SetErrorCov();
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

  void SetMeasureNoise(double measurement_px_noise,
    double measurement_py_noise, double measurement_heading_noise) {
      measurement_px_noise_ = measurement_px_noise;
      measurement_py_noise_ = measurement_py_noise;
      measurement_heading_noise_ = measurement_heading_noise;
    }

  void SetNoise(const double &init_noise,
    const double &process_noise,
    const double &measure_noise) {
    init_noise_ = init_noise;
    process_noise_ = process_noise;
    measure_noise_ = measure_noise;
  }
  void EnableSpeedLimit();
  void DisableSpeedLimit();
  // xzhj
  bool enable_filter_protect_;
 protected:
  // kf stage ,cnt less than 20, convergence stage
  int correct_count_;
  bool re_init_;
  int predict_count_;
  // width in image


  double init_noise_;
  double process_noise_;
  double measure_noise_;

  // 0:vcs 1:world
  int kf_coordinate_sys_;  //

  // measurement noise
  double measurement_px_noise_;
  double measurement_py_noise_;
  double measurement_heading_noise_;
};

//----------------------------------------------------------------------------------
class VehicleCYRAKF : public ExtendedKalmanFilter {
 public:
  explicit VehicleCYRAKF(int kf_coor_sys = 1);
  virtual ~VehicleCYRAKF();

  virtual void Init(cv::Point2d init_pose, double init_vel, double init_theta);

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

  virtual void SetErrorCov();
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

  void SetMeasureNoise(double measurement_px_noise,
    double measurement_py_noise, double measurement_heading_noise) {
      measurement_px_noise_ = measurement_px_noise;
      measurement_py_noise_ = measurement_py_noise;
      measurement_heading_noise_ = measurement_heading_noise;
    }

  void SetNoise(const double &init_noise,
    const double &process_noise,
    const double &measure_noise) {
    init_noise_ = init_noise;
    process_noise_ = process_noise;
    measure_noise_ = measure_noise;
  }
  void EnableSpeedLimit();
  void DisableSpeedLimit();
  // xzhj
  bool enable_filter_protect_;
 protected:
  // kf stage ,cnt less than 20, convergence stage
  int correct_count_;
  bool re_init_;
  int predict_count_;
  // width in image


  double init_noise_;
  double process_noise_;
  double measure_noise_;

  // 0:vcs 1:world
  int kf_coordinate_sys_;  //

  // measurement noise
  double measurement_px_noise_;
  double measurement_py_noise_;
  double measurement_heading_noise_;
};


}  // namespace common
}  // namespace apollo

#endif  // HOBOT_ADAS_UTILS_KALMAN_FILTER_GPS_KALMAN_FILTER_H_
