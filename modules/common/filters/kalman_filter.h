//
//

#ifndef HOBOT_ADAS_UTILS_KALMAN_FILTER_KALMAN_FILTER_H_
#define HOBOT_ADAS_UTILS_KALMAN_FILTER_KALMAN_FILTER_H_

#include "opencv2/opencv.hpp"


//#ifdef LOW_GCC_VERSION
//#define override
//#endif
namespace apollo {
namespace common {

class KalmanFilterInterface {
 public:
  KalmanFilterInterface() {
  }
  virtual ~KalmanFilterInterface() {
  }

  virtual void Init(int dynam_params, int measure_params,
                    int control_params, int type = CV_32F);

  virtual const cv::Mat& Predict(const cv::Mat& control) = 0;
  virtual const cv::Mat& Correct(const cv::Mat& measurement,
                                 cv::Mat& extra_measurement) = 0;


  cv::Mat state_pre_;              // predicted state
                                  // (x'(k)): x(k)=A*x(k-1)+B*u(k)
  cv::Mat state_post_;             // corrected state
                                  // (x(k)): x(k)=x'(k)+K(k)*(z(k)-H*x'(k))
  cv::Mat transition_matrix_;      // state transition matrix (A)
  cv::Mat control_matrix_;         // control matrix (B)
                                  // (not used if there is no control)
  cv::Mat measurement_matrix_;     // measurement matrix (H)
  cv::Mat process_noise_cov_;      // process noise covariance matrix (Q)
  cv::Mat measurement_noise_cov_;  // measurement noise covariance matrix (R)
  cv::Mat error_cov_pre_;          // priori error estimate covariance matrix
                                  // (P'(k)): P'(k)=A*P(k-1)*At + Q)*/
  cv::Mat gain_;                   // Kalman gain_ matrix
                                  // (K(k)): K(k)=P'(k)*Ht*inv(H*P'(k)*Ht+R)
  cv::Mat error_cov_post_;         // posteriori err estimate covariance matrix
                                  // (P(k)): P(k)=(I-K(k)*H)*P'(k)
};

class KalmanFilter : public KalmanFilterInterface {
 public:
  KalmanFilter() {
  }

  void Init(int dynam_params, int measure_params,
            int control_params, int type = CV_32F) override;

  const cv::Mat& Predict(const cv::Mat& control) override;
  const cv::Mat& Correct(const cv::Mat& measurement,
                         cv::Mat& extra_measurement) override;

 public:
  // temporary matrices
  cv::Mat temp1_;
  cv::Mat temp2_;
  cv::Mat temp3_;
  cv::Mat temp4_;
  cv::Mat temp5_;
};   // KalmanFilter


class PointKalmanFilter : public KalmanFilter {
 public:
  PointKalmanFilter() {
    KalmanFilter::Init(4, 2, 0);

    transition_matrix_ = (cv::Mat_<float>(4, 4) <<
      1, 0, 1, 0,
      0, 1, 0, 1,
      0, 0, 1, 0,
      0, 0, 0, 1);
    cv::setIdentity(measurement_matrix_,
      cv::Scalar(1.f));
    // kalman_filter_.control_matrix_->setTo(cv::Scalar::all(0));
    cv::setIdentity(process_noise_cov_,
      cv::Scalar::all(1e-5));
    cv::setIdentity(measurement_noise_cov_,
      cv::Scalar::all(1e-1));
    cv::setIdentity(error_cov_post_,
      cv::Scalar::all(1));
  }
};   // KalmanFilter

class ValueKalmanFilter : public KalmanFilter {
 public:
  ValueKalmanFilter() {
    KalmanFilter::Init(2, 1, 0);

    transition_matrix_ = (cv::Mat_<float>(2, 2) <<
                          1, 1,
                          0, 1);
    cv::setIdentity(measurement_matrix_,
                    cv::Scalar(1.f));
    // kalman_filter_.control_matrix_->setTo(cv::Scalar::all(0));
    cv::setIdentity(process_noise_cov_,
                    cv::Scalar::all(1e-5));
    cv::setIdentity(measurement_noise_cov_,
                    cv::Scalar::all(1e-3));
    cv::setIdentity(error_cov_pre_,
                    cv::Scalar::all(1e-1));
    cv::setIdentity(error_cov_post_,
                    cv::Scalar::all(1e-1));
  }

  const cv::Mat& Predict(const cv::Mat& control) override;
  const cv::Mat& Correct(const cv::Mat& measurement,
                         cv::Mat& extra_measurement) override;
};

}  // namespace apollo
}  // namespace common

#endif  // HOBOT_ADAS_UTILS_KALMAN_FILTER_KALMAN_FILTER_H_
