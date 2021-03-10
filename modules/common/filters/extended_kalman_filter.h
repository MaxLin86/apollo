//
// Copyright 2016 Horizon Robotics.
//
#ifndef HOBOT_ADAS_UTILS_KALMAN_FILTER_EXTENDED_KALMAN_FILTER_H_
#define HOBOT_ADAS_UTILS_KALMAN_FILTER_EXTENDED_KALMAN_FILTER_H_
#include "modules/common/filters/kalman_filter.h"

#include <vector>

//#include <hobot-adas/data-structure/type_def.h>
#include <opencv2/opencv.hpp>
//#include <hobot-adas/utils/camera/camera.h>
//#include <hobot-adas/utils/camera/camera_wrapper.h>
//#include <hobot-adas/data-structure/perception.h>
//#include <hobot-adas/data-structure/motion.h>

//class CameraOdo;

namespace apollo {
namespace common {
//class Line;

class ExtendedKalmanFilter : public KalmanFilterInterface {
 public:
  ExtendedKalmanFilter() {
  }

  virtual void Init(int dynam_params, int measure_params,
                    int control_params, int type = CV_32F);

  virtual const cv::Mat& Predict(const cv::Mat& control);
  virtual const cv::Mat& Correct(const cv::Mat& measurement,
                                 cv::Mat& extra_measurement);

  virtual void Transition(const cv::Mat &state_post,
                          cv::Mat &state_pre,
                          const cv::Mat &control) = 0;
  virtual void Measurement(const cv::Mat &steate_pre,
                           cv::Mat &measuremet,
                           cv::Mat &extra_measuremet) = 0;
  virtual void JacobianTrasition(cv::Mat &jacobian_trans,
                                 const cv::Mat &control, const cv::Mat &state_post) = 0;
  virtual void JacobianMeasurement(const cv::Mat &steate_pre,
                                   cv::Mat &jacobian_measurement,
                                   cv::Mat& extra_measurement) = 0;
  virtual void JacobianProcessNoise(cv::Mat &jacobian_process_noise,
                                    const cv::Mat &control) = 0;
  virtual void JacobianMeasurementNoise(cv::Mat &jacobian_measurement_noise)
                                                                           = 0;
  /*
  TimeStamp time_stamp;

  void SetCvtMatrix(const std::vector<const CameraOdo *> &cams,
    const OdometryFrame* cur_odo = nullptr);

  std::vector<const CameraOdo *> cams_;
  OdometryFrame cur_odo_;

  // used cvtImageToGround & cvtGroundToImage
  cv::Point2f cvtImageToVcsGnd(const cv::Point2f& pt, int idx) const;
  cv::Point2f cvtVcsGndToImage(const cv::Point2f& pt, int idx) const;
  // used cvtImageToWorld() & cvtWorldToImage()
  cv::Point2f cvtImageToWorldGnd(const cv::Point2f& pt, int idx) const;
  cv::Point2f cvtWorldGndToImage(const cv::Point2f& pt, int idx) const;
  // used cvt cvtVcsGndToWorld() & cvtWorldToVcsGnd()
  cv::Point2f cvtVcsGndToWorldGnd(const cv::Point2f& pt) const;
  cv::Point2f cvtWorldGndToVcsGnd(const cv::Point2f& pt) const;
  // used cvtVcsToCamgnd() & cvtCamgndToVcs()
  cv::Point2f cvtVcsGndToCamGnd(const cv::Point2f& pt, int idx) const;
  cv::Point2f cvtCamGndToVcsGnd(const cv::Point2f& pt, int idx) const;
  // used cvtWorldToCamgnd() & cvtCamgndToWorld
  cv::Point2f cvtWorldGndToCamGnd(const cv::Point2f& pt, int idx) const;
  cv::Point2f cvtCamGndToWorldGnd(const cv::Point2f& pt, int idx) const;
  // used cvtSpeedWorldToVcsGnd()
  cv::Point2f cvtSpeedWorldGndToVcsGnd(
    const cv::Point2f &world_speed) const;
  // used cvtSpeedVcsGndToWorld()
  Point2DF cvtSpeedVcsGndToWorldGnd(
    const Point2DF &world_speed) const;
  Point2DF cvtSpeedWorldGndToVcsGnd(
    const Point2DF &world_speed) const;
  // used cvtSpeedVcsGndToWorld()
  cv::Point2f cvtSpeedVcsGndToWorldGnd(
    const cv::Point2f &world_speed) const;
  cv::Point3f cvtSpeedWorldToVcs(const cv::Point3f &vcs_speed) const;
  cv::Point3f cvtSpeedVcsToWorld(const cv::Point3f &vcs_speed) const;
  // Point2DF cvtSpeedGlobalGndToVcsGnd(const Point2DF world_speed) const;
  // Point2DF cvtSpeedVcsGndToGlobalGnd(const Point2DF local_speed) const;
  */
  
  cv::Mat jacobian_trasition_;
  cv::Mat jacobian_measurement_;
  cv::Mat jacobian_process_noise_;
  cv::Mat jacobian_measurement_noise_;

 protected:
  cv::Mat process_noise_tmp_;
  cv::Mat measurement_noise_tmp_;

  // temporary matrices
  cv::Mat temp_state_;
  cv::Mat temp_measurement_;
  cv::Mat temp1_;
  cv::Mat temp2_;
  cv::Mat temp3_;
  cv::Mat temp4_;
  cv::Mat temp5_;
};
}  // namespace common
}  // namespace apollo

#endif  // HOBOT_ADAS_UTILS_KALMAN_FILTER_EXTENDED_KALMAN_FILTER_H_
