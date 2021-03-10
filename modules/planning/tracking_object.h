/******************************************************************************
 * 
 *****************************************************************************/

//#pragma once

//#ifndef HOBOT_ADAS_TRACKING_TRACKING_OBJECT_H_
//#define HOBOT_ADAS_TRACKING_TRACKING_OBJECT_H_
#include "modules/common/filters/tracking_obj_ekf.h"
#include "modules/planning/common/local_view.h"
#include <vector>
#include <string>
#include <memory>
#include "opencv2/opencv.hpp"




/**
 * @namespace apollo::planning
 * @brief apollo::planning
 */
namespace apollo {
namespace planning {
/**
 * @class planning
 *
 * @brief PlanningBase module main class.
 */

//class apollo::common::KalmanFilterInterface;
/*
class KalmanTrackingInterface {
 public:
  apollo::common::KalmanFilterInterface *kalman_filter_;
  //std::unique_ptr<apollo::common::KalmanFilterInterface> kalman_filter_;

  virtual apollo::common::KalmanFilterInterface* CreateKalmanFilter(int kf_coor_sys) = 0;
  virtual bool InitKalmanFilter(LocalView &local_view, const std::vector<GuidepostGroup> &guideposts) = 0;
  virtual void KalmanFilterPredict(LocalView &local_view) = 0;
  virtual void KalmanFilterCorrect(LocalView &local_view, const std::vector<GuidepostGroup> &guideposts) = 0;

};
*/
class TrackingSelfCarObject {
 public:
  apollo::common::KalmanFilterInterface *kalman_filter_;
  TrackingSelfCarObject(){
    kf_coor_sys_ = 1;
  }
  ~TrackingSelfCarObject(){
    delete kalman_filter_;
  }
  apollo::common::KalmanFilterInterface* CreateKalmanFilter(int kf_coor_sys);
  // Predict object rect
  bool InitKalmanFilter(LocalView &local_view, const std::vector<GuidepostGroup> &guideposts);

  void KalmanFilterPredict(LocalView &local_view);
  void KalmanFilterCorrect(LocalView &local_view, const std::vector<GuidepostGroup> &guideposts);
  /*int UpdateCurrentFrame(const std::vector<const CameraOdo *> &cameras,
      Rect<float> rect_obs,
      Rect<float> rect_obs_ori,
      float conf, int track_source, int type = 0) override;

  */
  static int kf_coor_sys_;



};
/*
class TrackingSelfCarObject : public KalmanTrackingInterface {
 public:
   TrackingSelfCarObject();
  virtual ~TrackingSelfCarObject();
  apollo::common::KalmanFilterInterface* CreateKalmanFilter(int kf_coor_sys) override;
  // Predict object rect
  bool InitKalmanFilter(LocalView &local_view, const std::vector<GuidepostGroup> &guideposts) override;

  void KalmanFilterPredict(LocalView &local_view) override;
  void KalmanFilterCorrect(LocalView &local_view, const std::vector<GuidepostGroup> &guideposts) override;

  static int kf_coor_sys_;



};
*/

}  // namespace planning
}  // namespace apollo
