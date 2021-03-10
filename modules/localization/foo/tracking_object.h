/******************************************************************************
 * 
 *****************************************************************************/

//#pragma once

//#ifndef HOBOT_ADAS_TRACKING_TRACKING_OBJECT_H_
//#define HOBOT_ADAS_TRACKING_TRACKING_OBJECT_H_
#include "modules/common/filters/ins_guidepost_fusion_ekf.h"
//#include "modules/map/relative_map/relative_map.h"
#include <vector>
#include <string>
#include <memory>
#include "opencv2/opencv.hpp"


#include "modules/routing/proto/routing.pb.h"//add 20200708
#include "modules/localization/proto/localization.pb.h"
#include "modules/canbus/proto/chassis.pb.h"

#include "modules/common/filters/self_car_localization_ekf.h"

/**
 * @namespace apollo::planning
 * @brief apollo::planning
 */
namespace apollo {
namespace localization {
/**
 * @class planning
 *
 * @brief PlanningBase module main class.
 */

  struct NewLocalView {
  std::shared_ptr<canbus::Chassis> chassis;
  std::shared_ptr<LocalizationEstimate> localization_estimate;
  std::shared_ptr<routing::RoutingResponse> routing;
  };
  /**/


    //路标组构造函数（自定义坐标系下） add by shzhw
struct NewGuidepostGroup
{
public:
  std::string id;//路标组路标1 id
	double x;//路标组路标1 x坐标
  double y;//路标组路标1 y坐标
};
//---------------------------------

class TrackingSelfCarObject {
 public:
  apollo::common::KalmanFilterInterface *ins_kalman_filter_;
  apollo::common::KalmanFilterInterface *guidepost_kalman_filter_;


  TrackingSelfCarObject(){
    kf_coor_sys_ = 1;
  }
  ~TrackingSelfCarObject(){
    delete ins_kalman_filter_;
    delete guidepost_kalman_filter_;
  }
  apollo::common::KalmanFilterInterface* CreateINSreKalmanFilter(int kf_coor_sys);
  apollo::common::KalmanFilterInterface* CreateGuidepostKalmanFilter(int kf_coor_sys);
  // Predict object rect
  bool InitKalmanFilter(bool ins_localization_valid, NewLocalView &local_view, const std::vector<NewGuidepostGroup> &guideposts);

  void KalmanFilterPredict(NewLocalView &local_view);
  void KalmanFilterGuidepostCorrect(NewLocalView &local_view, const std::vector<NewGuidepostGroup> &guideposts);
  void KalmanFilterINSCorrect(NewLocalView &local_view);
  /*int UpdateCurrentFrame(const std::vector<const CameraOdo *> &cameras,
      Rect<float> rect_obs,
      Rect<float> rect_obs_ori,
      float conf, int track_source, int type = 0) override;

  */
  static int kf_coor_sys_;
  NewLocalView new_local_view_;
  std::vector<NewGuidepostGroup> guidepost_groups_;

  uint64 cur_perception_guidepost_index_;


};


}  // namespace planning
}  // namespace apollo
