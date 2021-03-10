/******************************************************************************
 * 
 *****************************************************************************/

#pragma once

#include "modules/common/filters/self_car_localization_ekf.h"
#include "modules/common/filters/gps_kalman_filter.h"
#include <vector>
#include <string>
#include <memory>
#include "opencv2/opencv.hpp"

#include "modules/localization/proto/localization.pb.h"
#include "modules/canbus/proto/chassis.pb.h"
#include "modules/common/math/quaternion.h"

#include "modules/drivers/gnss/proto/gnss_best_pose.pb.h"
#include "modules/drivers/gnss/proto/heading.pb.h"

#include "modules/common/filters/ins_guidepost_fusion_ekf.h"
#include "modules/routing/proto/routing.pb.h"
#include "modules/localization/msf/common/util/frame_transform.h"




/**
 * @namespace apollo::planning
 * @brief apollo::planning
 */
namespace apollo {
namespace localization {
/**
 * @class localization
 *
 * @brief PlanningBase module main class.
 */

//class apollo::common::KalmanFilterInterface;


  struct LocalView {
  std::shared_ptr<canbus::Chassis> chassis;
  //canbus::Chassis chassis;
  std::shared_ptr<LocalizationEstimate> localization_estimate;
    std::shared_ptr<LocalizationStatus> localization_status;  
  std::shared_ptr<drivers::gnss::GnssBestPose> best_gnss_pos;
  std::shared_ptr<drivers::gnss::Heading> gnss_heading;
  std::shared_ptr<routing::RoutingResponse> routing;
  };
  
struct GuidepostGroup
{
public:
  std::string id;//路标组路标1 id
	double x;//路标组路标1 x坐标
  double y;//路标组路标1 y坐标
};

class SelfCarLocalizationTracking {
 public:
  apollo::common::KalmanFilterInterface *kalman_filter_;
  SelfCarLocalizationTracking(){
    kf_coor_sys_ = 1;
  }
  ~SelfCarLocalizationTracking(){
    delete kalman_filter_;
  }
  apollo::common::KalmanFilterInterface* CreateKalmanFilter(int kf_coor_sys);
  // Predict object rect

  bool InitKalmanFilter(LocalView &local_view, LocalizationEstimate &start_localization, LocalizationEstimate &init_localization);

  void KalmanFilterPredict(LocalView &local_view);
  void KalmanFilterCorrect(LocalView &local_view);

  static int kf_coor_sys_;
  LocalView local_view_;

};

class GPSLocalizationFilter {
   public:
  apollo::common::KalmanFilterInterface *kalman_filter_;
  GPSLocalizationFilter(){
    kf_coor_sys_ = 1;
  }
  ~GPSLocalizationFilter(){
    delete kalman_filter_;
  }
  apollo::common::KalmanFilterInterface* CreateKalmanFilter(int kf_coor_sys);
  // Predict object rect
  bool InitKalmanFilter0(LocalView &local_view, cv::Point2d &start_utm_xy,cv::Point2d &init_utm_xy, double gnss_heading);
  bool InitKalmanFilter(LocalView &local_view, cv::Point2d &start_utm_xy,cv::Point2d &init_utm_xy, double gnss_heading, double gnss_heading_std_dev);

  void KalmanFilterPredict(LocalView &local_view, double gnss_heading_std_dev);
  void KalmanFilterCorrect(LocalView &local_view, cv::Point2d utm_xy, cv::Point2d utm_xy_std_dev, 
        double gnss_heading, double gnss_heading_std_dev, bool is_msf_heading);

  static int kf_coor_sys_;
  LocalView local_view_;
};

class TrailerLocalizationFilter {
  public:
  apollo::common::KalmanFilterInterface *kalman_filter_;
  TrailerLocalizationFilter(){
    kf_coor_sys_ = 1;
  }
  ~TrailerLocalizationFilter(){
    delete kalman_filter_;
  }
  apollo::common::KalmanFilterInterface* CreateKalmanFilter(int kf_coor_sys);
  // Predict object rect
  bool InitKalmanFilter(cv::Point2d &init_vehicle_xy, double init_vehicle_heading, double init_trailer_heading);

  void KalmanFilterPredict(LocalView &local_view);
  void KalmanFilterCorrect(cv::Point2d vehicle_xy, cv::Point2d vehicle_xy_std_dev, 
        double vehicle_heading, double vehicle_heading_std_dev);

  static int kf_coor_sys_;
  LocalView local_view_;
};

class MSFLocalizationDRCorrectFilter {
   public:
  apollo::common::KalmanFilterInterface *kalman_filter_;
    apollo::common::KalmanFilterInterface *guidepost_kalman_filter_;
  MSFLocalizationDRCorrectFilter(){
    kf_coor_sys_ = 1;
  }
  ~MSFLocalizationDRCorrectFilter(){
    delete kalman_filter_;
    delete guidepost_kalman_filter_;
  }
  apollo::common::KalmanFilterInterface* CreateKalmanFilter(int kf_coor_sys);
  // Predict object rect
  bool InitKalmanFilter(LocalView &local_view);
  void KalmanFilterPredict(LocalView &local_view);
  void KalmanFilterCorrect(LocalView &local_view, bool is_update_gps);


  apollo::common::KalmanFilterInterface* CreateGuidepostKalmanFilter(int kf_coor_sys);
  void KalmanFilterGuidepostCorrect(LocalView &local_view, const std::vector<GuidepostGroup> &guideposts);
  

  static int kf_coor_sys_;
  LocalView local_view_;

  std::vector<GuidepostGroup> guidepost_groups_; 
  uint64 cur_perception_guidepost_index_;
};


}  // namespace localization
}  // namespace apollo
