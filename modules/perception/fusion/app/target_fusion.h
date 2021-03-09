#pragma once

#include <vector>
#include <memory>

#include "modules/perception/base/object.h"
#include "modules/perception/fusion/common/kalman_filter.h"
#include "modules/perception/camera/lib/obstacle/tracker/common/kalman_filter.h"
#include "modules/perception/onboard/proto/fusion_obstacle_tracker.pb.h"
#include "modules/perception/base/frame.h"



namespace apollo {
namespace perception {
namespace fusion {

struct TrackObj {
  int fusion_mode_;
  int radar_detid;
  int camera_detid;
  double timestamp;
  base::ObjectPtr object;
};
typedef std::shared_ptr<TrackObj> TrackObjPtr;
typedef std::vector<TrackObjPtr> FusedTrackObjPtrs;

class FusedTarget {
 public:
  FusedTarget(const TrackParam &param, base::ObjectPtr object);
  ~FusedTarget() {};
  void Init(const TrackParam &param, base::ObjectPtr object);
  
  camera::KalmanFilterConstVelocity imgPosKF_;
  camera::KalmanFilterConstVelocity PositionKF_;

  KalmanFilter positionkf_;

  Eigen::Matrix2d cam_measureVar_;
  Eigen::Matrix4d cam_processVar_;
  Eigen::Matrix2d radar_measureVar_;
  Eigen::Matrix4d radar_processVar_;

  void Add(base::ObjectPtr detObject);
  void Predict(base::FramePtr frame);
  // void predict();

  // void removeold();

  void Clear();

  void update();

  int Size() const;

  TrackObjPtr get_object(int index) const;
  TrackObjPtr operator[](int index) const;

  int track_id_;
  int lost_age_;
  int reverse_age_;
  int life_age_;
  float timestamp;

  TrackObjPtr latest_object_;
  FusedTrackObjPtrs fused_tracked_objs_;

 private:
  Eigen::Matrix<double,6,6> process_variance_;
  Eigen::Matrix4d measure_variance_;
  Eigen::Matrix<double,6,6> init_variance_;


};

} // namespace fusion
} // namespace perception
} // namespace apollo