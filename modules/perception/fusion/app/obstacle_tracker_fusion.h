
#pragma once

#include <memory>

#include "modules/perception/fusion/app/target_fusion.h"
#include "modules/perception/camera/common/util.h"
#include "modules/perception/common/geometry/common.h"
#include "modules/perception/base/frame.h"
#include "modules/perception/onboard/proto/fusion_obstacle_tracker.pb.h"
#include <opencv2/opencv.hpp>

namespace apollo {
namespace perception {
namespace fusion {
struct Hypothesis {
  int target;
  int object;
  float score;

  Hypothesis() {
    this->target = -1;
    this->object = -1;
    this->score = -1;
  }

  Hypothesis(int tar, int obj, float score) {
    this->target = tar;
    this->object = obj;
    this->score = score;
  }

  bool operator<(const Hypothesis &b) const { return score < b.score; }

  bool operator>(const Hypothesis &b) const { return score > b.score; }
};

class ObstacleTrackerFusion {
public:
  ObstacleTrackerFusion() {};
  ~ObstacleTrackerFusion(){};
  bool Init(std::string config_path);
  // void predict(base::FramePtr frame);
  void associate(std::vector<base::ObjectPtr> fusedObjs, base::FramePtr frame);
  void CreateNewTarget(std::vector<base::ObjectPtr> fusedObjs, base::FramePtr frame);
  void TrackProc(std::vector<base::ObjectPtr> &fusedObjs, base::FramePtr frame, 
  std::vector<base::ObjectPtr> &validObject, std::vector<cv::Point> radar_pt_in_camera);
  void ClearFusedTarget();
private:
  std::vector<bool> ismatched_detobj_;
  std::vector<FusedTarget> target_objs_;
  TrackParam tracker_param_;

  bool MatchbyTrackerId(base::ObjectPtr curObject, TrackObjPtr traObject);
  float MatchByCenterPt(base::ObjectPtr curObject,
                                FusedTarget targObject);
  float MatchByBox(base::ObjectPtr curObject, TrackObjPtr traObject);
  float MatchByShap(base::ObjectPtr curObject,
                                  TrackObjPtr trackObject);

private:
  static int global_track_id_;
  
};
} // namespace fusion
} // namespace perception
} // namespace apollo