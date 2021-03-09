
#include "cyber/common/file.h"
#include "modules/perception/fusion/app/obstacle_tracker_fusion.h"



namespace apollo {
namespace perception {
namespace fusion {
  using cyber::common::GetAbsolutePath;
  int ObstacleTrackerFusion::global_track_id_ = 0;
  bool ObstacleTrackerFusion::Init(std::string config_path){
    //导入配置文件
    std::string config_dir = GetAbsolutePath(config_path, "fusion_obstacle_tracker.pb.txt");    
    if(!cyber::common::GetProtoFromFile(config_dir,&tracker_param_)){
      return false;
    } 
    ismatched_detobj_.clear();
    target_objs_.clear();
    return true;
  }


  void ObstacleTrackerFusion::CreateNewTarget(
      std::vector<base::ObjectPtr> fusedObjs, base::FramePtr frame){
    for(size_t i = 0; i < fusedObjs.size(); i++) {
      if(!ismatched_detobj_[i]) {
        FusedTarget newTarget(tracker_param_, fusedObjs[i]);
        newTarget.track_id_ = global_track_id_;
        global_track_id_++;
        newTarget.Add(fusedObjs[i]);
        newTarget.latest_object_->timestamp = frame->timestamp;
        target_objs_.push_back(newTarget);
      }
    }
    return;
  }

  void ObstacleTrackerFusion::ClearFusedTarget() {
    int left = 0;
    int end = static_cast<int>(target_objs_.size() - 1);
    while (left <= end) {
      if ((target_objs_[left].Size() == 0)) {
        while ((left < end) && (target_objs_[end].Size() == 0)) {
          --end;
        }
        if (left >= end) {
          break;
        }
        target_objs_[left] = target_objs_[end];
        --end;
      }
      ++left;
    }
    target_objs_.erase(target_objs_.begin() + left, target_objs_.end());
  }
  
  void ObstacleTrackerFusion::associate(std::vector<base::ObjectPtr> fusedObjs, base::FramePtr frame){
    // predict
    for(auto &target:target_objs_){
      target.Predict(frame);
    }

    ismatched_detobj_.clear();
    ismatched_detobj_.assign(fusedObjs.size(), false);
    std::vector<bool> ismatched_target(target_objs_.size(), false);
    std::vector<Hypothesis> hypos;
    for(size_t i = 0; i < target_objs_.size(); i++) {
      Hypothesis pair;
      pair.target = i;
      auto target = target_objs_[i];
      if(target.fused_tracked_objs_.size() <= 0) {
        AERROR << "FUSED TRACKED OBJS SIZE LESS 0";
      }
      auto traobject = target[-1];
      for(size_t j = 0; j < fusedObjs.size(); j++){
        pair.object = j;
        auto object = fusedObjs[j];
        if(MatchbyTrackerId(object, traobject)) {
          pair.score = 0.6f;
        } else {
          pair.score = MatchByBox(object,traobject);
        }
        if(pair.score >= 0.6f) {
          hypos.push_back(pair);
        }
      }
    }

    std::sort(hypos.begin(), hypos.end(), std::less<Hypothesis>());
    for(auto &pair:hypos) {
      if(ismatched_detobj_[pair.object] || ismatched_target[pair.target]) {
        continue;
      }
      FusedTarget &target = target_objs_[pair.target];
      auto det_obj = fusedObjs[pair.object];
      target.Add(det_obj);
      target.latest_object_->timestamp = frame->timestamp;
      ismatched_target[pair.target] = true;
      ismatched_detobj_[pair.object] = true;
    }
    for(size_t i = 0; i < target_objs_.size(); i++) {
      if(!ismatched_target[i]) {
        target_objs_[i].lost_age_++;
      }
    }

    CreateNewTarget(fusedObjs, frame);

    for(auto &target:target_objs_) {
      if(target.lost_age_ > tracker_param_.reserve_age()) {
        target.Clear();
      } else {
        // correct
        target.update();
      }
    }

    ClearFusedTarget();
    return;
  }

  
  bool ObstacleTrackerFusion::MatchbyTrackerId(base::ObjectPtr curObject, TrackObjPtr traObject){
    bool retMatch = false;
    int cur_camera_id = curObject->raw_camera_detid_;
    int pre_camera_id = traObject->camera_detid;
    if((cur_camera_id != -1) && (cur_camera_id == pre_camera_id)) {
      retMatch = true;
    } 
    return retMatch;
  }
  
  float ObstacleTrackerFusion::MatchByBox(base::ObjectPtr curObject,
                              TrackObjPtr trackObject){
    // target use predict result
    auto curbox = curObject->camera_supplement.box;
    auto prebox = trackObject->object->camera_supplement.box;
    float fIoU = common::CalculateIOUBBox(curbox, prebox);
    return fIoU;
  }

  float ObstacleTrackerFusion::MatchByCenterPt(base::ObjectPtr curObject,
                                  FusedTarget target) {
    auto state = target.imgPosKF_.get_state();
    cv::Point targPt(state(0), state(1));
    auto detBox = curObject->camera_supplement.box;
    cv::Point detPt;
    detPt.x = (detBox.xmax - detBox.xmin) / 2;
    detPt.y = (detBox.ymax - detBox.ymin) / 2;
    float fdis = sqrt((targPt.x - detPt.x) * (targPt.x - detPt.x) +
    (targPt.y - detPt.y) * (targPt.y -detPt.y));
    return fdis;
  }

  float ObstacleTrackerFusion::MatchByShap(base::ObjectPtr curObject,
                                  TrackObjPtr trackObject) {
    base::RectF currect(curObject->camera_supplement.box);
    base::RectF rect(trackObject->object->camera_supplement.box);
    float s = static_cast<float>((currect.height - rect.height) *
                               (currect.width - rect.width) / (currect.height * currect.width));
    return -std::abs(s);
  }

  void ObstacleTrackerFusion::TrackProc(std::vector<base::ObjectPtr> &fusedObject,
                    base::FramePtr frame,
                    std::vector<base::ObjectPtr> &validObject,
                    std::vector<cv::Point> radar_pt_in_camera) {
    associate(fusedObject, frame);
    validObject.clear();
    for(auto target:target_objs_) {
      if(target.lost_age_ < tracker_param_.lost_age()) {
        base::ObjectPtr validobj = target.latest_object_->object;
        validobj->track_id = target.track_id_;
        validObject.push_back(target.latest_object_->object);
      }
    }
  }

} // namespace fusion
} // namespace perception
} // namespace apollo 