/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the License);
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an AS IS BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/
#include "modules/perception/camera/lib/obstacle/tracker/omt/target.h"

#include <algorithm>
#include <map>

#include "cyber/common/log.h"
#include "modules/perception/camera/common/math_functions.h"
#include "modules/perception/camera/common/util.h"
#include "modules/perception/common/geometry/basic.h"

namespace apollo {
namespace perception {
namespace camera {

int Target::global_track_id = 0;
int Target::Size() const { return static_cast<int>(tracked_objects.size()); }

void Target::Clear() { tracked_objects.clear(); }

TrackObjectPtr Target::operator[](int index) const { return get_object(index); }
TrackObjectPtr Target::get_object(int index) const {
  CHECK_GT(tracked_objects.size(), 0);
  CHECK_LT(index, static_cast<int>(tracked_objects.size()));
  CHECK_GE(index, -static_cast<int>(tracked_objects.size()));
  return tracked_objects[(index + tracked_objects.size()) %
                         tracked_objects.size()];
}
void Target::Add(TrackObjectPtr object) {
  if (tracked_objects.empty()) {
    start_ts = object->timestamp;
    id = Target::global_track_id++;
    type = object->object->sub_type;
  }
  object->object->track_id = id;
  object->object->tracking_time = object->timestamp - start_ts;
  object->object->latest_tracked_time = object->timestamp;
  latest_object = object;
  lost_age = 0;
  life_age_++;
  tracked_objects.push_back(object);
}
void Target::RemoveOld(int frame_id) {
  size_t index = 0;
  while (index < tracked_objects.size() &&
         tracked_objects[index]->indicator.frame_id < frame_id) {
    ++index;
  }
  tracked_objects.erase(tracked_objects.begin(),
                        tracked_objects.begin() + index);
}
void Target::Init(const omt::TargetParam &param) {
  target_param_ = param;
  id = -1;
  lost_age = 0;
  life_age_ = 0;
  vec_objtype_.clear();
  type_voter_.assign(static_cast<int>(base::ObjectSubType::MAX_OBJECT_TYPE),0);
  image_wh.SetAlpha(param.image_wh_update_rate());
  // TODO(gaohan)  should update variance when predict and update
  image_center.variance_ *= target_param_.image_center().init_variance();
  image_center.measure_noise_ *=
      target_param_.image_center().measure_variance();
  image_center.process_noise_ *=
      target_param_.image_center().process_variance();
  // record history state after kalman filter
  history_world_states_.set_capacity(param.world_state_history());
}
Target::Target(const omt::TargetParam &param) { Init(param); }

void Target::Predict(CameraFrame *frame) {
  auto delta_t =
      static_cast<float>(frame->timestamp - latest_object->timestamp);
  if (delta_t < 0) {
    return;
  }
  image_center.Predict(delta_t);
}

void Target::Update2D(CameraFrame *frame) {
  // measurements
  auto obj = latest_object->object;
  float width = static_cast<float>(frame->data_provider->src_width());
  float height = static_cast<float>(frame->data_provider->src_height());
  base::RectF rect(latest_object->projected_box);
  base::Point2DF center = rect.Center();
  if(lost_age == 0){//!isLost()
    Eigen::Vector2d measurement;
    measurement << rect.width, rect.height;
    image_wh.AddMeasure(measurement);
    measurement << center.x, center.y;
    image_center.Correct(measurement);
    // update 2d bbox size
    Eigen::Vector4d state;
    state = image_center.get_state();
    center.x = static_cast<float>(state(0));
    center.y = static_cast<float>(state(1));
    ADEBUG << "2d move:" << id << " " << state(2) << " " << state(3);
    auto shape = image_wh.get_state();
    rect.width = static_cast<float>(shape(0));
    rect.height = static_cast<float>(shape(1));
    rect.SetCenter(center);
    RefineBox(rect, width, height, &rect);
    latest_object->projected_box = rect;
  } else {
    Eigen::Vector4d state;
    state = image_center.get_state();
    center.x = static_cast<float>(state(0));
    center.y = static_cast<float>(state(1));
    ADEBUG << "2d move:" << id << " " << state(2) << " " << state(3);
    auto shape = image_wh.get_state();
    rect.width = static_cast<float>(shape(0));
    rect.height = static_cast<float>(shape(1));
    rect.SetCenter(center);
    RefineBox(rect, width, height, &rect);
    latest_object->projected_box = rect;
  }
}
void Target::UpdateType(CameraFrame *frame) {
  auto object = latest_object->object;
  if(lost_age == 0) {
    int ncurtype = static_cast<int>(object->sub_type);
    if( life_age_ < 16) {
      vec_objtype_.push_back(ncurtype);
      type_voter_[ncurtype]++;
    } else {
      int idx = life_age_%15;
      if((idx == 1) && (life_age_ > 2000)) {
        life_age_ = 16;
      }
      int ndeltype = vec_objtype_[idx];
      if(ndeltype != ncurtype){
        vec_objtype_[idx] = ncurtype;
        type_voter_[ncurtype]++;
        type_voter_[ndeltype]--;
      }
    }
    auto max_voter = std::max_element(type_voter_.begin(), type_voter_.end());
    auto index = static_cast<int>(std::distance(type_voter_.begin(), max_voter));
    type = static_cast<base::ObjectSubType>(index);
    object->sub_type = type;
  }
}
bool Target::isTracked() const {
  return Size() >= target_param_.tracked_life();
}
bool Target::isLost() const { return lost_age > 0; } // gqs

}  // namespace camera
}  // namespace perception
}  // namespace apollo
