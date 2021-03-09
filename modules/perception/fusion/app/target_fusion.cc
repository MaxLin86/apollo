#include "modules/perception/fusion/app/target_fusion.h"
#include "modules/perception/camera/common/util.h"
#include "cyber/common/log.h"
#include "modules/perception/camera/common/math_functions.h"
#include "modules/perception/common/geometry/basic.h"
#include <iostream>


namespace apollo {
namespace perception {
namespace fusion {

  int FusedTarget::Size() const {
    return static_cast<int>(fused_tracked_objs_.size());
  }

  void FusedTarget::Clear() { fused_tracked_objs_.clear(); }

  TrackObjPtr FusedTarget::operator[](int index) const {
    return get_object(index);
  }
  TrackObjPtr FusedTarget::get_object(int index) const {
    CHECK_GT(fused_tracked_objs_.size(), 0);
    CHECK_LT(index, static_cast<int>(fused_tracked_objs_.size()));
    CHECK_GE(index, -static_cast<int>(fused_tracked_objs_.size()));
    return fused_tracked_objs_[(index + fused_tracked_objs_.size()) %
                              fused_tracked_objs_.size()];
  }

  FusedTarget::FusedTarget(const TrackParam &param, base::ObjectPtr object) { Init(param, object); }

  void FusedTarget::Init(const TrackParam &param, base::ObjectPtr object) {
    lost_age_ = 0;
    life_age_ = 0;

    imgPosKF_.variance_ *= param.init_imgkf().init_variance();
    imgPosKF_.measure_noise_ *= param.init_imgkf().measure_variance();
    imgPosKF_.process_noise_ *= param.init_imgkf().process_variance();

    PositionKF_.variance_ *= param.init_poskf().init_variance();
    PositionKF_.measure_noise_ *= param.init_poskf().measure_variance();
    PositionKF_.process_noise_ *= param.init_poskf().process_variance();

    init_variance_ = Eigen::Matrix<double,6,6>::Identity() * param.init_poskf().init_variance();
    measure_variance_ = Eigen::Matrix4d::Identity() * param.init_poskf().measure_variance();
    process_variance_ = Eigen::Matrix<double,6,6>::Identity() * param.init_poskf().process_variance();

    Eigen::VectorXd init_state(6);
    init_state << object->center(0),object->center(1),0,0,0,0;
    positionkf_.Init(init_state, init_variance_);

    Eigen::MatrixXd control_matrix(4,6);
    control_matrix.block<4,4>(0,0) = Eigen::Matrix4d::Identity();
    control_matrix.block<4,2>(0,4) = Eigen::Matrix<double,4,2>::Zero();
    positionkf_.SetControlMatrix(control_matrix);
  }

  void FusedTarget::Add(base::ObjectPtr detObject) {
    TrackObjPtr newobject(new TrackObj);
    newobject->object = detObject;
    newobject->fusion_mode_ = detObject->fusion_mode_;
    newobject->radar_detid = detObject->raw_radar_detid_;
    newobject->camera_detid = detObject->raw_camera_detid_;
    life_age_++;
    lost_age_ = 0;
    latest_object_ = newobject;
    fused_tracked_objs_.push_back(newobject);
    return;
  }

  void FusedTarget::Predict(base::FramePtr frame) {
    auto delta_t =
      static_cast<float>(frame->timestamp - latest_object_->timestamp);
    if (delta_t < 0) {
      return;
    }
    imgPosKF_.Predict(delta_t);
    PositionKF_.Predict(delta_t);
    Eigen::Matrix<double,6,6> transform_matrix;
    transform_matrix << 1, 0, delta_t, 0, delta_t*delta_t/2.0, 0,
                        0, 1, 0, delta_t, 0, delta_t*delta_t/2.0,
                        0, 0, 1, 0, delta_t, 0,
                        0, 0, 0, 1, 0, delta_t,
                        0, 0, 0, 0, 1, 0,
                        0, 0, 0, 0, 0, 1;
    positionkf_.Predict(transform_matrix, process_variance_);

  }

  void FusedTarget::update() {
    auto &obj = latest_object_->object;
    base::RectF rect(obj->camera_supplement.box);
    Eigen::Vector3d position = obj->center;
    Eigen::Vector3f velocity = obj->velocity;
    base::Point2DF center = rect.Center();
    std::fstream sfile("/apollo/data/measure.txt",
    std::ios::app|std::ios::binary|std::ios::out);
    if(sfile.is_open()) {
      sfile << track_id_ << "\t" <<obj->fusion_mode_
      << "\t" << position(0) << "\t"
      << position(1) << "\t";
    }
    if(lost_age_ == 0) {
      base::Point2DF precenter;
      Eigen::Vector4d state;
      state = imgPosKF_.get_state();
      precenter.x = static_cast<float>(state(0));
      precenter.y = static_cast<float>(state(1));
      Eigen::Vector2d measurement;
      measurement << center.x, center.y;
      imgPosKF_.Correct(measurement);

      if(obj->fusion_mode_ == 3) {
        measurement(0) = position(0);
        measurement(1) = position(1);
        PositionKF_.Correct(measurement);
        std::fstream vfile("/apollo/data/variance.txt",
        std::ios::app|std::ios::binary|std::ios::out);
        if(vfile.is_open()) {
          vfile << track_id_ << "\t" << PositionKF_.variance_(0,0) << 
          "\t" << PositionKF_.variance_(1,1) << std::endl;
        }
        vfile.close();
      }

      if(obj->fusion_mode_ == 3) {
        Eigen::Vector4d observation;
        observation << position(0), position(1), velocity(0), velocity(1); 
        bool ret = positionkf_.Correct(observation, measure_variance_);
        if(ret) {
          

        }
      }
    }
    Eigen::Vector4d state;
    // update object position in image
    state = imgPosKF_.get_state();
    center.x = static_cast<float>(state(0));
    center.y = static_cast<float>(state(1));
    rect.SetCenter(center);
    float width = 1920;
    float height = 1080;
    camera::RefineBox(rect, width, height, &rect);
    obj->camera_supplement.box = rect;
    // update object position in car coordinate
    state = PositionKF_.get_state();
    position(0) = static_cast<float>(state(0));
    position(1) = static_cast<float>(state(1));
    if(sfile.is_open()) {
      sfile << position(0) << "\t" 
      << position(1) << std::endl; 
    }
    sfile.close();
    obj->center = position;
  }
} // namespace fusion
} // namespace perception
} // namespace apollo