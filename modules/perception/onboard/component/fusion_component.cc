/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/
#include "modules/perception/onboard/component/fusion_component.h"

#include "modules/common/time/time.h"
#include "modules/perception/base/object_pool_types.h"
#include "modules/perception/lib/utils/perf.h"
#include "modules/perception/onboard/common_flags/common_flags.h"
#include "modules/perception/onboard/msg_serializer/msg_serializer.h"
#include "yaml-cpp/yaml.h"//add by xuefeng
#include "modules/perception/common/sensor_manager/sensor_manager.h"//add by xuefeng
namespace apollo {
namespace perception {
namespace onboard {

using namespace cv;
uint32_t FusionComponent::s_seq_num_ = 0;
std::mutex FusionComponent::s_mutex_;

//add by xuefeng
// @description: load camera extrinsics from yaml file
static bool LoadExtrinsics(const std::string &yaml_file,
                           Eigen::Matrix4d *camera_extrinsic) {
  if (!apollo::cyber::common::PathExists(yaml_file)) {
    AINFO << yaml_file << " not exist!";
    return false;
  }
  YAML::Node node = YAML::LoadFile(yaml_file);
  double qw = 0.0;
  double qx = 0.0;
  double qy = 0.0;
  double qz = 0.0;
  double tx = 0.0;
  double ty = 0.0;
  double tz = 0.0;
  try {
    if (node.IsNull()) {
      AINFO << "Load " << yaml_file << " failed! please check!";
      return false;
    }
    qw = node["transform"]["rotation"]["w"].as<double>();
    qx = node["transform"]["rotation"]["x"].as<double>();
    qy = node["transform"]["rotation"]["y"].as<double>();
    qz = node["transform"]["rotation"]["z"].as<double>();
    tx = node["transform"]["translation"]["x"].as<double>();
    ty = node["transform"]["translation"]["y"].as<double>();
    tz = node["transform"]["translation"]["z"].as<double>();
  } catch (YAML::InvalidNode &in) {
    AERROR << "load camera extrisic file " << yaml_file
           << " with error, YAML::InvalidNode exception";
    return false;
  } catch (YAML::TypedBadConversion<double> &bc) {
    AERROR << "load camera extrisic file " << yaml_file
           << " with error, YAML::TypedBadConversion exception";
    return false;
  } catch (YAML::Exception &e) {
    AERROR << "load camera extrisic file " << yaml_file
           << " with error, YAML exception:" << e.what();
    return false;
  }
  camera_extrinsic->setConstant(0);
  Eigen::Quaterniond q;
  q.x() = qx;
  q.y() = qy;
  q.z() = qz;
  q.w() = qw;
  (*camera_extrinsic).block<3, 3>(0, 0) = q.normalized().toRotationMatrix();
  (*camera_extrinsic)(0, 3) = tx;
  (*camera_extrinsic)(1, 3) = ty;
  (*camera_extrinsic)(2, 3) = tz;
  (*camera_extrinsic)(3, 3) = 1;
  return true;
}
bool FusionComponent::Init() {
  FusionComponentConfig comp_config;
  if (!GetProtoConfig(&comp_config)) {
    return false;
  }
  AINFO << "Fusion Component Configs: " << comp_config.DebugString();

  // to load component configs
  fusion_method_ = comp_config.fusion_method();
  fusion_main_sensor_ = comp_config.fusion_main_sensor();
  object_in_roi_check_ = comp_config.object_in_roi_check();
  radius_for_roi_object_check_ = comp_config.radius_for_roi_object_check();

  // init algorithm plugin
  CHECK(InitAlgorithmPlugin()) << "Failed to init algorithm plugin.";
  writer_ = node_->CreateWriter<PerceptionObstacles>(
      comp_config.output_obstacles_channel_name());
  inner_writer_ = node_->CreateWriter<SensorFrameMessage>(
      comp_config.output_viz_fused_content_channel_name());
  //add by xuefeng
  //init extrinsic/intrinsic
  Eigen::Matrix4d ex_lidar2imu;
  LoadExtrinsics(FLAGS_obs_sensor_intrinsic_path + "/" +
                    "velodyne128_novatel_extrinsics.yaml",
                &ex_lidar2imu);
  // rotate 90 degree around z axis to make x point forward
  // double imu_height = 0;  // imu height should be considred later
  ex_imu2car_ << 0, 1, 0, 0,  // cos(90), sin(90), 0,
                -1, 0, 0, 0,  // -sin(90),  cos(90), 0,
                0, 0, 1, 0,  // 0,              0, 1
                0, 0, 0, 1;
  camera_names_.push_back("front_6mm_1");
  camera_names_.push_back("front_6mm_2");
  for (const auto &camera_name : camera_names_) {
    base::BaseCameraModelPtr model;
    model =
        common::SensorManager::Instance()->GetUndistortCameraModel(camera_name);
    auto pinhole = static_cast<base::PinholeCameraModel *>(model.get());
    Eigen::Matrix3f intrinsic = pinhole->get_intrinsic_params();
    intrinsic_map_[camera_name] = intrinsic.cast<double>();
    Eigen::Matrix4d extrinsic;
    LoadExtrinsics(FLAGS_obs_sensor_intrinsic_path + "/" + camera_name +
                       "_extrinsics.yaml",
                   &extrinsic);
    extrinsic_map_[camera_name] = extrinsic;
    ex_camera2imu_ = ex_lidar2imu * extrinsic_map_[camera_name];
    ex_camera2car_map_[camera_name] = ex_imu2car_ * ex_camera2imu_;
    ex_car2camera_map_[camera_name] = ex_camera2car_map_[camera_name].inverse();
  }
  cv::Mat output_image_temp(770, 1400, CV_8UC3, Scalar(160, 160, 160));
  output_image = output_image_temp.clone();
  return true;
}

bool FusionComponent::Proc(const std::shared_ptr<SensorFrameMessage>& message) {
  if (message->process_stage_ == ProcessStage::SENSOR_FUSION) {
    return true;
  }
  std::shared_ptr<PerceptionObstacles> out_message(new (std::nothrow)
                                                       PerceptionObstacles);
  std::shared_ptr<SensorFrameMessage> viz_message(new (std::nothrow)
                                                      SensorFrameMessage);
  bool status = InternalProc(message, out_message, viz_message);
    
  if (status) {
    // TODO(conver sensor id)
    if (0){//message->sensor_id_ != fusion_main_sensor_) {
      AINFO << "Fusion receive from " << message->sensor_id_ << "not from "
            << fusion_main_sensor_ << ". Skip send.";
    } else {
      // Send("/apollo/perception/obstacles", out_message);
      writer_->Write(out_message);
      cyber::common::SetProtoToASCIIFile(*out_message, "fusion_test.proto");
      AINFO << "Send fusion processing output message.";
      // send msg for visualization
      if (FLAGS_obs_enable_visualization) {
        // Send("/apollo/perception/inner/PrefusedObjects", viz_message);
        inner_writer_->Write(viz_message);
      }
    }
  }
  return status;
}

bool FusionComponent::InitAlgorithmPlugin() {
  fusion_.reset(new fusion::ObstacleMultiSensorFusion());
  fusion::ObstacleMultiSensorFusionParam param;
  param.main_sensor = fusion_main_sensor_;
  param.fusion_method = fusion_method_;
  CHECK(fusion_->Init(param)) << "Failed to init ObstacleMultiSensorFusion";

  if (FLAGS_obs_enable_hdmap_input && object_in_roi_check_) {
    hdmap_input_ = map::HDMapInput::Instance();
    CHECK(hdmap_input_->Init()) << "Failed to init hdmap input.";
  }
  AINFO << "Init algorithm successfully, onboard fusion: " << fusion_method_;
  
  return true;
}
bool FusionComponent::DrawCoor(cv::Mat img, int x_step,int x_num, int y_step, int y_num, int dis_per_step, int cor_origin_pos)
{
  cv::Point2f left_corner_radar_pos = cv::Point2f(-0.74, 0);
  cv::Point2f front_radar_pos = cv::Point2f(0.0, 0);
  cv::Point2f right_corner_radar_pos = cv::Point2f(0.74, 0);
  cv::Point lcr_fov_left_start_pt; //left corner radar
  cv::Point fr_fov_left_start_pt; //front radar
  cv::Point rcr_fov_left_start_pt; //left corner radar
  cv::Point lcr_fov_left_end_pt; //left corner radar
  cv::Point fr_fov_left_end_pt; //front radar
  cv::Point rcr_fov_left_end_pt; //left corner radar
  
  cv::Point lcr_fov_right_start_pt; //left corner radar
  cv::Point fr_fov_right_start_pt; //front radar
  cv::Point rcr_fov_right_start_pt; //left corner radar
  cv::Point lcr_fov_right_end_pt; //left corner radar
  cv::Point fr_fov_right_end_pt; //front radar
  cv::Point rcr_fov_right_end_pt; //left corner radar

  float lef_corner_radar_angle = 75.0 * M_PI / 180;
  float front_radar_angle = 57.0 * M_PI / 180;
  float right_corner_radar_angle = 75.0 * M_PI / 180;
  float corner_radar_rotate_angle = 15.0 * M_PI / 180;
  int y_cor_num = 3;
  cv::Point cor_origin_pos_img;
  float pix_per_meter = x_step * 1.0 / dis_per_step;
  cor_origin_pos_img.x = cor_origin_pos + x_step * (x_num / 2);

  
  cv::Scalar color, color1, color2;
  cv::Point start_pt = cv::Point(cor_origin_pos, img.rows - cor_origin_pos);
  color1 = cv::Scalar(0, 0, 0);
  color2 = cv::Scalar(245, 245, 245);
  int x_end = start_pt.x + x_step * (x_num-1);
  int y_end = start_pt.y + y_step * (y_num-1);
  char dis_str[4];
 
  cor_origin_pos_img.y = img.rows - cor_origin_pos + y_cor_num * y_step;
  //left_corner_radar_fov
  lcr_fov_left_start_pt.x = static_cast<int>(cor_origin_pos_img.x + pix_per_meter * left_corner_radar_pos.x + 0.5);
  lcr_fov_left_start_pt.y = static_cast<int>(cor_origin_pos_img.y - pix_per_meter * left_corner_radar_pos.y + 0.5);
  lcr_fov_left_end_pt.x = cor_origin_pos;
  lcr_fov_left_end_pt.y = static_cast<int>(tan(M_PI_2 - lef_corner_radar_angle - corner_radar_rotate_angle) 
                          * (lcr_fov_left_end_pt.x - lcr_fov_left_start_pt.x) + lcr_fov_left_start_pt.y + 0.5);
  lcr_fov_right_start_pt.x = lcr_fov_left_start_pt.x;
  lcr_fov_right_start_pt.y = lcr_fov_left_start_pt.y;
  lcr_fov_right_end_pt.x = x_end;
  lcr_fov_right_end_pt.y = static_cast<int>(-tan(M_PI_2 - lef_corner_radar_angle + corner_radar_rotate_angle)
                          * (lcr_fov_right_end_pt.x - lcr_fov_right_start_pt.x) 
                          + lcr_fov_right_start_pt.y + 0.5);
  //cv::line(img, lcr_fov_left_start_pt, lcr_fov_left_end_pt, cv::Scalar(255, 0, 0), 1, CV_AA);
  //cv::line(img, lcr_fov_right_start_pt,lcr_fov_right_end_pt, cv::Scalar(255, 0, 0), 1, CV_AA);
  //front_radar_fov
  fr_fov_left_start_pt.x = static_cast<int>(cor_origin_pos_img.x + pix_per_meter * front_radar_pos.x + 0.5);
  fr_fov_left_start_pt.y = static_cast<int>(cor_origin_pos_img.y - pix_per_meter * front_radar_pos.y + 0.5);
  fr_fov_left_end_pt.x = cor_origin_pos;
  fr_fov_left_end_pt.y = static_cast<int>(tan(M_PI_2 - front_radar_angle) 
                          * (fr_fov_left_end_pt.x - fr_fov_left_start_pt.x) + fr_fov_left_start_pt.y + 0.5);
  fr_fov_right_start_pt.x = fr_fov_left_start_pt.x;
  fr_fov_right_start_pt.y = fr_fov_left_start_pt.y;
  fr_fov_right_end_pt.x = x_end;
  fr_fov_right_end_pt.y = static_cast<int>(-tan(M_PI_2 - front_radar_angle)
                          * (fr_fov_right_end_pt.x - fr_fov_right_start_pt.x) 
                          + fr_fov_right_start_pt.y + 0.5);
  //cv::line(img, fr_fov_left_start_pt, fr_fov_left_end_pt, cv::Scalar(0, 255, 0), 1, CV_AA);
  //cv::line(img, fr_fov_right_start_pt,fr_fov_right_end_pt, cv::Scalar(0, 255, 0), 1, CV_AA);
    //right_corner_radar_fov
  rcr_fov_left_start_pt.x = static_cast<int>(cor_origin_pos_img.x + pix_per_meter * right_corner_radar_pos.x + 0.5);
  rcr_fov_left_start_pt.y = static_cast<int>(cor_origin_pos_img.y - pix_per_meter * right_corner_radar_pos.y + 0.5);
  rcr_fov_left_end_pt.x = cor_origin_pos;
  rcr_fov_left_end_pt.y = static_cast<int>(tan(M_PI_2 - right_corner_radar_angle + corner_radar_rotate_angle) 
                          * (rcr_fov_left_end_pt.x - rcr_fov_left_start_pt.x) + rcr_fov_left_start_pt.y + 0.5);
  rcr_fov_right_start_pt.x = rcr_fov_left_start_pt.x;
  rcr_fov_right_start_pt.y = rcr_fov_left_start_pt.y;
  rcr_fov_right_end_pt.x = x_end;
  rcr_fov_right_end_pt.y = static_cast<int>(-tan(M_PI_2 - right_corner_radar_angle - corner_radar_rotate_angle)
                          *(rcr_fov_right_end_pt.x - rcr_fov_right_start_pt.x) 
                          + rcr_fov_right_start_pt.y + 0.5);
  //cv::line(img, rcr_fov_left_start_pt, rcr_fov_left_end_pt, cv::Scalar(0, 0, 255), 1, CV_AA);
  //cv::line(img, rcr_fov_right_start_pt,rcr_fov_right_end_pt, cv::Scalar(0, 0, 255), 1, CV_AA);

  for (int i = 0; i < y_num; i++) {
    if (i == y_cor_num) {
      snprintf(dis_str, 4, "%3d", 0);
      color = color1;
      cv::line(img, cv::Point(start_pt.x, start_pt.y+y_step *i), cv::Point(x_end, start_pt.y + y_step * i), color1, 1,CV_AA);
      cv::putText(img, dis_str, cv::Point(start_pt.x - 40, start_pt.y + y_step * i), cv::FONT_HERSHEY_SIMPLEX, 0.5, color, 1, CV_AA);
    }
    else {
      snprintf(dis_str, 4, "%3d", -(y_num -1) * dis_per_step / 2 + (i + y_cor_num - 1) * dis_per_step);
      if (i == y_num - 1) {
        color = color1;
      }
      else {
        color = color2;
      }
      cv::line(img, cv::Point(start_pt.x,  start_pt.y + y_step * i), cv::Point(x_end, start_pt.y + y_step * i), color, 1, CV_AA);
      cv::putText(img, dis_str, cv::Point(start_pt.x - 40, start_pt.y + y_step * i), cv::FONT_HERSHEY_SIMPLEX,0.5, color1,1, CV_AA);
    }
  }

  for (int i = 0; i < x_num; i++) {
    if (i == 0) {
      color = color1;
      snprintf(dis_str, 4, "%3d", -(x_num-1) * dis_per_step/2);
      cv::line(img, start_pt, cv::Point(start_pt.x, y_end), color1, 1, CV_AA);
      cv::putText(img, dis_str, cv::Point(start_pt.x-20 , start_pt.y + y_step * y_cor_num +20), cv::FONT_HERSHEY_SIMPLEX, 0.5, color, 1, CV_AA);
    }
    else {
      snprintf(dis_str, 4, "%3d", -(x_num-1) * dis_per_step / 2 + i * dis_per_step);
      if (i == x_num - 1) {
        color = color1;
      }
      else {
        color = color2;
      }
      cv::line(img, cv::Point(start_pt.x + x_step * i, start_pt.y), cv::Point(start_pt.x + x_step * i, y_end), color, 1, CV_AA);
      cv::putText(img, dis_str, cv::Point(start_pt.x-20+ i*x_step, start_pt.y + y_step * y_cor_num + 20), cv::FONT_HERSHEY_SIMPLEX, 0.5, color1, 1, CV_AA);
    }
  }
  cv::putText(img, "Unit:m", cv::Point(img.cols / 2 - 30, 30), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 0, 0), 1, CV_AA);
  //cv::line(img, cv::Point(80, 740), cv::Point(120, 740), cv::Scalar(255, 0, 0), 1, CV_AA);
  // cv::putText(img, "left_radar", cv::Point(130, 740), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0), 1, CV_AA);
  // cv::line(img, cv::Point(280, 740), cv::Point(320, 740), cv::Scalar(0, 255, 0), 1, CV_AA);
  // cv::putText(img, "front_radar", cv::Point(330, 740), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0), 1, CV_AA);
  // cv::line(img, cv::Point(480, 740), cv::Point(520, 740), cv::Scalar(0, 0, 255), 1, CV_AA);
  // cv::putText(img, "right_radar", cv::Point(530, 740), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0), 1, CV_AA);
  return true;
}
bool FusionComponent::InternalProc(
    const std::shared_ptr<SensorFrameMessage const>& in_message,
    std::shared_ptr<PerceptionObstacles> out_message,
    std::shared_ptr<SensorFrameMessage> viz_message) {
  {
    std::unique_lock<std::mutex> lock(s_mutex_);
    s_seq_num_++;
  }
  static int obj_track_id = 0;///////////////

  PERCEPTION_PERF_BLOCK_START();
  const double timestamp = in_message->timestamp_;
  const uint64_t lidar_timestamp = in_message->lidar_timestamp_;
  std::vector<base::ObjectPtr> valid_objects;
  if (in_message->error_code_ != apollo::common::ErrorCode::OK) {
    if (!MsgSerializer::SerializeMsg(
            timestamp, lidar_timestamp, in_message->seq_num_, valid_objects,
            in_message->error_code_, out_message.get())) {
      AERROR << "Failed to gen PerceptionObstacles object.";
      return false;
    }
    if (FLAGS_obs_enable_visualization) {
      viz_message->process_stage_ = ProcessStage::SENSOR_FUSION;
      viz_message->error_code_ = in_message->error_code_;
    }
    AERROR << "Fusion receive message with error code, skip it.";
    return true;
  }
  base::FramePtr frame = in_message->frame_;
  std::vector<bool> camera_obj_is_match;
  std::vector<bool> radar_obj_is_match;
  std::vector<cv::Point> radar_pt_in_camera;
  std::map<int, std::vector<int>> camera_radar_obj_match;
  Eigen::Matrix<double, 3, 4> ex_car2camera;
  frame->timestamp = in_message->timestamp_;
  
  
   //add by xuefeng
  //Eigen::Matrix3d ex_car2camera_3 = ex_car2camera_.block(0,0,3,4);
  // if(FLAGS_debug_display){
    
  // }
  int margin_h = 20;
  int margin_v = 20;
  int radar_pt_in_camera_size = 50;
  int img_w = 1920;
  int img_h = 1080;
  static int frame_cnt = 0;
  std::vector<base::ObjectPtr> fused_objects;
  if(in_message->sensor_id_ != "front_6mm_1" && in_message->sensor_id_ != "front_6mm_2")
  {
    frame_cnt = 0;
    base::FramePtr &radar_frame = frame;
    radar_objects_ = radar_frame->objects;
    for (auto obj : radar_objects_){
      //fused_obj->center = obj->center;
      if(obj_track_id > 1000){
        obj_track_id = 0;
      }
      obj->track_id = obj_track_id;
      obj->fusion_mode = 2; //only radar
      obj->size = Eigen::Vector3f(1.0, 1.0, 1.0);//only radar, can set 3*3*3 to apart from camera
      obj->type = base::ObjectType::UNKNOWN;
      if(obj->center[0] < 50.0 && fabs(obj->center[1]) < 2.0){
        fused_objects.push_back(obj);
      }
      obj_track_id++;
    }
  }
  if(in_message->sensor_id_ == "front_6mm_1" || in_message->sensor_id_ == "front_6mm_2")
  {
    if(frame_cnt > 10){
      radar_objects_.clear();
    }
    frame_cnt++;
    cv::Mat img_front_src;
    camera_obj_is_match.resize(frame->objects.size(), false);
    ex_car2camera = ex_car2camera_map_[in_message->sensor_id_].block(0,0,3,4);
    base::FramePtr &camera_frame = frame;

    Point obstacle_pt_top_view;
    Point cor_origin_pos_img;
    //int n_win_width = 1400;
    //int n_win_hight = 770;
    int pix_per_step = 66;
    int dis_per_step = 10;
    float pix_per_meter = pix_per_step * 1.0 / dis_per_step;
    int radar_top_view_size = 750;
    //Mat output_image(n_win_hight, n_win_width, CV_8UC3, Scalar(160, 160, 160));
    Mat radar_top_view(radar_top_view_size, radar_top_view_size, CV_8UC3, Scalar(255, 255, 255));
    Rect radar_ROI_rect = Rect(10, 10, radar_top_view_size, radar_top_view_size);
    Mat radar_ROI_img = output_image(radar_ROI_rect);
    cv::Mat img_front_ROI; 
    if(FLAGS_fusion_debug_display){
      int step_num = 11;
      int cor_pos = 50;
      int y_cor_num = 3;//consistent with the function  DrawCoor
      cor_origin_pos_img.x = cor_pos + pix_per_step * (step_num / 2);
      cor_origin_pos_img.y = radar_top_view_size - cor_pos - y_cor_num * pix_per_step;
      Rect left_camera_rect = Rect(780, 40, 600, 338);
      Rect right_camera_rect = Rect(780, 420, 600, 338);
      Point left_camera_pix_pos = Point(1000, 30);
      Point right_camera_pix_pos = Point(1000, 410);
      DrawCoor(radar_top_view, pix_per_step, step_num, -pix_per_step, step_num, dis_per_step, cor_pos);
      cv::putText(output_image, "Left camera", left_camera_pix_pos, cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0));
      cv::putText(output_image, "Right camera", right_camera_pix_pos, cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0));
      
      //
      Point obstacle_pt_top_view;
      std::shared_ptr<base::Object> radar_obj1;
      radar_obj1.reset(new base::Object);

      // radar_objects_.clear();
      // radar_obj1->center[0] = 6.0;
      // radar_obj1->center[1] = 2.0;
      // obstacle_pt_top_view.x = cor_origin_pos_img.x + static_cast<int>(pix_per_meter * radar_obj1->center[1]);
      // obstacle_pt_top_view.y = cor_origin_pos_img.y - static_cast<int>(pix_per_meter * radar_obj1->center[0]);
      // cv::circle(radar_top_view, obstacle_pt_top_view, 5, Scalar(255, 0, 0), -1);
      // radar_objects_.push_back(radar_obj1);
      // std::shared_ptr<base::Object> radar_obj3;
      // radar_obj3.reset(new base::Object);
      // radar_obj3->center[0] = 6.0;
      // radar_obj3->center[1] = -2.0;
      // obstacle_pt_top_view.x = cor_origin_pos_img.x + static_cast<int>(pix_per_meter * radar_obj1->center[1]);
      // obstacle_pt_top_view.y = cor_origin_pos_img.y - static_cast<int>(pix_per_meter * radar_obj1->center[0]);
      // radar_objects_.push_back(radar_obj3);
      // std::shared_ptr<base::Object> radar_obj4;
      // radar_obj4.reset(new base::Object);
      // radar_obj4->center[0] = 10.0;
      // radar_obj4->center[1] = 2.0;
      // obstacle_pt_top_view.x = cor_origin_pos_img.x + static_cast<int>(pix_per_meter * radar_obj1->center[1]);
      // obstacle_pt_top_view.y = cor_origin_pos_img.y - static_cast<int>(pix_per_meter * radar_obj1->center[0]);
      // radar_objects_.push_back(radar_obj4);
      // AERROR<<radar_objects_.size();
      //
      if(in_message->sensor_id_ == "front_6mm_1"){
        cv::Mat img_front_left_src(1080, 1920, CV_8UC3,
                        cv::Scalar(250, 0, 0));
        base::Image8UPtr out_image;
        out_image = camera_frame->camera_frame_supplement.image_ptr;
        memcpy(img_front_left_src.data, out_image->cpu_data(),
              out_image->total() * sizeof(uint8_t));
        img_front_src = img_front_left_src;
        img_front_ROI = output_image(left_camera_rect);
      }
      if(in_message->sensor_id_ == "front_6mm_2"){
        cv::Mat img_front_right_src(1080, 1920, CV_8UC3,
                        cv::Scalar(250, 0, 0));
        base::Image8UPtr out_image;
        out_image = camera_frame->camera_frame_supplement.image_ptr;
        memcpy(img_front_right_src.data, out_image->cpu_data(),
              out_image->total() * sizeof(uint8_t));
        img_front_src = img_front_right_src;
        img_front_ROI = output_image(right_camera_rect);
      }
    }
    int radar_obj_cnt = 0;
    radar_obj_is_match.resize(radar_objects_.size(), false);
    radar_pt_in_camera.resize(radar_objects_.size());
    for (auto obj : radar_objects_){
      Eigen::Vector4d obstacle_radar_pt;
      Eigen::Matrix3d intrinsic = intrinsic_map_[in_message->sensor_id_];
      cv::Scalar tl_color;
      tl_color = cv::Scalar(255, 255, 255);
      obstacle_radar_pt(0) = obj->center(0);
      obstacle_radar_pt(1) = obj->center(1);
      obstacle_radar_pt(2) = obj->center(2)+0.82;
      obstacle_radar_pt(3) = 1.0;
      
      Eigen::Vector3d obstacle_radar_camera_pt = ex_car2camera * obstacle_radar_pt;
      Eigen::Vector3d obstacle_radar_img_pt = intrinsic * obstacle_radar_camera_pt;
      AERROR<<obstacle_radar_img_pt(0)<<","<<obstacle_radar_img_pt(1)<<","<<obstacle_radar_img_pt(2);
      int x = static_cast<int>(obstacle_radar_img_pt(0) / obstacle_radar_img_pt(2));
      int y = static_cast<int>(obstacle_radar_img_pt(1) / obstacle_radar_img_pt(2));
      if(FLAGS_fusion_debug_display){
        obstacle_pt_top_view.x = cor_origin_pos_img.x + static_cast<int>(pix_per_meter * obstacle_radar_pt(1));
        obstacle_pt_top_view.y = cor_origin_pos_img.y - static_cast<int>(pix_per_meter * obstacle_radar_pt(0));
        if(obstacle_pt_top_view.x > 0 && obstacle_pt_top_view.x < radar_top_view.cols
        && obstacle_pt_top_view.x > 0 && obstacle_pt_top_view.x < radar_top_view.rows){
          cv::circle(radar_top_view, obstacle_pt_top_view, 5, Scalar(255, 0, 0), -1);
        }
      }
      if(x > 0 && x < img_w && y > 0 && y < img_h){
        radar_pt_in_camera[radar_obj_cnt] = cv::Point(x,y);
        if(FLAGS_fusion_debug_display){
          cv::circle(img_front_src, cv::Point(x, y), 30, tl_color, -1);
        }
      }
      // AERROR<<obstacle_radar_pt(0)<<","<<obstacle_radar_pt(1)<<","<<obstacle_radar_pt(2);
      // AERROR<<obstacle_radar_camera_pt(0)<<","<<obstacle_radar_camera_pt(1)<<","<<obstacle_radar_camera_pt(2);
      // AERROR<<obstacle_radar_img_pt(0)<<","<<obstacle_radar_img_pt(1)<<","<<obstacle_radar_img_pt(2);
      // AERROR<<x<<","<<y;
      radar_obj_cnt++;
    }
    int camera_obj_cnt = 0;
    for (auto obj : camera_frame->objects) {
      auto &box = obj->camera_supplement.box;
      radar_obj_cnt = 0;
      for (auto obj : radar_objects_){
        int x = radar_pt_in_camera[radar_obj_cnt].x;
        int y = radar_pt_in_camera[radar_obj_cnt].y;
        if((x > box.xmin - margin_h) && (x < box.xmax + margin_h) 
          && (y > box.ymin - margin_v) && (y < box.ymax + margin_v)){
          radar_obj_is_match[radar_obj_cnt] = true;
          camera_obj_is_match[camera_obj_cnt] = true;
          camera_radar_obj_match[camera_obj_cnt].push_back(radar_obj_cnt);
        }
        radar_obj_cnt++;
      } 
      camera_obj_cnt++;
    }
    camera_obj_cnt = 0;
    for (auto obj : camera_frame->objects) {
      base::ObjectPtr fused_obj = nullptr;
      fused_obj.reset(new base::Object);
      if(camera_obj_is_match[camera_obj_cnt]){
        float dis_min = 1000000.0;
        float dis_temp = 0.0;
        int dis_min_index = 0;
        int size_temp = static_cast<int>(camera_radar_obj_match[camera_obj_cnt].size());
        for(auto i = 0; i < size_temp; i++){
          int index = camera_radar_obj_match[camera_obj_cnt][i];
          dis_temp = std::pow(radar_objects_[index]->center[0], 2) + std::pow(radar_objects_[index]->center[1], 2);
          if(dis_temp < dis_min){
            dis_min = dis_temp;
            dis_min_index = index;
          }
        }
        if(obj_track_id > 1000){
          obj_track_id = 0;
        }
        fused_obj->track_id = obj_track_id;
        fused_obj->center = radar_objects_[dis_min_index]->center;
        fused_obj->velocity = radar_objects_[dis_min_index]->velocity;
        fused_obj->camera_supplement.box = obj->camera_supplement.box;
        fused_obj->size = Eigen::Vector3f(1.0, 1.0, 1.0);
        fused_obj->type = obj->type;
        fused_obj->fusion_mode = 3; //camera and radar
        obj_track_id++;
      }
      else{
        fused_obj->center = obj->center;
        fused_obj->camera_supplement.box = obj->camera_supplement.box;
        float obj_img_h = obj->camera_supplement.box.ymax-obj->camera_supplement.box.ymin;
        float obj_world_h = obj->size[2];
        float dis_y = obj_world_h * 2000 / obj_img_h;
        if(obj_track_id > 1000){
          obj_track_id = 0;
        }
        fused_obj->track_id = obj_track_id;
        fused_obj->center[1] = dis_y;
        fused_obj->size = Eigen::Vector3f(1.0, 1.0, 1.0);
        fused_obj->type = obj->type;
        fused_obj->fusion_mode = 1; //only camera
        obj_track_id++;
      }
      if(fused_obj->center[0] < 50.0 && fabs(fused_obj->center[1]) < 2.0){
          fused_objects.push_back(fused_obj);
        }
      camera_obj_cnt++;
    }
    radar_obj_cnt = 0;
    for (auto obj : radar_objects_){
      base::ObjectPtr fused_obj = nullptr;
      fused_obj.reset(new base::Object);
      if(!radar_obj_is_match[radar_obj_cnt]){
        fused_obj->center = radar_objects_[radar_obj_cnt]->center;
        int xmin = radar_pt_in_camera[radar_obj_cnt].x - radar_pt_in_camera_size;
        int xmax = radar_pt_in_camera[radar_obj_cnt].x + radar_pt_in_camera_size;
        int ymin = radar_pt_in_camera[radar_obj_cnt].y - radar_pt_in_camera_size;
        int ymax = radar_pt_in_camera[radar_obj_cnt].y + radar_pt_in_camera_size;
        if(xmin > 0 && xmax < img_w && ymin > 0 && ymax < img_h){
          fused_obj->camera_supplement.box.xmin = xmin;
          fused_obj->camera_supplement.box.xmax = xmax;
          fused_obj->camera_supplement.box.ymin = ymin;
          fused_obj->camera_supplement.box.ymax = ymax;
          
        }
        if(obj_track_id > 1000){
          obj_track_id = 0;
        }
        fused_obj->track_id = obj_track_id;
        fused_obj->fusion_mode = 2; //only radar
        fused_obj->size = Eigen::Vector3f(1.0, 1.0, 1.0);
        fused_obj->type = base::ObjectType::UNKNOWN;
        if(fused_obj->center[0] < 50.0 && fabs(fused_obj->center[1]) < 2.0){
          fused_objects.push_back(fused_obj);
        }
        obj_track_id++;
      }
      radar_obj_cnt++;
    }
    
    AERROR<<"fused_objects"<<fused_objects.size();
    if(FLAGS_fusion_debug_display){
      for (auto obj : fused_objects) {
        auto &box = obj->camera_supplement.box;
        if (box.xmin < box.xmax && box.ymin < box.ymax) {
          //add by houxuefeng
          const cv::Rect rectified_rect(box.xmin, box.ymin,
                                      box.xmax-box.xmin, box.ymax-box.ymin);
          cv::Scalar tl_color;
          if(obj->fusion_mode == 1){
            tl_color = cv::Scalar(0, 255, 0);
          }
          if(obj->fusion_mode == 2){
            tl_color = cv::Scalar(255, 0, 0);
          }
          if(obj->fusion_mode == 3){
            tl_color = cv::Scalar(0, 0, 255);
          }
          std::string obj_name;
          switch (int(obj->type)) {
            case 0:
              obj_name = "unknown";
              break;
            case 1:
              obj_name = "unknown_movable";
              break;
            case 2:
              obj_name = "unknown_unmovable";
              break;
            case 3:
              obj_name = "pedestrian";
              break;
            case 4:
              obj_name = "bicycle";
              break;
            case 5:
              obj_name = "vehicle";
              break;
            default:
              obj_name = "error";
          break;
          }
          if(obj_name == "pedestrian"){
            //continue;
          }
          // float obj_img_h = box.ymax-box.ymin;
          // float obj_world_h = obj->size[2];
          // float dis = obj_world_h * 2000 / obj_img_h;
          float dis_x = obj->center[0];
          float dis_y = obj->center[1];
          std::string res;
          char dis_x_str[4];
          char dis_y_str[4];
          snprintf(dis_x_str, 4, "%3f", dis_x);
          snprintf(dis_y_str, 4, "%3f", dis_y);
          res = res + dis_x_str;
          res = res + ",";
          res = res + dis_y_str;
          cv::putText(img_front_src, res, cv::Point(box.xmin, box.ymin),
                  cv::FONT_HERSHEY_DUPLEX, 2.0, tl_color, 2);
          cv::rectangle(img_front_src, rectified_rect, tl_color, 2);
          //add by houxuefeng
        }
      }
      if(FLAGS_fusion_debug_display){
        cv::namedWindow("fused_img", CV_WINDOW_NORMAL);
        cv::resize(img_front_src, img_front_ROI, img_front_ROI.size());
        cv::resize(radar_top_view, radar_ROI_img, radar_ROI_img.size());
        cv::resizeWindow("fused_img", 640, 480);
        //add by houxuefeng
        cv::imshow("fused_img", output_image);
        //cv::imwrite("test.jpg", img_front_src);
        cvWaitKey(1);
      }
    }
    
  }

  
  AERROR<<"fusion module:"<<fused_objects.size();
  // if (!fusion_->Process(frame, &fused_objects)) {
  //   AERROR << "Failed to call fusion plugin.";
  //   return false;
  // }
  // PERCEPTION_PERF_BLOCK_END_WITH_INDICATOR(std::string("fusion_process"),
  //                                          in_message->sensor_id_);

  // if (in_message->sensor_id_ != fusion_main_sensor_) {
  //   return true;
  // }

  Eigen::Matrix4d sensor2world_pose = 
      in_message->frame_->sensor2world_pose.matrix();
  if (object_in_roi_check_ && FLAGS_obs_enable_hdmap_input) {
    // get hdmap
    base::HdmapStructPtr hdmap(new base::HdmapStruct());
    if (hdmap_input_) {
      base::PointD position;
      position.x = sensor2world_pose(0, 3);
      position.y = sensor2world_pose(1, 3);
      position.z = sensor2world_pose(2, 3);
      hdmap_input_->GetRoiHDMapStruct(position, radius_for_roi_object_check_,
                                      hdmap);
      // TODO(use check)
      // ObjectInRoiSlackCheck(hdmap, fused_objects, &valid_objects);
      valid_objects.assign(fused_objects.begin(), fused_objects.end());
    } else {
      valid_objects.assign(fused_objects.begin(), fused_objects.end());
    }
  } else {
    valid_objects.assign(fused_objects.begin(), fused_objects.end());
  }
  PERCEPTION_PERF_BLOCK_END_WITH_INDICATOR(std::string("fusion_roi_check"),
                                           in_message->sensor_id_);

  // produce visualization msg
  if (FLAGS_obs_enable_visualization) {
    viz_message->timestamp_ = in_message->timestamp_;
    viz_message->seq_num_ = in_message->seq_num_;
    viz_message->frame_ = base::FramePool::Instance().Get();
    viz_message->frame_->sensor2world_pose =
        in_message->frame_->sensor2world_pose;
    viz_message->sensor_id_ = in_message->sensor_id_;
    viz_message->hdmap_ = in_message->hdmap_;
    viz_message->process_stage_ = ProcessStage::SENSOR_FUSION;
    viz_message->error_code_ = in_message->error_code_;
    viz_message->frame_->objects = fused_objects;
  }
  // produce pb output msg
  apollo::common::ErrorCode error_code = apollo::common::ErrorCode::OK;
  if (!MsgSerializer::SerializeMsg(timestamp, lidar_timestamp,
                                   in_message->seq_num_, valid_objects,
                                   error_code, out_message.get())) {
    AERROR << "Failed to gen PerceptionObstacles object.";
    return false;
  }
  PERCEPTION_PERF_BLOCK_END_WITH_INDICATOR(
      std::string("fusion_serialize_message"), in_message->sensor_id_);

  const double cur_time = apollo::common::time::Clock::NowInSeconds();
  const double latency = (cur_time - timestamp) * 1e3;
  AINFO << "FRAME_STATISTICS:Obstacle:End:msg_time[" << timestamp
        << "]:cur_time[" << cur_time << "]:cur_latency[" << latency
        << "]:obj_cnt[" << valid_objects.size() << "]";
  AINFO << "publish_number: " << valid_objects.size() << " obj";
  return true;
}

}  // namespace onboard
}  // namespace perception
}  // namespace apollo
