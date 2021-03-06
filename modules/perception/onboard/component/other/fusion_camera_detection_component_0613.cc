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
#include "modules/perception/onboard/component/fusion_camera_detection_component.h"

#include "absl/strings/str_cat.h"
#include "boost/algorithm/string.hpp"
#include "boost/format.hpp"
#include "yaml-cpp/yaml.h"

#include "cyber/common/file.h"
#include "cyber/common/log.h"
#include "modules/common/math/math_utils.h"
#include "modules/common/time/time.h"
#include "modules/common/time/time_util.h"
#include "modules/common/util/point_factory.h"
#include "modules/common/adapters/adapter_gflags.h"
#include "modules/perception/common/perception_gflags.h"
#include "modules/perception/common/sensor_manager/sensor_manager.h"
#include "modules/perception/onboard/common_flags/common_flags.h"
#include "modules/perception/onboard/component/camera_perception_viz_message.h"
#include <opencv2/core/eigen.hpp>

namespace apollo {
namespace perception {
namespace onboard {

//using namespace cv;
//using namespace std;
using apollo::cyber::common::GetAbsolutePath;

static void fill_lane_msg(const base::LaneLineCubicCurve &curve_coord,
                          apollo::perception::LaneMarker *lane_marker) {
  lane_marker->set_c0_position(curve_coord.d);
  lane_marker->set_c1_heading_angle(curve_coord.c);
  lane_marker->set_c2_curvature(curve_coord.b);
  lane_marker->set_c3_curvature_derivative(curve_coord.a);
  // lane_marker->set_longitude_start(curve_coord.x_start); //delete by shzhw
  // lane_marker->set_longitude_end(curve_coord.x_end);
  lane_marker->set_view_range(curve_coord.x_end - curve_coord.x_start);
  lane_marker->set_quality(0.5); // add by shzhw, to do
}

static int GetGpuId(const camera::CameraPerceptionInitOptions &options) {
  camera::app::PerceptionParam perception_param;
  std::string work_root = camera::GetCyberWorkRoot();
  std::string config_file =
      GetAbsolutePath(options.root_dir, options.conf_file);
  config_file = GetAbsolutePath(work_root, config_file);
  if (!cyber::common::GetProtoFromFile(config_file, &perception_param)) {
    AERROR << "Read config failed: " << config_file;
    return -1;
  }
  if (!perception_param.has_gpu_id()) {
    AINFO << "gpu id not found.";
    return -1;
  }
  return perception_param.gpu_id();
}

bool SetCameraHeight(const std::string &sensor_name,
                     const std::string &params_dir, float default_camera_height,
                     float *camera_height) {
  float base_h = default_camera_height;
  float camera_offset = 0.0f;
  try {
    YAML::Node lidar_height =
        YAML::LoadFile(params_dir + "/" + "velodyne128_height.yaml");
    base_h = lidar_height["vehicle"]["parameters"]["height"].as<float>();
    AINFO << base_h;
    YAML::Node camera_ex =
        YAML::LoadFile(params_dir + "/" + sensor_name + "_extrinsics.yaml");
    camera_offset = camera_ex["transform"]["translation"]["z"].as<float>();
    AINFO << camera_offset;
    *camera_height = base_h + camera_offset;
  } catch (YAML::InvalidNode &in) {
    AERROR << "load camera extrisic file error, YAML::InvalidNode exception";
    return false;
  } catch (YAML::TypedBadConversion<float> &bc) {
    AERROR << "load camera extrisic file error, "
           << "YAML::TypedBadConversion exception";
    return false;
  } catch (YAML::Exception &e) {
    AERROR << "load camera extrisic file "
           << " error, YAML exception:" << e.what();
    return false;
  }
  return true;
}

// @description: load camera extrinsics from yaml file
bool LoadExtrinsics(const std::string &yaml_file,
                    Eigen::Matrix4d *camera_extrinsic) {
  if (!apollo::cyber::common::PathExists(yaml_file)) {
    AINFO << yaml_file << " does not exist!";
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

// @description: get project matrix
bool GetProjectMatrix(
    const std::vector<std::string> &camera_names,
    const std::map<std::string, Eigen::Matrix4d> &extrinsic_map,
    const std::map<std::string, Eigen::Matrix3f> &intrinsic_map,
    Eigen::Matrix3d *project_matrix, double *pitch_diff = nullptr) {
  // TODO(techoe): This condition should be removed.
  if (camera_names.size() != 2) {
    //AINFO << "camera number must be 2!";
    //return false;
  }
  *project_matrix =
      intrinsic_map.at(camera_names[0]).cast<double>() *
      extrinsic_map.at(camera_names[0]).block<3, 3>(0, 0).inverse() *
      extrinsic_map.at(camera_names[1]).block<3, 3>(0, 0) *
      intrinsic_map.at(camera_names[1]).cast<double>().inverse();
  // extract the pitch_diff = pitch_narrow - pitch_obstacle
  if (pitch_diff != nullptr) {
    Eigen::Vector3d euler =
        (extrinsic_map.at(camera_names[0]).block<3, 3>(0, 0).inverse() *
         extrinsic_map.at(camera_names[1]).block<3, 3>(0, 0))
            .eulerAngles(0, 1, 2);
    *pitch_diff = euler(0);
    AINFO << "pitch diff: " << *pitch_diff;
  }
  return true;
}

FusionCameraDetectionComponent::~FusionCameraDetectionComponent() {}

void FusionCameraDetectionComponent::initUndistortRectifyMapFisheye(cv::InputArray K, cv::InputArray D, cv::InputArray R, cv::InputArray P,
    const cv::Size& size, int m1type, cv::OutputArray map1, cv::OutputArray map2)
{
    map1.create( size, m1type <= 0 ? CV_16SC2 : m1type );
    map2.create( size, map1.type() == CV_16SC2 ? CV_16UC1 : CV_32F );

    //???????????????K????????????????????????fx,fy; cx,cy
    cv::Vec2d f, c;
    if (K.depth() == CV_32F)
    {
        cv::Matx33f camMat = K.getMat();
        f = cv::Vec2f(camMat(0, 0), camMat(1, 1));
        c = cv::Vec2f(camMat(0, 2), camMat(1, 2));
    }
    else
    {
        cv::Matx33d camMat = K.getMat();
        f = cv::Vec2d(camMat(0, 0), camMat(1, 1));
        c = cv::Vec2d(camMat(0, 2), camMat(1, 2));
    }
    //?????????????????????D?????????????????????k1,k2,k3,k4
    cv::Vec4d k = cv::Vec4d::all(0);
    if (!D.empty())
        k = D.depth() == CV_32F ? (cv::Vec4d)*D.getMat().ptr<cv::Vec4f>(): *D.getMat().ptr<cv::Vec4d>();

    //????????????RR?????????????????????CV_64F??????????????????????????????RR????????????
    cv::Matx33d RR  = cv::Matx33d::eye();
    if (!R.empty() && R.total() * R.channels() == 3)
    {
        cv::Vec3d rvec;
        R.getMat().convertTo(rvec, CV_64F);
        RR = cv::Affine3d(rvec).rotation();
    }
    else if (!R.empty() && R.size() == cv::Size(3, 3))
        R.getMat().convertTo(RR, CV_64F);
    
    //??????????????????PP?????????????????????CV_64F
    cv::Matx33d PP = cv::Matx33d::eye();
    if (!P.empty())
        P.getMat().colRange(0, 3).convertTo(PP, CV_64F);

    //?????????????????????????????????*???????????????????????????SVD?????????????????????iR???????????????
    cv::Matx33d iR = (PP * RR).inv(cv::DECOMP_SVD);

    //?????????????????????????????????????????????????????????????????????????????????????????????(u,v)????????????????????????(u,v)???mapx???mapy???
    for( int i = 0; i < size.height; ++i)
    {
        float* m1f = map1.getMat().ptr<float>(i);
        float* m2f = map2.getMat().ptr<float>(i);
        short*  m1 = (short*)m1f;
        ushort* m2 = (ushort*)m2f;

        //???????????????????????????->??????????????????
        double _x = i*iR(0, 1) + iR(0, 2),
               _y = i*iR(1, 1) + iR(1, 2),
               _w = i*iR(2, 1) + iR(2, 2);

        for( int j = 0; j < size.width; ++j)
        {
            //????????????????????????????????????????????????Z=1?????????
            double x = _x/_w, y = _y/_w;

            //??????????????????????????????r
            double r = sqrt(x*x + y*y);
            //???????????????????????????????????????????????????????????????Theta
            double theta = atan(r);
            //??????????????????theta_d?????????????????????????????????
            double theta2 = theta*theta, theta4 = theta2*theta2, theta6 = theta4*theta2, theta8 = theta4*theta4;
            double theta_d = theta * (1 + k[0]*theta2 + k[1]*theta4 + k[2]*theta6 + k[3]*theta8);
            //??????????????????Theta????????????????????????????????????????????????????????????????????????????????????????????????(j,i)????????????????????????(u,v)
            double scale = (r == 0) ? 1.0 : theta_d / r;
            double u = f[0]*x*scale + c[0];
            double v = f[1]*y*scale + c[1];

            //??????(u,v)?????????mapx,mapy
            if( m1type == CV_16SC2 )
            {
                int iu = cv::saturate_cast<int>(u*cv::INTER_TAB_SIZE);
                int iv = cv::saturate_cast<int>(v*cv::INTER_TAB_SIZE);
                m1[j*2+0] = (short)(iu >> cv::INTER_BITS);
                m1[j*2+1] = (short)(iv >> cv::INTER_BITS);
                m2[j] = (ushort)((iv & (cv::INTER_TAB_SIZE-1))*cv::INTER_TAB_SIZE + (iu & (cv::INTER_TAB_SIZE-1)));
            }
            else if( m1type == CV_32FC1 )
            {
                m1f[j] = (float)u;
                m2f[j] = (float)v;
            }

            //???????????????????????? ???//???????????????????????????->?????????????????????????????????????????????iR??????????????????????????????????????????
            _x += iR(0, 0);
            _y += iR(1, 0);
            _w += iR(2, 0);
        }
    }
}
bool FusionCameraDetectionComponent::Init() {
  if (InitConfig() != cyber::SUCC) {
    AERROR << "InitConfig() failed.";
    return false;
  }
  writer_ =
      node_->CreateWriter<PerceptionObstacles>(output_obstacles_channel_name_);
  sensorframe_writer_ =
      node_->CreateWriter<SensorFrameMessage>(prefused_channel_name_);
  camera_viz_writer_ = node_->CreateWriter<CameraPerceptionVizMessage>(
      camera_perception_viz_message_channel_name_);
  camera_debug_writer_ =
      node_->CreateWriter<apollo::perception::camera::CameraDebug>(
          camera_debug_channel_name_);
  cyber::ReaderConfig chassis_reader_config;
  chassis_reader_config.channel_name = FLAGS_chassis_topic;
  chassis_reader_config.pending_queue_size = 10;

  chassis_reader_ =
      node_->CreateReader<apollo::canbus::Chassis>(chassis_reader_config, nullptr);

  CHECK(chassis_reader_ != nullptr);
  if (InitSensorInfo() != cyber::SUCC) {
    AERROR << "InitSensorInfo() failed.";
    return false;
  }
  if (InitAlgorithmPlugin() != cyber::SUCC) {
    AERROR << "InitAlgorithmPlugin() failed.";
    return false;
  }
  if (InitCameraFrames() != cyber::SUCC) {
    AERROR << "InitCameraFrames() failed.";
    return false;
  }
  if (InitProjectMatrix() != cyber::SUCC) {
    AERROR << "InitProjectMatrix() failed.";
    return false;
  }
  if (InitCameraListeners() != cyber::SUCC) {
    AERROR << "InitCameraListeners() failed.";
    return false;
  }
  if (InitMotionService() != cyber::SUCC) {
    AERROR << "InitMotionService() failed.";
    return false;
  }

  SetCameraHeightAndPitch();

  // Init visualizer
  // TODO(techoe, yg13): homography from image to ground should be
  // computed from camera height and pitch.
  // Apply online calibration to adjust pitch/height automatically
  // Temporary code is used here for test

  double pitch_adj_degree = 0.0;
  double yaw_adj_degree = 0.0;
  double roll_adj_degree = 0.0;
  // load in lidar to imu extrinsic
  Eigen::Matrix4d ex_lidar2imu;
  LoadExtrinsics(FLAGS_obs_sensor_intrinsic_path + "/" +
                     "velodyne128_novatel_extrinsics.yaml",
                 &ex_lidar2imu);
  AINFO << "velodyne128_novatel_extrinsics: " << ex_lidar2imu;

  CHECK(visualize_.Init_all_info_single_camera(
      camera_names_, visual_camera_, intrinsic_map_, extrinsic_map_,
      ex_lidar2imu, pitch_adj_degree, yaw_adj_degree, roll_adj_degree,
      image_height_, image_width_));

  homography_im2car_ = visualize_.homography_im2car(visual_camera_);
  camera_obstacle_pipeline_->SetIm2CarHomography(homography_im2car_);

  if (enable_cipv_) {
    cipv_.Init(homography_im2car_, min_laneline_length_for_cipv_,
               average_lane_width_in_meter_, max_vehicle_width_in_meter_,
               average_frame_rate_, image_based_cipv_, debug_level_);
  }

  if (enable_visualization_) {
    if (write_visual_img_) {
      visualize_.write_out_img_ = true;
      visualize_.SetDirectory(visual_debug_folder_);
    }
  }
  apollo::cyber::proto::RoleAttributes attr;
  attr.set_channel_name(FLAGS_routing_response_topic);
  response_writer_ = node_->CreateWriter<apollo::routing::RoutingResponse>(attr);
  for(int i = 0; i < 4; i++){
    last_frame_left_line[i] = 0.0;
    last_frame_right_line[i] = 0.0;
  }
  mapx_ = cv::Mat(cv::Size(image_width_,image_height_), CV_32FC1);
  mapy_ = cv::Mat(cv::Size(image_width_,image_height_), CV_32FC1);
  cv::Mat R = cv::Mat::eye(3,3,CV_32F);
  cv::Matx33d intrinsic_matrix, new_intrinsic_matrix;
  intrinsic_matrix(0,0) = 787.92;
  intrinsic_matrix(1,1) = 792.67;
  intrinsic_matrix(0,2) = 956.72;
  intrinsic_matrix(1,2) = 578.41;
  intrinsic_matrix(2,2) = 1.0;
  cv::Vec4d distortion_coeffs;
  distortion_coeffs[0] = 0.06697;
  distortion_coeffs[1] = 0.035987;
  distortion_coeffs[2] = -0.03336;
  distortion_coeffs[3] = 0.013;
  new_intrinsic_matrix(0,0) = 787.92 * 0.7;
  new_intrinsic_matrix(1,1) = 792.67 * 0.7;
  new_intrinsic_matrix(0,2) = 960.00;
  new_intrinsic_matrix(1,2) = 540.00;
  new_intrinsic_matrix(2,2) = 1.0;
  initUndistortRectifyMapFisheye(intrinsic_matrix,distortion_coeffs,R,new_intrinsic_matrix,
                                       cv::Size(image_width_,image_height_),CV_32FC1,mapx_,mapy_);
  left_lane_invalid_frame_cnt_ = 0;
  right_lane_invalid_frame_cnt_ = 0;
  left_right_lane_invalid_frame_cnt_ = 0;
  if(std::atoi(FLAGS_guide_post_start_id.c_str()) < 0 || std::atoi(FLAGS_guide_post_end_id.c_str()) < 0){
    return false;
  }
  guide_post_start_id_ = std::atoi(FLAGS_guide_post_start_id.c_str());
  guide_post_end_id_ = std::atoi(FLAGS_guide_post_end_id.c_str());
  guide_post_cur_id_index_ = std::atoi(FLAGS_guide_post_start_id.c_str()) - 1;
  guide_post_direction_ = FLAGS_guide_post_direction;
  guide_post_last_pt_.x = 0.0;
  guide_post_last_pt_.y = 0.0;
  guide_post_time_start_ = false;
  veh_run_dis_ = 0.0;
  return true;
}

void FusionCameraDetectionComponent::OnReceiveImage(
    const std::shared_ptr<apollo::drivers::Image> &message,
    const std::string &camera_name) {
  std::lock_guard<std::mutex> lock(mutex_);
    std::shared_ptr<SensorFrameMessage> prefused_message(new (std::nothrow)
                                                           SensorFrameMessage);
  
  const double msg_timestamp = message->measurement_time() + timestamp_offset_;
  AINFO << "Enter FusionCameraDetectionComponent::Proc(), "
        << " camera_name: " << camera_name << " image ts: " << msg_timestamp;
  // timestamp should be almost monotonic
  if (last_timestamp_ - msg_timestamp > ts_diff_) {
    AINFO << "Received an old message. Last ts is " << std::setprecision(19)
          << last_timestamp_ << " current ts is " << msg_timestamp
          << " last - current is " << last_timestamp_ - msg_timestamp;
    //return;//??????????????????????????????????????????????????????
  }
  last_timestamp_ = msg_timestamp;
  ++seq_num_;

  // for e2e lantency statistics
  {
    const double cur_time = apollo::common::time::Clock::NowInSeconds();
    const double start_latency = (cur_time - message->measurement_time()) * 1e3;
    AINFO << "FRAME_STATISTICS:Camera:Start:msg_time[" << camera_name << "-"
          << GLOG_TIMESTAMP(message->measurement_time()) << "]:cur_time["
          << GLOG_TIMESTAMP(cur_time) << "]:cur_latency[" << start_latency
          << "]";
  }

  // protobuf msg
  std::shared_ptr<apollo::perception::PerceptionObstacles> out_message(
      new (std::nothrow) apollo::perception::PerceptionObstacles);
  apollo::common::ErrorCode error_code = apollo::common::OK;

  if (InternalProc(message, camera_name, &error_code, prefused_message.get(),
                   out_message.get()) != cyber::SUCC) {
    AERROR << "InternalProc failed, error_code: " << error_code<<FLAGS_obs_sensor_intrinsic_path;
    if (MakeProtobufMsg(msg_timestamp, seq_num_, {}, {}, error_code,
                        out_message.get()) != cyber::SUCC) {
      AERROR << "MakeProtobufMsg failed";
      return;
    }
    if (output_final_obstacles_) {
      writer_->Write(out_message);
      
    }
    return;
  }
  bool send_sensorframe_ret = sensorframe_writer_->Write(prefused_message);
  AINFO << "send out prefused msg, ts: " << msg_timestamp
        << "ret: " << send_sensorframe_ret;
  // Send output msg
  if (output_final_obstacles_) {
    writer_->Write(out_message);
    cyber::common::SetProtoToASCIIFile(*out_message, "lane_test.proto");
  }
  // for e2e lantency statistics
  {
    const double end_timestamp = apollo::common::time::Clock::NowInSeconds();
    const double end_latency =
        (end_timestamp - message->measurement_time()) * 1e3;
    AINFO << "FRAME_STATISTICS:Camera:End:msg_time[" << camera_name << "-"
          << GLOG_TIMESTAMP(message->measurement_time()) << "]:cur_time["
          << GLOG_TIMESTAMP(end_timestamp) << "]:cur_latency[" << end_latency
          << "]";
  }
}

int FusionCameraDetectionComponent::InitConfig() {
  // the macro READ_CONF would return cyber::FAIL if config not exists
  apollo::perception::onboard::FusionCameraDetection
      fusion_camera_detection_param;
  if (!GetProtoConfig(&fusion_camera_detection_param)) {
    AINFO << "load fusion camera detection component proto param failed";
    return false;
  }

  std::string camera_names_str = fusion_camera_detection_param.camera_names();
  boost::algorithm::split(camera_names_, camera_names_str,
                          boost::algorithm::is_any_of(","));
  if (camera_names_.size() != 2) {
   // AERROR << "Now FusionCameraDetectionComponent only support 2 cameras";
   // return cyber::FAIL;
  }

  std::string input_camera_channel_names_str =
      fusion_camera_detection_param.input_camera_channel_names();
  boost::algorithm::split(input_camera_channel_names_,
                          input_camera_channel_names_str,
                          boost::algorithm::is_any_of(","));
  if (input_camera_channel_names_.size() != camera_names_.size()) {
    AERROR << "wrong input_camera_channel_names_.size(): "
           << input_camera_channel_names_.size();
    return cyber::FAIL;
  }

  camera_perception_init_options_.root_dir =
      fusion_camera_detection_param.camera_obstacle_perception_conf_dir();
  camera_perception_init_options_.conf_file =
      fusion_camera_detection_param.camera_obstacle_perception_conf_file();
  camera_perception_init_options_.lane_calibration_working_sensor_name =
      fusion_camera_detection_param.lane_calibration_working_sensor_name();
  camera_perception_init_options_.use_cyber_work_root = true;
  frame_capacity_ = fusion_camera_detection_param.frame_capacity();
  image_channel_num_ = fusion_camera_detection_param.image_channel_num();
  enable_undistortion_ = fusion_camera_detection_param.enable_undistortion();
  enable_visualization_ = fusion_camera_detection_param.enable_visualization();
  output_obstacles_channel_name_ =
      fusion_camera_detection_param.output_obstacles_channel_name();
  camera_perception_viz_message_channel_name_ =
      fusion_camera_detection_param
          .camera_perception_viz_message_channel_name();
  visual_debug_folder_ = fusion_camera_detection_param.visual_debug_folder();
  visual_camera_ = fusion_camera_detection_param.visual_camera();
  output_final_obstacles_ =
      fusion_camera_detection_param.output_final_obstacles();
  prefused_channel_name_ =
      fusion_camera_detection_param.prefused_channel_name();
  default_camera_pitch_ =
      static_cast<float>(fusion_camera_detection_param.default_camera_pitch());
  default_camera_height_ =
      static_cast<float>(fusion_camera_detection_param.default_camera_height());
  output_camera_debug_msg_ =
      fusion_camera_detection_param.output_camera_debug_msg();
  camera_debug_channel_name_ =
      fusion_camera_detection_param.camera_debug_channel_name();
  ts_diff_ = fusion_camera_detection_param.ts_diff();
  write_visual_img_ = fusion_camera_detection_param.write_visual_img();

  min_laneline_length_for_cipv_ = static_cast<float>(
      fusion_camera_detection_param.min_laneline_length_for_cipv());
  average_lane_width_in_meter_ = static_cast<float>(
      fusion_camera_detection_param.average_lane_width_in_meter());
  max_vehicle_width_in_meter_ = static_cast<float>(
      fusion_camera_detection_param.max_vehicle_width_in_meter());
  average_frame_rate_ =
      static_cast<float>(fusion_camera_detection_param.average_frame_rate());

  image_based_cipv_ =
      static_cast<float>(fusion_camera_detection_param.image_based_cipv());

  debug_level_ = static_cast<int>(fusion_camera_detection_param.debug_level());
  enable_cipv_ = fusion_camera_detection_param.enable_cipv();
  lane_detect_camera_ = fusion_camera_detection_param.lane_detect_camera();
  std::string format_str = R"(
      FusionCameraDetectionComponent InitConfig success
      camera_names:    %s, %s
      camera_obstacle_perception_conf_dir:    %s
      camera_obstacle_perception_conf_file:    %s
      frame_capacity:    %d
      image_channel_num:    %d
      enable_undistortion:    %d
      enable_visualization:    %d
      output_obstacles_channel_name:    %s
      camera_perception_viz_message_channel_name:    %s
      visual_debug_folder_:     %s
      visual_camera_:     %s
      output_final_obstacles:    %s
      prefused_channel_name:    %s
      write_visual_img_:    %s)";
  std::string config_info_str =
      str(boost::format(format_str.c_str()) % camera_names_[0] %
          camera_names_[1] % camera_perception_init_options_.root_dir %
          camera_perception_init_options_.conf_file % frame_capacity_ %
          image_channel_num_ % enable_undistortion_ % enable_visualization_ %
          output_obstacles_channel_name_ %
          camera_perception_viz_message_channel_name_ % visual_debug_folder_ %
          visual_camera_ % output_final_obstacles_ % prefused_channel_name_ %
          write_visual_img_);
  AINFO << config_info_str;

  return cyber::SUCC;
}

int FusionCameraDetectionComponent::InitSensorInfo() {
  if (camera_names_.size() != 2) {
    //AERROR << "invalid camera_names_.size(): " << camera_names_.size();
    //return cyber::FAIL;
  }

  auto *sensor_manager = common::SensorManager::Instance();
  for (size_t i = 0; i < camera_names_.size(); ++i) {
    if (!sensor_manager->IsSensorExist(camera_names_[i])) {
      AERROR << ("sensor_name: " + camera_names_[i] + " not exists.");
      return cyber::FAIL;
    }

    base::SensorInfo sensor_info;
    if (!(sensor_manager->GetSensorInfo(camera_names_[i], &sensor_info))) {
      AERROR << "Failed to get sensor info, sensor name: " << camera_names_[i];
      return cyber::FAIL;
    }
    sensor_info_map_[camera_names_[i]] = sensor_info;

    std::string tf_camera_frame_id =
        sensor_manager->GetFrameId(camera_names_[i]);
    tf_camera_frame_id_map_[camera_names_[i]] = tf_camera_frame_id;
    std::shared_ptr<TransformWrapper> trans_wrapper(new TransformWrapper);
    trans_wrapper->Init(tf_camera_frame_id);
    camera2world_trans_wrapper_map_[camera_names_[i]] = trans_wrapper;
  }

  // assume all camera have same image size
  base::BaseCameraModelPtr camera_model_ptr =
      sensor_manager->GetUndistortCameraModel(camera_names_[0]);
  image_width_ = static_cast<int>(camera_model_ptr->get_width());
  image_height_ = static_cast<int>(camera_model_ptr->get_height());

  std::string format_str = R"(
      camera_names: %s %s
      tf_camera_frame_ids: %s %s
      image_width: %d
      image_height: %d
      image_channel_num: %d)";
  std::string sensor_info_str =
      str(boost::format(format_str.c_str()) % camera_names_[0] %
          camera_names_[1] % tf_camera_frame_id_map_[camera_names_[0]] %
          tf_camera_frame_id_map_[camera_names_[1]] % image_width_ %
          image_height_ % image_channel_num_);
  AINFO << sensor_info_str;

  return cyber::SUCC;
}

int FusionCameraDetectionComponent::InitAlgorithmPlugin() {
  camera_obstacle_pipeline_.reset(new camera::ObstacleCameraPerception);
  if (!camera_obstacle_pipeline_->Init(camera_perception_init_options_)) {
    AERROR << "camera_obstacle_pipeline_->Init() failed";
    return cyber::FAIL;
  }
  AINFO << "camera_obstacle_pipeline_->Init() succeed";
  return cyber::SUCC;
}

int FusionCameraDetectionComponent::InitCameraFrames() {
  if (camera_names_.size() != 2) {
    //AERROR << "invalid camera_names_.size(): " << camera_names_.size();
    //return cyber::FAIL;
  }
  // fixed size
  camera_frames_.resize(frame_capacity_);
  if (camera_frames_.empty()) {
    AERROR << "frame_capacity_ must > 0";
    return cyber::FAIL;
  }

  // init data_providers for each camera
  for (const auto &camera_name : camera_names_) {
    camera::DataProvider::InitOptions data_provider_init_options;
    data_provider_init_options.image_height = image_height_;
    data_provider_init_options.image_width = image_width_;
    data_provider_init_options.do_undistortion = enable_undistortion_;
    data_provider_init_options.sensor_name = camera_name;
    int gpu_id = GetGpuId(camera_perception_init_options_);
    if (gpu_id == -1) {
      return cyber::FAIL;
    }
    data_provider_init_options.device_id = gpu_id;
    AINFO << "data_provider_init_options.device_id: "
          << data_provider_init_options.device_id
          << " camera_name: " << camera_name;

    std::shared_ptr<camera::DataProvider> data_provider(
        new camera::DataProvider);
    data_provider->Init(data_provider_init_options);
    data_providers_map_[camera_name] = data_provider;
  }

  //  init extrinsic/intrinsic
  for (const auto &camera_name : camera_names_) {
    base::BaseCameraModelPtr model;
    model =
        common::SensorManager::Instance()->GetUndistortCameraModel(camera_name);
    auto pinhole = static_cast<base::PinholeCameraModel *>(model.get());
    Eigen::Matrix3f intrinsic = pinhole->get_intrinsic_params();
    intrinsic_map_[camera_name] = intrinsic;
    AINFO << "#intrinsics of " << camera_name << ": "
          << intrinsic_map_[camera_name];
    Eigen::Matrix4d extrinsic;
    LoadExtrinsics(FLAGS_obs_sensor_intrinsic_path + "/" + camera_name +
                       "_extrinsics.yaml",
                   &extrinsic);
    extrinsic_map_[camera_name] = extrinsic;
    AINFO << "#extrinsics of " << camera_name << ": "
          << extrinsic_map_[camera_name];
  }

  // Init camera height
  for (const auto &camera_name : camera_names_) {
    float height = 0.0f;
    SetCameraHeight(camera_name, FLAGS_obs_sensor_intrinsic_path,
                    default_camera_height_, &height);
    camera_height_map_[camera_name] = height;
  }

  for (auto &frame : camera_frames_) {
    frame.track_feature_blob.reset(new base::Blob<float>());
    frame.lane_detected_blob.reset(new base::Blob<float>());
  }

  return cyber::SUCC;
}

int FusionCameraDetectionComponent::InitProjectMatrix() {
  if (!GetProjectMatrix(camera_names_, extrinsic_map_, intrinsic_map_,
                        &project_matrix_, &pitch_diff_)) {
    AERROR << "GetProjectMatrix failed";
    return cyber::FAIL;
  }
  AINFO << "project_matrix_: " << project_matrix_;
  AINFO << "pitch_diff_:" << pitch_diff_;
  name_camera_pitch_angle_diff_map_[camera_names_[0]] = 0.f;
  name_camera_pitch_angle_diff_map_[camera_names_[1]] =
      static_cast<float>(pitch_diff_);
  name_camera_pitch_angle_diff_map_[camera_names_[2]] =
      static_cast<float>(pitch_diff_);
  name_camera_pitch_angle_diff_map_[camera_names_[3]] =
      static_cast<float>(pitch_diff_);
  name_camera_pitch_angle_diff_map_[camera_names_[4]] =
      static_cast<float>(pitch_diff_);
  return cyber::SUCC;
}

int FusionCameraDetectionComponent::InitCameraListeners() {
  for (size_t i = 0; i < camera_names_.size(); ++i) {
    const std::string &camera_name = camera_names_[i];
    const std::string &channel_name = input_camera_channel_names_[i];
    const std::string &listener_name = camera_name + "_fusion_camera_listener";
    AINFO << "listener name: " << listener_name;

    typedef std::shared_ptr<apollo::drivers::Image> ImageMsgType;
    std::function<void(const ImageMsgType &)> camera_callback =
        std::bind(&FusionCameraDetectionComponent::OnReceiveImage, this,
                  std::placeholders::_1, camera_name);
    auto camera_reader = node_->CreateReader(channel_name, camera_callback);
  }
  return cyber::SUCC;
}

int FusionCameraDetectionComponent::InitMotionService() {
  const std::string &channel_name_local = "/apollo/perception/motion_service";
  std::function<void(const MotionServiceMsgType &)> motion_service_callback =
      std::bind(&FusionCameraDetectionComponent::OnMotionService, this,
                std::placeholders::_1);
  auto motion_service_reader =
      node_->CreateReader(channel_name_local, motion_service_callback);
  // initialize motion buffer
  if (motion_buffer_ == nullptr) {
    motion_buffer_ = std::make_shared<base::MotionBuffer>(motion_buffer_size_);
  } else {
    motion_buffer_->set_capacity(motion_buffer_size_);
  }
  return cyber::SUCC;
}

// On receiving motion service input, convert it to motion_buff_
void FusionCameraDetectionComponent::OnMotionService(
    const MotionServiceMsgType &message) {
  // Comment: use the circular buff to do it smartly, only push the latest
  // circular_buff only saves only the incremental motion between frames.
  // motion_service is now hard-coded for camera front 6mm
  base::VehicleStatus vehicledata;
  vehicledata.roll_rate = message->vehicle_status()[0].roll_rate();
  vehicledata.pitch_rate = message->vehicle_status()[0].pitch_rate();
  vehicledata.yaw_rate = message->vehicle_status()[0].yaw_rate();
  vehicledata.velocity = message->vehicle_status()[0].velocity();
  vehicledata.velocity_x = message->vehicle_status()[0].velocity_x();
  vehicledata.velocity_y = message->vehicle_status()[0].velocity_y();
  vehicledata.velocity_z = message->vehicle_status()[0].velocity_z();
  vehicledata.time_ts = message->vehicle_status()[0].time_ts();
  vehicledata.time_d = message->vehicle_status()[0].time_d();

  base::MotionType motion_2d = base::MotionType::Identity();
  motion_2d(0, 0) = message->vehicle_status()[0].motion().m00();
  motion_2d(0, 1) = message->vehicle_status()[0].motion().m01();
  motion_2d(0, 2) = message->vehicle_status()[0].motion().m02();
  motion_2d(0, 3) = message->vehicle_status()[0].motion().m03();
  motion_2d(1, 0) = message->vehicle_status()[0].motion().m10();
  motion_2d(1, 1) = message->vehicle_status()[0].motion().m11();
  motion_2d(1, 2) = message->vehicle_status()[0].motion().m12();
  motion_2d(1, 3) = message->vehicle_status()[0].motion().m13();
  motion_2d(2, 0) = message->vehicle_status()[0].motion().m20();
  motion_2d(2, 1) = message->vehicle_status()[0].motion().m21();
  motion_2d(2, 2) = message->vehicle_status()[0].motion().m22();
  motion_2d(2, 3) = message->vehicle_status()[0].motion().m23();
  motion_2d(3, 0) = message->vehicle_status()[0].motion().m30();
  motion_2d(3, 1) = message->vehicle_status()[0].motion().m31();
  motion_2d(3, 2) = message->vehicle_status()[0].motion().m32();
  motion_2d(3, 3) = message->vehicle_status()[0].motion().m33();
  vehicledata.motion = motion_2d;

  motion_buffer_->push_back(vehicledata);
  // TODO(@yg13): output motion in text file
}

void FusionCameraDetectionComponent::SetCameraHeightAndPitch() {
  camera_obstacle_pipeline_->SetCameraHeightAndPitch(
      camera_height_map_, name_camera_pitch_angle_diff_map_,
      default_camera_pitch_);
}
bool FusionCameraDetectionComponent::DetectGuidePost(cv::Mat img, std::vector<cv::Point2f> &grd_pt) {
  if (img.empty()) {
    return false;
  }
  cv::Rect rect_roi = cv::Rect(0, 350, 1920, 600);
  cv::Mat ImageRoi = img(rect_roi);
  cv::Mat img_pt(3, 1, CV_64FC1);
  
  cv::Mat grayImage, binaryImage, edgeImage;
  binaryImage = cv::Mat(ImageRoi.rows, ImageRoi.cols, CV_8UC1);
  std::vector<cv::Mat> channels;
  split(ImageRoi, channels);
  //threshold(channels[0], binaryImage, 100, 255, CV_THRESH_OTSU);
 // binaryImage = 2* channels[0] - channels[1] - channels[2];
  for (int i = 0; i < ImageRoi.rows; i++) {
    for (int j = 0; j < ImageRoi.cols; j++) {
      if (channels[1].at<uchar>(i, j) *1.0 / channels[0].at<uchar>(i, j) < 0.8) {
        binaryImage.at<uchar>(i, j) = 255;
      }
      else {
        binaryImage.at<uchar>(i, j) = 0;
      }
    }
  }
  erode(binaryImage, binaryImage, cv::Mat());
  dilate(binaryImage, binaryImage, cv::Mat());
  GaussianBlur(binaryImage, binaryImage, cv::Size(3, 3), 0, 0, cv::BORDER_DEFAULT);
  Canny(binaryImage, edgeImage, 100, 200);

  std::vector<std::vector<cv::Point>> contours;
  std::vector<cv::Vec4i> hierarchy;
  findContours(edgeImage, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE, cv::Point());
  std::vector<std::vector<cv::Point>> contours_ploy(contours.size());
  std::vector<cv::Moments>mu(contours.size());
  std::vector<cv::Point2f>mc(contours.size());
  for (unsigned int i = 0; i < contours.size(); i++) {
    approxPolyDP(contours[i], contours_ploy[i], 20, true);
    mu[i] = moments(contours_ploy[i], false);
    mc[i] = cv::Point2f(mu[i].m10 / mu[i].m00, mu[i].m01 / mu[i].m00);
    int corners = contours_ploy[i].size();
    cv::RotatedRect rect = minAreaRect(contours[i]);
    float w_h_scale = rect.size.width / rect.size.height;
    if (w_h_scale > 6 || w_h_scale < 0.2) {
      continue;
    }
    cv::Point2f P[4];
    rect.points(P);
    struct {
      bool operator()(cv::Point2f a, cv::Point2f b) const {
        return a.y > b.y;
      }
    } cmp_y;
    std::vector<cv::Point> corner_pt;
    for (int j = 0; j <= 3; j++){
      corner_pt.push_back(P[j]);
    }
    sort(corner_pt.begin(), corner_pt.end(), cmp_y);
    if (corners != 3){
      continue;
    }
    std::vector<cv::Point2f> corners_grd_pt;
    cv::Mat temp;
    cv::Point2f grd_pt_temp;
    for(int j = 0; j < corners; j++){
      img_pt.at<double>(0, 0) = contours_ploy[i][j].x + rect_roi.x;
      img_pt.at<double>(1, 0) = contours_ploy[i][j].y + rect_roi.y;
      img_pt.at<double>(2, 0) = 1.0;
      temp = mat_homography_im2car_ * img_pt;
      grd_pt_temp.x = temp.at<double>(0, 0) / temp.at<double>(2, 0);
      grd_pt_temp.y = temp.at<double>(1, 0) / temp.at<double>(2, 0);
      corners_grd_pt.push_back(grd_pt_temp);
    }
    float side_len[3];
    side_len[0] = sqrt(pow(corners_grd_pt[0].x - corners_grd_pt[1].x,2)+pow(corners_grd_pt[0].y - corners_grd_pt[1].y,2));
    side_len[1] = sqrt(pow(corners_grd_pt[1].x - corners_grd_pt[2].x,2)+pow(corners_grd_pt[1].y - corners_grd_pt[2].y,2));
    side_len[2] = sqrt(pow(corners_grd_pt[2].x - corners_grd_pt[0].x,2)+pow(corners_grd_pt[2].y - corners_grd_pt[0].y,2));
    //AERROR<<"side_len[0]:"<<side_len[0];
    //AERROR<<"side_len[1]:"<<side_len[1];
    //AERROR<<"side_len[2]:"<<side_len[2];
    if(fabs(side_len[0] - side_len[1]) > 0.15){
      continue;
    }
    if(fabs(side_len[1] - side_len[2]) > 0.15){
      continue;
    }
    if(fabs(side_len[0] - side_len[2]) > 0.15){
      continue;
    }
    if(side_len[0] < 0.10 || side_len[0] > 0.45)
    {
      continue;
    }
    if(side_len[1] < 0.10 || side_len[1] > 0.45)
    {
      continue;
    }
    if(side_len[2] < 0.10 || side_len[2] > 0.45)
    {
      continue;
    }
    img_pt.at<double>(0, 0) = mc[i].x + rect_roi.x;
    img_pt.at<double>(1, 0) = mc[i].y + rect_roi.y;
    img_pt.at<double>(2, 0) = 1.0;
    temp = mat_homography_im2car_ * img_pt;
    grd_pt_temp.x = temp.at<double>(0, 0) / temp.at<double>(2, 0);
    grd_pt_temp.y = temp.at<double>(1, 0) / temp.at<double>(2, 0);
    if(grd_pt_temp.x < 5.2 && fabs(grd_pt_temp.y) > 1.2){
      continue;//????????????????????????????????????????????????
    }
    if(fabs(grd_pt_temp.y) > 4.0){
      continue;
    }
    grd_pt.push_back(grd_pt_temp);
    // if (FLAGS_guide_post_debug_display) {
    //   cv::circle(img, cv::Point(img_pt.at<double>(0, 0), img_pt.at<double>(1, 0)), 5, cv::Scalar(255, 255, 255), 5);
    // }
  }
  if(grd_pt.size() != 1){
    grd_pt.clear();
  }
  else{
    // if(guide_post_last_pt_.x < 0.01){
    //   guide_post_last_pt_ = grd_pt[0];
    // }
    if (FLAGS_guide_post_debug_display) {
     cv::circle(img, cv::Point(img_pt.at<double>(0, 0), img_pt.at<double>(1, 0) - 30), 50, cv::Scalar(255, 255, 255), 5);
    }
  }
  
  if (FLAGS_guide_post_debug_display) {
    //cv::namedWindow("?????????", CV_WINDOW_NORMAL);
    //cv::imshow("?????????", binaryImage);
    cv::namedWindow("?????????", CV_WINDOW_NORMAL);
    cv::imshow("?????????", edgeImage);
    cv::namedWindow("src", CV_WINDOW_NORMAL);
    cv::resizeWindow("src", 640, 480);
    cv::imshow("src", img);
    cvWaitKey(1);
  }
  
  return true;
}
bool FusionCameraDetectionComponent::polynomial_curve_fit(std::vector<cv::Point2f>& key_point, int n, cv::Mat& A)
{
  //Number of key points
  int N = key_point.size();

  //????????????X
  cv::Mat X = cv::Mat::zeros(n + 1, n + 1, CV_64FC1);
  for (int i = 0; i < n + 1; i++)
  {
    for (int j = 0; j < n + 1; j++)
    {
      for (int k = 0; k < N; k++)
      {
        X.at<double>(i, j) = X.at<double>(i, j) +
          std::pow(key_point[k].x, i + j);
      }
    }
  }

  //????????????Y
  cv::Mat Y = cv::Mat::zeros(n + 1, 1, CV_64FC1);
  for (int i = 0; i < n + 1; i++)
  {
    for (int k = 0; k < N; k++)
    {
      Y.at<double>(i, 0) = Y.at<double>(i, 0) +
        std::pow(key_point[k].x, i) * key_point[k].y;
    }
  }

  A = cv::Mat::zeros(n + 1, 1, CV_64FC1);
  //????????????A
  cv::solve(X, Y, A, cv::DECOMP_LU);
  return true;
}
bool FusionCameraDetectionComponent::FindLinePt(cv::Mat &image, cv::Mat gra_img, int  row, int col_start, int col_end, float &line_center_x, float &line_w) {
  if (image.empty())
  {
    return false;
  }
  
  int col_start_temp = 0;
  int col_end_temp = 0;
  for (int j = col_start; j < col_end; j++) {
    if(image.at<uchar>(row, j) != 0){
      col_start_temp = j;
      break;
    }
  }
  for (int j = col_end; j > col_start; j--) {
    if(image.at<uchar>(row, j) != 0){
      col_end_temp = j;
      break;
    }
  }
  int search_w = 40;
  float sum = 0.0, avr = 0.0;
  int size = col_end_temp - col_start_temp;
  if (size <= 100) {
    return false;
  }
  for (int j = col_start_temp; j < col_end_temp; j++) {
    sum += image.at<uchar>(row, j);
  }
  avr = sum / size;
  for (int j = col_start_temp; j <= col_end_temp;  j++)
  {
    float cur_pt_gray_val = image.at<uchar>(row, j);
    if(image.at<uchar>(row, j - search_w) == 0){
      continue;
    }
    if(image.at<uchar>(row, j + search_w) == 0){
      continue;
    }
    if (cur_pt_gray_val > avr + 20) {
      float grad_x_max_left = 0;
      float grad_x_max_right = 0;
      int edge_left = 0, edge_right = 0, line_w_temp = 0;
      for (int k = 0; k < search_w; k++) {
        if (gra_img.at<uchar>(row, j - search_w + k - 3) > grad_x_max_left) {
          grad_x_max_left = gra_img.at<uchar>(row, j - search_w + k);
          edge_left = j - search_w + k;
        }
        if (gra_img.at<uchar>(row, j + k + 3) > grad_x_max_right) {
          grad_x_max_right = gra_img.at<uchar>(row, j + k);
          edge_right = j + k;
        }
      }
      
      line_w_temp = edge_right - edge_left;
      if (line_w_temp > 18 && line_w_temp < 40) {
        sum = 0;
        for (int k = edge_left; k < edge_right; k++) {
          sum += image.at<uchar>(row, k);
        }
        avr = sum / line_w_temp;
        sum = 0;
        for (int k = edge_left - line_w_temp; k < edge_left; k++) {
          sum += image.at<uchar>(row, k);
        }
        float left_avr = sum / line_w_temp;
        sum = 0;
        for (int k = edge_right; k < edge_right + line_w_temp; k++) {
          sum += image.at<uchar>(row, k);
        }
        float right_avr = sum / line_w_temp;
        if ((avr > left_avr + 20)
          && (avr > right_avr + 20))
        {
          line_center_x = (edge_right + edge_left) / 2;
          line_w = line_w_temp;
          //image.at<uchar>(row, (edge_right + edge_left) / 2) = 0;
          return true;
        }
      }
    }
  }
  return false;
}
bool FusionCameraDetectionComponent::DetectLine(cv::Mat &image, cv::Mat img_grad, int line_pos, cv::Mat &line_coff){
  float last_line_center_x, last_line_w;
  float new_line_center_x, new_line_w;
  bool find_begin_row = false;
  float begin_row;
  std::vector<cv::Point2f> line_pts_img;
  int start_col;
  int end_col;
  if(line_pos == 0){
    start_col = 100;
    end_col = image.cols / 2 - 100;
  }
  else{
    start_col = image.cols / 2 + 100;
    end_col = image.cols - 100;
  }
  line_pts_img.clear();
  for (int i = 950; i > 0; i--) {
    if (!find_begin_row) {
      find_begin_row = FindLinePt(image, img_grad, i, start_col, end_col, new_line_center_x, new_line_w);
      if (find_begin_row) {
        line_pts_img.push_back(cv::Point2f(i, last_line_center_x));
        last_line_center_x = new_line_center_x;
        last_line_w = new_line_w;
        begin_row = i;
      }
    }
    else {
      start_col = last_line_center_x - 60;
      end_col = last_line_center_x + 60;
      if (FindLinePt(image, img_grad, i, start_col, end_col, new_line_center_x, new_line_w)) {
        if(fabs(new_line_center_x - last_line_center_x) < 10.0
          && fabs(new_line_w - last_line_w) < 10.0){
          line_pts_img.push_back(cv::Point2f(i, new_line_center_x));
          last_line_center_x = new_line_center_x;
          last_line_w = new_line_w;
        }
      }
    }
  }
  if (line_pts_img.size() > 250) {
    polynomial_curve_fit(line_pts_img, 2, line_coff);
    for (int i = begin_row; i > 0; i--) {
      float col;
      col = line_coff.at<double>(2, 0) * i * i + line_coff.at<double>(1, 0) * i + line_coff.at<double>(0, 0);
      if(col > 0 && col < image.cols){
        //cv::circle(image, cv::Point(col, i), 1, cv::Scalar(255, 255, 255));
      }
    }
    return true;
  }
  return false;
}
bool FusionCameraDetectionComponent::TrackLine(cv::Mat &image, cv::Mat img_grad, int line_pos, cv::Mat input_line_coff, cv::Mat & output_line_coff){
  float last_line_center_x, last_line_w;
  float new_line_center_x, new_line_w;
  int start_col;
  int end_col;
  bool find_begin_row = false;
  std::vector<cv::Point2f> line_pts_img;
  line_pts_img.clear();
  for (int i = 950; i > 0; i--){
    float col_temp = input_line_coff.at<double>(2, 0) * i * i + input_line_coff.at<double>(1, 0) * i + input_line_coff.at<double>(0, 0);
    start_col = col_temp - 80;
    end_col = col_temp + 80;
    if(!find_begin_row){
      find_begin_row = FindLinePt(image, img_grad, i, start_col, end_col, new_line_center_x, new_line_w);
      if(find_begin_row){
        line_pts_img.push_back(cv::Point2f(i, new_line_center_x));
        last_line_center_x = new_line_center_x;
        last_line_w = new_line_w;
      }
    }
    else{
      if(FindLinePt(image, img_grad, i, start_col, end_col, new_line_center_x, new_line_w)){
        if(fabs(new_line_center_x - last_line_center_x) < 10.0
        && fabs(new_line_w - last_line_w) < 10.0){
          line_pts_img.push_back(cv::Point2f(i, new_line_center_x));
          last_line_center_x = new_line_center_x;
          last_line_w = new_line_w;
        }
      }
    }
  }
  
  if (line_pts_img.size() > 250) {
    polynomial_curve_fit(line_pts_img, 2, output_line_coff);
    //for (int i = 700; i > 0; i--) {
     // float col;
     // col = output_line_coff.at<double>(2, 0) * i * i + output_line_coff.at<double>(1, 0) * i + output_line_coff.at<double>(0, 0);
      // if(col > 0 && col < image.cols){
      //   cv::circle(image, cv::Point(col, i), 1, cv::Scalar(0, 0, 0));
      // }
    //}
    return true;
  }
  return false;
}
bool FusionCameraDetectionComponent::DetectLaneNew(cv::Mat & image, camera::CameraFrame *frame) {
  if (image.empty())
  {
    return false;
  }
  cv::Point2f src_points[] = {
    cv::Point2f(756,275),
    cv::Point2f(784,238),
    cv::Point2f(1445,260),
    cv::Point2f(1386,223)
  };

  cv::Point2f dst_points[] = {
    cv::Point2f(900,650),
    cv::Point2f(900,550),
    cv::Point2f(1390,650),
    cv::Point2f(1390,550)
  };
  cv::Mat M = cv::getPerspectiveTransform(src_points, dst_points);
  cv::Mat M_inv = M.inv();
  cv::Mat perspective;
  cv::warpPerspective(image, perspective, M, cv::Size(image.cols, image.rows), cv::INTER_NEAREST);
  cv::Mat grad_x, abs_grad_x;
  int scale = 1;
  int delta = 0;
  int ddepth = CV_16S;
  cv::Sobel(perspective, grad_x, ddepth, 1, 0, 3, scale, delta, cv::BORDER_DEFAULT);
  cv::convertScaleAbs(grad_x, abs_grad_x);
  cv::Mat left_line_coff(3, 1, CV_64FC1);
  cv::Mat right_line_coff(3, 1, CV_64FC1);
  std::vector<cv::Point2f> left_line_pts_grd;
  cv::Point2f left_line_pt_grd;
  cv::Mat left_line_coff_grd(3, 1, CV_64FC1);
  std::vector<cv::Point2f> right_line_pts_grd;
  cv::Point2f right_line_pt_grd;
  cv::Mat right_line_coff_grd(3, 1, CV_64FC1);

  static int lane_w = 700;

  static int left_track_status = 0;
  static cv::Mat last_left_line_coff(3, 1, CV_64FC1);
  static cv::Mat new_left_line_coff(3, 1, CV_64FC1);
  static int left_detect_num = 0;
  static int left_track_num = 0; 

  static int right_track_status = 0;
  static cv::Mat last_right_line_coff(3, 1, CV_64FC1);
  static cv::Mat new_right_line_coff(3, 1, CV_64FC1);
  static int right_detect_num = 0;
  static int right_track_num = 0; 
  if(left_track_num > 5){
    left_detect_num = 0;
    left_track_num = 0;
    left_track_status = 0;
  }
  if(left_detect_num >= 3){
    left_track_status = 1;
    left_detect_num = 0;
    left_track_num = 0;
  }
  if(left_track_status == 0){
    left_track_num = 0;
    if(DetectLine(perspective, abs_grad_x, 0, new_left_line_coff)){
      last_left_line_coff = new_left_line_coff;
      left_detect_num++;
    }else{
      left_detect_num = 0;
    }
  }
  else{
    left_detect_num = 0;
    if(TrackLine(perspective, abs_grad_x, 0, last_left_line_coff, new_left_line_coff) == false){
       left_track_num++;
    }
    else{
      left_track_num = 0;
      last_left_line_coff = (new_left_line_coff + last_left_line_coff) / 2;
    }
    if(left_track_num > 5){
      if((right_track_num <= 2) && (right_track_status == 1)){
        last_left_line_coff.at<double>(2, 0) = last_right_line_coff.at<double>(2, 0);
        last_left_line_coff.at<double>(1, 0) = last_right_line_coff.at<double>(1, 0);
        last_left_line_coff.at<double>(0, 0) = last_right_line_coff.at<double>(0, 0) - lane_w;
        left_track_num--;
        AERROR<<"use_right_cal_left";
      }
    }
    
  }

  if(right_track_num > 5){
    right_detect_num = 0;
    right_track_num = 0;
    right_track_status = 0;
  }
  if(right_detect_num >= 3){
    right_track_status = 1;
    right_detect_num = 0;
    right_track_num = 0;
  }
  if(right_track_status == 0){
    right_track_num = 0;
    if(DetectLine(perspective, abs_grad_x, 1, new_right_line_coff)){
      last_right_line_coff = new_right_line_coff;
      right_detect_num++;
    }else{
      right_detect_num = 0;
    }
  }
  else{
    right_detect_num = 0;
    if(TrackLine(perspective, abs_grad_x, 0, last_right_line_coff, new_right_line_coff) == false){
       right_track_num++;
    }
    else{
      right_track_num = 0;
      last_right_line_coff = (new_right_line_coff + last_right_line_coff) / 2;
    }
    if(right_track_num > 5){
      if((left_track_num <= 2) && left_track_status == 1) {
        last_right_line_coff.at<double>(2, 0) = last_left_line_coff.at<double>(2, 0);
        last_right_line_coff.at<double>(1, 0) = last_left_line_coff.at<double>(1, 0);
        last_right_line_coff.at<double>(0, 0) = last_left_line_coff.at<double>(0, 0) + lane_w;
        right_track_num--;
        AERROR<<"use_left_cal_right";
      }
    }
    
  }
  if(fabs(last_left_line_coff.at<double>(1,0)) < 0.3){
      if(last_left_line_coff.at<double>(0, 0) > 1100){
        left_track_status = 0;
      }
  }
  if(fabs(last_right_line_coff.at<double>(1,0)) < 0.3){
      if(last_right_line_coff.at<double>(0, 0) < 900){
        right_track_status = 0;
      }
  }
  if(fabs(last_left_line_coff.at<double>(2,0)) > 0.0005){
    left_track_status = 0;
  }
  if(fabs(last_right_line_coff.at<double>(2,0)) > 0.0005){
    right_track_status = 0;
  }
  if(left_track_status == 1 && right_track_status == 1){
    lane_w = last_right_line_coff.at<double>(0, 0) - last_left_line_coff.at<double>(0, 0);
    if(lane_w > 1000 || lane_w < 500){
      lane_w = 900;
    }
    else{
      double col;
      Eigen::Vector3d img_pt_temp;
      Eigen::Vector3d grd_pt_temp;
      double temp,temp_x, temp_y;
      for (int i = 900; i > 0; i--) {
        col = last_left_line_coff.at<double>(2, 0) * i * i + last_left_line_coff.at<double>(1, 0) * i + last_left_line_coff.at<double>(0, 0);
        if(col > 0 && col < perspective.cols){
          temp = M_inv.at<double>(2, 0) * col + M_inv.at<double>(2, 1) * i + M_inv.at<double>(2, 2);
          temp_x = (M_inv.at<double>(0, 0) * col + M_inv.at<double>(0, 1) * i + M_inv.at<double>(0, 2)) / temp;
          temp_y = (M_inv.at<double>(1, 0) * col + M_inv.at<double>(1, 1) * i + M_inv.at<double>(1, 2)) / temp;
          img_pt_temp<<temp_x, temp_y, 1.0;
          grd_pt_temp = homography_im2car_ * img_pt_temp;
          left_line_pt_grd.x = grd_pt_temp[0] / grd_pt_temp[2];
          left_line_pt_grd.y  = grd_pt_temp[1] / grd_pt_temp[2];
          left_line_pts_grd.push_back(left_line_pt_grd);
          if(FLAGS_lane_debug_display){
            cv::circle(image, cv::Point(temp_x, temp_y), 1, cv::Scalar(0, 0, 0));
            cv::circle(perspective, cv::Point(col, i), 1, cv::Scalar(0, 0, 0));
          }
        }
      }
      polynomial_curve_fit(left_line_pts_grd, 2, left_line_coff_grd);
      for (int i = 900; i > 0; i--) {
        col = last_right_line_coff.at<double>(2, 0) * i * i + last_right_line_coff.at<double>(1, 0) * i + last_right_line_coff.at<double>(0, 0);
        if(col > 0 && col < perspective.cols){
          temp = M_inv.at<double>(2, 0) * col + M_inv.at<double>(2, 1) * i + M_inv.at<double>(2, 2);
          temp_x = (M_inv.at<double>(0, 0) * col + M_inv.at<double>(0, 1) * i + M_inv.at<double>(0, 2)) / temp;
          temp_y = (M_inv.at<double>(1, 0) * col + M_inv.at<double>(1, 1) * i + M_inv.at<double>(1, 2)) / temp;
          img_pt_temp<<temp_x, temp_y, 1.0;
          grd_pt_temp = homography_im2car_ * img_pt_temp;
          right_line_pt_grd.x = grd_pt_temp[0] / grd_pt_temp[2];
          right_line_pt_grd.y  = grd_pt_temp[1] / grd_pt_temp[2];
          right_line_pts_grd.push_back(right_line_pt_grd);
          if(FLAGS_lane_debug_display){
            cv::circle(image, cv::Point(temp_x, temp_y), 1, cv::Scalar(255, 255, 255));
            cv::circle(perspective, cv::Point(col, i), 1, cv::Scalar(255, 255, 255));
          }
        }
      }
      polynomial_curve_fit(right_line_pts_grd, 2, right_line_coff_grd);
      base::LaneLine left_line_obj;
      left_line_obj.curve_car_coord.x_start = 6.0;
      left_line_obj.curve_car_coord.x_end = 10.0;
      left_line_obj.curve_car_coord.a = 0.0;
      left_line_obj.curve_car_coord.b = left_line_coff_grd.at<double>(2,0);
      left_line_obj.curve_car_coord.c = left_line_coff_grd.at<double>(1,0);
      left_line_obj.curve_car_coord.d = left_line_coff_grd.at<double>(0,0);
      left_line_obj.pos_type = base::LaneLinePositionType::EGO_LEFT;
      left_line_obj.type = base::LaneLineType::YELLOW_SOLID;
      frame->lane_objects.push_back(left_line_obj);

      base::LaneLine right_line_obj;
      right_line_obj.curve_car_coord.x_start = 6.0;
      right_line_obj.curve_car_coord.x_end = 10.0;
      right_line_obj.curve_car_coord.a = 0.0;
      right_line_obj.curve_car_coord.b = right_line_coff_grd.at<double>(2,0);;
      right_line_obj.curve_car_coord.c = right_line_coff_grd.at<double>(1,0);;
      right_line_obj.curve_car_coord.d = right_line_coff_grd.at<double>(0,0);
      right_line_obj.pos_type = base::LaneLinePositionType::EGO_RIGHT;
      right_line_obj.type = base::LaneLineType::YELLOW_SOLID;
      frame->lane_objects.push_back(right_line_obj);
    }
  }
  if(FLAGS_lane_debug_display){
    cv::namedWindow("image", CV_WINDOW_NORMAL);
    cv::imshow("image", image);
    cv::namedWindow("perspective", CV_WINDOW_NORMAL);
    cv::imshow("perspective", perspective);
    cvWaitKey(1);
  }
  return true;
}
bool FusionCameraDetectionComponent::DetectLane(cv::Mat & image, std::vector<cv::Vec4i> &lines_img) {
  if (image.empty())
  {
    return false;
  }
  cv::Point2f src_points[] = {
    cv::Point2f(674,264),
    cv::Point2f(708,234),
    cv::Point2f(1279,260),
    cv::Point2f(1245,231)
  };
  cv::Point2f dst_points[] = {
    cv::Point2f(674,264),
    cv::Point2f(674,160),
    cv::Point2f(1279,264),
    cv::Point2f(1279,160)
  };
  cv::Mat M = cv::getPerspectiveTransform(src_points, dst_points);
  cv::Mat M_inv = M.inv();
  cv::Mat perspective;
  cv::warpPerspective(image, perspective, M, cv::Size(image.cols, image.rows), cv::INTER_NEAREST);
  cv::Vec4i line_pt;
  cv::Vec4i line_pt_temp;
  cv::Point pt_start, pt_end;
  cv::Mat new_img;
  float scale = 1.0;
  
  cv::Rect rect_roi = cv::Rect(000, 0, 1920, 800);
  pt_start.y = 100;
  pt_end.y = 500;
  cv::resize(perspective, new_img, cv::Size(0, 0), scale, scale, cv::INTER_NEAREST);
  cv::Mat srcImageRoi = new_img(rect_roi);
  cv::Mat edgeImage;
  /*cv::Mat binaryImage(srcImageRoi.rows, srcImageRoi.cols, CV_8UC1);;
  cv::Mat gradX, absGradX;
  float sum1 = 0.0;
  float sum2 = 0.0;
  float avr1 = 0.0;
  float avr2 = 0.0;
  int block_size = 5;
  for (int i = 20; i < srcImageRoi.rows - 20; i++) {
    for (int j = 20; j < srcImageRoi.cols - 20; j++) {
      for (int k = 0; k < block_size; k++) {
        sum1 += srcImageRoi.at<float>(i, j + k);
        sum2 += srcImageRoi.at<float>(i, j - 1 - k);
      }
      avr1 = sum1 / block_size;
      avr2 = sum2 / block_size;
      if (avr1 - avr2 > 50) {
        binaryImage.at<float>(i, j) = 255;
      }
      else {
        binaryImage.at<float>(i, j) = 0;
      }
      sum1 = 0;
      sum2 = 0;
    }
  }*/
  //cv::Sobel(srcImageRoi,gradX,-1,1,0,CV_SCHARR);
  //cv::convertScaleAbs(gradX,absGradX);
  //cv::threshold(absGradX, edgeImage, 100, 255, CV_THRESH_OTSU);//??????????????????CV_THRESH_OTSU
  /*for (int j = 0; j < srcImageRoi.cols; j++){
    for (int i = 0; i < srcImageRoi.rows; i++) {
      if (srcImageRoi.at<uchar>(i, j) == 0) {
        int temp = srcImageRoi.at<uchar>(i - 1, j);
        srcImageRoi.at<uchar>(i, j) = srcImageRoi.at<uchar>(i -1, j);
      }
    }
  }*/
  cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
  //cv::erode(binaryImage,binaryImage,element);//??????
 
 // GaussianBlur(srcImageRoi, srcImageRoi, Size(3, 3), 0, 0, BORDER_DEFAULT);
  cv::Canny(srcImageRoi, edgeImage, 100, 200);
  //cv::erode(edgeImage,edgeImage,element);//??????
  cv::dilate(edgeImage, edgeImage, element);//??????
  std::vector<bool> used;
  std::map<int, std::vector<cv::Vec4i>> map_lines;
  // ??????????????????????????????90??????????????????50??????????????????10 
  std::vector<cv::Vec4i> lines;
  std::vector<cv::Vec4i> lines_temp;
  cv::HoughLinesP(edgeImage, lines, 1, CV_PI / 180, 80, 60, 20);
  if (lines.size() == 0) {
    return false;
  }
  used.resize(lines.size(), false);
  float slope;
  float intercept;
  float slope_temp;
  std::map<int, std::vector<cv::Vec4i>>::iterator iter;
  for (unsigned int i = 0; i < lines.size(); i++) {
    if (used[i]) {
      continue;
    }
    slope = (lines[i][0] - lines[i][2]) * 1.0 / (lines[i][1] - lines[i][3]);
    intercept = lines[i][0] - slope * lines[i][1];
    map_lines[i].push_back(lines[i]);
    for (unsigned j = i + 1; j < lines.size(); j++) {
      if (used[j]) continue;
      slope_temp = (lines[j][0] - lines[j][2]) * 1.0 / (lines[j][1] - lines[j][3]);
      float x_avr = (lines[j][0] + lines[j][2]) / 2;
      float y_avr = (lines[j][1] + lines[j][3]) / 2;
      float dis_pt_to_line = fabs(y_avr* slope + intercept - x_avr) / sqrt(slope* slope + 1);
      if (fabs(slope - slope_temp) < 0.15 && dis_pt_to_line < 50) {
        used[j] = true;
        map_lines[i].push_back(lines[j]);
      }
    }
  }
  iter = map_lines.begin();
  float slope_avr = 0;
  float intercept_avr = 0;
  while (iter != map_lines.end()) {
    float slope_sum = 0;
    float intercept_sum = 0;

    float slope_temp = 0;
    float intercept_temp = 0;
    bool invalid = true;
    for (unsigned int i = 0; i < iter->second.size(); i++) {
      if (iter->second[i][1] == iter->second[i][3]) {
        invalid = false;
        break;
      }
      slope_temp = (iter->second[i][0] - iter->second[i][2])* 1.0 / (iter->second[i][1] - iter->second[i][3]);
      intercept_temp = iter->second[i][0] - slope_temp * iter->second[i][1];
      slope_sum += slope_temp;
      intercept_sum += intercept_temp;
    }
    if (invalid == false) {
      iter++;
      continue;
    }
    slope_avr = slope_sum / iter->second.size();
    intercept_avr = intercept_sum / iter->second.size();
    //drawDetectLines(srcImageRoi, iter->second, Scalar(0, 255, 0));
    iter++;
    pt_start.x = pt_start.y * slope_avr + intercept_avr;
    pt_end.x = pt_end.y * slope_avr + intercept_avr;
    float gray_sum = 0.0;
    float gray_avr = 0.0;
    int pix_num = 0;
    for (int i = 0; i < srcImageRoi.rows; i++) {
      float x_temp = i * slope_avr + intercept_avr;
      for (int j = x_temp - 5; j < x_temp + 5; j++) {
        
        if (j < 0 || j >= srcImageRoi.cols-1) {
          continue;
        }
        else {
          gray_sum += srcImageRoi.at<unsigned char>(i, j);
          pix_num++;
        }
     }
    }
    if(pix_num != 0){
      gray_avr = gray_sum / pix_num;
    }
    
    if(gray_avr < 120){//???????????????????????????????????????????????????
      continue;
    }
    if(FLAGS_lane_debug_display){
      //cv::line(edgeImage, cv::Point(line_pt_temp[0], line_pt_temp[1]), cv::Point(line_pt_temp[2], line_pt_temp[3]), cv::Scalar(255, 255, 255), 2);
      cv::line(srcImageRoi, pt_start, pt_end, cv::Scalar(255, 255, 255), 2);
    }
    pt_start.x += rect_roi.x;
    pt_end.x += rect_roi.x;
    if (pt_start.x < 0 || pt_start.y < 0 || pt_start.x >= new_img.cols || pt_start.y >= new_img.rows) {
      continue;
    }
    if (pt_end.x < 0 || pt_end.y < 0 || pt_end.x >= new_img.cols || pt_end.y >= new_img.rows) {
      continue;
    }
    
    line_pt_temp[0] = pt_start.x / scale;
    line_pt_temp[1] = pt_start.y / scale;
    line_pt_temp[2] = pt_end.x / scale;
    line_pt_temp[3] = pt_end.y / scale;
    float temp;
    temp = M_inv.at<double>(2, 0) * line_pt_temp[0] + M_inv.at<double>(2, 1) * line_pt_temp[1] + M_inv.at<double>(2, 2);
    line_pt[0] = (M_inv.at<double>(0, 0) * line_pt_temp[0] + M_inv.at<double>(0, 1) * line_pt_temp[1] + M_inv.at<double>(0, 2)) / temp;
    line_pt[1] = (M_inv.at<double>(1, 0) * line_pt_temp[0] + M_inv.at<double>(1, 1) * line_pt_temp[1] + M_inv.at<double>(1, 2)) / temp;
    temp = M_inv.at<double>(2, 0) * line_pt_temp[2] + M_inv.at<double>(2, 1) * line_pt_temp[3] + M_inv.at<double>(2, 2);
    line_pt[2] = (M_inv.at<double>(0, 0) * line_pt_temp[2] + M_inv.at<double>(0, 1) * line_pt_temp[3] + M_inv.at<double>(0, 2)) / temp;
    line_pt[3] = (M_inv.at<double>(1, 0) * line_pt_temp[2] + M_inv.at<double>(1, 1) * line_pt_temp[3] + M_inv.at<double>(1, 2) / temp);
    if ((line_pt[0] + line_pt[2]) / 2 < 10)
    {
      continue;
    }
    if ((line_pt[0] + line_pt[2]) / 2 > 1900)
    {
      continue;
    }
    lines_temp.push_back(line_pt_temp);
    
  }
  std::vector<bool> used_temp;
  std::map<int, std::vector<cv::Vec4i>> map_lines_temp;
  used_temp.resize(lines_temp.size(), false);
  for (unsigned int i = 0; i < lines_temp.size(); i++) {
    if (used_temp[i]) {
      continue;
    }
    slope = fabs((lines_temp[i][0] - lines_temp[i][2]) * 1.0 / (lines_temp[i][1] - lines_temp[i][3]));
    intercept = lines_temp[i][0] - slope * lines_temp[i][1];
    map_lines_temp[i].push_back(lines_temp[i]);
    for (unsigned j = i + 1; j < lines_temp.size(); j++) {
      if (used_temp[j]) continue;
      slope_temp =fabs((lines_temp[j][0] - lines_temp[j][2]) * 1.0 / (lines_temp[j][1] - lines_temp[j][3]));
      float x_avr = (lines_temp[j][0] + lines_temp[j][2]) / 2;
      float y_avr = (lines_temp[j][1] + lines_temp[j][3]) / 2;
      float dis_pt_to_line = fabs(y_avr* slope + intercept - x_avr) / sqrt(slope* slope + 1);
      if (fabs(slope - slope_temp) < 0.20 && dis_pt_to_line > 600 && dis_pt_to_line < 900) {
        used_temp[j] = true;
        map_lines_temp[i].push_back(lines_temp[j]);
      }
    }
  }
  for (unsigned int i = 0; i < map_lines_temp.size(); i++) {
    if (map_lines_temp[i].size() > 1) {
      for (unsigned int j = 0; j < map_lines_temp[i].size(); j++) {
        float temp;
        temp = M_inv.at<double>(2, 0) * map_lines_temp[i][j][0] + M_inv.at<double>(2, 1) * map_lines_temp[i][j][1] + M_inv.at<double>(2, 2);
        line_pt[0] = (M_inv.at<double>(0, 0) * map_lines_temp[i][j][0] + M_inv.at<double>(0, 1) * map_lines_temp[i][j][1] + M_inv.at<double>(0, 2)) / temp;
        line_pt[1] = (M_inv.at<double>(1, 0) * map_lines_temp[i][j][0] + M_inv.at<double>(1, 1) * map_lines_temp[i][j][1] + M_inv.at<double>(1, 2)) / temp;
        temp = M_inv.at<double>(2, 0) * map_lines_temp[i][j][2] + M_inv.at<double>(2, 1) * map_lines_temp[i][j][3] + M_inv.at<double>(2, 2);
        line_pt[2] = (M_inv.at<double>(0, 0) * map_lines_temp[i][j][2] + M_inv.at<double>(0, 1) * map_lines_temp[i][j][3] + M_inv.at<double>(0, 2)) / temp;
        line_pt[3] = (M_inv.at<double>(1, 0) * map_lines_temp[i][j][2] + M_inv.at<double>(1, 1) * map_lines_temp[i][j][3] + M_inv.at<double>(1, 2) / temp);
        lines_img.push_back(line_pt);
        if(FLAGS_lane_debug_display){
          cv::line(image, cv::Point(line_pt[0], line_pt[1]), cv::Point(line_pt[2], line_pt[3]), cv::Scalar(0, 255,0), 2);
        }
        
      }
    }
  }
  if(FLAGS_lane_debug_display)
  {
    cv::namedWindow("?????????", CV_WINDOW_NORMAL);
    cv::imshow("?????????", edgeImage);
    cv::namedWindow("srcImageRoi", CV_WINDOW_NORMAL);
    cv::imshow("srcImageRoi", srcImageRoi);
    cv::namedWindow("src", CV_WINDOW_NORMAL);
    cv::imshow("src", image);
    cvWaitKey(1);
  }
  return true;
}
bool FusionCameraDetectionComponent::FilterLines(std::vector<cv::Vec4i> in_lines, std::vector<cv::Vec4i> &out_lines, camera::CameraFrame *frame){
  //std::vector<std::vector<float>> lines_left_grd;
  //std::vector<std::vector<float>> lines_right_grd;
  std::vector<float> line_temp;
  cv::Vec4i left_line_pt;
  cv::Vec4i right_line_pt;
  Eigen::Vector3d img_pt_temp;
  Eigen::Vector3d grd_pt_temp;
  Eigen::Vector4d line_grd_pt;
  double pi = 3.1415926;
  double angle_threshold = 40 * pi / 180;
  double slope;
  double intercept;
  double slope_angle;
  double left_pt_y = 1000.0;;
  double right_pt_y = -1000.0;
  Eigen::Vector2d left_line(0.0, 0.0);
  Eigen::Vector2d right_line(0.0, 0.0);
  Eigen::Vector2d left_line_grd_x(0.0, 0.0);
  Eigen::Vector2d right_line_grd_x(0.0, 0.0);
  bool left_line_valide = false, right_line_valide = false;
  for(unsigned int i = 0; i < in_lines.size(); i++){
    img_pt_temp<<static_cast<double>(in_lines[i][0]), static_cast<double>(in_lines[i][1]), 1.0;
    grd_pt_temp = homography_im2car_ * img_pt_temp;
    line_grd_pt[0] = grd_pt_temp[0] / grd_pt_temp[2];
    line_grd_pt[1] = grd_pt_temp[1] / grd_pt_temp[2];

    img_pt_temp<<static_cast<double>(in_lines[i][2]), static_cast<double>(in_lines[i][3]), 1.0;
    grd_pt_temp = homography_im2car_ * img_pt_temp;
    line_grd_pt[2] = grd_pt_temp[0] / grd_pt_temp[2];
    line_grd_pt[3] = grd_pt_temp[1] / grd_pt_temp[2];
    slope = (line_grd_pt[3] - line_grd_pt[1]) / (line_grd_pt[2] - line_grd_pt[0]);
    intercept = line_grd_pt[1] - slope * line_grd_pt[0];
    slope_angle = atan(slope);
    if(fabs(slope_angle) > angle_threshold){
      continue;
    }
    if(fabs(line_grd_pt[3]) > 15.0){
      continue;
    }
    if(line_grd_pt[3] > 0){
      if(line_grd_pt[3] < left_pt_y){//near the car
        left_line[0] = slope;
        left_line[1] = intercept;
        left_pt_y = line_grd_pt[3];
        left_line_grd_x[0] = line_grd_pt[2];//start point
        left_line_grd_x[1] = line_grd_pt[0];//end point
        for(unsigned int j = 0; j < 4; j++){
          left_line_pt[j] = in_lines[i][j];
        } 
        left_line_valide = true;
      }
    }
    if(line_grd_pt[3] < 0){
      if(line_grd_pt[3] > right_pt_y){//near the car
        right_line[0] = slope;
        right_line[1] = intercept;
        right_pt_y = line_grd_pt[3];
        right_line_grd_x[0] = line_grd_pt[2];//start point
        right_line_grd_x[1] = line_grd_pt[0];//end point
        for(unsigned int j = 0; j < 4; j++){
          right_line_pt[j] = in_lines[i][j];
        } 
        right_line_valide = true;
      }
    }
  }
  if(left_line_valide){
    out_lines.push_back(left_line_pt);
    //AERROR<<left_line_pt[0]<<","<<left_line_pt[1]<<","<<left_line_pt[2]<<","<<left_line_pt[3];
  }
  if(right_line_valide){
    out_lines.push_back(right_line_pt);
    //AERROR<<right_line_pt[0]<<","<<right_line_pt[1]<<","<<right_line_pt[2]<<","<<right_line_pt[3];
  }
  if(!left_line_valide || !right_line_valide){
    //return false;
  }
  if(left_line_valide){
    base::LaneLine left_line_obj;
    left_line_obj.curve_car_coord.x_start = left_line_grd_x[0];
    left_line_obj.curve_car_coord.x_end = left_line_grd_x[1];
    left_line_obj.curve_car_coord.a = 0.0;
    left_line_obj.curve_car_coord.b = 0.0;
    left_line_obj.curve_car_coord.c = left_line[0];
    left_line_obj.curve_car_coord.d = left_line[1];
    left_line_obj.pos_type = base::LaneLinePositionType::EGO_LEFT;
    left_line_obj.type = base::LaneLineType::YELLOW_SOLID;
    frame->lane_objects.push_back(left_line_obj);
  }
  if(right_line_valide){
    base::LaneLine right_line_obj;
    right_line_obj.curve_car_coord.x_start = right_line_grd_x[0];
    right_line_obj.curve_car_coord.x_end = right_line_grd_x[1];
    right_line_obj.curve_car_coord.a = 0.0;
    right_line_obj.curve_car_coord.b = 0.0;
    right_line_obj.curve_car_coord.c = right_line[0];
    right_line_obj.curve_car_coord.d = right_line[1];
    right_line_obj.pos_type = base::LaneLinePositionType::EGO_RIGHT;
    right_line_obj.type = base::LaneLineType::YELLOW_SOLID;
    frame->lane_objects.push_back(right_line_obj);
  }
  return true;
}
int FusionCameraDetectionComponent::InternalProc(
    const std::shared_ptr<apollo::drivers::Image const> &in_message,
    const std::string &camera_name, apollo::common::ErrorCode *error_code,
    SensorFrameMessage *prefused_message,
    apollo::perception::PerceptionObstacles *out_message) {
  const double msg_timestamp =
      in_message->measurement_time() + timestamp_offset_;
  const int frame_size = static_cast<int>(camera_frames_.size());
  camera::CameraFrame &camera_frame = camera_frames_[frame_id_ % frame_size];
  prefused_message->timestamp_ = msg_timestamp;
  prefused_message->seq_num_ = seq_num_;
  prefused_message->process_stage_ = ProcessStage::MONOCULAR_CAMERA_DETECTION;
  prefused_message->sensor_id_ = camera_name;
  prefused_message->frame_ = base::FramePool::Instance().Get();
  prefused_message->frame_->sensor_info = sensor_info_map_[camera_name];
  prefused_message->frame_->timestamp = msg_timestamp;

  // Get sensor to world pose from TF
  Eigen::Affine3d camera2world_trans =  Eigen::Affine3d::Identity();
  if (0){//!camera2world_trans_wrapper_map_[camera_name]->GetSensor2worldTrans(
          //msg_timestamp, &camera2world_trans)) {
    const std::string err_str =
        absl::StrCat("failed to get camera to world pose, ts: ", msg_timestamp,
                     " camera_name: ", camera_name);
    AERROR << err_str;
    *error_code = apollo::common::ErrorCode::PERCEPTION_ERROR_TF;
    prefused_message->error_code_ = *error_code;
    return cyber::FAIL;
  }
  Eigen::Affine3d world2camera = camera2world_trans.inverse();

  prefused_message->frame_->sensor2world_pose = camera2world_trans;

  // Fill camera frame
  // frame_size != 0, see InitCameraFrames()
  camera_frame.camera2world_pose = camera2world_trans;
  camera_frame.data_provider = data_providers_map_[camera_name].get();
  camera_frame.data_provider->FillImageData(
      image_height_, image_width_,
      reinterpret_cast<const uint8_t *>(in_message->data().data()),
      in_message->encoding());

  camera_frame.frame_id = frame_id_;
  camera_frame.timestamp = msg_timestamp;
  // get narrow to short projection matrix
  if (camera_frame.data_provider->sensor_name() == camera_names_[1]) {
    camera_frame.project_matrix = project_matrix_;
  } else {
    camera_frame.project_matrix.setIdentity();
  }

  ++frame_id_;
  // Run camera perception pipeline
  camera_obstacle_pipeline_->GetCalibrationService(
      &camera_frame.calibration_service);
  // cv::Mat src_gray_image(image_height_, image_width_, CV_8UC1);
  // base::Image8U src_gray_8U(image_height_, image_width_, base::Color::GRAY);
  // camera::DataProvider::ImageOptions image_options_gray;
  // image_options_gray.target_color = base::Color::GRAY;
  // camera_frame.data_provider->GetImage(image_options_gray, &src_gray_8U);
  // memcpy(src_gray_image.data, src_gray_8U.cpu_data(),
  //       src_gray_8U.total() * sizeof(uint8_t)); 
  
  cv::Mat output_image(image_height_, image_width_, CV_8UC3,
                       cv::Scalar(250, 0, 0));
  base::Image8U out_image(image_height_, image_width_, base::Color::RGB);
  camera::DataProvider::ImageOptions image_options;
  image_options.target_color = base::Color::BGR;
  camera_frame.data_provider->GetImage(image_options, &out_image);
  memcpy(output_image.data, out_image.cpu_data(),
         out_image.total() * sizeof(uint8_t));

  std::vector<cv::Vec4i> lines_img;
  std::vector<cv::Vec4i> after_filter_lines;
  camera_frame.lane_objects.clear();

  cv::Mat undistort_img(output_image.rows,output_image.cols,CV_32FC3);
  cv::remap(output_image,undistort_img,mapx_,mapy_,cv::INTER_NEAREST);
  // cv::Mat src_img = cv::imread("test2.png", 1);
  // cv::Mat new_src_img;
  // cv::resize(src_img, new_src_img, cv::Size(1920, 1080), 0, 0, cv::INTER_NEAREST);
  // cv::cvtColor(new_src_img, src_gray_image, CV_BGR2GRAY);
  if(camera_name == lane_detect_camera_){
    cv::eigen2cv(homography_im2car_,mat_homography_im2car_);
    std::vector<cv::Point2f> guide_post_grd_pt;
    DetectGuidePost(undistort_img, guide_post_grd_pt);
    GuidepostRoutingMap routing_map;
    cyber::common::GetProtoFromASCIIFile("/apollo/modules/perception/production/conf/perception/camera/guidepost_routing_map.pb.txt", &routing_map);
    //routing_map.routing_pt_size();
    
    const auto vec_routing_pts = routing_map.routing_pt();
    
    float cur_speed = 0.0;
    double diff;
    if(guide_post_time_start_){
      double cur_time = apollo::common::time::Clock::NowInSeconds();
      diff = cur_time - last_time_;
      chassis_reader_->Observe();
      auto msg = chassis_reader_->GetLatestObserved();
      if (msg)
      {
        cur_speed = msg->speed_mps();
        
      }
      veh_run_dis_ += diff * cur_speed;
      last_time_ = cur_time;
      //AERROR<<"veh_run_dis_:"<<veh_run_dis_<<","<<diff<<","<<cur_speed;
      //std::cout << std::fixed << msg->header().timestamp_sec()<< std::endl;
    }
    if(guide_post_grd_pt.size() > 0){
      if(guide_post_last_pt_.x < 0.01){
        guide_post_cur_id_index_ = guide_post_start_id_ - 1;
        guide_post_last_pt_ = guide_post_grd_pt[0];
        guide_post_time_start_ = 1;
        last_time_ = apollo::common::time::Clock::NowInSeconds();
        veh_run_dis_ = 0;
       // AERROR<<"guide post:"<<guide_post_cur_id_<<","<<veh_run_dis_<<","<<guide_post_grd_pt[0].x<<","<<guide_post_grd_pt[0].x - guide_post_last_pt_.x;
      }
      else{
        if((veh_run_dis_ > 1.8) || ((guide_post_grd_pt[0].x > guide_post_last_pt_.x + 0.5) && (veh_run_dis_ > 1.0))){
          AERROR<<"guide post:"<<guide_post_cur_id_<<","<<veh_run_dis_<<","<<guide_post_grd_pt[0].x<<","<<guide_post_grd_pt[0].y;
        //if((guide_post_grd_pt[0].x < 6.5) && (guide_post_grd_pt[0].x > guide_post_last_pt_.x + 0.5)){
          veh_run_dis_ = 0.0;
          guide_post_cur_id_index_++;
          if(guide_post_direction_ == true){
            guide_post_cur_id_index_ = guide_post_cur_id_index_  % routing_map.routing_pt_size();
          }
          else{
            guide_post_cur_id_index_ = routing_map.routing_pt_size() - guide_post_cur_id_index_ % routing_map.routing_pt_size();
          }
        }
        guide_post_last_pt_ = guide_post_grd_pt[0];
      }
      
      guide_post_cur_id_ = vec_routing_pts[guide_post_cur_id_index_].guidepost_seq();
      
      //guide_post_end_id_ = vec_routing_pts[routing_map.routing_pt_size() - 1].guidepost_seq();
      //AERROR<<"guide post:"<<guide_post_cur_id_<<","<<veh_run_dis_<<","<<guide_post_grd_pt[0].x<<;
      SendRoutingMsg(std::to_string(guide_post_cur_id_),std::to_string(guide_post_end_id_),
                     guide_post_grd_pt[0], vec_routing_pts[guide_post_cur_id_index_].lan_type());
    }
    cv::Mat undistort_img_gray;
    //auto end_time1 = std::chrono::system_clock::now();
    cv::cvtColor(undistort_img,undistort_img_gray,CV_BGR2GRAY);
    DetectLaneNew(undistort_img_gray, &camera_frame);
    //DetectLane(undistort_img_gray, lines_img);
    //FilterLines(lines_img, after_filter_lines, &camera_frame);
    
  //   cv::Point2f lane_grd_pt[4];//per lane two points
  //   if(camera_frame.lane_objects.size() == 0){
  //     left_right_lane_invalid_frame_cnt_++;
  //     if(left_right_lane_invalid_frame_cnt_ > 1){
  //       last_frame_left_line[3] = 0.0;
  //       last_frame_right_line[3] = 0.0;
  //     }
  //     else{
  //       left_right_lane_invalid_frame_cnt_ = 0;
  //     }
  //   }
    
  //   for(unsigned int i = 0; i < camera_frame.lane_objects.size(); i++){
  //     if(camera_frame.lane_objects[i].pos_type == base::LaneLinePositionType::EGO_LEFT){
  //       if(last_frame_left_line[3] < 0.0001){
  //         last_frame_left_line[0] = camera_frame.lane_objects[i].curve_car_coord.a;
  //         last_frame_left_line[1] = camera_frame.lane_objects[i].curve_car_coord.b;
  //         last_frame_left_line[2] = camera_frame.lane_objects[i].curve_car_coord.c;
  //         last_frame_left_line[3] = camera_frame.lane_objects[i].curve_car_coord.d;
  //         //AERRORAERROR<<"left init";
  //       }
  //       if(fabs(last_frame_left_line[3] - camera_frame.lane_objects[i].curve_car_coord.d) > 0.8 ||
  //           fabs(last_frame_left_line[2] - camera_frame.lane_objects[i].curve_car_coord.c) > 0.3){
  //           camera_frame.lane_objects[i].curve_car_coord.a = last_frame_left_line[0];
  //           camera_frame.lane_objects[i].curve_car_coord.b = last_frame_left_line[1];
  //           camera_frame.lane_objects[i].curve_car_coord.c = last_frame_left_line[2];
  //           camera_frame.lane_objects[i].curve_car_coord.d = last_frame_left_line[3];
  //           left_lane_invalid_frame_cnt_++;
  //           if(left_lane_invalid_frame_cnt_ > 2){
  //             last_frame_left_line[3] = 0.0;
  //           }
  //           //AERROR<<"left diff too large";
  //       }
  //       else{
  //         float temp_a, temp_b, temp_c, temp_d;
  //         left_lane_invalid_frame_cnt_ = 0;
  //         temp_a = camera_frame.lane_objects[i].curve_car_coord.a;
  //         temp_b = camera_frame.lane_objects[i].curve_car_coord.b;
  //         temp_c = camera_frame.lane_objects[i].curve_car_coord.c;
  //         temp_d = camera_frame.lane_objects[i].curve_car_coord.d;
  //         camera_frame.lane_objects[i].curve_car_coord.a = temp_a * 0.8 + last_frame_left_line[0] * 0.2;
  //         camera_frame.lane_objects[i].curve_car_coord.b = temp_b * 0.8 + last_frame_left_line[1] * 0.2;
  //         camera_frame.lane_objects[i].curve_car_coord.c = temp_c * 0.8 + last_frame_left_line[2] * 0.2;
  //         camera_frame.lane_objects[i].curve_car_coord.d = temp_d * 0.8 + last_frame_left_line[3] * 0.2;
  //         last_frame_left_line[0] = camera_frame.lane_objects[i].curve_car_coord.a;
  //         last_frame_left_line[1] = camera_frame.lane_objects[i].curve_car_coord.b;
  //         last_frame_left_line[2] = camera_frame.lane_objects[i].curve_car_coord.c;
  //         last_frame_left_line[3] = camera_frame.lane_objects[i].curve_car_coord.d;
  //         //AERROR<<"left diff good";
  //       }
  //       lane_grd_pt[0].x = 10.0;
  //       lane_grd_pt[0].y = camera_frame.lane_objects[i].curve_car_coord.c * lane_grd_pt[0].x + 
  //                           camera_frame.lane_objects[i].curve_car_coord.d;
  //       lane_grd_pt[1].x = 7.0;
  //       lane_grd_pt[1].y = camera_frame.lane_objects[i].curve_car_coord.c * lane_grd_pt[1].x + 
  //                           camera_frame.lane_objects[i].curve_car_coord.d;
  //     }
  //     if(camera_frame.lane_objects[i].pos_type == base::LaneLinePositionType::EGO_RIGHT){
  //       if(fabs(last_frame_right_line[3]) < 0.0001){
  //         last_frame_right_line[0] = camera_frame.lane_objects[i].curve_car_coord.a;
  //         last_frame_right_line[1] = camera_frame.lane_objects[i].curve_car_coord.b;
  //         last_frame_right_line[2] = camera_frame.lane_objects[i].curve_car_coord.c;
  //         last_frame_right_line[3] = camera_frame.lane_objects[i].curve_car_coord.d;
  //         //AERROR<<"right init";
  //       }
  //       if(fabs(last_frame_right_line[3] - camera_frame.lane_objects[i].curve_car_coord.d) > 0.8 ||
  //           fabs(last_frame_right_line[2] - camera_frame.lane_objects[i].curve_car_coord.c) > 0.3){
  //           camera_frame.lane_objects[i].curve_car_coord.a = last_frame_right_line[0];
  //           camera_frame.lane_objects[i].curve_car_coord.b = last_frame_right_line[1];
  //           camera_frame.lane_objects[i].curve_car_coord.c = last_frame_right_line[2];
  //           camera_frame.lane_objects[i].curve_car_coord.d = last_frame_right_line[3];
  //           right_lane_invalid_frame_cnt_++;
  //           if(right_lane_invalid_frame_cnt_ > 2){
  //             last_frame_right_line[3] = 0.0;
  //           }
  //           //AERROR<<"right diff too large";
  //       }
  //       else{
  //         float temp_a, temp_b, temp_c, temp_d;
  //         right_lane_invalid_frame_cnt_ = 0;
  //         temp_a = camera_frame.lane_objects[i].curve_car_coord.a;
  //         temp_b = camera_frame.lane_objects[i].curve_car_coord.b;
  //         temp_c = camera_frame.lane_objects[i].curve_car_coord.c;
  //         temp_d = camera_frame.lane_objects[i].curve_car_coord.d;
  //         camera_frame.lane_objects[i].curve_car_coord.a = temp_a * 0.8 + last_frame_right_line[0] * 0.2;
  //         camera_frame.lane_objects[i].curve_car_coord.b = temp_b * 0.8 + last_frame_right_line[1] * 0.2;
  //         camera_frame.lane_objects[i].curve_car_coord.c = temp_c * 0.8 + last_frame_right_line[2] * 0.2;
  //         camera_frame.lane_objects[i].curve_car_coord.d = temp_d * 0.8 + last_frame_right_line[3] * 0.2;
  //         last_frame_right_line[0] = camera_frame.lane_objects[i].curve_car_coord.a;
  //         last_frame_right_line[1] = camera_frame.lane_objects[i].curve_car_coord.b;
  //         last_frame_right_line[2] = camera_frame.lane_objects[i].curve_car_coord.c;
  //         last_frame_right_line[3] = camera_frame.lane_objects[i].curve_car_coord.d;
  //         //AERROR<<"right diff good";
  //       }
  //       lane_grd_pt[2].x = 10.0;
  //       lane_grd_pt[2].y = camera_frame.lane_objects[i].curve_car_coord.c * lane_grd_pt[2].x + 
  //                           camera_frame.lane_objects[i].curve_car_coord.d;
  //       lane_grd_pt[3].x = 7.0;
  //       lane_grd_pt[3].y = camera_frame.lane_objects[i].curve_car_coord.c * lane_grd_pt[3].x + 
  //                           camera_frame.lane_objects[i].curve_car_coord.d;
        
  //     }
  //   }
    
  //   if(FLAGS_lane_debug_display){
  //     Eigen::Matrix3d homography_car2im = homography_im2car_.inverse();
  //     cv::eigen2cv(homography_car2im,mat_homography_car2im_);
  //     cv::Mat grd_pt(3,1,CV_64FC1);
  //     cv::Point img_pt[4];
  //     cv::Mat temp;
  //     for(unsigned int i = 0; i < 2; i++){
  //       grd_pt.at<double>(0,0) = lane_grd_pt[2*i].x;
  //       grd_pt.at<double>(1,0) = lane_grd_pt[2*i].y;
  //       grd_pt.at<double>(2,0) = 1.0;
  //       temp = mat_homography_car2im_ * grd_pt;
  //       img_pt[2*i].x = temp.at<double>(0, 0) / temp.at<double>(2, 0);
  //       img_pt[2*i].y = temp.at<double>(1, 0) / temp.at<double>(2, 0);
  //       grd_pt.at<double>(0,0) = lane_grd_pt[2*i+1].x;
  //       grd_pt.at<double>(1,0) = lane_grd_pt[2*i+1].y;
  //       grd_pt.at<double>(2,0) = 1.0;
  //       temp = mat_homography_car2im_ * grd_pt;
  //       img_pt[2*i+1].x = temp.at<double>(0, 0) / temp.at<double>(2, 0);
  //       img_pt[2*i+1].y = temp.at<double>(1, 0) / temp.at<double>(2, 0);
  //       cv::line(undistort_img, img_pt[2*i], 
  //               img_pt[2*i+1], cv::Scalar(0, 255, 0), 2);
  //     }
  //     cv::namedWindow("lane_filter", CV_WINDOW_NORMAL);
  //     cv::imshow("lane_filter", undistort_img);
  //     cvWaitKey(1);
  //   }
  }
  
  if (0){//!camera_obstacle_pipeline_->Perception(camera_perception_options_,&camera_frame)) {
    AERROR << "camera_obstacle_pipeline_->Perception() failed"
           << " msg_timestamp: " << msg_timestamp;
    *error_code = apollo::common::ErrorCode::PERCEPTION_ERROR_PROCESS;
    prefused_message->error_code_ = *error_code;
    return cyber::FAIL;
  }
  AINFO << "##" << camera_name << ": pitch "
        << camera_frame.calibration_service->QueryPitchAngle()
        << " | camera_grond_height "
        << camera_frame.calibration_service->QueryCameraToGroundHeight();
  prefused_message->frame_->objects = camera_frame.tracked_objects;
  // TODO(gaohan02, wanji): check the boxes with 0-width in perception-camera
    //add by houxuefeng
    //char str[100];

// cv::Mat output_image(image_height_, image_width_, CV_8UC3,
//                        cv::Scalar(250, 0, 0));
//   base::Image8U out_image(image_height_, image_width_, base::Color::RGB);
//   camera::DataProvider::ImageOptions image_options;
//   image_options.target_color = base::Color::BGR;
//   camera_frame.data_provider->GetImage(image_options, &out_image);
//   memcpy(output_image.data, out_image.cpu_data(),
//          out_image.total() * sizeof(uint8_t));
  const cv::Rect rectified_rect(100, 100,
                                  100, 100);
  cv::Scalar tl_color;
  tl_color = cv::Scalar(255, 0, 0);
  prefused_message->frame_->camera_frame_supplement.image_ptr =  camera_frame.data_provider->bgr_;           
//add by houxuefeng
//   prefused_message->frame_->objects.clear();
//   for (auto obj : camera_frame.tracked_objects) {
//     auto &box = obj->camera_supplement.box;
//     if (box.xmin < box.xmax && box.ymin < box.ymax) {
//       prefused_message->frame_->objects.push_back(obj);
//       //add by houxuefeng
//       const cv::Rect rectified_rect(box.xmin, box.ymin,
//                                   box.xmax-box.xmin, box.ymax-box.ymin);
//       cv::Scalar tl_color;
//       tl_color = cv::Scalar(0, 255, 0);
//       std::string obj_name;
//       switch (int(obj->type)) {
//         case 0:
//           obj_name = "unknown";
//           break;
//         case 1:
//           obj_name = "unknown_movable";
//           break;
//         case 2:
//           obj_name = "unknown_unmovable";
//           break;
//         case 3:
//           obj_name = "pedestrian";
//           break;
//         case 4:
//           obj_name = "bicycle";
//           break;
//         case 5:
//           obj_name = "vehicle";
//           break;
//         default:
//           obj_name = "error";
//       break;
//       }
//       if(obj_name == "pedestrian"){
//         //continue;
//       }
//       float obj_img_h = box.ymax-box.ymin;
//       float obj_world_h = obj->size[2];
//       if(box.xmax - box.xmin > 450)
//       {
//         //continue;//
//       }
//       float dis = obj_world_h * 2000 / obj_img_h;
//       std::string res;
//       res = obj_name + std::to_string(dis);
//       cv::putText(output_image, obj_name, cv::Point(box.xmin, box.ymin),
//               cv::FONT_HERSHEY_DUPLEX, 2.0, tl_color, 3);
//       cv::rectangle(output_image, rectified_rect, tl_color, 2);
//       //add by houxuefeng
//     }
//   }
 //cv::namedWindow(camera_name, CV_WINDOW_NORMAL);
 //cv::resizeWindow(camera_name, 640, 480);
//   //add by houxuefeng
// cv::imshow(camera_name, output_image);
 // cvWaitKey(1);                      
//add by houxuefeng
  // process success, make pb msg

  const std::vector<base::ObjectPtr> objects_temp;
  if (output_final_obstacles_ &&
      MakeProtobufMsg(msg_timestamp, seq_num_, objects_temp,    //modify camera_frame.tracked_objects to objects_temp, only send lane msg
                      camera_frame.lane_objects, *error_code,
                      out_message) != cyber::SUCC) {
    AERROR << "MakeProtobufMsg failed ts: " << msg_timestamp;
    *error_code = apollo::common::ErrorCode::PERCEPTION_ERROR_UNKNOWN;
    prefused_message->error_code_ = *error_code;
    return cyber::FAIL;
  }
  
  *error_code = apollo::common::ErrorCode::OK;
  prefused_message->error_code_ = *error_code;
  prefused_message->frame_->camera_frame_supplement.on_use = true;
  if (FLAGS_obs_enable_visualization) {
    camera::DataProvider::ImageOptions image_options;
    image_options.target_color = base::Color::RGB;

    // Use Blob to pass image data
    prefused_message->frame_->camera_frame_supplement.image_blob.reset(
        new base::Blob<uint8_t>);
    camera_frame.data_provider->GetImageBlob(
        image_options,
        prefused_message->frame_->camera_frame_supplement.image_blob.get());
  }

  //  Determine CIPV
  if (enable_cipv_) {
    CipvOptions cipv_options;
    if (motion_buffer_ != nullptr) {
      if (motion_buffer_->size() == 0) {
        AWARN << "motion_buffer_ is empty";
        cipv_options.velocity = 5.0f;
        cipv_options.yaw_rate = 0.0f;
      } else {
        cipv_options.velocity = motion_buffer_->back().velocity;
        cipv_options.yaw_rate = motion_buffer_->back().yaw_rate;
      }
      ADEBUG << "[CIPV] velocity " << cipv_options.velocity
             << ", yaw rate: " << cipv_options.yaw_rate;
      cipv_.DetermineCipv(camera_frame.lane_objects, cipv_options, world2camera,
                          &camera_frame.tracked_objects);

      // Get Drop points
      if (motion_buffer_->size() > 0) {
        cipv_.CollectDrops(motion_buffer_, world2camera,
                           &camera_frame.tracked_objects);
      } else {
        AWARN << "motion_buffer is empty";
      }
    }
  }

  // Send msg for visualization
  if (enable_visualization_) {
    camera::DataProvider::ImageOptions image_options;
    image_options.target_color = base::Color::BGR;
    std::shared_ptr<base::Blob<uint8_t>> image_blob(new base::Blob<uint8_t>);
    camera_frame.data_provider->GetImageBlob(image_options, image_blob.get());
    std::shared_ptr<CameraPerceptionVizMessage> viz_msg(
        new (std::nothrow) CameraPerceptionVizMessage(
            camera_name, msg_timestamp, camera2world_trans.matrix(), image_blob,
            camera_frame.tracked_objects, camera_frame.lane_objects,
            *error_code));
    bool send_viz_ret = camera_viz_writer_->Write(viz_msg);
    AINFO << "send out camera visualization msg, ts: " << msg_timestamp
          << " send_viz_ret: " << send_viz_ret;

    cv::Mat output_image(image_height_, image_width_, CV_8UC3,
                         cv::Scalar(0, 0, 0));
    base::Image8U out_image(image_height_, image_width_, base::Color::RGB);
    camera_frame.data_provider->GetImage(image_options, &out_image);
    memcpy(output_image.data, out_image.cpu_data(),
           out_image.total() * sizeof(uint8_t));
    visualize_.ShowResult_all_info_single_camera(output_image, camera_frame,
                                                 motion_buffer_, world2camera);
  }

  // send out camera debug message
  if (output_camera_debug_msg_) {
    // camera debug msg
    std::shared_ptr<apollo::perception::camera::CameraDebug> camera_debug_msg(
        new (std::nothrow) apollo::perception::camera::CameraDebug);
    if (MakeCameraDebugMsg(msg_timestamp, camera_name, camera_frame,
                           camera_debug_msg.get()) != cyber::SUCC) {
      AERROR << "make camera_debug_msg failed";
      return cyber::FAIL;
    }
    camera_debug_writer_->Write(camera_debug_msg);
  }
  
  return cyber::SUCC;
}
int FusionCameraDetectionComponent::SendRoutingMsg(std::string cur_id,std::string end_id,cv::Point2f guide_post_pos, LanType lane_type){
  auto response = std::make_shared<apollo::routing::RoutingResponse>();
  apollo::routing::RoutingResponse* const routing_response = response.get();
  double publish_time = apollo::common::time::Clock::NowInSeconds();
  apollo::common::Header *header = routing_response->mutable_header();
  header->set_timestamp_sec(publish_time);
  header->set_module_name("perception_camera");
  header->set_sequence_num(seq_num_);  
  auto routing_request = routing_response->mutable_routing_request();
  header = routing_request->mutable_header();
  header->set_timestamp_sec(publish_time);
  header->set_module_name("perception_camera");
  header->set_sequence_num(seq_num_);
  routing_request->set_is_use_guidepost(lane_type == LanType::curve);
  
  routing_request->set_end_id(end_id);
  auto *start_point = routing_request->add_waypoint();
  start_point->set_id(cur_id);
  start_point->set_s(0.0);
  auto point0 = apollo::common::util::PointFactory::ToPointENU(0, 0);
  start_point->mutable_pose()->CopyFrom(point0);

  auto *end_point = routing_request->add_waypoint();
  end_point->set_id(cur_id);
  end_point->set_s(0.0);
  auto point1 = apollo::common::util::PointFactory::ToPointENU(guide_post_pos.x,guide_post_pos.y);
  end_point->mutable_pose()->CopyFrom(point1);

  routing_response->set_map_version("1.5");
  apollo::common::StatusPb* const error_code = routing_response->mutable_status();
  error_code->set_error_code(apollo::common::ErrorCode::OK);
  error_code->set_msg("Success");  
  response_writer_->Write(response);
  return true;
}
int FusionCameraDetectionComponent::MakeProtobufMsg(
    double msg_timestamp, int seq_num,
    const std::vector<base::ObjectPtr> &objects,
    const std::vector<base::LaneLine> &lane_objects,
    const apollo::common::ErrorCode error_code,
    apollo::perception::PerceptionObstacles *obstacles) {
  double publish_time = apollo::common::time::Clock::NowInSeconds();
  apollo::common::Header *header = obstacles->mutable_header();
  header->set_timestamp_sec(publish_time);
  header->set_module_name("perception_camera");
  header->set_sequence_num(seq_num);
  // in nanosecond
  // PnC would use lidar timestamp to predict
  header->set_lidar_timestamp(static_cast<uint64_t>(msg_timestamp * 1e9));
  header->set_camera_timestamp(static_cast<uint64_t>(msg_timestamp * 1e9));
  // header->set_radar_timestamp(0);

  // write out obstacles in world coordinates
  obstacles->set_error_code(error_code);
  for (const auto &obj : objects) {
    apollo::perception::PerceptionObstacle *obstacle =
        obstacles->add_perception_obstacle();
    if (ConvertObjectToPb(obj, obstacle) != cyber::SUCC) {
      AERROR << "ConvertObjectToPb failed, Object:" << obj->ToString();
      return cyber::FAIL;
    }
  }

  // write out lanes in ego coordinates
  apollo::perception::LaneMarkers *lane_markers =
      obstacles->mutable_lane_marker();
  apollo::perception::LaneMarker *lane_marker_l0 =
      lane_markers->mutable_left_lane_marker();
  apollo::perception::LaneMarker *lane_marker_r0 =
      lane_markers->mutable_right_lane_marker();
  apollo::perception::LaneMarker *lane_marker_l1 =
      lane_markers->add_next_left_lane_marker();
  apollo::perception::LaneMarker *lane_marker_r1 =
      lane_markers->add_next_right_lane_marker();

  for (const auto &lane : lane_objects) {
    base::LaneLineCubicCurve curve_coord = lane.curve_car_coord;
    
    switch (lane.pos_type) {
      case base::LaneLinePositionType::EGO_LEFT:
        fill_lane_msg(curve_coord, lane_marker_l0);
        break;
      case base::LaneLinePositionType::EGO_RIGHT:
        fill_lane_msg(curve_coord, lane_marker_r0);
        break;
      case base::LaneLinePositionType::ADJACENT_LEFT:
        fill_lane_msg(curve_coord, lane_marker_l1);
        break;
      case base::LaneLinePositionType::ADJACENT_RIGHT:
        fill_lane_msg(curve_coord, lane_marker_r1);
        break;
      default:
        break;
    }
  }

  return cyber::SUCC;
}

int FusionCameraDetectionComponent::ConvertObjectToPb(
    const base::ObjectPtr &object_ptr,
    apollo::perception::PerceptionObstacle *pb_msg) {
  if (!object_ptr || !pb_msg) {
    return cyber::FAIL;
  }

  pb_msg->set_id(object_ptr->track_id);
  pb_msg->set_theta(object_ptr->theta);

  apollo::common::Point3D *obj_center = pb_msg->mutable_position();
  obj_center->set_x(object_ptr->center(0));
  obj_center->set_y(object_ptr->center(1));
  obj_center->set_z(object_ptr->center(2));

  apollo::common::Point3D *obj_velocity = pb_msg->mutable_velocity();
  obj_velocity->set_x(object_ptr->velocity(0));
  obj_velocity->set_y(object_ptr->velocity(1));
  obj_velocity->set_z(object_ptr->velocity(2));

  pb_msg->set_length(object_ptr->size(0));
  pb_msg->set_width(object_ptr->size(1));
  pb_msg->set_height(object_ptr->size(2));

  // convert 3d bbox to polygon
  int polygon_point_size = static_cast<int>(object_ptr->polygon.size());
  for (int i = 0; i < polygon_point_size; ++i) {
    auto &pt = object_ptr->polygon.at(i);
    apollo::common::Point3D *p = pb_msg->add_polygon_point();
    p->set_x(pt.x);
    p->set_y(pt.y);
    p->set_z(pt.z);
  }

  // for camera results, set object's center as anchor point
  apollo::common::Point3D *obj_anchor_point = pb_msg->mutable_anchor_point();
  obj_anchor_point->set_x(object_ptr->center(0));
  obj_anchor_point->set_y(object_ptr->center(1));
  obj_anchor_point->set_z(object_ptr->center(2));

  apollo::perception::BBox2D *obj_bbox2d = pb_msg->mutable_bbox2d();
  const base::BBox2DF &box = object_ptr->camera_supplement.box;
  obj_bbox2d->set_xmin(box.xmin);
  obj_bbox2d->set_ymin(box.ymin);
  obj_bbox2d->set_xmax(box.xmax);
  obj_bbox2d->set_ymax(box.ymax);

  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      pb_msg->add_position_covariance(object_ptr->center_uncertainty(i, j));
      pb_msg->add_velocity_covariance(object_ptr->velocity_uncertainty(i, j));
    }
  }

  pb_msg->set_tracking_time(object_ptr->tracking_time);
  pb_msg->set_type(static_cast<PerceptionObstacle::Type>(object_ptr->type));
  pb_msg->set_sub_type(
      static_cast<PerceptionObstacle::SubType>(object_ptr->sub_type));
  pb_msg->set_timestamp(object_ptr->latest_tracked_time);  // in seconds.

  pb_msg->set_height_above_ground(object_ptr->size(2));

  if (object_ptr->type == base::ObjectType::VEHICLE) {
    apollo::perception::LightStatus *light_status =
        pb_msg->mutable_light_status();
    const base::CarLight &car_light = object_ptr->car_light;
    light_status->set_brake_visible(car_light.brake_visible);
    light_status->set_brake_switch_on(car_light.brake_switch_on);

    light_status->set_left_turn_visible(car_light.left_turn_visible);
    light_status->set_left_turn_switch_on(car_light.left_turn_switch_on);

    light_status->set_right_turn_visible(car_light.right_turn_visible);
    light_status->set_right_turn_switch_on(car_light.right_turn_switch_on);
  }

  return cyber::SUCC;
}

int FusionCameraDetectionComponent::ConvertObjectToCameraObstacle(
    const base::ObjectPtr &object_ptr,
    apollo::perception::camera::CameraObstacle *camera_obstacle) {
  if (camera_obstacle == nullptr) {
    AERROR << "camera_obstacle is not available";
    return false;
  }
  apollo::perception::PerceptionObstacle *obstacle =
      camera_obstacle->mutable_obstacle();
  ConvertObjectToPb(object_ptr, obstacle);

  camera_obstacle->set_type(
      static_cast<apollo::perception::camera::CameraObstacle::CameraType>(
          object_ptr->type));
  for (const auto &prob : object_ptr->type_probs) {
    camera_obstacle->add_type_probs(prob);
  }
  camera_obstacle->mutable_upper_left()->set_x(
      object_ptr->camera_supplement.box.xmin);
  camera_obstacle->mutable_upper_left()->set_y(
      object_ptr->camera_supplement.box.ymin);
  camera_obstacle->mutable_lower_right()->set_x(
      object_ptr->camera_supplement.box.xmax);
  camera_obstacle->mutable_lower_right()->set_y(
      object_ptr->camera_supplement.box.ymax);

  std::stringstream type_score_msg;
  type_score_msg << apollo::perception::camera::CameraObstacle::CameraType_Name(
                        camera_obstacle->type())
                 << ": "
                 << camera_obstacle->type_probs(camera_obstacle->type());
  camera_obstacle->add_debug_message(type_score_msg.str());

  return cyber::SUCC;
}

int FusionCameraDetectionComponent::ConvertLaneToCameraLaneline(
    const base::LaneLine &lane_line,
    apollo::perception::camera::CameraLaneLine *camera_laneline) {
  if (camera_laneline == nullptr) {
    AERROR << "camera_laneline is not available";
    return false;
  }
  // fill the lane line attribute
  apollo::perception::camera::LaneLineType line_type =
      static_cast<apollo::perception::camera::LaneLineType>(lane_line.type);
  camera_laneline->set_type(line_type);
  apollo::perception::camera::LaneLinePositionType pos_type =
      static_cast<apollo::perception::camera::LaneLinePositionType>(
          lane_line.pos_type);
  camera_laneline->set_pos_type(pos_type);
  camera_laneline->set_track_id(lane_line.track_id);
  camera_laneline->set_confidence(lane_line.confidence);
  apollo::perception::camera::LaneLineUseType use_type =
      static_cast<apollo::perception::camera::LaneLineUseType>(
          lane_line.use_type);
  camera_laneline->set_use_type(use_type);
  // fill the curve camera coord
  camera_laneline->mutable_curve_camera_coord()->set_longitude_min(
      lane_line.curve_camera_coord.x_start);
  camera_laneline->mutable_curve_camera_coord()->set_longitude_max(
      lane_line.curve_camera_coord.x_end);
  camera_laneline->mutable_curve_camera_coord()->set_a(
      lane_line.curve_camera_coord.a);
  camera_laneline->mutable_curve_camera_coord()->set_b(
      lane_line.curve_camera_coord.b);
  camera_laneline->mutable_curve_camera_coord()->set_c(
      lane_line.curve_camera_coord.c);
  camera_laneline->mutable_curve_camera_coord()->set_d(
      lane_line.curve_camera_coord.d);
  // fill the curve image coord
  camera_laneline->mutable_curve_image_coord()->set_longitude_min(
      lane_line.curve_image_coord.x_start);
  camera_laneline->mutable_curve_image_coord()->set_longitude_max(
      lane_line.curve_image_coord.x_end);
  camera_laneline->mutable_curve_image_coord()->set_a(
      lane_line.curve_image_coord.a);
  camera_laneline->mutable_curve_image_coord()->set_b(
      lane_line.curve_image_coord.b);
  camera_laneline->mutable_curve_image_coord()->set_c(
      lane_line.curve_image_coord.c);
  camera_laneline->mutable_curve_image_coord()->set_d(
      lane_line.curve_image_coord.d);
  // fill the curve image point set
  for (size_t i = 0; i < lane_line.curve_image_point_set.size(); i++) {
    const base::Point2DF &image_point2d = lane_line.curve_image_point_set[i];
    apollo::common::Point2D *lane_point_2d =
        camera_laneline->add_curve_image_point_set();
    lane_point_2d->set_x(image_point2d.x);
    lane_point_2d->set_y(image_point2d.y);
  }
  // fill the curve camera point set
  for (size_t i = 0; i < lane_line.curve_camera_point_set.size(); i++) {
    const base::Point3DF &point3d = lane_line.curve_camera_point_set[i];
    apollo::common::Point3D *lane_point_3d =
        camera_laneline->add_curve_camera_point_set();
    lane_point_3d->set_x(point3d.x);
    lane_point_3d->set_y(point3d.y);
    lane_point_3d->set_z(point3d.z);
  }
  // fill the line end point set
  for (size_t i = 0; i < lane_line.image_end_point_set.size(); i++) {
    const base::EndPoints &end_points = lane_line.image_end_point_set[i];
    apollo::perception::camera::EndPoints *lane_end_points =
        camera_laneline->add_image_end_point_set();
    lane_end_points->mutable_start()->set_x(end_points.start.x);
    lane_end_points->mutable_start()->set_y(end_points.start.y);
    lane_end_points->mutable_end()->set_x(end_points.end.x);
    lane_end_points->mutable_end()->set_y(end_points.end.y);
  }
  return cyber::SUCC;
}

int FusionCameraDetectionComponent::MakeCameraDebugMsg(
    double msg_timestamp, const std::string &camera_name,
    const camera::CameraFrame &camera_frame,
    apollo::perception::camera::CameraDebug *camera_debug_msg) {
  if (camera_debug_msg == nullptr) {
    AERROR << "camera_debug_msg is not available";
    return false;
  }

  auto itr = std::find(camera_names_.begin(), camera_names_.end(), camera_name);
  if (itr == camera_names_.end()) {
    AERROR << "invalid camera_name: " << camera_name;
    return cyber::FAIL;
  }
  int input_camera_channel_names_idx =
      static_cast<int>(itr - camera_names_.begin());
  int input_camera_channel_names_size =
      static_cast<int>(input_camera_channel_names_.size());
  if (input_camera_channel_names_idx < 0 ||
      input_camera_channel_names_idx >= input_camera_channel_names_size) {
    AERROR << "invalid input_camera_channel_names_idx: "
           << input_camera_channel_names_idx
           << " input_camera_channel_names_.size(): "
           << input_camera_channel_names_.size();
    return cyber::FAIL;
  }
  std::string source_channel_name =
      input_camera_channel_names_[input_camera_channel_names_idx];
  camera_debug_msg->set_source_topic(source_channel_name);

  // Fill header info.
  apollo::common::Header *header = camera_debug_msg->mutable_header();
  header->set_camera_timestamp(static_cast<uint64_t>(msg_timestamp * 1.0e9));

  // Fill the tracked objects
  const std::vector<base::ObjectPtr> &objects = camera_frame.tracked_objects;
  for (const auto &obj : objects) {
    apollo::perception::camera::CameraObstacle *camera_obstacle =
        camera_debug_msg->add_camera_obstacle();
    ConvertObjectToCameraObstacle(obj, camera_obstacle);
  }

  // Fill the laneline objects
  const std::vector<base::LaneLine> &lane_objects = camera_frame.lane_objects;
  for (const auto &lane_obj : lane_objects) {
    apollo::perception::camera::CameraLaneLine *camera_laneline =
        camera_debug_msg->add_camera_laneline();
    ConvertLaneToCameraLaneline(lane_obj, camera_laneline);
  }

  // Fill the calibrator information(pitch angle)
  float pitch_angle = camera_frame.calibration_service->QueryPitchAngle();
  camera_debug_msg->mutable_camera_calibrator()->set_pitch_angle(pitch_angle);
  return cyber::SUCC;
}

}  // namespace onboard
}  // namespace perception
}  // namespace apollo
