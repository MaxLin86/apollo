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
#pragma once

#include <map>
#include <memory>
#include <string>
#include <vector>

#include "cyber/component/component.h"
#include "modules/drivers/proto/sensor_image.pb.h"
#include "modules/perception/base/object.h"
#include "modules/perception/base/object_types.h"
#include "modules/perception/base/point.h"
#include "modules/perception/camera/app/cipv_camera.h"
#include "modules/perception/camera/app/obstacle_camera_perception.h"
#include "modules/perception/camera/app/perception.pb.h"
#include "modules/perception/camera/common/util.h"
#include "modules/perception/camera/lib/interface/base_camera_perception.h"
#include "modules/perception/camera/tools/offline/visualizer.h"
#include "modules/perception/onboard/component/camera_perception_viz_message.h"
#include "modules/perception/onboard/inner_component_messages/inner_component_messages.h"
#include "modules/perception/onboard/proto/fusion_camera_detection_component.pb.h"
#include "modules/perception/onboard/transform_wrapper/transform_wrapper.h"
#include "modules/perception/proto/motion_service.pb.h"
#include "modules/perception/proto/perception_camera.pb.h"
#include "modules/perception/proto/perception_obstacle.pb.h"
#include "modules/routing/proto/routing.pb.h"
#include "modules/perception/proto/guidepost_routing_map.pb.h"
#include "modules/canbus/proto/chassis.pb.h"
#include <opencv2/opencv.hpp>
typedef std::shared_ptr<apollo::perception::Motion_Service>
    MotionServiceMsgType;

namespace apollo {
namespace perception {
namespace onboard {

class FusionCameraDetectionComponent : public apollo::cyber::Component<> {
 public:
  FusionCameraDetectionComponent() : seq_num_(0) {}
  ~FusionCameraDetectionComponent();

  FusionCameraDetectionComponent(const FusionCameraDetectionComponent&) =
      delete;
  FusionCameraDetectionComponent& operator=(
      const FusionCameraDetectionComponent&) = delete;

  bool Init() override;

 private:
  void OnReceiveImage(const std::shared_ptr<apollo::drivers::Image>& in_message,
                      const std::string& camera_name);
  void initUndistortRectifyMapFisheye(cv::InputArray K, cv::InputArray D, cv::InputArray R, cv::InputArray P,
    const cv::Size& size, int m1type, cv::OutputArray map1, cv::OutputArray map2);
  int InitConfig();
  int InitSensorInfo();
  int InitAlgorithmPlugin();
  int InitCameraFrames();
  int InitProjectMatrix();
  int InitCameraListeners();
  int InitMotionService();
  void SetCameraHeightAndPitch();
  void OnMotionService(const MotionServiceMsgType& message);
  bool DetectGuidePost(cv::Mat img, std::vector<cv::Point2f> &grd_pt);
  bool polynomial_curve_fit(std::vector<cv::Point2f>& key_point, int n, cv::Mat& A);
  bool FindLinePt(cv::Mat &image, cv::Mat gra_img, int  row, int col_start, int col_end, float &line_center_x, float &line_w); 
  bool DetectLine(cv::Mat &image, cv::Mat img_grad, int line_pos, cv::Mat &line_coff);
  bool TrackLine(cv::Mat &image, cv::Mat img_grad, int line_pos, const cv::Mat input_line_coff, cv::Mat & output_line_coff);
  bool DetectLaneNew(cv::Mat & image, camera::CameraFrame *frame);
  bool DetectLane(cv::Mat & image, std::vector<cv::Vec4i> &lines);
  bool FilterLines(std::vector<cv::Vec4i> in_lines, std::vector<cv::Vec4i> &out_lines, 
                   camera::CameraFrame *frame);
  int InternalProc(
      const std::shared_ptr<apollo::drivers::Image const>& in_message,
      const std::string& camera_name, apollo::common::ErrorCode* error_code,
      SensorFrameMessage* prefused_message,
      apollo::perception::PerceptionObstacles* out_message);

  int MakeProtobufMsg(double msg_timestamp, int seq_num,
                      const std::vector<base::ObjectPtr>& objects,
                      const std::vector<base::LaneLine>& lane_objects,
                      const apollo::common::ErrorCode error_code,
                      apollo::perception::PerceptionObstacles* obstacles);

  int ConvertObjectToPb(const base::ObjectPtr& object_ptr,
                        apollo::perception::PerceptionObstacle* pb_msg);

  int ConvertObjectToCameraObstacle(
      const base::ObjectPtr& object_ptr,
      apollo::perception::camera::CameraObstacle* camera_obstacle);

  int ConvertLaneToCameraLaneline(
      const base::LaneLine& lane_line,
      apollo::perception::camera::CameraLaneLine* camera_laneline);

  int MakeCameraDebugMsg(
      double msg_timestamp, const std::string& camera_name,
      const camera::CameraFrame& camera_frame,
      apollo::perception::camera::CameraDebug* camera_debug_msg);
  int SendRoutingMsg(std::string cur_id,std::string end_id,cv::Point2f guide_post_pos, LanType lane_type);
 private:
  std::mutex mutex_;
  uint32_t seq_num_;

  std::vector<std::shared_ptr<cyber::Node>> camera_listener_nodes_;

  std::vector<std::string> camera_names_;  // camera sensor names
  std::vector<std::string> input_camera_channel_names_;

  // camera name -> SensorInfo
  std::map<std::string, base::SensorInfo> sensor_info_map_;

  // camera_height
  std::map<std::string, float> camera_height_map_;

  // camera_pitch_angle_diff
  // the pitch_diff = pitch_narrow - pitch_obstacle
  std::map<std::string, float> name_camera_pitch_angle_diff_map_;

  // TF stuff
  std::map<std::string, std::string> tf_camera_frame_id_map_;
  std::map<std::string, std::shared_ptr<TransformWrapper>>
      camera2world_trans_wrapper_map_;

  // pre-allocaated-mem data_provider;
  std::map<std::string, std::shared_ptr<camera::DataProvider>>
      data_providers_map_;

  // map for store params
  std::map<std::string, Eigen::Matrix4d> extrinsic_map_;
  std::map<std::string, Eigen::Matrix3f> intrinsic_map_;
  Eigen::Matrix3d homography_im2car_;
  cv::Mat mat_homography_im2car_;
  cv::Mat mat_homography_car2im_;
  int left_lane_invalid_frame_cnt_;
  int right_lane_invalid_frame_cnt_;
  int left_right_lane_invalid_frame_cnt_;
  int guide_post_start_id_;
  int guide_post_end_id_;
  int guide_post_cur_id_;
  int guide_post_cur_id_index_;
  bool guide_post_direction_;
  cv::Point2f guide_post_last_pt_;
  cv::Mat m_perspective_;
  cv::Mat m_perspective_inv_;
  std::shared_ptr<::apollo::cyber::Writer<apollo::routing::RoutingResponse>> response_writer_ = nullptr;
  std::shared_ptr<apollo::cyber::Reader<apollo::canbus::Chassis>>
      chassis_reader_;
  GuidepostRoutingMap routing_map_;
  // camera obstacle pipeline
  camera::CameraPerceptionInitOptions camera_perception_init_options_;
  camera::CameraPerceptionOptions camera_perception_options_;
  std::unique_ptr<camera::ObstacleCameraPerception> camera_obstacle_pipeline_;

  // fixed size camera frames
  int frame_capacity_ = 20;
  int frame_id_ = 0;
  std::vector<camera::CameraFrame> camera_frames_;

  // image info.
  int image_width_ = 1920;
  int image_height_ = 1080;
  int image_channel_num_ = 3;
  int image_data_size_ = -1;

  // default camera pitch angle & height
  float default_camera_pitch_ = 0.f;
  float default_camera_height_ = 1.6f;

  // options for DataProvider
  bool enable_undistortion_ = false;

  double timestamp_offset_ = 0.0;

  std::string prefused_channel_name_;

  bool enable_visualization_ = false;
  std::string camera_perception_viz_message_channel_name_;
  std::string visual_debug_folder_;
  std::string visual_camera_;

  bool output_final_obstacles_ = false;
  std::string output_obstacles_channel_name_;

  bool output_camera_debug_msg_ = false;
  std::string camera_debug_channel_name_;

  Eigen::Matrix3d project_matrix_;
  double pitch_diff_ = 0.0;

  double last_timestamp_ = 0.0;
  double ts_diff_ = 1.0;
  
  std::shared_ptr<
      apollo::cyber::Writer<apollo::perception::PerceptionObstacles>>
      writer_;

  std::shared_ptr<apollo::cyber::Writer<SensorFrameMessage>>
      sensorframe_writer_;

  std::shared_ptr<apollo::cyber::Writer<CameraPerceptionVizMessage>>
      camera_viz_writer_;

  std::shared_ptr<
      apollo::cyber::Writer<apollo::perception::camera::CameraDebug>>
      camera_debug_writer_;

  // variable for motion service
  base::MotionBufferPtr motion_buffer_;
  const int motion_buffer_size_ = 100;

  // // variables for CIPV
  bool enable_cipv_ = false;
  Cipv cipv_;
  float min_laneline_length_for_cipv_ = kMinLaneLineLengthForCIPV;
  float average_lane_width_in_meter_ = kAverageLaneWidthInMeter;
  float max_vehicle_width_in_meter_ = kMaxVehicleWidthInMeter;
  float average_frame_rate_ = kAverageFrameRate;
  bool image_based_cipv_ = false;
  int debug_level_ = 0;
  // variables for visualization
  camera::Visualizer visualize_;
  bool write_visual_img_;
  float last_frame_left_line[4];
  float last_frame_right_line[4];
  std::string lane_detect_camera_;
  double last_time_;
  bool guide_post_time_start_;
  float veh_run_dis_;
  cv::Mat mapx_;
  cv::Mat mapy_;
};

CYBER_REGISTER_COMPONENT(FusionCameraDetectionComponent);

}  // namespace onboard
}  // namespace perception
}  // namespace apollo
