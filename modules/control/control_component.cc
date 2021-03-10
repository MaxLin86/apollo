/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
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
#include "modules/control/control_component.h"

#include "absl/strings/str_cat.h"
#include "cyber/common/file.h"
#include "cyber/common/log.h"
#include "modules/common/math/math_utils.h"
#include "modules/common/adapters/adapter_gflags.h"
#include "modules/common/latency_recorder/latency_recorder.h"
#include "modules/common/time/time.h"
#include "modules/common/vehicle_state/vehicle_state_provider.h"
#include "modules/control/common/control_gflags.h"

        
#define ARRIVING_SPEED_KMH  2.0
#define KP_SPD_CTRL   0.5//0.3
#define KD_SPD_CTRL   0.3//0.05
#define KP_THROTTLE_CTRL 6.0//5.0
#define THROTTLE_COMPENSATION_MAX_PER_SEC 3.5//3.0
#define THROTTLE_RELEASE_MAX_PER_SEC 4.5//5.5
#define THROTTLE_ALLOWED_MAX  20.0
#define ALLOWED_SPEED_ERROR_ZONE  (0.1/3.6)
#define THROTTLE_START_VAL 10.0

namespace apollo {
namespace control {

using apollo::canbus::Chassis;
using apollo::common::ErrorCode;
using apollo::common::Status;
using apollo::common::VehicleStateProvider;
using apollo::common::time::Clock;
using apollo::localization::LocalizationEstimate;
using apollo::planning::ADCTrajectory;

ControlComponent::ControlComponent()
    : monitor_logger_buffer_(common::monitor::MonitorMessageItem::CONTROL) {}

void ExecuteMoveAction(double &moving_distance, LocalView &local_view_t, ControlCommand &control_cmd,
            int &drive_gear_cnt, int &reverse_gear_cnt,  double &vehicle_move_length, bool &start_brake_flag, bool &excut_completed) {
    //add for test drive length, add by shzhw 20200807
    if (moving_distance > 1e-4) {
      // put int  drive gear
      if (local_view_t.chassis().gear_location() !=
             Chassis::GEAR_DRIVE) { 
        if (local_view_t.chassis().gear_location() !=
             Chassis::GEAR_NEUTRAL) {
          control_cmd.set_speed(0.0);
          control_cmd.set_throttle(0.0);
          control_cmd.set_steering_target(0.0);
          control_cmd.set_brake(50.0);

          //for (int jj=0; jj< 60;jj++) {
            control_cmd.set_gear_location(Chassis::GEAR_NEUTRAL);
          //}
          
        }
        if (local_view_t.chassis().gear_location() ==
             Chassis::GEAR_NEUTRAL) {
          control_cmd.set_speed(0.0);
          control_cmd.set_throttle(0.0);
          control_cmd.set_steering_target(0.0);
          control_cmd.set_brake(50.0);
          drive_gear_cnt += 1;
          if (drive_gear_cnt > 5) {
            control_cmd.set_gear_location(Chassis::GEAR_NEUTRAL);
            //AERROR << "SET NEUTRAL.2.. " << drive_gear_cnt << ", " << local_view_t.chassis().gear_location();
          } 
          if (drive_gear_cnt > 10) {
            control_cmd.set_gear_location(Chassis::GEAR_DRIVE);
            //AERROR << "SET DRIVE.2.. " << drive_gear_cnt << ", " << local_view_t.chassis().gear_location();
          }
        }
      } else if (local_view_t.chassis().gear_location() ==
              Chassis::GEAR_DRIVE) {
          drive_gear_cnt = 0;
          //AERROR << "GEAR DRIVE,ALREADY.";
      }
    } else  if (moving_distance < - 1e-4) {
      // put intpo reverse gear
      if (local_view_t.chassis().gear_location() !=
             Chassis::GEAR_REVERSE) { 
        if (local_view_t.chassis().gear_location() !=
             Chassis::GEAR_NEUTRAL) {
          control_cmd.set_speed(0.0);
          control_cmd.set_throttle(0.0);
          control_cmd.set_steering_target(0.0);
          control_cmd.set_brake(50.0);

          //for (int jj=0; jj< 60;jj++) {
            control_cmd.set_gear_location(Chassis::GEAR_NEUTRAL);
          //}
        }
        if (local_view_t.chassis().gear_location() ==
             Chassis::GEAR_NEUTRAL) {
          control_cmd.set_speed(0.0);
          control_cmd.set_throttle(0.0);
          control_cmd.set_steering_target(0.0);
          control_cmd.set_brake(50.0);
          reverse_gear_cnt += 1;
          if (reverse_gear_cnt > 5) {
            control_cmd.set_gear_location(Chassis::GEAR_NEUTRAL);
          } 
          if (reverse_gear_cnt > 10) {
            control_cmd.set_gear_location(Chassis::GEAR_REVERSE);
          }
        }
      } else if (local_view_t.chassis().gear_location() ==
              Chassis::GEAR_REVERSE) {
          reverse_gear_cnt = 0;
      }
    } else {
        control_cmd.set_speed(0.0);
        control_cmd.set_throttle(0.0);
        control_cmd.set_steering_target(0.0);
        control_cmd.set_brake(50.0);
        excut_completed = true;
    }
    // excute move action
    if ((moving_distance < - 1e-4 && local_view_t.chassis().gear_location() ==
             Chassis::GEAR_REVERSE) || (moving_distance > 1e-4 &&local_view_t.chassis().gear_location() ==
             Chassis::GEAR_DRIVE )) {
               //AERROR << "EXCUTE";
      const double vehicle_speed = local_view_t.chassis().speed_mps();
      double abs_vehicle_speed = std::fabs(vehicle_speed);

      static double pre_timestamp = apollo::common::time::Clock::NowInSeconds();
      double cur_timestamp = apollo::common::time::Clock::NowInSeconds();
      double time_diff = cur_timestamp - pre_timestamp;
      pre_timestamp = cur_timestamp;
    
      

      vehicle_move_length += time_diff * abs_vehicle_speed;
      //modified 20210207
      double braking_distance = 0.0;//0.8535*abs_vehicle_speed - 0.01345 +0.01*std::fabs(moving_distance); //change 20201021
       //braking_distance = 0.5175*abs_vehicle_speed - 0.01881;
       braking_distance = 0.414*abs_vehicle_speed + 0.03895;
      //only front
      //braking_distance = 3.267*abs_vehicle_speed*abs_vehicle_speed*abs_vehicle_speed -3.965*abs_vehicle_speed*abs_vehicle_speed + 1.973*abs_vehicle_speed - 0.06479;
      //all front and back
      //braking_distance =4.49*abs_vehicle_speed*abs_vehicle_speed*abs_vehicle_speed -5.448*abs_vehicle_speed*abs_vehicle_speed +2.602*abs_vehicle_speed - 0.1469;
      

      if (start_brake_flag){
        braking_distance = 999.0;
      }

      AERROR << std::fixed << "excute.. time: " << local_view_t.chassis().header().timestamp_sec() << " , vehicle dis:" << vehicle_move_length << " brake dis:" << braking_distance << " vel:" << abs_vehicle_speed << " msg dis:" << moving_distance;
      if((vehicle_move_length + braking_distance) <= std::fabs(moving_distance)) {
        /*control_cmd.set_throttle(FLAGS_drive_len_test_throttle);
        if (abs_vehicle_speed < 0.8/3.6) {
          control_cmd.set_throttle(12);
        } else if (abs_vehicle_speed > 3.0/3.6) {
          control_cmd.set_throttle(11);
        }
        */
        

        /******add 2020115 wrc start********/

        //double target_ctl_spd_mps = ARRIVING_SPEED_KMH/3.6;
	      double target_ctl_spd_mps = 3.2/3.6;

        if(std::fabs(moving_distance) - vehicle_move_length <= 5.0) {
          target_ctl_spd_mps = ARRIVING_SPEED_KMH/3.6;
        //} else if (std::fabs(moving_distance) - vehicle_move_length <= 5.0) {
        //  target_ctl_spd_mps = 3.6/3.6;
        }

        static double last_timestamp_s = apollo::common::time::Clock::NowInSeconds();
        double cur_timestamp_s = apollo::common::time::Clock::NowInSeconds();
        double time_diff_s = cur_timestamp_s - last_timestamp_s;

        // const double x_velocity_imu = local_view_t.localization().pose().linear_velocity().x();
        // const double y_velocity_imu = local_view_t.localization().pose().linear_velocity().y();

        // static double last_velocity_mps = std::sqrt(x_velocity_imu * x_velocity_imu + y_velocity_imu * y_velocity_imu);
        // const double cur_velocity_mps = std::sqrt(x_velocity_imu * x_velocity_imu + y_velocity_imu * y_velocity_imu);

        static double last_velocity_mps = local_view_t.chassis().speed_mps();
        const double cur_velocity_mps = local_view_t.chassis().speed_mps();
        auto cur_gear_position = local_view_t.chassis().gear_location();
        static double last_velocity_error = target_ctl_spd_mps - cur_velocity_mps;
        double cur_velocity_error = target_ctl_spd_mps - cur_velocity_mps;
        double diff_spd_error = cur_velocity_error - last_velocity_error;
        double target_acceleration_m2ps = 0.0;
        double cur_acceleration_m2ps = 0.0;
        static double throttle_cal_value = local_view_t.chassis().throttle_percentage();
        const double cur_throttle_value = local_view_t.chassis().throttle_percentage();
        double throttle_compensation = 0.0;
        double brake_cmd = 0.0;
        double throttle_cmd =0.0;
        static double last_throttle_cmd = 0.0;
        static double last_brake_cmd = 0.0;
        
        
        if (time_diff_s > 0.15 || 
             (std::fabs(cur_velocity_mps - last_velocity_mps) > 1e-4  && time_diff_s > 0.07) ){             
          cur_acceleration_m2ps = (cur_velocity_mps - last_velocity_mps) / time_diff_s;
          target_acceleration_m2ps = cur_velocity_error * KP_SPD_CTRL + diff_spd_error * KD_SPD_CTRL;
          throttle_compensation = (target_acceleration_m2ps - cur_acceleration_m2ps) * KP_THROTTLE_CTRL * time_diff_s;
          throttle_compensation = common::math::Clamp(throttle_compensation, 
                                                      -THROTTLE_COMPENSATION_MAX_PER_SEC * time_diff_s, 
                                                      THROTTLE_RELEASE_MAX_PER_SEC * time_diff_s);
          if(std::fabs(cur_velocity_mps) < 0.01 && cur_throttle_value > 13.0 && 
              throttle_compensation > THROTTLE_COMPENSATION_MAX_PER_SEC/2.5 * time_diff_s){
            throttle_compensation = THROTTLE_COMPENSATION_MAX_PER_SEC/2.5 * time_diff_s;
          }//起步时防止油门响应延迟，导致发的过快，然后猛突进
          throttle_cal_value += throttle_compensation;
          if((throttle_cal_value - cur_throttle_value) > THROTTLE_COMPENSATION_MAX_PER_SEC){
            //in case realtime throttle not catch up, eg. AEBS let throttle return 0.0, then recover.
            throttle_cal_value = cur_throttle_value + THROTTLE_COMPENSATION_MAX_PER_SEC*0.5;
          }
          throttle_cal_value = common::math::Clamp(throttle_cal_value,
                                                  -THROTTLE_ALLOWED_MAX,
                                                  THROTTLE_ALLOWED_MAX);
         
          if(cur_velocity_mps < 0.92*target_ctl_spd_mps /*&& target_ctl_spd_mps >= ARRIVING_SPEED_KMH/3.6*/
             && throttle_cal_value < THROTTLE_START_VAL) {
            throttle_cal_value = THROTTLE_START_VAL;
          }
          if(cur_velocity_mps > 1.4 * target_ctl_spd_mps
              && throttle_cal_value > 0.0){//此时油门应该全部松开了
            throttle_cal_value = 0.0;
          }


          if((local_view_t.chassis().brake_percentage() > 20 && throttle_cal_value >= 0.0)
                   || (cur_gear_position != canbus::Chassis::GEAR_DRIVE &&
                        cur_gear_position != canbus::Chassis::GEAR_REVERSE)){
            //when detect brake(like AEB) or stop, reset throttle value.
            throttle_cal_value = 0.0;
          }

          if(throttle_cal_value >= 0.0){
            throttle_cmd = throttle_cal_value;
            brake_cmd = 0.0;
          }else{//负油门，表示需要刹车
            throttle_cmd = 0.0;
            brake_cmd = std::fabs(throttle_cal_value) /THROTTLE_ALLOWED_MAX * 80.0;
            brake_cmd = std::min(brake_cmd, 60.0);            
          }

          last_velocity_mps = cur_velocity_mps;
          last_velocity_error = cur_velocity_error;
          last_throttle_cmd = throttle_cmd;
          last_brake_cmd = brake_cmd;
          last_timestamp_s = cur_timestamp_s;
        }else {
          throttle_cmd = last_throttle_cmd;
          brake_cmd = last_brake_cmd;
        }
        // throttle_cmd = 12.0;
        // brake_cmd = 0.0;
        control_cmd.set_throttle(throttle_cmd);
        control_cmd.set_brake(brake_cmd);
	
        /******add 20201115 wrc end********/


        //if (std::fabs(vehicle_move_length-std::fabs(moving_distance))<1.0) {
        //  //control_cmd.set_throttle(0);
        //}
        //    control_cmd.set_brake(0);
        ////control_cmd.set_steering_target(1.8);
        control_cmd.set_steering_target(local_view_t.trajectory().simple_control_cmd().steering_target());
//control_cmd.set_steering_target(1.87);      
        control_cmd.set_gear_location(local_view_t.chassis().gear_location());
        //AERROR << "Driving ing..." << ",  abs_vehicle_speed:" << abs_vehicle_speed << " ,vehicle_move_length:" << control_vehicle_move_length;
      } else {
        control_cmd.set_speed(0.0);
        control_cmd.set_throttle(0.0);
        control_cmd.set_brake(65);//65
        control_cmd.set_gear_location(local_view_t.chassis().gear_location());
	start_brake_flag = true;
       AERROR << std::fixed << "stop ing.. time: " << local_view_t.chassis().header().timestamp_sec() << " , vehicle dis:" << vehicle_move_length << " brake dis:" << braking_distance << " vel:" << abs_vehicle_speed << " msg dis:" << moving_distance;
       #if 1
      std::ofstream pad_excute_out;
  pad_excute_out.open("pad_excute_out.txt",std::ios::app);
  pad_excute_out << std::fixed << local_view_t.chassis().header().timestamp_sec() << "\t" << vehicle_move_length << "\t" << braking_distance << "\t" << abs_vehicle_speed << "\t" << moving_distance << std::endl;
  pad_excute_out.close();
  #endif 
       
       if (std::fabs(local_view_t.chassis().speed_mps()) < 1e-4) {
	  excut_completed = true;
    AERROR << std::fixed << "stop completed.. time: " << local_view_t.chassis().header().timestamp_sec() << " , vehicle dis:" << vehicle_move_length << " vel:" << abs_vehicle_speed ;
	}
        //AERROR << "Stop ing...";
      }
   
    }
}

bool ControlComponent::Init() {
  init_time_ = Clock::Now();

  AINFO << "Control init, starting ...";

  CHECK(
      cyber::common::GetProtoFromFile(FLAGS_control_conf_file, &control_conf_))
      << "Unable to load control conf file: " + FLAGS_control_conf_file;

  AINFO << "Conf file: " << FLAGS_control_conf_file << " is loaded.";

  AINFO << "Conf file: " << ConfigFilePath() << " is loaded.";

  // initial controller agent when not using control submodules
  ADEBUG << "FLAGS_use_control_submodules: " << FLAGS_use_control_submodules;
  if (!FLAGS_use_control_submodules &&
      !controller_agent_.Init(&control_conf_).ok()) {
    // set controller
    ADEBUG << "original control";
    monitor_logger_buffer_.ERROR("Control init controller failed! Stopping...");
    return false;
  }

  cyber::ReaderConfig chassis_reader_config;
  chassis_reader_config.channel_name = FLAGS_chassis_topic;
  chassis_reader_config.pending_queue_size = FLAGS_chassis_pending_queue_size;

  chassis_reader_ =
      node_->CreateReader<Chassis>(chassis_reader_config, nullptr);
  CHECK(chassis_reader_ != nullptr);

  cyber::ReaderConfig planning_reader_config;
  planning_reader_config.channel_name = FLAGS_planning_trajectory_topic;
  planning_reader_config.pending_queue_size = FLAGS_planning_pending_queue_size;

  trajectory_reader_ =
      node_->CreateReader<ADCTrajectory>(planning_reader_config, nullptr);
  CHECK(trajectory_reader_ != nullptr);

  cyber::ReaderConfig localization_reader_config;
  localization_reader_config.channel_name = FLAGS_localization_topic;
  localization_reader_config.pending_queue_size =
      FLAGS_localization_pending_queue_size;

  localization_reader_ = node_->CreateReader<LocalizationEstimate>(
      localization_reader_config, nullptr);
  CHECK(localization_reader_ != nullptr);

  cyber::ReaderConfig pad_msg_reader_config;
  pad_msg_reader_config.channel_name = FLAGS_pad_topic;
  pad_msg_reader_config.pending_queue_size = FLAGS_pad_msg_pending_queue_size;

  pad_msg_reader_ =
      node_->CreateReader<PadMessage>(pad_msg_reader_config, nullptr);
  CHECK(pad_msg_reader_ != nullptr);

  cyber::ReaderConfig abb_msg_reader_config;
  abb_msg_reader_config.channel_name = "/apollo/alignement";//TODO
  abb_msg_reader_config.pending_queue_size = FLAGS_pad_msg_pending_queue_size;

  abb_msg_reader_ =
      node_->CreateReader<PadMessage>(abb_msg_reader_config, nullptr);
  CHECK(abb_msg_reader_ != nullptr);

  cyber::ReaderConfig benewakelidar_reader_config;
  benewakelidar_reader_config.channel_name = "apollo/lidar";
  benewakelidar_reader_config.pending_queue_size = FLAGS_pad_msg_pending_queue_size;

  benewakelidar_reader_ =
      node_->CreateReader<lidar::BenewakeLidar>(benewakelidar_reader_config, nullptr);
  CHECK(benewakelidar_reader_ != nullptr);

  if (!FLAGS_use_control_submodules) {
    control_cmd_writer_ =
        node_->CreateWriter<ControlCommand>(FLAGS_control_command_topic);
    CHECK(control_cmd_writer_ != nullptr);
  } else {
    local_view_writer_ =
        node_->CreateWriter<LocalView>(FLAGS_control_local_view_topic);
    CHECK(local_view_writer_ != nullptr);
  }

  // set initial vehicle state by cmd
  // need to sleep, because advertised channel is not ready immediately
  // simple test shows a short delay of 80 ms or so
  AINFO << "Control resetting vehicle state, sleeping for 1000 ms ...";
  std::this_thread::sleep_for(std::chrono::milliseconds(1000));

  // should init_vehicle first, let car enter work status, then use status msg
  // trigger control

  AINFO << "Control default driving action is "
        << DrivingAction_Name(control_conf_.action());
  pad_msg_.set_action(control_conf_.action());

  open_highbeam_minute_ = control_conf_.open_highbeam_minute();
  close_highbeam_minute_ = control_conf_.close_highbeam_minute();
  return true;
}

void ControlComponent::OnPad(const std::shared_ptr<PadMessage> &pad) {
  std::lock_guard<std::mutex> lock(mutex_);
  pad_msg_.CopyFrom(*pad);
  ADEBUG << "Received Pad Msg:" << pad_msg_.DebugString();
  //AERROR_IF(!pad_msg_.has_action()) << "pad message check failed!";
}

void ControlComponent::OnAbb(const std::shared_ptr<PadMessage> &abb) {
  std::lock_guard<std::mutex> lock(mutex_);
  abb_msg_.CopyFrom(*abb);
  ADEBUG << "Received ABB Msg:" << abb_msg_.DebugString();
  AERROR_IF(!abb_msg_.has_action()) << "ABB message check failed!";
}

void ControlComponent::OnBenewakeLidar(const std::shared_ptr<apollo::lidar::BenewakeLidar> &benewakelidar) {
  std::lock_guard<std::mutex> lock(mutex_);
  benewakelidar_msg_.CopyFrom(*benewakelidar);
}

void ControlComponent::OnChassis(const std::shared_ptr<Chassis> &chassis) {
  ADEBUG << "Received chassis data: run chassis callback.";
  std::lock_guard<std::mutex> lock(mutex_);
  latest_chassis_.CopyFrom(*chassis);
}

void ControlComponent::OnPlanning(
    const std::shared_ptr<ADCTrajectory> &trajectory) {
  ADEBUG << "Received chassis data: run trajectory callback.";
  std::lock_guard<std::mutex> lock(mutex_);
  latest_trajectory_.CopyFrom(*trajectory);
}

void ControlComponent::OnLocalization(
    const std::shared_ptr<LocalizationEstimate> &localization) {
  ADEBUG << "Received control data: run localization message callback.";
  std::lock_guard<std::mutex> lock(mutex_);
  latest_localization_.CopyFrom(*localization);
  //AERROR <<std::fixed<< "RRRRRRRRR:" << latest_localization_.pose().position().x() << "," << latest_localization_.pose().position().y();

}

void ControlComponent::OnMonitor(
    const common::monitor::MonitorMessage &monitor_message) {
  for (const auto &item : monitor_message.item()) {
    if (item.log_level() == common::monitor::MonitorMessageItem::FATAL) {
      estop_ = true;
      return;
    }
  }
}

Status ControlComponent::ProduceControlCommand(
    ControlCommand *control_command) {
  Status status = CheckInput(&local_view_);
  // check data

  if (!status.ok()) {
    AERROR_EVERY(100) << "Control input data failed: "
                      << status.error_message();
    control_command->mutable_engage_advice()->set_advice(
        apollo::common::EngageAdvice::DISALLOW_ENGAGE);
    control_command->mutable_engage_advice()->set_reason(
        status.error_message());
    estop_ = true;
    estop_reason_ = status.error_message();
  } else {
    Status status_ts = CheckTimestamp(local_view_);
    if (!status_ts.ok()) {
      AERROR << "Input messages timeout";
      estop_ = true;
      status = status_ts;
      if (local_view_.chassis().driving_mode() !=
          apollo::canbus::Chassis::COMPLETE_AUTO_DRIVE) {
        control_command->mutable_engage_advice()->set_advice(
            apollo::common::EngageAdvice::DISALLOW_ENGAGE);
        control_command->mutable_engage_advice()->set_reason(
            status.error_message());
      }
    } else {
      control_command->mutable_engage_advice()->set_advice(
          apollo::common::EngageAdvice::READY_TO_ENGAGE);
    }
  }

  // check estop
  estop_ = control_conf_.enable_persistent_estop()
               ? estop_ || local_view_.trajectory().estop().is_estop()
               : local_view_.trajectory().estop().is_estop();

  if (local_view_.trajectory().estop().is_estop()) {
    estop_ = true;
    estop_reason_ = "estop from planning : ";
    estop_reason_ += local_view_.trajectory().estop().reason();
  }

  if (local_view_.trajectory().trajectory_point().empty()) {
    AWARN_EVERY(100) << "planning has no trajectory point. ";
    estop_ = true;
    estop_reason_ = "estop for empty planning trajectory, planning headers: " +
                    local_view_.trajectory().header().ShortDebugString();
  }

  if (FLAGS_enable_gear_drive_negative_speed_protection) {
    const double kEpsilon = 0.001;
    auto first_trajectory_point = local_view_.trajectory().trajectory_point(0);
    if (local_view_.chassis().gear_location() == Chassis::GEAR_DRIVE &&
        first_trajectory_point.v() < -1 * kEpsilon) {
      estop_ = true;
      estop_reason_ = "estop for negative speed when gear_drive";
    }
  }

  if (!estop_) {
    if (local_view_.chassis().driving_mode() == Chassis::COMPLETE_MANUAL) {
      controller_agent_.Reset();
      AINFO_EVERY(100) << "Reset Controllers in Manual Mode";
    }

    auto debug = control_command->mutable_debug()->mutable_input_debug();
    debug->mutable_localization_header()->CopyFrom(
        local_view_.localization().header());
    debug->mutable_canbus_header()->CopyFrom(local_view_.chassis().header());
    debug->mutable_trajectory_header()->CopyFrom(
        local_view_.trajectory().header());

    if (local_view_.trajectory().is_replan()) {
      latest_replan_trajectory_header_ = local_view_.trajectory().header();
    }

    if (latest_replan_trajectory_header_.has_sequence_num()) {
      debug->mutable_latest_replan_trajectory_header()->CopyFrom(
          latest_replan_trajectory_header_);
    }
    // controller agent
    Status status_compute = controller_agent_.ComputeControlCommand(
        &local_view_.localization(), &local_view_.chassis(),
        &local_view_.trajectory(), control_command);

    if (!status_compute.ok()) {
      AERROR << "Control main function failed"
             << " with localization: "
             << local_view_.localization().ShortDebugString()
             << " with chassis: " << local_view_.chassis().ShortDebugString()
             << " with trajectory: "
             << local_view_.trajectory().ShortDebugString()
             << " with cmd: " << control_command->ShortDebugString()
             << " status:" << status_compute.error_message();
      estop_ = true;
      estop_reason_ = status_compute.error_message();
      status = status_compute;
    }
  }
  // if planning set estop, then no control process triggered
  static int gear_count = 0;
 // control_command->set_gear_location(local_view_.chassis().gear_location());
  if (estop_) {
    AWARN_EVERY(100) << "Estop triggered! No control core method executed!";
    // set Estop command
    control_command->set_speed(0.0);
    control_command->set_throttle(0.0);
    control_command->set_brake(control_conf_.soft_estop_brake());
    control_command->set_gear_location(Chassis::GEAR_DRIVE);
    //AERROR << "33333";
  //} else if (local_view_.chassis().gear_location() ==
   //         Chassis::GEAR_NEUTRAL) {  // add by why
  } else if (local_view_.chassis().gear_location()!=
             Chassis::GEAR_DRIVE) {  // modified by shzhw 20200812
    control_command->set_speed(0.0);
    control_command->set_throttle(0.0);
    control_command->set_steering_target(0.0);
    control_command->set_brake(control_conf_.soft_estop_brake());
    gear_count += 1;
    if (gear_count > 5) {
      control_command->set_gear_location(Chassis::GEAR_NEUTRAL);
    } 
    if (gear_count > 10) {
      control_command->set_gear_location(Chassis::GEAR_DRIVE);
    }
  } else if (local_view_.chassis().gear_location() == Chassis::GEAR_DRIVE) {
    gear_count = 0;
  }

  if (control_command->throttle()>12.0) {
    control_command->set_throttle(12.0);
  }
  // check signal
  if (local_view_.trajectory().decision().has_vehicle_signal()) {
    control_command->mutable_signal()->CopyFrom(
        local_view_.trajectory().decision().vehicle_signal());
  }
  return status;
}

bool ControlComponent::Proc() {
  static bool pad_msg_valid = false;
  static bool excut_pad_move_action_completed = false;
  static int drive_gear_count = 0;
  static int reverse_gear_count = 0;
  static double control_vehicle_move_length = 0.0; 
  static double pad_moving_distance = 0.0; //pad moving distance
  static double excut_pad_action_completed_time = 0.0;// add 20201223

  static bool start_excut_brake_flag = false;

  static int reset_cnt = 0;

  // ABB sencond positioning
  static bool abb_msg_valid = false;
  static bool execute_abb_second_positioning_completed = true;
  static double abb_control_vehicle_move_length = 0.0; 
  static double abb_moving_distance = 0.0;

  bool is_in_yard_valid_zone = false;
  bool is_in_crane_valid_zone = false;


  const auto start_time = Clock::Now();
  estop_ = false; 
  chassis_reader_->Observe();
  const auto &chassis_msg = chassis_reader_->GetLatestObserved();
  if (chassis_msg == nullptr) {
    AERROR << "Chassis msg is not ready!";
    return false;
  }

  OnChassis(chassis_msg);

  trajectory_reader_->Observe();
  const auto &trajectory_msg = trajectory_reader_->GetLatestObserved();
  if (trajectory_msg == nullptr) {
    AERROR << "planning msg is not ready!";
    return false;
  }
  OnPlanning(trajectory_msg);

  localization_reader_->Observe();
  const auto &localization_msg = localization_reader_->GetLatestObserved();
  if (localization_msg == nullptr) {
    AERROR << "localization msg is not ready!";
    return false;
  }
  OnLocalization(localization_msg);

  pad_msg_reader_->Observe();
  const auto &pad_msg = pad_msg_reader_->GetLatestObserved();
  if (pad_msg != nullptr) {
    OnPad(pad_msg);
  }

  abb_msg_reader_->Observe();
  const auto &abb_msg = abb_msg_reader_->GetLatestObserved();
  if (abb_msg != nullptr) {
    OnAbb(abb_msg);
  }

  benewakelidar_reader_->Observe();
  const auto &benewakelidar_msg = benewakelidar_reader_->GetLatestObserved();
  if (benewakelidar_msg != nullptr) {
    OnBenewakeLidar(benewakelidar_msg);
  }

  {
    // TODO(SHU): to avoid redundent copy
    auto pad_msg_is_valid = latest_trajectory_.pad_msg_valid();
    
    std::lock_guard<std::mutex> lock(mutex_);
    local_view_.mutable_chassis()->CopyFrom(latest_chassis_);
    local_view_.mutable_trajectory()->CopyFrom(latest_trajectory_);
    local_view_.mutable_localization()->CopyFrom(latest_localization_);

    // dui chang
    if (std::fabs(latest_localization_.pose().heading() - (-1.24)) < 0.2) {
      double PositionEst_vaild_startx = local_view_.trajectory().target_position_coord().x() + 
                (-6.3) * cos(-1.24); //modified 20201223
      double PositionEst_vaild_starty = local_view_.trajectory().target_position_coord().y() + 
                (-6.3) * sin(-1.24); //modified 20201223
      double PositionEst_vaild_endx = local_view_.trajectory().target_position_coord().x() + 
                (7.0+7.0) * cos(-1.24);
      double PositionEst_vaild_endy = local_view_.trajectory().target_position_coord().y() + 
                (7.0+7.0)* sin(-1.24);

      if ((latest_localization_.pose().position().x() - PositionEst_vaild_startx >=0 && 
        latest_localization_.pose().position().y() - PositionEst_vaild_starty<=0) && 
        (latest_localization_.pose().position().x() - PositionEst_vaild_endx <=0 && 
        latest_localization_.pose().position().y() - PositionEst_vaild_endy>=0)) {
          is_in_yard_valid_zone = true;   
      }
    }

    // an qiao
    if (std::fabs(latest_localization_.pose().heading() - 1.9) < 0.2) {
      double PositionEst_vaild_startx = local_view_.trajectory().target_position_coord().x() + 
                (-14.0) * cos(1.9);
      double PositionEst_vaild_starty = local_view_.trajectory().target_position_coord().y() + 
                (-14.0) * sin(1.9);
      double PositionEst_vaild_endx = local_view_.trajectory().target_position_coord().x() + 
                14.0 * cos(1.9);
      double PositionEst_vaild_endy = local_view_.trajectory().target_position_coord().y() + 
                14.0* sin(1.9);

      if ((latest_localization_.pose().position().x() - PositionEst_vaild_startx <=0 && 
        latest_localization_.pose().position().y() - PositionEst_vaild_starty>=0) && 
        (latest_localization_.pose().position().x() - PositionEst_vaild_endx >=0 && 
        latest_localization_.pose().position().y() - PositionEst_vaild_endy<=0)) {
          //if (pad_msg_.has_moving_distance() && pad_msg_.moving_distance() > 6.5) {
            is_in_crane_valid_zone = true;
         // }
	 //AERROR << "in carne zone 9999999999999999999999";
      }
    }

    if ((pad_msg_.has_header()) && 
              (!local_view_.has_pad_msg() ||
              local_view_.pad_msg().header().timestamp_sec() != pad_msg_.header().timestamp_sec())
         && pad_msg_.header().module_name() != "perception_camera") {
      
      local_view_.mutable_pad_msg()->CopyFrom(pad_msg_);
      if (pad_msg_.header().module_name() == "PositionEst") {
        
        if (pad_msg_is_valid) {
          pad_msg_valid = true;
        }        

        if (is_in_yard_valid_zone || is_in_crane_valid_zone) {
          pad_msg_valid = true;
        }

        //if (local_view_.localization().pose().position().x() - 794820.360759 >=0 && 
        //    local_view_.localization().pose().position().y() - 2490790.908271<=0 ) {
        //      pad_msg_valid = true;
        //   } //add shzhw 1121 TODO, zone throlde
      } else {
      pad_msg_valid = true;
      }
      AERROR << "PAD VALID 55555555555:  "<< pad_msg_valid;
      if (pad_msg_.action() == DrivingAction::RESET) {
        pad_msg_valid = false;
        abb_msg_valid = false;// add, TODO
        excut_pad_action_completed_time = 0.0;
        reset_cnt++;
        
      }

      if (pad_msg_.has_moving_distance()) {
        pad_moving_distance =pad_msg_.moving_distance();
        if (pad_msg_.header().module_name() == "PositionEst") {
           pad_moving_distance -=0.0;//0.10; //TODO
          if (benewakelidar_msg_.has_distance() &&
                (benewakelidar_msg_.distance()/100.0 > 1.5 && benewakelidar_msg_.distance()/100.0 < 2.50)) {
            if (is_in_crane_valid_zone) {
       //       pad_moving_distance +=( benewakelidar_msg_.distance()/100.0 - 1.80); //TODO 
	      AERROR << "benewake lidar dist: " << benewakelidar_msg_.distance() << "," << pad_moving_distance;
	    } else {
        //      pad_moving_distance +=( benewakelidar_msg_.distance()/100.0 - 2.02); //TODO 
	    }	    
          }          
        }
      }

      if (pad_msg_.has_moving_distance() &&  
                  std::fabs(pad_moving_distance) >1e-4 ) {
        excut_pad_move_action_completed = false;
        //drive_gear_count = 0;
        //reverse_gear_count = 0;
        control_vehicle_move_length = 0.0;
        start_excut_brake_flag = false;
      } else {
        excut_pad_move_action_completed = true;
      }
    }

    //ABB msg proc
    if (abb_msg_.has_header() && 
              (!local_view_.has_abb_msg() ||
              std::fabs(local_view_.abb_msg().header().timestamp_sec() - abb_msg_.header().timestamp_sec()) > 1e-3)
    ) {
      
      local_view_.mutable_abb_msg()->CopyFrom(abb_msg_); 
     AERROR << "abb msg get--------------"; 

      if (is_in_yard_valid_zone && excut_pad_move_action_completed) {
        //abb_msg_valid = true;
	if (std::fabs(excut_pad_action_completed_time) > 1e-6) {
          if (local_view_.chassis().header().timestamp_sec() - excut_pad_action_completed_time > 1.0 &&
          local_view_.chassis().header().timestamp_sec() - excut_pad_action_completed_time < 3.0) {
            abb_msg_valid = true;
          }
        }
	//abb_msg_valid = false; //...........................
	AERROR << "abb msg get--------------" << "speed: " << local_view_.chassis().speed_mps();
      }
      if (is_in_crane_valid_zone){
	if (std::fabs(abb_msg_.moving_distance()) > 0.04 && excut_pad_move_action_completed) {
	 // abb_msg_valid = true;
	 if (std::fabs(excut_pad_action_completed_time) > 1e-6) {
          if (local_view_.chassis().header().timestamp_sec() - excut_pad_action_completed_time > 1.0 &&
          local_view_.chassis().header().timestamp_sec() - excut_pad_action_completed_time < 2.0) {
            abb_msg_valid = true;
          }
        }
	}
      }
 
      if (abb_msg_.has_moving_distance() && std::fabs(abb_msg_.moving_distance()) >= 0.05 && execute_abb_second_positioning_completed) { //20210104
        abb_moving_distance =abb_msg_.moving_distance();
	if (is_in_crane_valid_zone){
          abb_moving_distance = -abb_msg_.moving_distance();
	}
	if (abb_moving_distance < 0.0) {
	  abb_moving_distance = std::min(abb_moving_distance,-0.07);
	} else {
	   abb_moving_distance = std::max(abb_moving_distance,0.07);
	}
	
        AERROR << "abb distance:  " << abb_moving_distance;
        AERROR << "ABB VAILD: " << abb_msg_valid;	
      }

      if (execute_abb_second_positioning_completed) {
        if (abb_msg_.has_moving_distance() &&  
                    std::fabs(pad_moving_distance) >= 0.05 ) { //20210104
	  if (abb_msg_valid) {
          execute_abb_second_positioning_completed = false;
          //drive_gear_count = 0;
          //reverse_gear_count = 0;
          abb_control_vehicle_move_length = 0.0;
          start_excut_brake_flag = false;
	  }
        } else {
          execute_abb_second_positioning_completed = true;
        }
      }

    }
    //------ABB msg----------
  }

  // use control submodules
  if (FLAGS_use_control_submodules) {
    local_view_.mutable_header()->set_lidar_timestamp(
        local_view_.trajectory().header().lidar_timestamp());
    local_view_.mutable_header()->set_camera_timestamp(
        local_view_.trajectory().header().camera_timestamp());
    local_view_.mutable_header()->set_radar_timestamp(
        local_view_.trajectory().header().radar_timestamp());
    common::util::FillHeader(FLAGS_control_local_view_topic, &local_view_);

    const auto end_time = Clock::Now();

    // measure latency
    static apollo::common::LatencyRecorder latency_recorder(
        FLAGS_control_local_view_topic);
    latency_recorder.AppendLatencyRecord(
        local_view_.trajectory().header().lidar_timestamp(), start_time,
        end_time);

    local_view_writer_->Write(local_view_);
    return true;
  }

  if (pad_msg != nullptr) {
    ADEBUG << "pad_msg: " << pad_msg_.ShortDebugString();
    if (pad_msg_.action() == DrivingAction::RESET) {
      AINFO << "Control received RESET action!";
      estop_ = false;
      estop_reason_.clear();
    }
    pad_received_ = true;
  }

  if (control_conf_.is_control_test_mode() &&
      control_conf_.control_test_duration() > 0 &&
      absl::ToDoubleSeconds(start_time - init_time_) >
          control_conf_.control_test_duration()) {
    AERROR << "Control finished testing. exit";
    return false;
  }

  ControlCommand control_command;
  Status status;
  if (!pad_msg_valid) { //no pad msg , normal auto
    status = ProduceControlCommand(&control_command);
    AERROR_IF(!status.ok()) << "Failed to produce control command:"
                            << status.error_message();

    // add by shzhw, ProduceControlCommand() is used to gear
    
      // just for test
    if (1 && local_view_.chassis().gear_location() ==
          Chassis::GEAR_DRIVE) {
      AERROR << "222222222222222";
      static bool finish_flag = false;
      int test_brake = control_conf_.test_brake();
      float test_speed = control_conf_.test_speed();
      int test_throttle = control_conf_.test_throttle();
      if (local_view_.chassis().speed_mps() < test_speed/3.6) {
       control_command.set_brake(0);
        control_command.set_throttle(test_throttle);
	      //control_command.set_steering_target(local_view_.trajectory().simple_control_cmd().steering_target());
       control_command.set_steering_target(0);
      } else {
        control_command.set_brake(test_brake);
        control_command.set_throttle(0);
        control_command.set_steering_target(0);
        //control_command.set_steering_target(local_view_.trajectory().simple_control_cmd().steering_target());
        finish_flag = true;
      }
      if (finish_flag) {
        control_command.set_brake(test_brake);
        control_command.set_throttle(0);
        control_command.set_steering_target(0);
        static double d_length = 0.0;
        static double pre_timestamp_test = apollo::common::time::Clock::NowInSeconds();
        static double init_timestamp = apollo::common::time::Clock::NowInSeconds();
        double cur_timestamp_test = apollo::common::time::Clock::NowInSeconds();
        double time_diff_test = cur_timestamp_test - pre_timestamp_test;
        double brake_time = cur_timestamp_test - init_timestamp;
        pre_timestamp_test = cur_timestamp_test;
        const double  abs_vehicle_speed_test = std::fabs(local_view_.chassis().speed_mps());
        d_length += time_diff_test * abs_vehicle_speed_test;
        AERROR << std::fixed << "test: " << "timesptamp:" << cur_timestamp_test << ",diff time: " << time_diff_test  << ",speed: " << abs_vehicle_speed_test  << ",brake" << test_brake <<", length: " << d_length;
        std::fstream sfile("/apollo/data/brake_test.txt", 
        std::ios::out|std::ios::app|std::ios::binary);
        if(sfile.is_open()){
          sfile << abs_vehicle_speed_test << "\t" << test_brake <<"\t"<< cur_timestamp_test <<"\t"<< brake_time << "\t" << d_length <<std::endl;
        }
        sfile.close();
      }
    }
      //-----------------------

    else {
    if (local_view_.trajectory().is_use_routing() && local_view_.chassis().gear_location() ==
          Chassis::GEAR_DRIVE) {
      Status status_ts = CheckTimestamp(local_view_);
      if (!status_ts.ok()) {
        control_command.set_speed(0.0);
        control_command.set_throttle(0.0);
        control_command.set_brake(control_conf_.soft_estop_brake());
      } else {
        control_command.set_brake(local_view_.trajectory().simple_control_cmd().brake());
        control_command.set_throttle(local_view_.trajectory().simple_control_cmd().throttle());
        control_command.set_steering_target(local_view_.trajectory().simple_control_cmd().steering_target());
      }


      //test
      /**/
      #if 0
      #define STOP_PX 596565.668
      #define STOP_PY 2722363.543
      #define STOP_YAW (1.571)
      static bool stop_p =false;

      const double  speed_abs_temp = std::fabs(local_view_.chassis().speed_mps());
      double x_st_stopPt_coordinate = 0.0;
      double curPos_x = local_view_.localization().pose().position().x();
      double curPos_y = local_view_.localization().pose().position().y();
      double curPos_yaw = local_view_.localization().pose().heading();      
      x_st_stopPt_coordinate = curPos_x * cos(STOP_YAW) + curPos_y * sin(STOP_YAW) - (STOP_PX*cos(STOP_YAW) + STOP_PY*sin(STOP_YAW));

      if(x_st_stopPt_coordinate >= -0.8535*speed_abs_temp && x_st_stopPt_coordinate < 3.0 && fabs(curPos_yaw - STOP_YAW) < 0.2){
      //if (std::fabs(local_view_.localization().pose().position().x()-794634.900190429)+
      //      std::fabs(local_view_.localization().pose().position().y()-2490881.77754037) < 0.08) {
      // if ((local_view_.localization().pose().position().x()-STOP_PX)>=0.0 &&
      //       (local_view_.localization().pose().position().y()-STOP_PY) <=0.0) {
        stop_p = true;
        AERROR << "stop flag: " << stop_p;
      }else if(x_st_stopPt_coordinate >3.0){
        stop_p = false;
        reset_cnt = 0;
      }
      if (stop_p && reset_cnt == 0) {
        control_command.set_speed(0.0);
        control_command.set_throttle(0.0);
        control_command.set_brake(40/*70*/);
      }

      #endif     

    } else if (estop_) {
      AWARN_EVERY(100) << "Estop triggered! No control core method executed!";
      // set Estop command
      control_command.set_speed(0.0);
      control_command.set_throttle(0.0);
      control_command.set_brake(control_conf_.soft_estop_brake());
      control_command.set_gear_location(Chassis::GEAR_DRIVE);
    }

    }//---------------

  } else if (pad_msg_valid && !excut_pad_move_action_completed)  {
	  AERROR << "pad move distance0000000000000: " << pad_moving_distance;
    ExecuteMoveAction(pad_moving_distance, local_view_, control_command,
           drive_gear_count, reverse_gear_count,  control_vehicle_move_length, start_excut_brake_flag, excut_pad_move_action_completed);
    //TODO reset pad_msg_valid = false
    //----------------------------------------
  }
  // excute pad move action completed, stop, gear park
  if (excut_pad_move_action_completed && pad_msg_valid) {
    if (std::fabs(excut_pad_action_completed_time) < 1e-6) {
      excut_pad_action_completed_time = local_view_.chassis().header().timestamp_sec();
    }
    //an qiao
    if (0 && is_in_crane_valid_zone) {
      control_command.set_speed(0.0);
      control_command.set_throttle(0.0);
      control_command.set_brake(65);//65
      control_command.set_gear_location(local_view_.chassis().gear_location());
      if (local_view_.chassis().speed_mps() <= 0.01) {
        control_command.set_gear_location(Chassis::GEAR_PARKING);
      }
    }

    //chang qiao
    if (is_in_crane_valid_zone || is_in_yard_valid_zone) {
      AERROR << "pad completed, abb msg vaild: " << abb_msg_valid;
      /*
      if (!abb_msg_valid) { 
        if (local_view_.chassis().header().timestamp_sec() - excut_pad_action_completed_time > 10.0) { //no abb msg, default alignement ok
          control_command.set_speed(0.0);
          control_command.set_throttle(0.0);
          control_command.set_brake(65);//65
          control_command.set_gear_location(local_view_.chassis().gear_location());
          if (local_view_.chassis().speed_mps() <= 0.01) {
            control_command.set_gear_location(Chassis::GEAR_PARKING);
          }
        } else {
          //do nothing
	        control_command.set_speed(0.0);
          control_command.set_throttle(0.0);
          control_command.set_brake(65);//65
	        control_command.set_gear_location(Chassis::GEAR_NEUTRAL);
        }
      } else { // has abb msg
        if (!execute_abb_second_positioning_completed) {
          if (std::fabs(abb_moving_distance) > 0.1) {
          ExecuteMoveAction(abb_moving_distance, local_view_, control_command,
              drive_gear_count, reverse_gear_count,  abb_control_vehicle_move_length, execute_abb_second_positioning_completed);
          } else {
            execute_abb_second_positioning_completed = true;
          }
        } else if (execute_abb_second_positioning_completed) {
          control_command.set_speed(0.0);
          control_command.set_throttle(0.0);
          control_command.set_brake(65);//65
          control_command.set_gear_location(local_view_.chassis().gear_location());
          if (local_view_.chassis().speed_mps() <= 0.01) {
            control_command.set_gear_location(Chassis::GEAR_PARKING);
          }
        }

      }
      */
      
      if (abb_msg_valid && !execute_abb_second_positioning_completed) {
        if (std::fabs(abb_moving_distance) > 0.05) {
        ExecuteMoveAction(abb_moving_distance, local_view_, control_command,
            drive_gear_count, reverse_gear_count,  abb_control_vehicle_move_length, start_excut_brake_flag, execute_abb_second_positioning_completed);
        } else {
          execute_abb_second_positioning_completed = true;
        }
      } /*else if (abb_msg_valid && execute_abb_second_positioning_completed) {
        control_command.set_speed(0.0);
        control_command.set_throttle(0.0);
        control_command.set_brake(65);//65
        control_command.set_gear_location(local_view_.chassis().gear_location());
        if (local_view_.chassis().speed_mps() <= 0.01) {
          control_command.set_gear_location(Chassis::GEAR_PARKING);
        }
      } else {//no abb msg, default alignement ok
	      AERROR << "CHANGQIAO, abb msg valid:------------" << abb_msg_valid << ", " << abb_moving_distance;
	      control_command.set_speed(0.0);
        control_command.set_throttle(0.0);
        control_command.set_brake(65);//64
        control_command.set_gear_location(local_view_.chassis().gear_location());
	      if (local_view_.chassis().speed_mps() <= 0.01) {
          control_command.set_gear_location(Chassis::GEAR_PARKING);
        }
      }
      */
      /**/
      if (execute_abb_second_positioning_completed) {
        if (local_view_.chassis().header().timestamp_sec() - excut_pad_action_completed_time > 2.0) { //no abb msg, default alignement ok
          control_command.set_speed(0.0);
          control_command.set_throttle(0.0);
          control_command.set_brake(65);//65
          control_command.set_gear_location(local_view_.chassis().gear_location());
          if (local_view_.chassis().speed_mps() <= 0.01) {
            control_command.set_gear_location(Chassis::GEAR_PARKING);
          }
        } else {
          //do nothing
	      control_command.set_speed(0.0);
          control_command.set_throttle(0.0);
          control_command.set_brake(65);//65
	      control_command.set_gear_location(Chassis::GEAR_NEUTRAL);  
       }

      }
      
      
      
    }
  }

  if (pad_received_) {
    control_command.mutable_pad_msg()->CopyFrom(pad_msg_);
    pad_received_ = false;
  }

  // forward estop reason among following control frames.
  if (estop_) {
    control_command.mutable_header()->mutable_status()->set_msg(estop_reason_);
  }

  // set header
  control_command.mutable_header()->set_lidar_timestamp(
      local_view_.trajectory().header().lidar_timestamp());
  control_command.mutable_header()->set_camera_timestamp(
      local_view_.trajectory().header().camera_timestamp());
  control_command.mutable_header()->set_radar_timestamp(
      local_view_.trajectory().header().radar_timestamp());

  common::util::FillHeader(node_->Name(), &control_command);

  ADEBUG << control_command.ShortDebugString();
  if (control_conf_.is_control_test_mode()) {
    ADEBUG << "Skip publish control command in test mode";
    return true;
  }

  const auto end_time = Clock::Now();

  const double time_diff_ms = absl::ToDoubleMilliseconds(end_time - start_time);
  control_command.mutable_latency_stats()->set_total_time_ms(time_diff_ms);
  control_command.mutable_latency_stats()->set_total_time_exceeded(
      time_diff_ms > control_conf_.control_period());
  ADEBUG << "control cycle time is: " << time_diff_ms << " ms.";
  status.Save(control_command.mutable_header()->mutable_status());

  // measure latency
  if (local_view_.trajectory().header().has_lidar_timestamp()) {
    static apollo::common::LatencyRecorder latency_recorder(
        FLAGS_control_command_topic);
    latency_recorder.AppendLatencyRecord(
        local_view_.trajectory().header().lidar_timestamp(), start_time,
        end_time);
  }

  struct timespec time;
  struct tm nowTime;
  clock_gettime(CLOCK_REALTIME, &time); //获取相对于1970到现在的秒数
  localtime_r(&time.tv_sec, &nowTime);

  int nowMinute = nowTime.tm_hour * 60 + nowTime.tm_min;	    
  if((nowMinute >= open_highbeam_minute_) || (nowMinute < close_highbeam_minute_)) {
	  control_command.mutable_signal()->set_high_beam(true);
	  control_command.mutable_signal()->set_low_beam(true);
  }
  if((nowMinute >= close_highbeam_minute_) && (nowMinute < open_highbeam_minute_)) {
	  control_command.mutable_signal()->set_high_beam(false);
	  control_command.mutable_signal()->set_low_beam(false);
  }

  //detect N, brake
  if(local_view_.chassis().gear_location() ==
          Chassis::GEAR_NEUTRAL){
    control_command.set_brake(60.0);
  }

  control_cmd_writer_->Write(control_command);
  return true;
}

Status ControlComponent::CheckInput(LocalView *local_view) {
  ADEBUG << "Received localization:"
         << local_view->localization().ShortDebugString();
  ADEBUG << "Received chassis:" << local_view->chassis().ShortDebugString();

  // add by shzhw
  /**/
  if (local_view->trajectory().is_use_routing()) {
    return Status::OK();
  }  
  //-------------------------------------------------------------------------

  if ((!local_view->trajectory().estop().is_estop() &&
      local_view->trajectory().trajectory_point().empty()) || 
      local_view->trajectory().total_path_length()< 0.1) {
    AWARN_EVERY(100) << "planning has no trajectory point. ";
    const std::string msg =
        absl::StrCat("planning has no trajectory point. planning_seq_num:",
                     local_view->trajectory().header().sequence_num());
    return Status(ErrorCode::CONTROL_COMPUTE_ERROR, msg);
  }

  for (auto &trajectory_point :
       *local_view->mutable_trajectory()->mutable_trajectory_point()) {
    if (std::abs(trajectory_point.v()) <
            control_conf_.minimum_speed_resolution() &&
        std::abs(trajectory_point.a()) <
            control_conf_.max_acceleration_when_stopped()) {
      trajectory_point.set_v(0.0);
      trajectory_point.set_a(0.0);
    }
  }

  VehicleStateProvider::Instance()->Update(local_view->localization(),
                                           local_view->chassis());

  return Status::OK();
}

Status ControlComponent::CheckTimestamp(const LocalView &local_view) {
  if (!control_conf_.enable_input_timestamp_check() ||
      control_conf_.is_control_test_mode()) {
    ADEBUG << "Skip input timestamp check by gflags.";
    return Status::OK();
  }
  double current_timestamp = Clock::NowInSeconds();
  double localization_diff =
      current_timestamp - local_view.localization().header().timestamp_sec();
  if (localization_diff > (control_conf_.max_localization_miss_num() *
                           control_conf_.localization_period())) {
    AERROR << "Localization msg lost for " << std::setprecision(6)
           << localization_diff << "s";
    monitor_logger_buffer_.ERROR("Localization msg lost");
    return Status(ErrorCode::CONTROL_COMPUTE_ERROR, "Localization msg timeout");
  }

  double chassis_diff =
      current_timestamp - local_view.chassis().header().timestamp_sec();
  if (chassis_diff >
      (control_conf_.max_chassis_miss_num() * control_conf_.chassis_period())) {
    AERROR << "Chassis msg lost for " << std::setprecision(6) << chassis_diff
           << "s";
    monitor_logger_buffer_.ERROR("Chassis msg lost");
    return Status(ErrorCode::CONTROL_COMPUTE_ERROR, "Chassis msg timeout");
  }

  double trajectory_diff =
      current_timestamp - local_view.trajectory().header().timestamp_sec();
  if (trajectory_diff > (control_conf_.max_planning_miss_num() *
                         control_conf_.trajectory_period())) {
    AERROR << "Trajectory msg lost for " << std::setprecision(6)
           << trajectory_diff << "s";
    monitor_logger_buffer_.ERROR("Trajectory msg lost");
    return Status(ErrorCode::CONTROL_COMPUTE_ERROR, "Trajectory msg timeout");
  }
  return Status::OK();
}

}  // namespace control
}  // namespace apollo
