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
#include "modules/guardian/guardian_component.h"

#include "cyber/common/log.h"
#include "modules/common/adapters/adapter_gflags.h"
#include "modules/common/util/message_util.h"

namespace apollo {
namespace guardian {

using apollo::canbus::Chassis;
using apollo::control::ControlCommand;
using apollo::monitor::SystemStatus;
using apollo::manuctrl::manuctrlMsg;
using apollo::tmc::tmcMsg;
using apollo::urgency::AEBCommand;

bool GuardianComponent::Init() {
  if (!GetProtoConfig(&guardian_conf_)) {
    AERROR << "Unable to load canbus conf file: " << ConfigFilePath();
    return false;
  }

  vcu_fault_level_ = Chassis::NO_FAILURE;
  vcu_fault_level2_timeout_flag_ = 0;
  vcu_fault_level2_timeout_time_pre_ = 0.0;

  chassis_reader_ = node_->CreateReader<Chassis>(
      FLAGS_chassis_topic, [this](const std::shared_ptr<Chassis>& chassis) {
        ADEBUG << "Received chassis data: run chassis callback.";
        std::lock_guard<std::mutex> lock(mutex_);
        chassis_.CopyFrom(*chassis);
      });

  control_cmd_reader_ = node_->CreateReader<ControlCommand>(
      FLAGS_control_command_topic,
      [this](const std::shared_ptr<ControlCommand>& cmd) {
        ADEBUG << "Received control data: run control callback.";
        std::lock_guard<std::mutex> lock(mutex_);
        control_cmd_.CopyFrom(*cmd);
      });

  system_status_reader_ = node_->CreateReader<SystemStatus>(
      FLAGS_system_status_topic,
      [this](const std::shared_ptr<SystemStatus>& status) {
        ADEBUG << "Received system status data: run system status callback.";
        std::lock_guard<std::mutex> lock(mutex_);
        system_status_.CopyFrom(*status);
      });

  aeb_cmd_reader_ = node_->CreateReader<AEBCommand>(
      FLAGS_aeb_command_topic,
      [this](const std::shared_ptr<AEBCommand>& cmd) {
        ADEBUG << "Received aeb data: run aeb callback.";
        std::lock_guard<std::mutex> lock(mutex_);
        aeb_cmd_.CopyFrom(*cmd);
      });
  
  serial_cmd_reader_ = node_->CreateReader<manuctrlMsg>(
      FLAGS_manuctrl_topic,
      [this](const std::shared_ptr<manuctrlMsg>& cmd) {
        ADEBUG << "Received manuctrl data: run manu control callback.";
        std::lock_guard<std::mutex> lock(mutex_);
        manu_ctrl_cmd_.CopyFrom(*cmd);
      });

  remote_cmd_reader_ = node_->CreateReader<tmcMsg>(
      FLAGS_tmc_topic,
      [this](const std::shared_ptr<tmcMsg>& cmd) {
        ADEBUG << "Received manuctrl data: run remote control callback.";
        std::lock_guard<std::mutex> lock(mutex_);
        remote_control_cmd_.CopyFrom(*cmd);
      });

  guardian_cmd_.clear_control_command();
  guardian_cmd_.mutable_control_command()->set_gear_location(Chassis::GEAR_PARKING);
  guardian_writer_ = node_->CreateWriter<GuardianCommand>(FLAGS_guardian_topic);
  
  miss_monitor_time_ = guardian_conf_.miss_monitor_time();
     
  return true;
}

bool GuardianComponent::Proc() {
  ADEBUG << "Timer is triggered: publish GuardianComponent result";
  // bool safety_mode_triggered = false;
  // if (guardian_conf_.guardian_enable()) {
  //   std::lock_guard<std::mutex> lock(mutex_);
  //   safety_mode_triggered = system_status_.has_safety_mode_trigger_time();
  // }

  // if (safety_mode_triggered) {
  //   ADEBUG << "Safety mode triggered, enable safety mode";
  //   TriggerSafetyMode();
  // } else {
  //   ADEBUG << "Safety mode not triggered, bypass control command";
  //   PassThroughControlCommand();
  // }

  PassThroughControlCommand();
  
  if((int)(chassis_.vcu_status())>=2) {
    guardian_cmd_.mutable_control_command()->mutable_signal()->set_warning_lamp(true);
  } else {
    guardian_cmd_.mutable_control_command()->mutable_signal()->set_warning_lamp(false);
  }

  /* Set top warning lamp: from high priority to low
   *  1. red:       level3 fault, 
   *                restart this module to recover
   *  2. red flash: level2 fault, 
   *                restart this module or press RMS_Fault_Reset button to recover
   *  3. green:     auto driving mode
   *  4. green flash: manuctrl & remotectrl mode
   *  5. close:     other conditions
   */
  /* Get vcu fault level */
  Chassis::FaultLevel vcu_fault_level_tmp = chassis_.vcu_status();
  if (static_cast<uint32_t>(vcu_fault_level_tmp) > static_cast<uint32_t>(vcu_fault_level_)) {
    vcu_fault_level_ = vcu_fault_level_tmp;
    AERROR << "Get vcu_fault_level = " << vcu_fault_level_tmp;
  }

  /* Reset level2 fault, when timeout */
  if ((vcu_fault_level_ == Chassis::LEVEL2_FAULT) &&
      (static_cast<uint32_t>(vcu_fault_level_tmp) < static_cast<uint32_t>(Chassis::LEVEL2_FAULT))) {
    if (vcu_fault_level2_timeout_flag_ == 0) {
      vcu_fault_level2_timeout_flag_ = 1;
      vcu_fault_level2_timeout_time_pre_ = apollo::common::time::Clock::NowInSeconds();
    }
    if (apollo::common::time::Clock::NowInSeconds() - vcu_fault_level2_timeout_time_pre_ > 30.0) {
      vcu_fault_level2_timeout_flag_ = 0;
      vcu_fault_level_ = vcu_fault_level_tmp;
    }
  } else {
    vcu_fault_level2_timeout_flag_ = 0;
  }
  /* Set top warning lamp */
  if (vcu_fault_level_ == Chassis::LEVEL3_FAULT) {
    // red
    guardian_cmd_.mutable_control_command()->mutable_signal()->set_top_warn_lamp(common::VehicleSignal::TOPWARNLAMPSTAT_RED);
  } else if (vcu_fault_level_ == Chassis::LEVEL2_FAULT) {
    // red flash
    guardian_cmd_.mutable_control_command()->mutable_signal()->set_top_warn_lamp(common::VehicleSignal::TOPWARNLAMPSTAT_FLASH_RED);
  } else {
    if (guardian_cmd_.has_command_from()) {
      switch (guardian_cmd_.command_from()) {
        case MANU_CTRL:
        case REMOTE_CTRL:
          // green flash
          guardian_cmd_.mutable_control_command()->mutable_signal()->set_top_warn_lamp(common::VehicleSignal::TOPWARNLAMPSTAT_FLASH_GREEN);
          break;
        case AUTO_DRIVING:
          // green
          guardian_cmd_.mutable_control_command()->mutable_signal()->set_top_warn_lamp(common::VehicleSignal::TOPWARNLAMPSTAT_GREEN);
          break;
        default:
          // close
          guardian_cmd_.mutable_control_command()->mutable_signal()->set_top_warn_lamp(common::VehicleSignal::TOPWARNLAMPSTAT_CLOSE);
          break;
      }
    } else {
      // close
      guardian_cmd_.mutable_control_command()->mutable_signal()->set_top_warn_lamp(common::VehicleSignal::TOPWARNLAMPSTAT_CLOSE);
    }
  }


  common::util::FillHeader(node_->Name(), &guardian_cmd_);
  guardian_writer_->Write(guardian_cmd_);
  return true;
}

void GuardianComponent::PassThroughControlCommand() {
  std::lock_guard<std::mutex> lock(mutex_);
  double time_current = apollo::common::time::Clock::NowInSeconds();
  //guardian_cmd_.clear_control_command();
  // system error situation
  guardian_cmd_.mutable_control_command()->clear_error_msg();
  guardian_cmd_.clear_command_from();
  //reset
  guardian_cmd_.mutable_control_command()->set_throttle(0.0);
  guardian_cmd_.mutable_control_command()->set_brake(
          guardian_conf_.guardian_cmd_soft_stop_percentage());
  guardian_cmd_.mutable_control_command()->set_adstatusreq(1);

  if (system_status_.has_header()) {
    double timediff_monitor =
        time_current - system_status_.header().timestamp_sec();
    if (fabs(timediff_monitor) < miss_monitor_time_) {
      guardian_cmd_.mutable_control_command()->set_error_code(
          system_status_.summary_status().error_code());
      guardian_cmd_.mutable_control_command()->set_error_msg(
          system_status_.summary_status().message());

      if (system_status_.summary_status().status() ==
          monitor::ComponentStatus_Status_WARN) {
        guardian_cmd_.mutable_control_command()->set_error_level(1);
      } else if (system_status_.summary_status().status() ==
                 monitor::ComponentStatus_Status_ERROR) {
        guardian_cmd_.mutable_control_command()->set_error_level(2);
        guardian_cmd_.set_command_from(MONITOR);
        // guardian_cmd_.mutable_control_command()->set_throttle(0.0);
        // guardian_cmd_.mutable_control_command()->set_brake(guardian_conf_.guardian_cmd_emergency_stop_percentage());
        return;
      } else if (system_status_.summary_status().status() ==
                 monitor::ComponentStatus_Status_FATAL) {
        // if(system_status_.summary_status().message() == "lost heart beat
        // accpet error Resource temporarily unavailable errno=11") {
        if (system_status_.summary_status().message() ==
            "lost heart beat recieve data fail!") {
            // guardian_cmd_.mutable_control_command()->set_error_level(3);
            // //delete by shzhw 20200609
            // guardian_cmd_.mutable_control_command()->set_throttle(0.0);
            // guardian_cmd_.mutable_control_command()->set_brake(guardian_conf_.guardian_cmd_emergency_stop_percentage());
            // return; // delete by shzhw 20200609
        } else {
          guardian_cmd_.set_command_from(MONITOR);
          guardian_cmd_.mutable_control_command()->set_error_level(3);
          // guardian_cmd_.mutable_control_command()->set_throttle(0.0);
          // guardian_cmd_.mutable_control_command()->set_brake(guardian_conf_.guardian_cmd_emergency_stop_percentage());
          return;
        }
      } else {
        guardian_cmd_.mutable_control_command()->set_error_level(0);
      }
    } else {
      guardian_cmd_.set_command_from(MISS_MESSAGE);
      guardian_cmd_.mutable_control_command()->set_throttle(0.0);
      guardian_cmd_.mutable_control_command()->set_brake(
          guardian_conf_.guardian_cmd_soft_stop_percentage());
      guardian_cmd_.mutable_control_command()->set_error_msg(
          "miss monitor message a long time since last time");
      return;
    }
  }

  // auto drive case
  double timediff_control =
      time_current - control_cmd_.header().timestamp_sec();
  if (control_cmd_.has_header()) {
    if (fabs(timediff_control) < guardian_conf_.miss_auto_message_time()) {
  //    guardian_cmd_.clear_control_command();
      guardian_cmd_.set_command_from(AUTO_DRIVING);
      guardian_cmd_.mutable_control_command()->CopyFrom(control_cmd_);
      guardian_cmd_.mutable_control_command()->set_max_speed_limit(5.0);
    } else {
  //    guardian_cmd_.clear_control_command();
      guardian_cmd_.set_command_from(MISS_MESSAGE);
      guardian_cmd_.mutable_control_command()->set_throttle(0.0);
      guardian_cmd_.mutable_control_command()->set_brake(
          guardian_conf_.guardian_cmd_soft_stop_percentage());
      guardian_cmd_.mutable_control_command()->set_error_msg(
          "miss control message a long time since last time");
    }
  }

  // // //  // remote control case
  double timediff_remote =
      time_current - remote_control_cmd_.header().timestamp_sec();
  if (remote_control_cmd_.has_header()) {
    if (fabs(timediff_remote) < guardian_conf_.miss_remote_message_time()) {
      if (remote_control_cmd_.flag()) {
        if (control_cmd_.has_header() &&
            fabs(time_current - control_cmd_.header().timestamp_sec()) < 0.1f) {
          const std::string& first_dag =
              "/apollo/modules/control/dag/control.dag";
          auto kill_cmd = absl::StrCat("pkill -f \"", first_dag, "\"");
          const int ret = std::system(kill_cmd.data());
          if (ret == 0) {
            AERROR << "Success!";
          }
        }
    //    guardian_cmd_.clear_control_command();
        guardian_cmd_.set_command_from(REMOTE_CTRL);
        guardian_cmd_.mutable_control_command()->CopyFrom(
            remote_control_cmd_.control_command());
	      guardian_cmd_.mutable_control_command()->set_max_speed_limit(10.0);
      }
    } else {
     // guardian_cmd_.clear_control_command();
      guardian_cmd_.set_command_from(MISS_MESSAGE);
      guardian_cmd_.mutable_control_command()->set_throttle(0.0);
      guardian_cmd_.mutable_control_command()->set_brake(
          guardian_conf_.guardian_cmd_soft_stop_percentage());
      guardian_cmd_.mutable_control_command()->set_error_msg(
          "miss remote message a long time since last time");
    }
/*     if(!remote_control_cmd_.flag_net_status()) {
      guardian_cmd_.set_command_from(REMOTE_CTRL);
      guardian_cmd_.mutable_control_command()->CopyFrom(
            remote_control_cmd_.control_command());
      guardian_cmd_.mutable_control_command()->set_error_msg(
          "5G network interruption");
      return;
    } */
  }

  // joystick control
  double timediff_serial =
      time_current - manu_ctrl_cmd_.header().timestamp_sec();
  if (manu_ctrl_cmd_.has_header()) {
    if (fabs(timediff_serial) < guardian_conf_.miss_manu_message_time()) {
      if (manu_ctrl_cmd_.flag()) {
        if (control_cmd_.has_header() &&
            fabs(time_current - control_cmd_.header().timestamp_sec()) < 0.1f) {
          const std::string& first_dag =
              "/apollo/modules/control/dag/control.dag";
          auto kill_cmd = absl::StrCat("pkill -f \"", first_dag, "\"");
          const int ret = std::system(kill_cmd.data());
          if (ret == 0) {
            AERROR << "Success!";
          }
        }

      //  guardian_cmd_.clear_control_command();
        guardian_cmd_.set_command_from(MANU_CTRL);
        guardian_cmd_.mutable_control_command()->CopyFrom(
            manu_ctrl_cmd_.control_command());
	      guardian_cmd_.mutable_control_command()->set_max_speed_limit(5.0);
      }
    } else {
     // guardian_cmd_.clear_control_command();
      guardian_cmd_.set_command_from(MISS_MESSAGE);
      guardian_cmd_.mutable_control_command()->set_throttle(0.0);
      guardian_cmd_.mutable_control_command()->set_brake(
          guardian_conf_.guardian_cmd_soft_stop_percentage());
      guardian_cmd_.mutable_control_command()->set_error_msg(
          "miss manu message a long time since last time");
    }
  }

  // aeb flag true
   double timediff_aeb = time_current - aeb_cmd_.header().timestamp_sec();
   if (aeb_cmd_.has_header()) {
     if (fabs(timediff_aeb) <  guardian_conf_.miss_aeb_message_time()) {
       if (aeb_cmd_.control_command().brake() > 0 &&
           remote_control_cmd_.aebsflag()) {
         guardian_cmd_.set_command_from(AEBS);
         guardian_cmd_.mutable_control_command()->set_throttle(0.0);
         guardian_cmd_.mutable_control_command()->set_brake(
              aeb_cmd_.control_command().brake());
         if((aeb_cmd_.steer_flag())
           &&(remote_control_cmd_.flag())) {
           AERROR << "Steer_Flag : " << aeb_cmd_.steer_flag();
           AERROR << "guardian fence flag true";
           guardian_cmd_.mutable_control_command()->set_steering_target(
             aeb_cmd_.control_command().steering_target());
         }
       }
     } else {
     //  guardian_cmd_.clear_control_command();
/*        guardian_cmd_.set_command_from(MISS_MESSAGE);
       guardian_cmd_.mutable_control_command()->set_throttle(0.0);
       guardian_cmd_.mutable_control_command()->set_brake(
           guardian_conf_.guardian_cmd_soft_stop_percentage());
       guardian_cmd_.mutable_control_command()->set_error_msg(
           "miss AEB message a long time since last time"); */
     }
   }else{
      // guardian_cmd_.set_command_from(AEBS);
      // guardian_cmd_.mutable_control_command()->set_throttle(0.0);
      // guardian_cmd_.mutable_control_command()->set_brake(
      //      guardian_conf_.guardian_cmd_emergency_stop_percentage());
      // guardian_cmd_.mutable_control_command()->set_error_msg(
      //     "AEB function is closed,please open AEB!");
   }
}

void GuardianComponent::TriggerSafetyMode() {
  AINFO << "Safety state triggered, with system safety mode trigger time : "
        << system_status_.safety_mode_trigger_time();
  std::lock_guard<std::mutex> lock(mutex_);
  bool sensor_malfunction = false, obstacle_detected = false;
  if (!chassis_.surround().sonar_enabled() ||
      chassis_.surround().sonar_fault()) {
    AINFO << "Ultrasonic sensor not enabled for faulted, will do emergency "
             "stop!";
    sensor_malfunction = true;
  } else {
    // TODO(QiL) : Load for config
    for (int i = 0; i < chassis_.surround().sonar_range_size(); ++i) {
      if ((chassis_.surround().sonar_range(i) > 0.0 &&
           chassis_.surround().sonar_range(i) < 2.5) ||
          chassis_.surround().sonar_range(i) > 30) {
        AINFO << "Object detected or ultrasonic sensor fault output, will do "
                 "emergency stop!";
        obstacle_detected = true;
      }
    }
  }

  guardian_cmd_.mutable_control_command()->set_throttle(0.0);
  guardian_cmd_.mutable_control_command()->set_steering_target(0.0);
  guardian_cmd_.mutable_control_command()->set_steering_rate(25.0);
  guardian_cmd_.mutable_control_command()->set_is_in_safe_mode(true);

  // TODO(QiL) : Remove this one once hardware re-alignment is done.
  sensor_malfunction = false;
  obstacle_detected = false;
  AINFO << "Temporarily ignore the ultrasonic sensor output during hardware "
           "re-alignment!";

  if (system_status_.require_emergency_stop() || sensor_malfunction ||
      obstacle_detected) {
    AINFO << "Emergency stop triggered! with system status from monitor as : "
          << system_status_.require_emergency_stop();
    guardian_cmd_.mutable_control_command()->set_brake(
        guardian_conf_.guardian_cmd_emergency_stop_percentage());
  } else {
    AINFO << "Soft stop triggered! with system status from monitor as : "
          << system_status_.require_emergency_stop();
    guardian_cmd_.mutable_control_command()->set_brake(
        guardian_conf_.guardian_cmd_soft_stop_percentage());
  }
}

}  // namespace guardian
}  // namespace apollo
