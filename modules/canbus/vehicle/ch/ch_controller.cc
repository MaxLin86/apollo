/******************************************************************************
 * Copyright 2019 The Apollo Authors. All Rights Reserved.
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

#include "modules/canbus/vehicle/ch/ch_controller.h"

#include "modules/common/proto/vehicle_signal.pb.h"

#include "cyber/common/log.h"
#include "modules/canbus/vehicle/ch/ch_message_manager.h"
#include "modules/canbus/vehicle/vehicle_controller.h"
#include "modules/common/time/time.h"
#include "modules/drivers/canbus/can_comm/can_sender.h"
#include "modules/drivers/canbus/can_comm/protocol_data.h"


namespace apollo {
namespace canbus {
namespace ch {

using ::apollo::drivers::canbus::ProtocolData;
using ::apollo::common::ErrorCode;
using ::apollo::control::ControlCommand;

namespace {

const int32_t kMaxFailAttempt = 10;
const int32_t CHECK_RESPONSE_STEER_UNIT_FLAG = 1;
const int32_t CHECK_RESPONSE_SPEED_UNIT_FLAG = 2;
}

ErrorCode ChController::Init(
	const VehicleParameter& params,
	CanSender<::apollo::canbus::ChassisDetail> *const can_sender,
    MessageManager<::apollo::canbus::ChassisDetail> *const message_manager) {
  if (is_initialized_) {
    AINFO << "ChController has already been initiated.";
    return ErrorCode::CANBUS_ERROR;
  }
  
  // common set to vcu
  throttle_percentage = 0.0;
  brake_percentage = 0.0;

  // common return from vcu
  power_status_vcu_return_ = Vcu_vehicleinfo3_98f003d0::VCU_VEHICLEPOWERSTATUS_LV_UP;
  gear_status_vcu_return_ = Vcu_vehicleinfo3_98f003d0::VCU_GEARSTATUS_NEUTRAL;
  hand_brake_status_vcu_return_ = 0;
  steer_angle_asc_return_ = 0.0;

#ifdef ENABLE_STEER_ADJUST
  steer_angle_curr_ = 0.0;
  steer_angle_target_pre_ = 0.0;
  is_first_steer = false;
#endif

#ifdef ENABLE_STEER_FAULT_DETECT_AND_RECOVER
  steer_error_detect_start_time_ = 0.0;
  steer_error_detect_start_flag_ = 0;
  steer_fault_level_ = Chassis::NO_FAILURE;
#endif

#ifdef ENABLE_ADU_BRAKECTRL_BACKUP
  brake_detect_start_time_ = 0.0;
  brake_detect_start_flag_ = 0;
  adu_brake1_air_pressure_ = 0;
  adu_brake2_air_pressure_ = 0;
#endif

  vehicle_params_.CopyFrom(
    common::VehicleConfigHelper::Instance()->GetConfig().vehicle_param());
  params_.CopyFrom(params);
  if (!params_.has_driving_mode()) {
    AERROR << "Vehicle conf pb not set driving_mode.";
    return ErrorCode::CANBUS_ERROR;
  }

  if (can_sender == nullptr) {
    return ErrorCode::CANBUS_ERROR;
  }
  can_sender_ = can_sender;

  if (message_manager == nullptr) {
    AERROR << "protocol manager is null.";
    return ErrorCode::CANBUS_ERROR;
  }
  message_manager_ = message_manager;

  // sender part
  adu_bodyctrl_8cff1600_ = dynamic_cast<Adubodyctrl8cff1600*>
          (message_manager_->GetMutableProtocolDataById(Adubodyctrl8cff1600::ID));
  if (adu_bodyctrl_8cff1600_ == nullptr) {
     AERROR << "Adubodyctrl8cff1600 does not exist in the ChMessageManager!";
     return ErrorCode::CANBUS_ERROR;
  }

  adu_drivectrl_8cff1400_ = dynamic_cast<Adudrivectrl8cff1400*>
          (message_manager_->GetMutableProtocolDataById(Adudrivectrl8cff1400::ID));
  if (adu_drivectrl_8cff1400_ == nullptr) {
     AERROR << "Adudrivectrl8cff1400 does not exist in the ChMessageManager!";
     return ErrorCode::CANBUS_ERROR;
  }

/*   adu_steeringctrl_8cff1500_ = dynamic_cast<Adusteeringctrl8cff1500*>
          (message_manager_->GetMutableProtocolDataById(Adusteeringctrl8cff1500::ID));
  if (adu_steeringctrl_8cff1500_ == nullptr) {
     AERROR << "Adusteeringctrl8cff1500 does not exist in the ChMessageManager!";
     return ErrorCode::CANBUS_ERROR;
  } */

  adu_autosteering_98ffefe8_ = dynamic_cast<Aduautosteering98ffefe8*>
          (message_manager_->GetMutableProtocolDataById(Aduautosteering98ffefe8::ID));
  if (adu_autosteering_98ffefe8_ == nullptr) {
     AERROR << "Aduautosteering98ffefe8 does not exist in the ChMessageManager!";
     return ErrorCode::CANBUS_ERROR;
  }

  adu_brakectrl_8c04eb0a_ = dynamic_cast<Adubrakectrl8c04eb0a*>
          (message_manager_->GetMutableProtocolDataById(Adubrakectrl8c04eb0a::ID));
  if (adu_brakectrl_8c04eb0a_ == nullptr) {
     AERROR << "Adubrakectrl8c04eb0a does not exist in the ChMessageManager!";
     return ErrorCode::CANBUS_ERROR;
  }

  can_sender_->AddMessage(Adubodyctrl8cff1600::ID, adu_bodyctrl_8cff1600_, false);
  can_sender_->AddMessage(Adudrivectrl8cff1400::ID, adu_drivectrl_8cff1400_, false);
  /* can_sender_->AddMessage(Adusteeringctrl8cff1500::ID, adu_steeringctrl_8cff1500_, false); */
  can_sender_->AddMessage(Aduautosteering98ffefe8::ID, adu_autosteering_98ffefe8_, false);
  can_sender_->AddMessage(Adubrakectrl8c04eb0a::ID, adu_brakectrl_8c04eb0a_, false);

  // need sleep to ensure all messages received
  AINFO << "ChController is initialized.";

  is_initialized_ = true;
  return ErrorCode::OK;
}

ChController::~ChController() {}

bool ChController::Start() {
  if (!is_initialized_) {
    AERROR << "ChController has NOT been initiated.";
    return false;
  }
  const auto& update_func = [this] { SecurityDogThreadFunc(); };
  thread_.reset(new std::thread(update_func));

  return true;
}

void ChController::Stop() {
  if (!is_initialized_) {
    AERROR << "ChController stops or starts improperly!";
    return;
  }

  if (thread_ != nullptr && thread_->joinable()) {
    thread_->join();
    thread_.reset();
    AINFO << "ChController stopped.";
  }
}

Chassis ChController::chassis() {
  chassis_.Clear();

  ChassisDetail chassis_detail;
  message_manager_->GetSensorData(&chassis_detail);

  // 21, 22, previously 1, 2
  if (driving_mode() == Chassis::EMERGENCY_MODE) {
    set_chassis_error_code(Chassis::NO_ERROR);
  }

  chassis_.set_driving_mode(driving_mode());
  chassis_.set_error_code(chassis_error_code());

  // 3
  chassis_.set_engine_started(true);


  /* ADD YOUR OWN CAR CHASSIS OPERATION
  */
  // get vcu power status
  if (chassis_detail.ch().has_vcu_vehicleinfo3_98f003d0() &&
      chassis_detail.ch().vcu_vehicleinfo3_98f003d0().has_vcu_vehiclepowerstatus()) {
    power_status_vcu_return_ = chassis_detail.ch().vcu_vehicleinfo3_98f003d0().vcu_vehiclepowerstatus();
  }
  // get vcu gear status
  if(chassis_detail.ch().has_vcu_vehicleinfo3_98f003d0() && 
     chassis_detail.ch().vcu_vehicleinfo3_98f003d0().has_vcu_gearstatus()){
    gear_status_vcu_return_ = chassis_detail.ch().vcu_vehicleinfo3_98f003d0().vcu_gearstatus();
  }
  // get hand brake status
  if (chassis_detail.ch().has_vcu_vehicleinfo3_98f003d0() &&
    chassis_detail.ch().vcu_vehicleinfo3_98f003d0().has_vcu_handbrakestatus()) {
    hand_brake_status_vcu_return_ = chassis_detail.ch().vcu_vehicleinfo3_98f003d0().vcu_handbrakestatus();
  }
  // get steer position: angle
  if(chassis_detail.ch().has_asc_steeringinfo2_98fff013() && chassis_detail.ch().asc_steeringinfo2_98fff013().has_asc_absolute_steering_position()){
      if (chassis_detail.ch().asc_steeringinfo2_98fff013().asc_absolute_steering_position_qf()) {
        steer_angle_asc_return_ = chassis_detail.ch().asc_steeringinfo2_98fff013().asc_absolute_steering_position();
      } else {
        //chassis_.set_asc_status(Chassis::LEVEL2_FAULT);
        AERROR << " asc invalid steering position： ret= "
          << chassis_detail.ch().asc_steeringinfo2_98fff013().asc_absolute_steering_position();
      }
  }


  // asc fault detect
  steer_fault_detect_recover(chassis_detail);
  // adu brake ctrl
  brake_ctrl_backup(chassis_detail);


  // send vcu fault level
  if (chassis_detail.ch().has_vcu_vehicleinfo1_98f001d0() &&
      chassis_detail.ch().vcu_vehicleinfo1_98f001d0().has_vcu_modulestatus()) {
    chassis_.set_vcu_status(static_cast<Chassis::FaultLevel>(chassis_detail.ch().vcu_vehicleinfo1_98f001d0().vcu_modulestatus()));
  } else {
    chassis_.set_vcu_status(Chassis::NO_FAILURE);
  } 

  // set led screen, when HV
  switch (power_status_vcu_return_) {
    case Vcu_vehicleinfo3_98f003d0::VCU_VEHICLEPOWERSTATUS_READY:
    case Vcu_vehicleinfo3_98f003d0::VCU_VEHICLEPOWERSTATUS_HV_UP_WITH_FAULT:
      adu_bodyctrl_8cff1600_->set_ad_ledscreenreq(Adu_bodyctrl_8cff1600::AD_LEDSCREENREQ_OPEN);
      break;
    default:
      adu_bodyctrl_8cff1600_->set_ad_ledscreenreq(Adu_bodyctrl_8cff1600::AD_LEDSCREENREQ_CLOSE);
      break;
  }

  // send current vehicle speed
  if (chassis_detail.ch().has_vcu_vehicleinfo3_98f003d0() &&
      chassis_detail.ch().vcu_vehicleinfo3_98f003d0().has_vcu_vehiclespeed()) {
    chassis_.set_speed_mps(
        static_cast<float>(chassis_detail.ch().vcu_vehicleinfo3_98f003d0().vcu_vehiclespeed() * 1000 / 3600));
  } else {
    chassis_.set_speed_mps(0);
  }


  // send real gear location
  if (chassis_detail.ch().has_vcu_vehicleinfo3_98f003d0() &&
      chassis_detail.ch().vcu_vehicleinfo3_98f003d0().has_vcu_gearstatus()) {
    Chassis::GearPosition gear_pos = Chassis::GEAR_INVALID;

    if (chassis_detail.ch().vcu_vehicleinfo3_98f003d0().vcu_gearstatus() ==
        Vcu_vehicleinfo3_98f003d0::VCU_GEARSTATUS_NEUTRAL) {
      gear_pos = Chassis::GEAR_NEUTRAL;
    }
    if (chassis_detail.ch().vcu_vehicleinfo3_98f003d0().vcu_gearstatus() ==
        Vcu_vehicleinfo3_98f003d0::VCU_GEARSTATUS_DRIVE) {
      gear_pos = Chassis::GEAR_DRIVE;
    }
    if (chassis_detail.ch().vcu_vehicleinfo3_98f003d0().vcu_gearstatus() ==
        Vcu_vehicleinfo3_98f003d0::VCU_GEARSTATUS_REVERSE) {
      gear_pos = Chassis::GEAR_REVERSE;
    }
    if (chassis_detail.ch().vcu_vehicleinfo3_98f003d0().vcu_gearstatus() ==
        Vcu_vehicleinfo3_98f003d0::VCU_GEARSTATUS_PARK) {
      gear_pos = Chassis::GEAR_PARKING;
    }
    chassis_.set_gear_location(gear_pos);
  } else {
    chassis_.set_gear_location(Chassis::GEAR_NONE);
  }

  // TODO
  // send real steering location
  if (chassis_detail.ch().has_asc_steeringinfo2_98fff013() &&
      chassis_detail.ch().asc_steeringinfo2_98fff013().has_asc_absolute_steering_position()) {
    double temp = static_cast<float>(
                    chassis_detail.ch().asc_steeringinfo2_98fff013().asc_absolute_steering_position() 
                    / (vehicle_params_.max_steer_angle() / M_PI * 180) * 100 * -1);
    chassis_.set_steering_percentage(temp);
    //AERROR << "set_steering_percentage: " << temp
    //  << " angle=" << chassis_detail.ch().asc_steeringinfo2_98fff013().asc_absolute_steering_position();
    chassis_.set_throttle_percentage(throttle_percentage);
    chassis_.set_brake_percentage(brake_percentage);
  } else {
    chassis_.set_steering_percentage(0);
    chassis_.set_throttle_percentage(0);
    chassis_.set_brake_percentage(0);
  }

  if (chassis_error_mask_) {
    chassis_.set_chassis_error_mask(chassis_error_mask_);
  }

  if (chassis_detail.has_surround()) {
    chassis_.mutable_surround()->CopyFrom(chassis_detail.surround());
  }

  if (!chassis_error_mask_ && !chassis_.parking_brake() &&
      (chassis_.throttle_percentage() == 0.0)) {
    chassis_.mutable_engage_advice()->set_advice(
        apollo::common::EngageAdvice::READY_TO_ENGAGE);
  } else {
    chassis_.mutable_engage_advice()->set_advice(
        apollo::common::EngageAdvice::DISALLOW_ENGAGE);
    chassis_.mutable_engage_advice()->set_reason(
        "CANBUS not ready, firmware error or emergency button pressed!");
  }

  return chassis_;
}

void ChController::Emergency() {
  set_driving_mode(Chassis::EMERGENCY_MODE);
  ResetProtocol();
}

ErrorCode ChController::EnableAutoMode() {
  if (driving_mode() == Chassis::COMPLETE_AUTO_DRIVE) {
    AINFO << "already in COMPLETE_AUTO_DRIVE mode";
    return ErrorCode::OK;
  }
  return ErrorCode::OK;
  /* ADD YOUR OWN CAR CHASSIS OPERATION
  brake_60_->set_enable();
  throttle_62_->set_enable();
  steering_64_->set_enable();

  can_sender_->Update();
  const int32_t flag =
      CHECK_RESPONSE_STEER_UNIT_FLAG | CHECK_RESPONSE_SPEED_UNIT_FLAG;
  if (!CheckResponse(flag, true)) {
    AERROR << "Failed to switch to COMPLETE_AUTO_DRIVE mode.";
    Emergency();
    set_chassis_error_code(Chassis::CHASSIS_ERROR);
    return ErrorCode::CANBUS_ERROR;
  }
  set_driving_mode(Chassis::COMPLETE_AUTO_DRIVE);
  AINFO << "Switch to COMPLETE_AUTO_DRIVE mode ok.";
  return ErrorCode::OK;
  */
  // TODO
  adu_drivectrl_8cff1400_->set_ad_statusreq(Adu_drivectrl_8cff1400::AD_STATUSREQ_AD_REQUEST);
  can_sender_->Update();
  set_driving_mode(Chassis::COMPLETE_AUTO_DRIVE);
  AINFO << "Switch to COMPLETE_AUTO_DRIVE mode ok.";
  return ErrorCode::OK;
}

ErrorCode ChController::DisableAutoMode() {
  ResetProtocol();
  can_sender_->Update();
  set_driving_mode(Chassis::COMPLETE_MANUAL);
  set_chassis_error_code(Chassis::NO_ERROR);
  AINFO << "Switch to COMPLETE_MANUAL ok.";
  return ErrorCode::OK;
}

ErrorCode ChController::EnableSteeringOnlyMode() {
  if (driving_mode() == Chassis::COMPLETE_AUTO_DRIVE ||
      driving_mode() == Chassis::AUTO_STEER_ONLY) {
    set_driving_mode(Chassis::AUTO_STEER_ONLY);
    AINFO << "Already in AUTO_STEER_ONLY mode.";
    return ErrorCode::OK;
  }
  /* ADD YOUR OWN CAR CHASSIS OPERATION
  brake_60_->set_disable();
  throttle_62_->set_disable();
  steering_64_->set_enable();

  can_sender_->Update();
  if (!CheckResponse(CHECK_RESPONSE_STEER_UNIT_FLAG, true)) {
    AERROR << "Failed to switch to AUTO_STEER_ONLY mode.";
    Emergency();
    set_chassis_error_code(Chassis::CHASSIS_ERROR);
    return ErrorCode::CANBUS_ERROR;
  }
  set_driving_mode(Chassis::AUTO_STEER_ONLY);
  AINFO << "Switch to AUTO_STEER_ONLY mode ok.";
  return ErrorCode::OK;
  */
  set_driving_mode(Chassis::AUTO_STEER_ONLY);
  AINFO << "Switch to AUTO_STEER_ONLY mode ok.";
  return ErrorCode::OK;
}

ErrorCode ChController::EnableSpeedOnlyMode() {
  if (driving_mode() == Chassis::COMPLETE_AUTO_DRIVE ||
      driving_mode() == Chassis::AUTO_SPEED_ONLY) {
    set_driving_mode(Chassis::AUTO_SPEED_ONLY);
    AINFO << "Already in AUTO_SPEED_ONLY mode";
    return ErrorCode::OK;
  }
  /* ADD YOUR OWN CAR CHASSIS OPERATION
  brake_60_->set_enable();
  throttle_62_->set_enable();
  steering_64_->set_disable();

  can_sender_->Update();
  if (!CheckResponse(CHECK_RESPONSE_SPEED_UNIT_FLAG, true)) {
    AERROR << "Failed to switch to AUTO_STEER_ONLY mode.";
    Emergency();
    set_chassis_error_code(Chassis::CHASSIS_ERROR);
    return ErrorCode::CANBUS_ERROR;
  }
  set_driving_mode(Chassis::AUTO_SPEED_ONLY);
  AINFO << "Switch to AUTO_SPEED_ONLY mode ok.";
  return ErrorCode::OK;
  */
  // TODO
  set_driving_mode(Chassis::AUTO_SPEED_ONLY);
  AINFO << "Switch to AUTO_SPEED_ONLY mode ok.";
  return ErrorCode::OK;
}

// NEUTRAL, REVERSE, DRIVE
void ChController::Gear(Chassis::GearPosition gear_position) {
  if (driving_mode() != Chassis::COMPLETE_AUTO_DRIVE &&
      driving_mode() != Chassis::AUTO_SPEED_ONLY) {
    AINFO << "This drive mode no need to set gear.";
    return;
  }

  // ad_request until HV ready or up
  switch (power_status_vcu_return_) {
    case Vcu_vehicleinfo3_98f003d0::VCU_VEHICLEPOWERSTATUS_READY:
    case Vcu_vehicleinfo3_98f003d0::VCU_VEHICLEPOWERSTATUS_HV_UP_WITH_FAULT:
      adu_drivectrl_8cff1400_->set_ad_statusreq(Adu_drivectrl_8cff1400::AD_STATUSREQ_AD_REQUEST);
      break;
    default:
      break;
  }

  switch (gear_position) {
    case Chassis::GEAR_NEUTRAL: {
      adu_drivectrl_8cff1400_->set_ad_gearreq(Adu_drivectrl_8cff1400::AD_GEARREQ_NATURAL);
      break;
    }
    case Chassis::GEAR_REVERSE: {
      adu_drivectrl_8cff1400_->set_ad_gearreq(Adu_drivectrl_8cff1400::AD_GEARREQ_REVERSE);
      break;
    }
    case Chassis::GEAR_DRIVE: {
      adu_drivectrl_8cff1400_->set_ad_gearreq(Adu_drivectrl_8cff1400::AD_GEARREQ_DRIVE);
      break;
    }
    case Chassis::GEAR_PARKING: {
      adu_drivectrl_8cff1400_->set_ad_gearreq(Adu_drivectrl_8cff1400::AD_GEARREQ_PARK);
      break;
    }
    case Chassis::GEAR_INVALID: {
      AERROR << "Gear command is invalid!";
      adu_drivectrl_8cff1400_->set_ad_gearreq(Adu_drivectrl_8cff1400::AD_GEARREQ_NATURAL);
      break;
    }
    default: {
      adu_drivectrl_8cff1400_->set_ad_gearreq(Adu_drivectrl_8cff1400::AD_GEARREQ_NATURAL);
      break;
    }
  }
}

// brake with new acceleration
// acceleration:0.00~99.99, unit:
// acceleration:0.0 ~ 7.0, unit:m/s^2
// acceleration_spd:60 ~ 100, suggest: 90
// -> pedal
void ChController::Brake(double pedal) {
  // double real_value = params_.max_acc() * acceleration / 100;
  // TODO(All) :  Update brake value based on mode
  if (driving_mode() != Chassis::COMPLETE_AUTO_DRIVE &&
      driving_mode() != Chassis::AUTO_SPEED_ONLY) {
    AINFO << "The current drive mode does not need to set brake pedal.";
    return;
  }
  
  if (pedal > 0) {
    adu_drivectrl_8cff1400_->set_ad_brakereq(Adu_drivectrl_8cff1400::AD_BRAKEREQ_APPLY_BRAKE);
  } else {
    adu_drivectrl_8cff1400_->set_ad_brakereq(Adu_drivectrl_8cff1400::AD_BRAKEREQ_NO_BRAKE);
  }
  
  brake_percentage = pedal;
  /* Modify@20201223: according to Rev13 matrix, unit change from /100 to /1000 */
  adu_drivectrl_8cff1400_->set_ad_brakepercent(pedal * 10.0);
}

// drive with old acceleration
// gas:0.00~99.99 unit:
void ChController::Throttle(double pedal) {
  if (driving_mode() != Chassis::COMPLETE_AUTO_DRIVE &&
      driving_mode() != Chassis::AUTO_SPEED_ONLY) {
    AINFO << "The current drive mode does not need to set throttle pedal.";
    return;
  }
  /* ADD YOUR OWN CAR CHASSIS OPERATION
  throttle_62_->set_pedal(pedal);
  */
  throttle_percentage = pedal;
  /* Modify@20201223: according to Rev13 matrix, unit change from /100 to /1000 */
  adu_drivectrl_8cff1400_->set_ad_torquepercent(pedal * 10.0);
}

// confirm the car is driven by acceleration command or throttle/brake pedal
// drive with acceleration/deceleration
// acc:-7.0 ~ 5.0, unit:m/s^2
void ChController::Acceleration(double acc) {
  if (driving_mode() != Chassis::COMPLETE_AUTO_DRIVE ||
      driving_mode() != Chassis::AUTO_SPEED_ONLY) {
    AINFO << "The current drive mode does not need to set acceleration.";
    return;
  }
  /* ADD YOUR OWN CAR CHASSIS OPERATION
  */
}

// ch default, -470 ~ 470, left:+, right:-
// need to be compatible with control module, so reverse
// steering with old angle speed
// angle:-99.99~0.00~99.99, unit:, left:-, right:+
void ChController::Steer(double angle) {
  if (driving_mode() != Chassis::COMPLETE_AUTO_DRIVE &&
      driving_mode() != Chassis::AUTO_STEER_ONLY) {
    AINFO << "The current driving mode does not need to set steer.";
    return;
  }
  const double steer_angle_target =
      angle / 100 * (vehicle_params_.max_steer_angle() / M_PI * 180) * -1;

  double steer_angle_next = steer_adjust(steer_angle_target);

/*   const double real_angle_spd =
      ProtocolData<::apollo::canbus::ChassisDetail>::BoundedValue(
          vehicle_params_.min_steer_angle_rate() / M_PI * 180,
          vehicle_params_.max_steer_angle_rate() / M_PI * 180,
          150.0);
  adu_steeringctrl_8cff1500_->set_ad_steeringturnangle(steer_angle_target);
  adu_steeringctrl_8cff1500_->set_ad_wheelspeed(real_angle_spd);
  adu_steeringctrl_8cff1500_->set_ad_turncmdvalid(Adu_steeringctrl_8cff1500::AD_TURNCMDVALID_NORMAL);
  adu_steeringctrl_8cff1500_->set_ad_adustatus(Adu_steeringctrl_8cff1500::AD_ADUSTATUS_NORMAL); */
  //adu_autosteering_98ffefe8_->set_ad_desiredoperatingmode(Adu_autosteering_98ffefe8::AD_DESIREDOPERATINGMODE_AUTODRIVE);
  adu_autosteering_98ffefe8_->set_ad_command_from_external_controller(steer_angle_next);
}
  
// steering with new angle speed
// angle:-99.99~0.00~99.99, unit:, left:-, right:+
// angle_spd:0.00~99.99, unit:deg/s
void ChController::Steer(double angle, double angle_spd) {
  if (driving_mode() != Chassis::COMPLETE_AUTO_DRIVE &&
      driving_mode() != Chassis::AUTO_STEER_ONLY) {
    AINFO << "The current driving mode does not need to set steer.";
    return;
  }
  
  const double steer_angle_target =
      angle / 100 * (vehicle_params_.max_steer_angle() / M_PI * 180) * -1;

  double steer_angle_next = steer_adjust(steer_angle_target);

/*   const double real_angle_spd =
      ProtocolData<::apollo::canbus::ChassisDetail>::BoundedValue(
          vehicle_params_.min_steer_angle_rate() / M_PI * 180,
          vehicle_params_.max_steer_angle_rate() / M_PI * 180,
          vehicle_params_.max_steer_angle_rate() / M_PI * 180 * angle_spd / 100.0); 
  adu_steeringctrl_8cff1500_->set_ad_steeringturnangle(steer_angle_target);
  adu_steeringctrl_8cff1500_->set_ad_wheelspeed(real_angle_spd);
  adu_steeringctrl_8cff1500_->set_ad_turncmdvalid(Adu_steeringctrl_8cff1500::AD_TURNCMDVALID_NORMAL);
  adu_steeringctrl_8cff1500_->set_ad_adustatus(Adu_steeringctrl_8cff1500::AD_ADUSTATUS_NORMAL); */
  //adu_autosteering_98ffefe8_->set_ad_desiredoperatingmode(Adu_autosteering_98ffefe8::AD_DESIREDOPERATINGMODE_AUTODRIVE);
  adu_autosteering_98ffefe8_->set_ad_command_from_external_controller(steer_angle_next);
}

void ChController::SetEpbBreak(const ControlCommand& command) {
  if (command.has_parking_brake()) {
    if (command.parking_brake()) {
      adu_drivectrl_8cff1400_->set_ad_epb(Adu_drivectrl_8cff1400::AD_EPB_APPLY_PARKING_BRAKE);
    } else {
      adu_drivectrl_8cff1400_->set_ad_epb(Adu_drivectrl_8cff1400::AD_EPB_STANDBY);
    }
  }
}

void ChController::SetBeam(const ControlCommand& command) {
/*   if (command.signal().has_high_beam()) {
    if (command.signal().high_beam()) {
      adu_bodyctrl_8cff1600_->set_ad_headlightfullbeamreq(Adu_bodyctrl_8cff1600::AD_HEADLIGHTFULLBEAMREQ_OPEN);
    } else {
      adu_bodyctrl_8cff1600_->set_ad_headlightfullbeamreq(Adu_bodyctrl_8cff1600::AD_HEADLIGHTFULLBEAMREQ_CLOSE);
    }
  }

  if (command.signal().has_low_beam()) {
    if (command.signal().low_beam()) {
      adu_bodyctrl_8cff1600_->set_ad_dippedheadlampreq(Adu_bodyctrl_8cff1600::AD_DIPPEDHEADLAMPREQ_OPEN);
    } else {
      adu_bodyctrl_8cff1600_->set_ad_dippedheadlampreq(Adu_bodyctrl_8cff1600::AD_DIPPEDHEADLAMPREQ_CLOSE);
    }
  } */
  if (command.signal().has_head_lamp()) {
    switch (command.signal().head_lamp()){
      case common::VehicleSignal::HEADLAMPSTAT_DIPPEDLAMP:
        adu_bodyctrl_8cff1600_->set_ad_headlampreq(Adu_bodyctrl_8cff1600::AD_HEADLAMPREQ_DIPPEDHEADLAMP);
        break;
      case common::VehicleSignal::HEADLAMPSTAT_FULLBEAM:
        adu_bodyctrl_8cff1600_->set_ad_headlampreq(Adu_bodyctrl_8cff1600::AD_HEADLAMPREQ_HEADLIGHTFULLBEAM);
        break;
      default:
        adu_bodyctrl_8cff1600_->set_ad_headlampreq(Adu_bodyctrl_8cff1600::AD_HEADLAMPREQ_CLOSE);
        break;
    }
  }


  if (command.signal().has_horn()) {
    if (command.signal().horn()) {
      adu_bodyctrl_8cff1600_->set_ad_hornreq(Adu_bodyctrl_8cff1600::AD_HORNREQ_OPEN);
    } else {
      adu_bodyctrl_8cff1600_->set_ad_hornreq(Adu_bodyctrl_8cff1600::AD_HORNREQ_CLOSE);
    }
  }

  if (command.signal().has_warning_lamp()) {
    if (command.signal().warning_lamp()) {
      adu_bodyctrl_8cff1600_->set_ad_warninglampreq(Adu_bodyctrl_8cff1600::AD_WARNINGLAMPREQ_OPEN);
    } else {
      adu_bodyctrl_8cff1600_->set_ad_warninglampreq(Adu_bodyctrl_8cff1600::AD_WARNINGLAMPREQ_CLOSE);
    }
  }

  if (command.signal().has_position_lamp()) {
    if (command.signal().position_lamp()) {
      adu_bodyctrl_8cff1600_->set_ad_positionlampreq(Adu_bodyctrl_8cff1600::AD_POSITIONLAMPREQ_OPEN);
    } else {
      adu_bodyctrl_8cff1600_->set_ad_positionlampreq(Adu_bodyctrl_8cff1600::AD_POSITIONLAMPREQ_CLOSE);
    }
  }

  if (command.signal().has_front_fog_lamp()) {
    if (command.signal().front_fog_lamp()) {
      adu_bodyctrl_8cff1600_->set_ad_frontfoglampreq(Adu_bodyctrl_8cff1600::AD_FRONTFOGLAMPREQ_OPEN);
    } else {
      adu_bodyctrl_8cff1600_->set_ad_frontfoglampreq(Adu_bodyctrl_8cff1600::AD_FRONTFOGLAMPREQ_CLOSE);
    }
  }

  if (command.signal().has_rear_fog_lamp()) {
    if (command.signal().rear_fog_lamp()) {
      adu_bodyctrl_8cff1600_->set_ad_rearfoglampreq(Adu_bodyctrl_8cff1600::AD_REARFOGLAMPREQ_OPEN);
    } else {
      adu_bodyctrl_8cff1600_->set_ad_rearfoglampreq(Adu_bodyctrl_8cff1600::AD_REARFOGLAMPREQ_CLOSE);
    }
  }

/*   if (command.signal().has_led_screen()) {
    if (command.signal().led_screen()) {
      adu_bodyctrl_8cff1600_->set_ad_ledscreenreq(Adu_bodyctrl_8cff1600::AD_LEDSCREENREQ_OPEN);
    } else {
      adu_bodyctrl_8cff1600_->set_ad_ledscreenreq(Adu_bodyctrl_8cff1600::AD_LEDSCREENREQ_CLOSE);
    }
  } */

  if (command.signal().has_top_warn_lamp()) {
    common::VehicleSignal_TopWarnLampStat top_warn_lamp_curr = command.signal().top_warn_lamp();
    switch (top_warn_lamp_curr) {
      case common::VehicleSignal::TOPWARNLAMPSTAT_GREEN:
        adu_bodyctrl_8cff1600_->set_ad_topwarninglampreq(Adu_bodyctrl_8cff1600::AD_TOPWARNINGLAMPREQ_GREEN);
        break;
      case common::VehicleSignal::TOPWARNLAMPSTAT_RED:
        adu_bodyctrl_8cff1600_->set_ad_topwarninglampreq(Adu_bodyctrl_8cff1600::AD_TOPWARNINGLAMPREQ_RED);
        break;
      case common::VehicleSignal::TOPWARNLAMPSTAT_FLASH_GREEN:
        if (top_warn_lamp_curr != top_warn_lamp_pre) {
          lamp_flash_counter_ = 0;
        }
        if (lamp_flash_counter_ < 25) {
          adu_bodyctrl_8cff1600_->set_ad_topwarninglampreq(Adu_bodyctrl_8cff1600::AD_TOPWARNINGLAMPREQ_GREEN);
        } else {
          adu_bodyctrl_8cff1600_->set_ad_topwarninglampreq(Adu_bodyctrl_8cff1600::AD_TOPWARNINGLAMPREQ_CLOSE);
        }
        lamp_flash_counter_ = (lamp_flash_counter_ + 1) % 50;
        break;
      case common::VehicleSignal::TOPWARNLAMPSTAT_FLASH_RED:
        if (top_warn_lamp_curr != top_warn_lamp_pre) {
          lamp_flash_counter_ = 0;
        }
        if (lamp_flash_counter_ < 25) {
          adu_bodyctrl_8cff1600_->set_ad_topwarninglampreq(Adu_bodyctrl_8cff1600::AD_TOPWARNINGLAMPREQ_RED);
        } else {
          adu_bodyctrl_8cff1600_->set_ad_topwarninglampreq(Adu_bodyctrl_8cff1600::AD_TOPWARNINGLAMPREQ_CLOSE);
        }
        lamp_flash_counter_ = (lamp_flash_counter_ + 1) % 50;
        break;
      default:
        adu_bodyctrl_8cff1600_->set_ad_topwarninglampreq(Adu_bodyctrl_8cff1600::AD_TOPWARNINGLAMPREQ_CLOSE);
        break;
    }

    top_warn_lamp_pre =  top_warn_lamp_curr;
  }
}

void ChController::SetHorn(const ControlCommand& command) {
  if (command.signal().horn()) {
    // None
  } else {
    // None
  }
}

void ChController::SetTurningSignal(const ControlCommand& command) {
  if (!command.signal().has_turn_signal()) {
    return;
  }

  auto signal = command.signal().turn_signal();
  if (signal == common::VehicleSignal::TURN_LEFT) {
    adu_bodyctrl_8cff1600_->set_ad_leftturnlampreq(Adu_bodyctrl_8cff1600::AD_LEFTTURNLAMPREQ_OPEN);
    adu_bodyctrl_8cff1600_->set_ad_rightturnlampreq(Adu_bodyctrl_8cff1600::AD_RIGHTTURNLAMPREQ_CLOSE);
  } else if (signal == common::VehicleSignal::TURN_RIGHT) {
    adu_bodyctrl_8cff1600_->set_ad_leftturnlampreq(Adu_bodyctrl_8cff1600::AD_LEFTTURNLAMPREQ_CLOSE);
    adu_bodyctrl_8cff1600_->set_ad_rightturnlampreq(Adu_bodyctrl_8cff1600::AD_RIGHTTURNLAMPREQ_OPEN);
  } else {
    adu_bodyctrl_8cff1600_->set_ad_leftturnlampreq(Adu_bodyctrl_8cff1600::AD_LEFTTURNLAMPREQ_CLOSE);
    adu_bodyctrl_8cff1600_->set_ad_rightturnlampreq(Adu_bodyctrl_8cff1600::AD_RIGHTTURNLAMPREQ_CLOSE);
  }
}

void ChController::ResetProtocol() {
  message_manager_->ResetSendMessages();
}

bool ChController::CheckChassisError() {
  /* ADD YOUR OWN CAR CHASSIS OPERATION
  */
  return false;
}

void ChController::SecurityDogThreadFunc() {
  int32_t vertical_ctrl_fail = 0;
  int32_t horizontal_ctrl_fail = 0;

  if (can_sender_ == nullptr) {
    AERROR << "Failed to run SecurityDogThreadFunc() because can_sender_ is "
              "nullptr.";
    return;
  }
  while (!can_sender_->IsRunning()) {
    std::this_thread::yield();
  }

  std::chrono::duration<double, std::micro> default_period{50000};
  int64_t start = 0;
  int64_t end = 0;
  while (can_sender_->IsRunning()) {
    start = absl::ToUnixMicros(::apollo::common::time::Clock::Now());
    const Chassis::DrivingMode mode = driving_mode();
    bool emergency_mode = false;

    // 1. horizontal control check
    if ((mode == Chassis::COMPLETE_AUTO_DRIVE ||
         mode == Chassis::AUTO_STEER_ONLY) &&
        CheckResponse(CHECK_RESPONSE_STEER_UNIT_FLAG, false) == false) {
      ++horizontal_ctrl_fail;
      if (horizontal_ctrl_fail >= kMaxFailAttempt) {
        emergency_mode = true;
        set_chassis_error_code(Chassis::MANUAL_INTERVENTION);
      }
    } else {
      horizontal_ctrl_fail = 0;
    }

    // 2. vertical control check
    if ((mode == Chassis::COMPLETE_AUTO_DRIVE ||
         mode == Chassis::AUTO_SPEED_ONLY) &&
        !CheckResponse(CHECK_RESPONSE_SPEED_UNIT_FLAG, false)) {
      ++vertical_ctrl_fail;
      if (vertical_ctrl_fail >= kMaxFailAttempt) {
        emergency_mode = true;
        set_chassis_error_code(Chassis::MANUAL_INTERVENTION);
      }
    } else {
      vertical_ctrl_fail = 0;
    }
    if (CheckChassisError()) {
      set_chassis_error_code(Chassis::CHASSIS_ERROR);
      emergency_mode = true;
    }

    if (emergency_mode && mode != Chassis::EMERGENCY_MODE) {
      set_driving_mode(Chassis::EMERGENCY_MODE);
      message_manager_->ResetSendMessages();
    }
    end = absl::ToUnixMicros(::apollo::common::time::Clock::Now());
    std::chrono::duration<double, std::micro> elapsed{end - start};
    if (elapsed < default_period) {
      std::this_thread::sleep_for(default_period - elapsed);
    } else {
      AERROR
          << "Too much time consumption in ChController looping process:"
          << elapsed.count();
    }
  }
}

bool ChController::CheckResponse(const int32_t flags, bool need_wait) {
  /* ADD YOUR OWN CAR CHASSIS OPERATION
  */
  // TODO
  return true;
}

void ChController::set_chassis_error_mask(const int32_t mask) {
  std::lock_guard<std::mutex> lock(chassis_mask_mutex_);
  chassis_error_mask_ = mask;
}

int32_t ChController::chassis_error_mask() {
  std::lock_guard<std::mutex> lock(chassis_mask_mutex_);
  return chassis_error_mask_;
}

Chassis::ErrorCode ChController::chassis_error_code() {
  std::lock_guard<std::mutex> lock(chassis_error_code_mutex_);
  return chassis_error_code_;
}

void ChController::set_chassis_error_code(
    const Chassis::ErrorCode& error_code) {
  std::lock_guard<std::mutex> lock(chassis_error_code_mutex_);
  chassis_error_code_ = error_code;
}

void ChController::SetFaultLevel(bool has_error_level, bool has_error_dtc, int32_t error_level, int32_t dtc) {
  bool set_external_dtc_flag = false;

#ifdef ENABLE_STEER_FAULT_DETECT_AND_RECOVER
  // set dtc 199: asc/ehps fault, when asc_error timeout
  if (steer_fault_level_ == Chassis::LEVEL3_FAULT) {
    dtc = STEER_FAULT_DETECT_DTC;
    if (has_error_level == false) {
      error_level = 0;
    }
    set_external_dtc_flag = true;
  }
#endif

  if ((has_error_level && has_error_dtc) || (set_external_dtc_flag == true)) {
    adu_drivectrl_8cff1400_->set_ad_dtc(dtc);
    adu_drivectrl_8cff1400_->set_ad_adcstatus(static_cast<Adu_drivectrl_8cff1400::Ad_adcstatusType>(error_level));
  }
  
}

void ChController::MaxSpeedLimit(double limit) {
  adu_drivectrl_8cff1400_->set_ad_maxspeedlimitreq(limit * 3600 / 1000);
}

void ChController::SetAdStatusReq(int32_t request){
  fflush(NULL);
  printf("request:%d\n", request);
  adu_drivectrl_8cff1400_->set_ad_statusreq(static_cast<Adu_drivectrl_8cff1400::Ad_statusreqType>(request));
}


double ChController::steer_adjust(const double steer_angle_target) {
#ifdef ENABLE_STEER_ADJUST
  int32_t is_gear_d_r;
  if (gear_status_vcu_return_ == Vcu_vehicleinfo3_98f003d0::VCU_GEARSTATUS_DRIVE ||
      gear_status_vcu_return_ == Vcu_vehicleinfo3_98f003d0::VCU_GEARSTATUS_REVERSE) {
    is_gear_d_r = 1;
  } else {
    is_gear_d_r = 0;
  }

  if (is_first_steer == false) {
    is_first_steer = true;
    steer_angle_curr_ = steer_angle_asc_return_;
  }

  if ((is_gear_d_r == 1)
  #ifdef ENABLE_STEER_ADJUST_SELF
      && (fabs(steer_angle_target_pre_ - steer_angle_target) > STEER_EPS) 
  #endif
     ){
    steer_angle_curr_ = steer_angle_asc_return_;
  }
  
  double steer_angle_next = steer_angle_curr_;
  double steer_dif = steer_angle_target - steer_angle_curr_;
  if (is_gear_d_r == 1) {
    if (steer_dif > STEER_EPS) {
      steer_angle_next = steer_angle_curr_ + ((steer_dif < STEER_ADJUST_STEP) ? steer_dif : STEER_ADJUST_STEP); 
    } else if (steer_dif < (-1.0 * STEER_EPS)) {
      steer_angle_next = steer_angle_curr_ + (((-steer_dif) < STEER_ADJUST_STEP) ? steer_dif : (-1.0 * STEER_ADJUST_STEP)); 
    }
  }
  /* AERROR << "steer: target=" << steer_angle_target
    << " target_pre=" << steer_angle_asc_return_
    << " ret=" << steer_angle_target_pre_
    << " curr=" << steer_angle_curr_
    << " next=" << steer_angle_next; */
  #ifdef ENABLE_STEER_ADJUST_SELF
  steer_angle_curr_ = steer_angle_next;
  steer_angle_target_pre_ = steer_angle_target;
  #endif

#else
  double steer_angle_next = steer_angle_target;
#endif

  return steer_angle_next;
}


void ChController::steer_fault_detect_recover(ChassisDetail& chassis_detail) {
#ifdef ENABLE_STEER_FAULT_DETECT_AND_RECOVER
  /* Detect and try to recover: asc fault */
  int32_t asc_error;
  Asc_steeringinfo2_98fff013::Asc_actual_mode asc_actual_mode = Asc_steeringinfo2_98fff013::ASC_POSITION_CONTROL_FROM_REMOTE_INPUT;
  if (chassis_detail.ch().has_asc_steeringinfo2_98fff013() &&
       chassis_detail.ch().asc_steeringinfo2_98fff013().has_asc_status_error()) {
    // bit0~7: 0 1 2 3    x x 6 x
    asc_error = chassis_detail.ch().asc_steeringinfo2_98fff013().asc_status_error() & 0x4F;
    if (asc_error) {
      // get first error time
      if (steer_error_detect_start_flag_ == 0){
        steer_error_detect_start_flag_ = 1;
        steer_error_detect_start_time_ = apollo::common::time::Clock::NowInSeconds();
      }
      // get asc actual mode
      if (chassis_detail.ch().has_asc_steeringinfo2_98fff013() &&
          chassis_detail.ch().asc_steeringinfo2_98fff013().has_asc_actual_mode()) {
        asc_actual_mode = chassis_detail.ch().asc_steeringinfo2_98fff013().asc_actual_mode();
      }
      // timeout
      if ((apollo::common::time::Clock::NowInSeconds() - steer_error_detect_start_time_) > STEER_FAULT_DETECT_TIMEOUT_SEC) {
        steer_fault_level_ = Chassis::LEVEL3_FAULT;
      } else {
        steer_fault_level_ = Chassis::LEVEL2_FAULT;
      }
    } else {
      steer_fault_level_ = Chassis::NO_FAILURE;
      // clear timeout
      steer_error_detect_start_flag_ = 0;
    }
  } else {
    steer_fault_level_ = Chassis::LEVEL1_FAULT;
  } 

  chassis_.set_asc_status(steer_fault_level_);

  // try to recover
  adu_autosteering_98ffefe8_->set_ad_desiredoperatingmode( \
          (asc_actual_mode == Asc_steeringinfo2_98fff013::ASC_NORMAL_TORQUE_CONTROL) ? \
          Adu_autosteering_98ffefe8::AD_DESIREDOPERATINGMODE_DEFAULT : \
          Adu_autosteering_98ffefe8::AD_DESIREDOPERATINGMODE_AUTODRIVE);
#endif
}


void ChController::brake_ctrl_backup(ChassisDetail& chassis_detail) {
#ifdef ENABLE_ADU_BRAKECTRL_BACKUP
  // 1. get msg
  float brake_percentage_cur_ = brake_percentage;
  int32_t vcu_brake1_air_pressure = 0;
  int32_t vcu_brake2_air_pressure = 0;
  // get vcu brake air pressure
  if (chassis_detail.ch().has_vcu_brakeinfo1_8c04cb0a() &&
    chassis_detail.ch().vcu_brakeinfo1_8c04cb0a().has_vcu_brake1_air_pressure()) {
    vcu_brake1_air_pressure = chassis_detail.ch().vcu_brakeinfo1_8c04cb0a().vcu_brake1_air_pressure();
  }
  if (chassis_detail.ch().has_vcu_brakeinfo2_8c04db0a() &&
    chassis_detail.ch().vcu_brakeinfo2_8c04db0a().has_vcu_brake2_air_pressure()) {
    vcu_brake2_air_pressure = chassis_detail.ch().vcu_brakeinfo2_8c04db0a().vcu_brake2_air_pressure();
  }

  // 2. brake detect: air brake && no vcu brake and handbrake
  if (brake_detect_start_flag_ == 0 || brake_detect_start_flag_ == 1) {
    if (IS_AIR_BRAKE(brake_percentage_cur_) && 
        (vcu_brake1_air_pressure == 0 && 
        vcu_brake2_air_pressure == 0 && 
        hand_brake_status_vcu_return_ == 0)) {
      
      if (brake_detect_start_flag_ == 0) {
        // set detect flag to time
        brake_detect_start_flag_ = 1;
        brake_detect_start_time_ = apollo::common::time::Clock::NowInSeconds();
      } else if (brake_detect_start_flag_ == 1) {
        if ((apollo::common::time::Clock::NowInSeconds() - brake_detect_start_time_) > ADU_BRAKECTRL_DETECT_TIMEOUT_SEC) {
          // set detect flag to brake ctrl
          brake_detect_start_flag_ = 2;
          // reset adu brake air pressure
          adu_brake1_air_pressure_ = 0;
          adu_brake2_air_pressure_ = 0;
        }
      }
    } else {
      // reset detect flag
      brake_detect_start_flag_ = 0;
    }
  }

  // 3. brake ctrl & recover
  if (brake_detect_start_flag_ == 2) {
    // get target pressure
    // 0%=0kpa 20%=0kpa 40%=200kpa 60%=400kpa 80%=600kpa 100%=800kpa
    int32_t adu_brake_air_pressure_target = 0;
    BRAKE_PERCENTAGE_TO_AIRPRESSURE(adu_brake_air_pressure_target, brake_percentage_cur_);
    
    // calc next brake air pressure
    int32_t adu_brake_air_pressure_next_ = adu_brake1_air_pressure_;
    int32_t brake_air_pressure_dif = adu_brake_air_pressure_target - adu_brake1_air_pressure_;
    if (brake_air_pressure_dif > 0) 
    {
      adu_brake_air_pressure_next_ +=  \
        ((brake_air_pressure_dif < ADU_BRAKECTRL_ADJUST_STEP) ? \
          brake_air_pressure_dif : ADU_BRAKECTRL_ADJUST_STEP); 
    } 
    else if (brake_air_pressure_dif < 0) 
    {
      adu_brake_air_pressure_next_ +=  \
        (((-brake_air_pressure_dif) < ADU_BRAKECTRL_ADJUST_STEP) ? \
          brake_air_pressure_dif : (-1.0 * ADU_BRAKECTRL_ADJUST_STEP)); 
    }
    else
    {
      //
    }
    adu_brake1_air_pressure_ = adu_brake_air_pressure_next_;
    adu_brake2_air_pressure_ = adu_brake_air_pressure_next_;

    // recover
    if ((adu_brake_air_pressure_next_ == 0) ||
        ((vcu_brake1_air_pressure > 0 && vcu_brake2_air_pressure > 0) &&
          (((vcu_brake1_air_pressure + ADU_BRAKECTRL_RECOVER_AIRPRESSURE_DELAY) > adu_brake1_air_pressure_) ||
           ((vcu_brake2_air_pressure + ADU_BRAKECTRL_RECOVER_AIRPRESSURE_DELAY) > adu_brake2_air_pressure_)
          )
        )
       )
    {
      // reset detect flag
      brake_detect_start_flag_ = 0;
      // reset adu brake air pressure
      adu_brake1_air_pressure_ = 0;
      adu_brake2_air_pressure_ = 0;
    }
    //AERROR << "brake_percentage_=" << brake_percentage_cur_
    //  << "pressure=" << adu_brake_air_pressure_target
    //  << "next=" << adu_brake_air_pressure_next_;
  }


  // 4. set msg
  adu_brakectrl_8c04eb0a_->set_adu_brake1_air_pressure(adu_brake1_air_pressure_);
  adu_brakectrl_8c04eb0a_->set_adu_brake2_air_pressure(adu_brake2_air_pressure_);
#endif
}

}  // namespace ch
}  // namespace canbus
}  // namespace apollo
