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

  adu_steeringctrl_8cff1500_ = dynamic_cast<Adusteeringctrl8cff1500*>
          (message_manager_->GetMutableProtocolDataById(Adusteeringctrl8cff1500::ID));
  if (adu_steeringctrl_8cff1500_ == nullptr) {
     AERROR << "Adusteeringctrl8cff1500 does not exist in the ChMessageManager!";
     return ErrorCode::CANBUS_ERROR;
  }

  can_sender_->AddMessage(Adubodyctrl8cff1600::ID, adu_bodyctrl_8cff1600_, false);
  can_sender_->AddMessage(Adudrivectrl8cff1400::ID, adu_drivectrl_8cff1400_, false);
  can_sender_->AddMessage(Adusteeringctrl8cff1500::ID, adu_steeringctrl_8cff1500_, false);

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
  // send current vehicle speed
  if (chassis_detail.ch().has_vcu_vehicleinfo3_98f003d0() &&
      chassis_detail.ch().vcu_vehicleinfo3_98f003d0().has_vcu_vehiclespeed()) {
    chassis_.set_speed_mps(
        static_cast<float>(chassis_detail.ch().vcu_vehicleinfo3_98f003d0().vcu_vehiclespeed() * 1000 / 3600));
  } else {
    chassis_.set_speed_mps(0);
  }
  // send vcu fault level
  if (chassis_detail.ch().has_vcu_vehicleinfo1_98f001d0() &&
      chassis_detail.ch().vcu_vehicleinfo1_98f001d0().has_vcu_modulestatus()) {
    chassis_.set_vcu_status(
        static_cast<Chassis::FaultLevel>(chassis_detail.ch().vcu_vehicleinfo1_98f001d0().vcu_modulestatus()));
  } else {
    chassis_.set_vcu_status(Chassis::NO_FAILURE);
  }
  // send asu fault level
  if (chassis_detail.ch().has_asc_steeringinfo_8c02a0a2() &&
      chassis_detail.ch().asc_steeringinfo_8c02a0a2().has_asc_faultlevel()) {
    chassis_.set_asc_status(
        static_cast<Chassis::FaultLevel>(chassis_detail.ch().asc_steeringinfo_8c02a0a2().asc_faultlevel()));
  } else {
    chassis_.set_asc_status(Chassis::NO_FAILURE);
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
  if (chassis_detail.ch().has_asc_steeringinfo_8c02a0a2() &&
      chassis_detail.ch().asc_steeringinfo_8c02a0a2().has_asc_steeringwheelturnangle()) {
    chassis_.set_steering_percentage(static_cast<float>(
        chassis_detail.ch().asc_steeringinfo_8c02a0a2().asc_steeringwheelturnangle() 
        / (vehicle_params_.max_steer_angle() / M_PI * 180) * 100 * -1));
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

  adu_drivectrl_8cff1400_->set_ad_statusreq(Adu_drivectrl_8cff1400::AD_STATUSREQ_AD_REQUEST);
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
  adu_drivectrl_8cff1400_->set_ad_brakepercent(pedal);
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
  adu_drivectrl_8cff1400_->set_ad_torquepercent(pedal);
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
  const double real_angle =
      angle / 100 * (vehicle_params_.max_steer_angle() / M_PI * 180) * -1;
  const double real_angle_spd =
      ProtocolData<::apollo::canbus::ChassisDetail>::BoundedValue(
          vehicle_params_.min_steer_angle_rate() / M_PI * 180,
          vehicle_params_.max_steer_angle_rate() / M_PI * 180,
          150.0);
  adu_steeringctrl_8cff1500_->set_ad_steeringturnangle(real_angle);
  adu_steeringctrl_8cff1500_->set_ad_wheelspeed(real_angle_spd);
  adu_steeringctrl_8cff1500_->set_ad_turncmdvalid(Adu_steeringctrl_8cff1500::AD_TURNCMDVALID_NORMAL);
  adu_steeringctrl_8cff1500_->set_ad_adustatus(Adu_steeringctrl_8cff1500::AD_ADUSTATUS_NORMAL);
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
  
  const double real_angle =
      angle / 100 * (vehicle_params_.max_steer_angle() / M_PI * 180) * -1;
  const double real_angle_spd =
      ProtocolData<::apollo::canbus::ChassisDetail>::BoundedValue(
          vehicle_params_.min_steer_angle_rate() / M_PI * 180,
          vehicle_params_.max_steer_angle_rate() / M_PI * 180,
          vehicle_params_.max_steer_angle_rate() / M_PI * 180 * angle_spd / 100.0);
  adu_steeringctrl_8cff1500_->set_ad_steeringturnangle(real_angle);
  adu_steeringctrl_8cff1500_->set_ad_wheelspeed(real_angle_spd);
  adu_steeringctrl_8cff1500_->set_ad_turncmdvalid(Adu_steeringctrl_8cff1500::AD_TURNCMDVALID_NORMAL);
  adu_steeringctrl_8cff1500_->set_ad_adustatus(Adu_steeringctrl_8cff1500::AD_ADUSTATUS_NORMAL);
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
  if (command.signal().has_high_beam()) {
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

void ChController::SetFaultLevel(int32_t error_level, int32_t dtc) {
  adu_drivectrl_8cff1400_->set_ad_adcstatus(static_cast<Adu_drivectrl_8cff1400::Ad_adcstatusType>(error_level));
  adu_drivectrl_8cff1400_->set_ad_dtc(dtc);
}

void ChController::MaxSpeedLimit(double limit) {
  adu_drivectrl_8cff1400_->set_ad_maxspeedlimitreq(limit * 3600 / 1000);
}

void ChController::SetAdStatusReq(int32_t request){
  fflush(NULL);
  printf("request:%d\n", request);
  adu_drivectrl_8cff1400_->set_ad_statusreq(static_cast<Adu_drivectrl_8cff1400::Ad_statusreqType>(request));
}

}  // namespace ch
}  // namespace canbus
}  // namespace apollo
