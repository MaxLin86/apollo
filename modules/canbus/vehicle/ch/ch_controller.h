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

#pragma once

#include <memory>
#include <thread>

#include "modules/canbus/vehicle/vehicle_controller.h"

#include "modules/canbus/proto/canbus_conf.pb.h"
#include "modules/canbus/proto/chassis.pb.h"
#include "modules/canbus/proto/vehicle_parameter.pb.h"
#include "modules/common/proto/error_code.pb.h"
#include "modules/control/proto/control_cmd.pb.h"

#include "modules/canbus/vehicle/ch/protocol/adu_bodyctrl_8cff1600.h"
#include "modules/canbus/vehicle/ch/protocol/adu_drivectrl_8cff1400.h"
//#include "modules/canbus/vehicle/ch/protocol/adu_steeringctrl_8cff1500.h"
#include "modules/canbus/vehicle/ch/protocol/adu_autosteering_98ffefe8.h"
#include "modules/canbus/vehicle/ch/protocol/asc_steeringinfo2_98fff013.h"
#include "modules/canbus/vehicle/ch/protocol/adu_brakectrl_8c04eb0a.h"


/* Steer adjust */
#define ENABLE_STEER_ADJUST
/* Steer adjust selft, without asc_return_value
   Avoid asc_shaking, and asc_no_action when STEER_ADJUST_STEP too small */
//#define ENABLE_STEER_ADJUST_SELF
#define STEER_ADJUST_STEP   (15.0)
#define STEER_EPS           (0.000001)

/* Steer fault detect & recover */
#define ENABLE_STEER_FAULT_DETECT_AND_RECOVER
#define STEER_FAULT_DETECT_DTC                  (199)
#define STEER_FAULT_DETECT_TIMEOUT_SEC          (2.0)

/* Adu brake ctrl backup */
#define ENABLE_ADU_BRAKECTRL_BACKUP
#define ADU_BRAKECTRL_DETECT_TIMEOUT_SEC          (0.12)
#define ADU_BRAKECTRL_RECOVER_AIRPRESSURE_DELAY   (100)
#define ADU_BRAKECTRL_ADJUST_STEP                 (2)
#define ADU_BRAKECTRL_AIRBRAKE_THRESH_PECENTAGE   (0.2)
#define ADU_BRAKECTRL_EPS                         (0.000001)
#define IS_AIR_BRAKE(percentage)  \
  ((percentage) > ADU_BRAKECTRL_AIRBRAKE_THRESH_PECENTAGE)
#define BRAKE_PERCENTAGE_TO_AIRPRESSURE(air_pressure, percentage) \
  do{ \
    if (IS_AIR_BRAKE(percentage)) { \
      air_pressure = static_cast<int32_t>( \
        ((percentage) - (ADU_BRAKECTRL_AIRBRAKE_THRESH_PECENTAGE * 100.0)) * 10.0); \
    } else { \
      air_pressure = 0; \
    } \
  }while(0)


namespace apollo {
namespace canbus {
namespace ch {

class ChController final : public VehicleController {
 public:

  explicit ChController() {};

  virtual ~ChController();

  ::apollo::common::ErrorCode Init(
      const VehicleParameter& params,
      CanSender<::apollo::canbus::ChassisDetail> *const can_sender,
      MessageManager<::apollo::canbus::ChassisDetail> *const message_manager) override;

  bool Start() override;

  /**
   * @brief stop the vehicle controller.
   */
  void Stop() override;

  /**
   * @brief calculate and return the chassis.
   * @returns a copy of chassis. Use copy here to avoid multi-thread issues.
   */
  Chassis chassis() override;

 private:
  // main logical function for operation the car enter or exit the auto driving
  void Emergency() override;
  ::apollo::common::ErrorCode EnableAutoMode() override;
  ::apollo::common::ErrorCode DisableAutoMode() override;
  ::apollo::common::ErrorCode EnableSteeringOnlyMode() override;
  ::apollo::common::ErrorCode EnableSpeedOnlyMode() override;

  // NEUTRAL, REVERSE, DRIVE
  void Gear(Chassis::GearPosition state) override;

  // brake with new acceleration
  // acceleration:0.00~99.99, unit:
  // acceleration_spd: 60 ~ 100, suggest: 90
  void Brake(double acceleration) override;

  // drive with old acceleration
  // gas:0.00~99.99 unit:
  void Throttle(double throttle) override;

  // drive with acceleration/deceleration
  // acc:-7.0~5.0 unit:m/s^2
  void Acceleration(double acc) override;

  // steering with old angle speed
  // angle:-99.99~0.00~99.99, unit:, left:+, right:-
  void Steer(double angle) override;

  // steering with new angle speed
  // angle:-99.99~0.00~99.99, unit:, left:+, right:-
  // angle_spd:0.00~99.99, unit:deg/s
  void Steer(double angle, double angle_spd) override;

  // set Electrical Park Brake
  void SetEpbBreak(const ::apollo::control::ControlCommand& command) override;
  void SetBeam(const ::apollo::control::ControlCommand& command) override;
  void SetHorn(const ::apollo::control::ControlCommand& command) override;
  void SetTurningSignal(
      const ::apollo::control::ControlCommand& command) override;

  void ResetProtocol();
  bool CheckChassisError();
  
  void SetFaultLevel(bool has_error_level, bool has_error_dtc, int32_t error_level, int32_t dtc) override;
  void MaxSpeedLimit(double limit) override;
  
  void SetAdStatusReq(int32_t request) override;

 private:
  void SecurityDogThreadFunc();
  virtual bool CheckResponse(const int32_t flags, bool need_wait);
  void set_chassis_error_mask(const int32_t mask);
  int32_t chassis_error_mask();
  Chassis::ErrorCode chassis_error_code();
  void set_chassis_error_code(const Chassis::ErrorCode& error_code);
  double steer_adjust(const double steer_angle_target);
  void steer_fault_detect_recover(ChassisDetail& chassis_detail);
  void brake_ctrl_backup(ChassisDetail& chassis_detail);

 private:
  // control protocol
  Adubodyctrl8cff1600* adu_bodyctrl_8cff1600_ = nullptr;
  Adudrivectrl8cff1400* adu_drivectrl_8cff1400_ = nullptr;
  //Adusteeringctrl8cff1500* adu_steeringctrl_8cff1500_ = nullptr;
  Aduautosteering98ffefe8* adu_autosteering_98ffefe8_ = nullptr;
  Adubrakectrl8c04eb0a* adu_brakectrl_8c04eb0a_ = nullptr;

  Chassis chassis_;
  std::unique_ptr<std::thread> thread_;
  bool is_chassis_error_ = false;

  std::mutex chassis_error_code_mutex_;
  Chassis::ErrorCode chassis_error_code_ = Chassis::NO_ERROR;

  std::mutex chassis_mask_mutex_;
  int32_t chassis_error_mask_ = 0;

  // common set to vcu
  float throttle_percentage = 0.0;
  float brake_percentage = 0.0;

  // common return from vcu
  Vcu_vehicleinfo3_98f003d0::Vcu_vehiclepowerstatusType power_status_vcu_return_ = Vcu_vehicleinfo3_98f003d0::VCU_VEHICLEPOWERSTATUS_LV_UP;
  Vcu_vehicleinfo3_98f003d0::Vcu_gearstatusType gear_status_vcu_return_ = Vcu_vehicleinfo3_98f003d0::VCU_GEARSTATUS_NEUTRAL;
  int32_t hand_brake_status_vcu_return_ = 0;
  double steer_angle_asc_return_ = 0.0;

  // top warn lamp
  common::VehicleSignal_TopWarnLampStat top_warn_lamp_pre;
  uint32_t lamp_flash_counter_ = 0;

#ifdef ENABLE_STEER_ADJUST
  double steer_angle_curr_ = 0.0;
  double steer_angle_target_pre_ = 0.0;
  bool is_first_steer = false;
#endif

#ifdef ENABLE_STEER_FAULT_DETECT_AND_RECOVER
  double steer_error_detect_start_time_ = 0.0;
  int32_t steer_error_detect_start_flag_ = 0;
  Chassis::FaultLevel steer_fault_level_ = Chassis::NO_FAILURE;
#endif

#ifdef ENABLE_ADU_BRAKECTRL_BACKUP
  double brake_detect_start_time_ = 0.0;
  int32_t brake_detect_start_flag_ = 0;
  int32_t adu_brake1_air_pressure_ = 0;
  int32_t adu_brake2_air_pressure_ = 0;
#endif
};

}  // namespace ch
}  // namespace canbus
}  // namespace apollo
