#include <cstdint>
#include "modules/manuctrl/manuctrl_component.h"

#include "modules/manuctrl/common/manuctrl_gflags.h"
#include "modules/common/adapters/adapter_gflags.h"
#include "modules/common/util/message_util.h"

#include "modules/manuctrl/proto/manuctrl_conf.pb.h"
#include "modules/canbus/proto/chassis.pb.h"

#include "modules/drivers/joystick/stream/stream.h"
#include "modules/common/time/time.h"

using namespace apollo::drivers::joystick;

namespace apollo {
namespace manuctrl {

using apollo::canbus::Chassis;
using apollo::common::time::Clock;

bool manuctrlComponent::CreateStream(const ManuctrlConf &conf) {
  Stream *s = nullptr;

  if (!conf.serial_port().has_device()) {
    AERROR << "serial_port def has no device field.";
    return false;
  }
  if (!conf.serial_port().has_baud_rate()) {
    AERROR << "serial_port def has no baud_rate field. Use default baud rate "
           << conf.serial_port().baud_rate();
    return false;
  }
  s = Stream::create_serial(conf.serial_port().device().c_str(),
                               conf.serial_port().baud_rate());

  if (s == nullptr) {
    AERROR << "Failed to create data stream.";
    return false;
  }
  data_stream_.reset(s);
  return true;
}

bool manuctrlComponent::slip_decode_add_byte(slip_t *p_slip, uint8_t c) {
  if (p_slip == NULL) {
    return false;
  }

  if (p_slip->cur_idx == p_slip->buf_len) {
    p_slip->state = SLIP_STATE_CLEARING_INVALID_PACKET;
    return false;
  }

  switch (p_slip->state) {
    case SLIP_STATE_DECODING:
      switch (c) {
        case SLIP_BYTE_END:
          // finished reading packet
          return true;

        case SLIP_BYTE_ESC:
          // wait for
          p_slip->state = SLIP_STATE_ESC_RECEIVED;
          break;

        default:
          // add byte to buffer
          p_slip->p_buf[p_slip->cur_idx++] = c;
          break;
      }
        break;

    case SLIP_STATE_ESC_RECEIVED:
      switch (c) {
        case SLIP_BYTE_ESC_END:
          p_slip->p_buf[p_slip->cur_idx++] = SLIP_BYTE_END;
          p_slip->state = SLIP_STATE_DECODING;
          break;

        case SLIP_BYTE_ESC_ESC:
          p_slip->p_buf[p_slip->cur_idx++] = SLIP_BYTE_ESC;
          p_slip->state = SLIP_STATE_DECODING;
          break;

        default:
          // protocol violation
          p_slip->state = SLIP_STATE_CLEARING_INVALID_PACKET;
          return false;
      }
        break;

    case SLIP_STATE_CLEARING_INVALID_PACKET:
      if (c == SLIP_BYTE_END) {
        p_slip->state = SLIP_STATE_DECODING;
        p_slip->cur_idx = 0;
      }
      break;
    }

    return false;
}

void manuctrlComponent::ctrl_vehicle_brake_chg()
{
  float max_brake;

  ctrl.torque = 0;
  
  if (!ctrl.rc_status.key.lb) {
    max_brake = CTRL_BRAKE_MAX;
  } else {
    max_brake = CTRL_BRAKE_DEFAULT;
  }

  if (!ctrl.rc_status.key.lt) {
    if (ctrl.brake >= CTRL_BRAKE_SLOPE) {
      ctrl.brake -= CTRL_BRAKE_SLOPE;
    } else if (ctrl.brake > 0) {
      ctrl.brake = 0;
    }
  } else if (!ctrl.manual_ch || (ctrl.manual_ch && !ctrl.rc_status.key.lb)) {
    if (ctrl.brake <= max_brake - CTRL_BRAKE_SLOPE) {
      ctrl.brake += CTRL_BRAKE_SLOPE;
    } else if (ctrl.brake < max_brake) {
      ctrl.brake = max_brake;
    }
  }
}

void manuctrlComponent::ctrl_vehicle_torque_chg()
{
  float max_torque;

  ctrl.brake = 0;
  
  if (!ctrl.rc_status.key.lb) {
    max_torque = CTRL_TORQUE_MAX;
  } else {
    max_torque = CTRL_TORQUE_DEFAULT;
  }

  if (ctrl.gear != VEHICLE_GEAR_DRIVE
      && ctrl.gear != VEHICLE_GEAR_REVERSE) {
    return;
  }

  if (!ctrl.rc_status.key.lt) {
    if (ctrl.torque >= CTRL_TORQUE_SLOPE) {
      ctrl.torque -= CTRL_TORQUE_SLOPE;
    } else if (ctrl.torque > 0) {
      ctrl.torque = 0;
    }
  } else if (!ctrl.manual_ch || (ctrl.manual_ch && !ctrl.rc_status.key.lb)) {
    if (ctrl.torque <= max_torque - CTRL_TORQUE_SLOPE) {
      ctrl.torque += CTRL_TORQUE_SLOPE;
    } else if (ctrl.torque < max_torque) {
      ctrl.torque = max_torque;
    }
  }
}

bool manuctrlComponent::ctrl_vehicle_ctrl()
{
  if ((!ctrl.rc_status.key.down && ctrl.rc_key_pre_status.down)
        || (!ctrl.rc_status.key.up && ctrl.rc_key_pre_status.up)) {
    ctrl.manual_ch = false;
  }

  if (!ctrl.rc_status.key.lb || !ctrl.rc_status.key.lt) {
    ctrl.manual_ch = true;
  }

  if (!ctrl.rc_status.key.down) {
    ctrl_vehicle_brake_chg();
  } else {
    ctrl.brake = 0;
    if (!ctrl.rc_status.key.up) {
      ctrl_vehicle_torque_chg();
    } else {
      ctrl.torque = 0;
    }
  }

  if (ctrl.rc_status.key.x && !ctrl.rc_key_pre_status.x) {
    if (ctrl.gear == VEHICLE_GEAR_NEUTRAL && ctrl.brake > 0) {
      ctrl.gear = VEHICLE_GEAR_PARK;
    }
  } else if (ctrl.rc_status.key.b && !ctrl.rc_key_pre_status.b) {
    if (ctrl.gear != VEHICLE_GEAR_NEUTRAL && ctrl.brake > 0) {
      ctrl.gear = VEHICLE_GEAR_NEUTRAL;
    }
  } else if (ctrl.rc_status.key.a && !ctrl.rc_key_pre_status.a) {
    if (ctrl.gear == VEHICLE_GEAR_NEUTRAL && ctrl.brake > 0) {
        ctrl.gear = VEHICLE_GEAR_REVERSE;
    }
  } else if (ctrl.rc_status.key.y && !ctrl.rc_key_pre_status.y) {
      if (ctrl.gear == VEHICLE_GEAR_NEUTRAL && ctrl.brake > 0) {
          ctrl.gear = VEHICLE_GEAR_DRIVE;
      }
  }

  if (ctrl.rc_status.rocker.rx < CTRL_ROCKER_MID - CTRL_ROCKER_TRIG_TH) {
    ctrl.steer_angle     = CTRL_STEER_ANGLE_MAX;
    ctrl.steer_speed     = CTRL_STEER_SPEED_CAL((CTRL_ROCKER_MID - CTRL_ROCKER_TRIG_TH) - ctrl.rc_status.rocker.rx);
    ctrl.turn_signal     = common::VehicleSignal::TURN_LEFT;
  } else if (ctrl.rc_status.rocker.rx > CTRL_ROCKER_MID + CTRL_ROCKER_TRIG_TH) {
    ctrl.steer_angle     = -CTRL_STEER_ANGLE_MAX;
    ctrl.steer_speed     = CTRL_STEER_SPEED_CAL(ctrl.rc_status.rocker.rx - (CTRL_ROCKER_MID + CTRL_ROCKER_TRIG_TH));
    ctrl.turn_signal     = common::VehicleSignal::TURN_RIGHT;
  } else {
    if (latest_chassis_.has_steering_percentage()) {
      ctrl.steer_angle = latest_chassis_.steering_percentage();
    } else {
      ctrl.steer_angle = 0;
    }
    ctrl.steer_speed     = CTRL_STEER_SPEED_MIN;
    ctrl.turn_signal     = common::VehicleSignal::TURN_NONE;
  }

  if (!ctrl.rc_status.key.rr) {
    ctrl.steer_angle     = 0;
    ctrl.steer_speed     = CTRL_STEER_SPEED_MIN;
    ctrl.turn_signal     = common::VehicleSignal::TURN_NONE;
  }

  if (ctrl.rc_status.key.rt && !ctrl.rc_key_pre_status.rt) {
    // TODO
  }

  if (ctrl.rc_status.key.rb && !ctrl.rc_key_pre_status.rb) {
    if (ctrl.full_beam_head_lamp == false) {
      ctrl.full_beam_head_lamp = true;
    } else {
      ctrl.full_beam_head_lamp = false;
    }
    if (ctrl.dipped_head_lamp == false) {
      ctrl.dipped_head_lamp = true;
    } else {
      ctrl.dipped_head_lamp = false;
    }
  }

  return true;
}

bool manuctrlComponent::ctrl_signal_proc()
{
  if (!ctrl.rc_status_valid) {
    ctrl.rc_key_pre_status = ctrl.rc_status.key;
    ctrl.rc_status_valid   = true;
    return false;
  }

  if (ctrl.vehicle_status == VEHICLE_STATUS_PWROFF) {
    if (ctrl.rc_key_pre_status.start == 0x0 && 
      ctrl.rc_status.key.start == 0x1) {
      ctrl.vehicle_status = VEHICLE_STATUS_IGNITION;
    }
  } else {
    if (ctrl.rc_key_pre_status.start == 0x0 && 
      ctrl.rc_status.key.start == 0x1) {
      ctrl.vehicle_status = VEHICLE_STATUS_PWROFF;
    } else if (ctrl.rc_key_pre_status.select == 0x0 && 
      ctrl.rc_status.key.select == 0x1) {
      if (ctrl.vehicle_status == VEHICLE_STATUS_IGNITION) {
        ctrl.vehicle_status = VEHICLE_STATUS_AD;
        exiting_ad_ = false;
      } else if (ctrl.vehicle_status == VEHICLE_STATUS_AD) {
        ctrl.vehicle_status = VEHICLE_STATUS_IGNITION;
        exit_ad_start_time_ = Clock::NowInSeconds();
        exiting_ad_ = true;
      }
    }

    if (ctrl.vehicle_status == VEHICLE_STATUS_IGNITION
        || ctrl.vehicle_status == VEHICLE_STATUS_AD) {
      ctrl_vehicle_ctrl();
    }

    if (!ctrl.rc_status.key.rr && !ctrl.rc_status.key.lr) {
        ctrl.fault_level = ADU_LEVEL3_FAULT;
        ctrl.dtc         = 0;
    } else {
        ctrl.fault_level = ADU_NO_FAILURE;
        ctrl.dtc         = 0;
    }
  }
  ctrl.rc_key_pre_status = ctrl.rc_status.key;

  return true;
}

void manuctrlComponent::ReadStream() {
  while (true) {
    size_t length = data_stream_->read(data_buffer_, DATA_BUFFER_SIZE);
    if (length > 0) {
      uint8_t *ptr = data_buffer_;
      while (length > 0) {
        if (slip_decode_add_byte(&slip, *ptr)) {
          /*
          uint32_t l = slip.cur_idx;
          uint8_t *p = slip.p_buf;
          while (l > 0) {
            AERROR << "0x" <<std::hex << static_cast<int>(*p) << std::endl;
            p++;
            l--;
          }
          */
          if (rc_status_parse(&ctrl.rc_status, slip.p_buf, slip.cur_idx)) {
            last_rcved_time_ = Clock::NowInSeconds();
            ctrl_signal_proc();
          }
          slip.cur_idx = 0;
        }
        ptr++;
        length--;
      }
    }
  }
}

void manuctrlComponent::StartStream()
{
  data_thread_ptr_.reset(new std::thread(&manuctrlComponent::ReadStream, this));
}

bool manuctrlComponent::Init()
{
  if (!GetProtoConfig(&manuctrl_conf_)) {
    AERROR << "Unable to load manuctrl conf file: " << ConfigFilePath();
    return false;
  }

  radio_timeout_sec_ = manuctrl_conf_.radio_timeout_sec();
  exit_ad_delay_sec_ = manuctrl_conf_.exit_ad_delay_sec();

  AINFO << "The manuctrl conf file is loaded: " << FLAGS_manuctrl_conf_file;
  ADEBUG << "manuctrl_conf:" << manuctrl_conf_.ShortDebugString();

  manuctrl_writer_ = node_->CreateWriter<manuctrlMsg>(FLAGS_manuctrl_topic);
  control_cmd_writer_ = node_->CreateWriter<apollo::control::ControlCommand>(FLAGS_control_command_topic);

  cyber::ReaderConfig chassis_reader_config;
  chassis_reader_config.channel_name = FLAGS_chassis_topic;
  chassis_reader_config.pending_queue_size = 10;

  chassis_reader_ =
      node_->CreateReader<apollo::canbus::Chassis>(chassis_reader_config, nullptr);
  CHECK(chassis_reader_ != nullptr);

  memset(&ctrl, 0, sizeof(ctrl));
  ctrl.vehicle_status = VEHICLE_STATUS_IGNITION;
  ctrl.fault_level = ADU_NO_FAILURE;
  ctrl.gear = VEHICLE_GEAR_NEUTRAL;

  if (!CreateStream(manuctrl_conf_)) {
    AERROR << "Init manuctrl stream failed";
    return false;
  }
  StartStream();

  return true;
}

void manuctrlComponent::OnChassis(const std::shared_ptr<Chassis> &chassis) {
  ADEBUG << "Received chassis data: run chassis callback.";
  std::lock_guard<std::mutex> lock(mutex_);
  latest_chassis_.CopyFrom(*chassis);
}

bool manuctrlComponent::Proc()
{
  chassis_reader_->Observe();
  const auto &chassis_msg = chassis_reader_->GetLatestObserved();
  if (chassis_msg != nullptr) {
    OnChassis(chassis_msg);
  }
  
  bool signal_lost = true;

  if (Clock::NowInSeconds() - last_rcved_time_ < radio_timeout_sec_) {
    signal_lost = false;
  }

  if (exiting_ad_) {
    if (Clock::NowInSeconds() - exit_ad_start_time_ > exit_ad_delay_sec_) {
      exiting_ad_ = false;
    }
  }
  
  apollo::canbus::Chassis::GearPosition gear_position;
  switch (ctrl.gear) {
    case VEHICLE_GEAR_NEUTRAL:
      gear_position = apollo::canbus::Chassis::GEAR_NEUTRAL;
      break;

    case VEHICLE_GEAR_DRIVE:
      gear_position = apollo::canbus::Chassis::GEAR_DRIVE;
      break;

    case VEHICLE_GEAR_REVERSE:
      gear_position = apollo::canbus::Chassis::GEAR_REVERSE;
      break;

    case VEHICLE_GEAR_PARK:
      gear_position = apollo::canbus::Chassis::GEAR_PARKING;
      break;
    
    default:
      gear_position = apollo::canbus::Chassis::GEAR_INVALID;
      break;
  }
  /*
  apollo::control::ControlCommand control_command;
  common::util::FillHeader(node_->Name(), &control_command);
  if (ctrl.vehicle_status == VEHICLE_STATUS_AD) {
    control_cmd_writer_->Write(control_command);
  }
  control_command.set_throttle(ctrl.torque);
  control_command.set_brake(ctrl.brake);
  control_command.set_gear_location(gear_position);
  control_command.set_steering_target(ctrl.steer_angle);
  control_command.set_steering_rate(ctrl.steer_speed);
  */
  manuctrl::manuctrlMsg manuctrl_msg;
  common::util::FillHeader(node_->Name(), &manuctrl_msg);
  if (signal_lost) {
    manuctrl_msg.mutable_control_command()->set_throttle(0);
    manuctrl_msg.mutable_control_command()->set_brake(30);
    manuctrl_msg.mutable_control_command()->mutable_signal()->set_warning_lamp(true);
  } else {
    manuctrl_msg.mutable_control_command()->set_throttle(ctrl.torque);
    manuctrl_msg.mutable_control_command()->set_brake(ctrl.brake);
    manuctrl_msg.mutable_control_command()->mutable_signal()->set_warning_lamp(false);
  }
  manuctrl_msg.mutable_control_command()->set_gear_location(gear_position);
  manuctrl_msg.mutable_control_command()->set_steering_target(ctrl.steer_angle);
  manuctrl_msg.mutable_control_command()->set_steering_rate(ctrl.steer_speed);
  manuctrl_msg.mutable_control_command()->set_error_level(ctrl.fault_level);
  manuctrl_msg.mutable_control_command()->mutable_signal()->set_turn_signal(ctrl.turn_signal);
  manuctrl_msg.mutable_control_command()->mutable_signal()->set_high_beam(ctrl.full_beam_head_lamp);
  manuctrl_msg.mutable_control_command()->mutable_signal()->set_low_beam(ctrl.dipped_head_lamp);
  if (ctrl.vehicle_status == VEHICLE_STATUS_AD) {
    manuctrl_msg.mutable_control_command()->mutable_signal()->set_position_lamp(true);
  } else {
    manuctrl_msg.mutable_control_command()->mutable_signal()->set_position_lamp(false);
  }
  manuctrl_msg.set_rc_voltage(ctrl.rc_status.voltage);

  if (ctrl.vehicle_status == VEHICLE_STATUS_AD || ctrl.fault_level != 0 || exiting_ad_) {
    manuctrl_msg.set_flag(true);
  } else {
    manuctrl_msg.set_flag(false);
  }
  manuctrl_writer_->Write(manuctrl_msg);

  return true;
}

bool manuctrlComponent::rc_status_parse(rc_status_t *p_status, uint8_t const *p_raw_data, size_t len)
{
  memcpy(&p_status->key, p_raw_data, sizeof(rc_key_status_t));
  p_raw_data += sizeof(rc_key_status_t);
  memcpy(&p_status->rocker, p_raw_data, sizeof(rc_rocker_status_t));
  p_raw_data += sizeof(rc_rocker_status_t);
  memcpy(&p_status->voltage, p_raw_data, sizeof(uint16_t));
  p_raw_data += sizeof(uint16_t);
  
  if (p_status->rocker.lx > 100
      || p_status->rocker.ly > 100
      || p_status->rocker.rx > 100
      || p_status->rocker.ry > 100) {
    return false;
  }
  return true;
}

}  // namespace manuctrl
}  // namespace apollo
