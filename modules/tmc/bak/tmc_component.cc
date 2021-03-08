#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <stdbool.h>
#include <string.h>
#include <cstdint>
#include <sys/types.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <netinet/in.h>

#include "modules/tmc/tmc_component.h"

#include "modules/tmc/common/tmc_gflags.h"
#include "modules/common/adapters/adapter_gflags.h"
#include "modules/common/util/message_util.h"
#include "modules/tmc/proto/tmc_conf.pb.h"
#include "modules/canbus/proto/chassis.pb.h"

#include "modules/common/time/time.h"

namespace apollo {
namespace tmc {

using apollo::canbus::Chassis;
using apollo::drivers::ContiRadar;
using apollo::drivers::Ultrasonic;
using apollo::common::time::Clock;

bool tmcComponent::Init()
{
  if (!GetProtoConfig(&tmc_conf_)) {
    AERROR << "Unable to load tmc conf file: " << ConfigFilePath();
    return false;
  }

  AERROR << "struct mwrada size = " << sizeof(struct mwradar_obs_s);

  AINFO << "The tmc conf file is loaded: " << FLAGS_tmc_conf_file;
  ADEBUG << "tmc_conf:" << tmc_conf_.ShortDebugString();

  tmc_writer_ = node_->CreateWriter<tmcMsg>(FLAGS_tmc_topic);
  rc_pkt_timeout_sec_ = tmc_conf_.rc_signal_timeout_sec();

  cyber::ReaderConfig chassis_reader_config;
  chassis_reader_config.channel_name = FLAGS_chassis_topic;
  chassis_reader_config.pending_queue_size = 10;
  chassis_reader_ =
      node_->CreateReader<apollo::canbus::Chassis>(chassis_reader_config, nullptr);
  CHECK(chassis_reader_ != nullptr);

  cyber::ReaderConfig mwradar_reader_config;
  mwradar_reader_config.channel_name = FLAGS_fused_radar_topic;
  mwradar_reader_config.pending_queue_size = 10;
  mwradar_reader_ =
      node_->CreateReader<ContiRadar>(mwradar_reader_config, nullptr);
  CHECK(mwradar_reader_ != nullptr);

  cyber::ReaderConfig utradar_reader_config;
  utradar_reader_config.channel_name = FLAGS_ultrasonic_radar_topic;
  utradar_reader_config.pending_queue_size = 10;
  utradar_reader_ =
      node_->CreateReader<Ultrasonic>(utradar_reader_config, nullptr);
  CHECK(utradar_reader_ != nullptr);

  cyber::ReaderConfig guardian_reader_config;
  guardian_reader_config.channel_name = FLAGS_guardian_topic;
  guardian_reader_config.pending_queue_size = 10;
  guardian_reader_ =
      node_->CreateReader<apollo::guardian::GuardianCommand>(guardian_reader_config, nullptr);
  CHECK(guardian_reader_ != nullptr);

  StartRcSocket();
  StartCtrlSocket();

  return true;
}

void tmcComponent::HexDump(uint8_t *buf, int32_t len)
{
  int32_t digits_per_line = 16;
  int32_t cur_digits = 0;
  char print_buf[digits_per_line * 3];

  for (int32_t i = 0; i < len; i++) {
    sprintf(print_buf + cur_digits * 3, "%02X ", buf[i]);
    if (++cur_digits == digits_per_line) {
      cur_digits = 0;
      print_buf[cur_digits * 3 - 1] = '\0';
      AERROR << print_buf << std::endl;
    }
  }
  if (cur_digits > 0) {
    print_buf[cur_digits * 3 - 1] = '\0';
    AERROR << print_buf << std::endl;
  }
}

uint8_t tmcComponent::GetUint8_t(const uint8_t *p_data, int32_t *idx)
{
  uint8_t value;
  if (p_data == NULL || idx == NULL) {
    AERROR << "Invalid pointer";
    return 0;
  }
  value = *(p_data + *idx);
  *idx += sizeof(value);
  return value;
}

uint16_t tmcComponent::GetUint16_t(const uint8_t *p_data, int32_t *idx)
{
  uint16_t value;
  if (p_data == NULL || idx == NULL) {
    AERROR << "Invalid pointer";
    return 0;
  }
  memcpy(&value, p_data + *idx, sizeof(value));
  *idx += sizeof(value);
  value = ntohs(value);

  return value;
}

uint32_t tmcComponent::GetUint32_t(const uint8_t *p_data, int32_t *idx)
{
  uint32_t value;
  if (p_data == NULL || idx == NULL) {
    AERROR << "Invalid pointer";
    return 0;
  }
  memcpy(&value, p_data + *idx, sizeof(value));
  *idx += sizeof(value);
  value = ntohl(value);

  return value;
}

void tmcComponent::SetUint8_t(uint8_t value, uint8_t *p_data, int32_t *idx)
{
  if (p_data == NULL || idx == NULL) {
    AERROR << "Invalid pointer";
    return;
  }
  *(p_data + *idx) = value;
  *idx += sizeof(value);
}

void tmcComponent::SetUint16_t(uint16_t value, uint8_t *p_data, int32_t *idx)
{
  if (p_data == NULL || idx == NULL) {
    AERROR << "Invalid pointer";
    return;
  }
  value = htons(value);
  memcpy(p_data + *idx, &value, sizeof(value));
  *idx += sizeof(value);
}

void tmcComponent::SetUint32_t(uint32_t value, uint8_t *p_data, int32_t *idx)
{
  if (p_data == NULL || idx == NULL) {
    AERROR << "Invalid pointer";
    return;
  }
  value = htonl(value);
  memcpy(p_data + *idx, &value, sizeof(value));
  *idx += sizeof(value);
}

int16_t tmcComponent::GetInt16_t(const uint8_t *p_data, int32_t *idx)
{
  int16_t value;
  if (p_data == NULL) {
    AERROR << "Invalid pointer";
    return 0;
  }
  memcpy(&value, p_data + *idx, sizeof(value));
  *idx += sizeof(value);
  value = ntohs(value);

  return value;
}

bool tmcComponent::ParseRcStream(const uint8_t *p_data, int32_t len)
{
  int32_t msg_id;
  int32_t msg_len;
  int32_t idx = 0;
  if (p_data == NULL) {
    AERROR << "Invalid buffer pointer";
    return false;
  }
  // Parse head
  if (len < 8) {
    AERROR << "Insufficient buffer lengh";
    return false;
  }
  len -= 8;
  uint8_t version = GetUint8_t(p_data, &idx);
  if (version != proto_ver_) {
    AERROR << "Protocol version mismatch";
    return false;
  }
  rc_status_ = GetUint8_t(p_data, &idx);  
  rc_id_ = GetUint8_t(p_data, &idx);
  enable_aebs_ = GetUint8_t(p_data, &idx);

  uint16_t seq = GetUint16_t(p_data, &idx);
  (void)seq;

  uint16_t service_id = GetUint16_t(p_data, &idx);
  // TODO
  if (service_id != rc_ctrl_id_) {
  //  AERROR << "Service ID mismatch";
  //  return false;
  }
  // Parse ADU drive control, ID 14
  if (len < 16) {
    AERROR << "Insufficient buffer lengh";
    return false;
  }
  len -= 16;

  msg_id = GetUint8_t(p_data, &idx);
  if (msg_id != 14) {
    AERROR << "Unexpected msg ID: 0x" << std::hex << (int32_t)msg_id;
    return false;
  }
  msg_len = GetUint16_t(p_data, &idx);
  if (msg_len != 13) {
    AERROR << "Unexpected msg length";
    return false;
  }
  ad_status_req_ = GetUint8_t(p_data, &idx);
  uint8_t gear = GetUint8_t(p_data, &idx);
  
  switch (gear) {
    case 0:
      ad_gear_req_ = apollo::canbus::Chassis::GEAR_NEUTRAL;
      break;

    case 1:
      ad_gear_req_ = apollo::canbus::Chassis::GEAR_DRIVE;
      break;

    case 2:
      ad_gear_req_ = apollo::canbus::Chassis::GEAR_REVERSE;
      break;

    case 3:
      ad_gear_req_ = apollo::canbus::Chassis::GEAR_PARKING;
      break;
    
    default:
      ad_gear_req_ = apollo::canbus::Chassis::GEAR_INVALID;
      break;
  }

  uint16_t torque = GetUint16_t(p_data, &idx);
  if (torque > 100) {
    torque = 100;
  }
  ad_torque_req_ = torque;
  ad_max_speed_limit_ = GetUint16_t(p_data, &idx);
  ad_vcu_sw_req_ = GetUint8_t(p_data, &idx);
  ad_brake_req_ = GetUint8_t(p_data, &idx);

  uint16_t brake = GetUint16_t(p_data, &idx);
  if (brake > 100) {
    brake = 100;
  }
  ad_brake_percent_ = brake;
  ad_epb_ = GetUint8_t(p_data, &idx);
  ad_status_ = GetUint8_t(p_data, &idx);
  ad_dtc_ = GetUint8_t(p_data, &idx);

  printf("ad_brake_req_ = %d,  ad_brake_percent_ = %d !\r\n", ad_brake_req_, ad_brake_percent_);

  // Parse ADU steer control, ID 15
  if (len < 11) {
    AERROR << "Insufficient buffer lengh";
    return false;
  }
  len -= 11;

  msg_id = GetUint8_t(p_data, &idx);
  if (msg_id != 15) {
    AERROR << "Unexpected msg ID: 0x" << std::hex << (int32_t)msg_id;
    return false;
  }
  msg_len = GetUint16_t(p_data, &idx);
  if (msg_len != 8) {
    AERROR << "Unexpected msg length";
    return false;
  }
  
  ad_turn_angle_ = (double)GetInt16_t(p_data, &idx) *100 / 900;
  ad_work_mode_ = GetUint8_t(p_data, &idx);
  ad_adu_status_ = GetUint8_t(p_data, &idx);
  ad_turn_cmd_valid_ = GetUint8_t(p_data, &idx);
  ad_turn_added_torque_ = GetUint8_t(p_data, &idx);
  ad_wheel_speed_ = GetUint16_t(p_data, &idx);

  // Parse ADU lamp control, ID 16
  if (len < 11) {
    AERROR << "Insufficient buffer lengh";
    return false;
  }
  len -= 11;

  msg_id = GetUint8_t(p_data, &idx);
  if (msg_id != 16) {
    AERROR << "Unexpected msg ID: 0x" << std::hex << (int32_t)msg_id;
    return false;
  }
  msg_len = GetUint16_t(p_data, &idx);
  if (msg_len != 8) {
    AERROR << "Unexpected msg length";
    return false;
  }
  ad_warning_lamp_req_ = GetUint8_t(p_data, &idx) ? true : false;
  ad_left_turn_lamp_req_ = GetUint8_t(p_data, &idx) ? true : false;
  ad_right_turn_lamp_req_ = GetUint8_t(p_data, &idx) ? true : false;
  ad_position_lamp_req_ = GetUint8_t(p_data, &idx) ? true : false;
  ad_dipped_lamp_req_ = GetUint8_t(p_data, &idx) ? true : false;
  ad_full_beam_lamp_req_ = GetUint8_t(p_data, &idx) ? true : false;
  ad_front_fog_lamp_req_ = GetUint8_t(p_data, &idx) ? true : false;
  ad_rear_fog_lamp_req_ = GetUint8_t(p_data, &idx) ? true : false;

  if (ad_left_turn_lamp_req_) {
    ad_turn_signal_ = common::VehicleSignal::TURN_LEFT;
  } else if (ad_right_turn_lamp_req_) {
    ad_turn_signal_ = common::VehicleSignal::TURN_RIGHT;
  } else {
    ad_turn_signal_ = common::VehicleSignal::TURN_NONE;
  }

  return true;
}

void tmcComponent::RcReadStream()
{
  int dgram_len;
  uint8_t rx_buf[4096];
  double cur_time;

  if (rc_sock_fd_ < 0) {
    AERROR << "Socket fd invalid";
    return;
  }

  while (1) {
    dgram_len = recvfrom(rc_sock_fd_, 
                         rx_buf, 
                         sizeof(rx_buf), 
                         MSG_WAITALL, 
                         (struct sockaddr *)&rc_client_addr_, 
                         &rc_client_addr_len_);
    
    cur_time = Clock::NowInSeconds();

    if (!rc_enable_) {
      continue;
    }

    if (dgram_len > 0) {
      //HexDump(rx_buf, dgram_len);
      if (ParseRcStream(rx_buf, dgram_len)) {
        if (rc_rcvd_first_pkt_) {
          rc_pkt_interval_sec_ = cur_time - last_rc_pkt_rcvd_time_;
        } else {
          rc_pkt_interval_sec_ = rc_pkt_period_sec_;
          rc_rcvd_first_pkt_ = true;
        }
        if (fabs(rc_pkt_interval_sec_ - rc_pkt_period_sec_) > 0.03) {
          RcFbTx();
        }
        last_rc_pkt_rcvd_time_ = cur_time;
      }
    }
  }
}

void tmcComponent::RcFbTx()
{
  uint8_t tx_buf[1024];
  int32_t tx_len;
  rc_fb_seq_++;
  tx_len = sizeof(tx_buf);
  RcFbPktAssemble(tx_buf, &tx_len);
  sendto(rc_sock_fd_, tx_buf, tx_len, 
        MSG_CONFIRM, (const struct sockaddr *) &rc_client_addr_,  
        rc_client_addr_len_); 
}

void tmcComponent::RcFbStream()
{
  while (1) {
    if (rc_enable_ && rc_rcvd_first_pkt_) {
      RcFbTx();
    }
    usleep(rc_fb_pkt_period_sec_ * 1000 * 1000);
  }
}

bool tmcComponent::StartRcSocket()
{
  rc_sock_fd_ = socket(AF_INET, SOCK_DGRAM, 0);
    if (rc_sock_fd_ < 0) {
      AERROR << "Socket creation failed";
      return false;
  }

  struct sockaddr_in server_addr;
  server_addr.sin_family      = AF_INET;
  server_addr.sin_addr.s_addr = INADDR_ANY;
  server_addr.sin_port        = htons(tmc_conf_.rc_udp_port());
  if (bind(rc_sock_fd_, (struct sockaddr *)&server_addr, sizeof(server_addr)) < 0) {
    AERROR << "Bind failed";
    rc_sock_fd_ = -1;
    return false;
  }

  rc_rxthread_ptr_.reset(new std::thread(&tmcComponent::RcReadStream, this));
  rc_txthread_ptr_.reset(new std::thread(&tmcComponent::RcFbStream, this));
  return true;
}

bool tmcComponent::StopRcSocket()
{
  close(rc_sock_fd_);
  rc_sock_fd_ = -1;
  return true;
}

void tmcComponent::AssembleMWObs(uint8_t *buf, int32_t *p_idx, int32_t *p_len)
{
  if (buf == NULL || p_idx == NULL || p_len == NULL) {
    AERROR << "Invalid pointer";
    return;
  }
  int32_t obs_data_len = mwradar_obs_cnt_ * sizeof(struct mwradar_obs_s);
  if (*p_len < 3 + obs_data_len) {
    return;
  }
  *p_len -= (3 + obs_data_len);
  SetUint8_t(11, buf, p_idx);
  SetUint16_t(obs_data_len, buf, p_idx);
  if (obs_data_len > 0) {
    memcpy(buf + *p_idx, mwradar_obs_list_, obs_data_len);
    *p_idx += obs_data_len;
  }
}

void tmcComponent::AssembleUTObs(uint8_t *buf, int32_t *p_idx, int32_t *p_len)
{
  if (buf == NULL || p_idx == NULL || p_len == NULL) {
    AERROR << "Invalid pointer";
    return;
  }
  int32_t obs_data_len = utradar_obs_cnt_ * sizeof(struct utradar_obs_s);
  if (*p_len < 3 + obs_data_len) {
    return;
  }
  *p_len -= (3 + obs_data_len);
  SetUint8_t(10, buf, p_idx);
  SetUint16_t(obs_data_len, buf, p_idx);
  if (obs_data_len > 0) {
    memcpy(buf + *p_idx, utradar_obs_list_, obs_data_len);
    *p_idx += obs_data_len;
  }
}

bool tmcComponent::AbstractMWObs()
{
  //if (!rcvd_mwradar_msg_) {
  //  return false;
  //}
  std::lock_guard<std::mutex> lock(mutex_);
  mwradar_obs_cnt_ = 0;
  for (int32_t num = 0; num < MWRADAR_NUM; num++) {
    for (int32_t i = 0; i < latest_mwradar_[num].contiobs_size(); i++) {
      if (mwradar_obs_cnt_ == MAX_MWRADAR_OBS) {
        break;
      }
      const auto& contiobs = latest_mwradar_[num].contiobs(i);
      if (contiobs.pos_x() < 10 && contiobs.pos_y() < 10 && contiobs.pos_z() < 10) {
        mwradar_obs_list_[mwradar_obs_cnt_].pos_x = contiobs.pos_x() * 10;
        mwradar_obs_list_[mwradar_obs_cnt_].pos_y = contiobs.pos_y() * 10;
        mwradar_obs_list_[mwradar_obs_cnt_].pos_z = contiobs.pos_z() * 10;
        mwradar_obs_list_[mwradar_obs_cnt_].vel_x 
            = contiobs.longitude_vel() < 120 ? contiobs.longitude_vel() : 120;
        mwradar_obs_list_[mwradar_obs_cnt_].pos_z 
            = contiobs.lateral_vel() < 120 ? contiobs.lateral_vel() : 120;
        mwradar_obs_cnt_++;
      }
    }
  }
  
  return mwradar_obs_cnt_ > 0 ? true : false;
}

bool tmcComponent::AbstractUTObs()
{
  std::lock_guard<std::mutex> lock(mutex_);
  utradar_obs_cnt_ = 0;

  for (int32_t nIndex = 0; nIndex < latest_utradar_.ranges_size(); nIndex++) 
  {
       if(nIndex >= MAX_UTRADAR_OBS)
       {
             break;
       }

      const auto conutobs = latest_utradar_.ranges(nIndex);
      utradar_obs_list_[utradar_obs_cnt_].nID = nIndex;
      utradar_obs_list_[utradar_obs_cnt_].nDistance = conutobs * 100;
      utradar_obs_cnt_++;
  }

  return utradar_obs_cnt_ > 0 ? true : false;
}

bool tmcComponent::RcFbPktAssemble(uint8_t *buf, int32_t *p_len)
{
  if (buf == NULL) {
    AERROR << "Invalid buffer pointer";
    return false;
  }
  // Add head
  int32_t idx = 0;
  int32_t len = *p_len;
  if (len < 8) {
    AERROR << "Insufficient buffer lengh";
    return false;
  }
  len -= 8;

  switch(latest_guardian_.command_from())
  {
	  case guardian::MANU_CTRL: rc_id_ = 2;  break;
	  case guardian::REMOTE_CTRL: rc_id_ = 1;  break;
    case guardian::AEBS: rc_id_ = 3;  break;
	  default: rc_id_ = 0;   break;
  }

  SetUint8_t(proto_ver_, buf, &idx);
  SetUint8_t(rc_status_, buf, &idx);
  SetUint8_t(rc_id_, buf, &idx);
  SetUint8_t(0x0, buf, &idx);
  SetUint16_t(rc_fb_seq_, buf, &idx);
  SetUint16_t(rc_ctrl_id_, buf, &idx);

  // Add IP status
  if (len < 8) {
    AERROR << "Insufficient buffer lengh";
    return false;
  }
  len -= 8;
  SetUint8_t(1, buf, &idx);
  SetUint16_t(5, buf, &idx);
  SetUint16_t(rc_pkt_interval_sec_ * 1000, buf, &idx);
  SetUint8_t(cpu_util_percent_, buf, &idx);
  SetUint8_t(mem_util_percent_, buf, &idx);
  SetUint8_t(cpu_temp_, buf, &idx);
  
  // Add error code
  if (len < 5) {
    AERROR << "Insufficient buffer lengh";
    return false;
  }
  len -= 5;
  SetUint8_t(2, buf, &idx);
  SetUint16_t(2, buf, &idx);
  SetUint8_t(dtc_, buf, &idx);
  SetUint8_t(status_, buf, &idx);

// Add radar obs
  AbstractUTObs();
  AssembleUTObs(buf, &idx, &len);

  // Add radar obs
  AbstractMWObs();
  AssembleMWObs(buf, &idx, &len);
  
  // Add MD5
  if (len < 16) {
    AERROR << "Insufficient buffer lengh";
    return false;
  }
  len -= 16;
  memset(buf + idx, 0, 16);
  idx += 16;

  *p_len = idx;
  return true;
}

bool tmcComponent::ParseCtrlPkt(const uint8_t *p_data, int32_t len)
{
  int32_t idx = 0;
  if (p_data == NULL) {
    AERROR << "Invalid buffer pointer";
    return false;
  }
  // Parse head
  if (len < 31) {
    AERROR << "Insufficient buffer lengh";
    return false;
  }
  len -= 31;

  uint8_t version = GetUint8_t(p_data, &idx);
  if (version != proto_ver_) {
    AERROR << "Protocol version mismatch";
    return false;
  }
  rcvd_ctrl_pkt_type_ = GetUint8_t(p_data, &idx);

  rc_ctrl_id_ = GetUint16_t(p_data, &idx);
  GetUint16_t(p_data, &idx);
  // Chassis data sender IP/port
  GetUint32_t(p_data, &idx);
  GetUint16_t(p_data, &idx);
  // Chassis data receiver IP/port
  chassis_dest_ip_ = GetUint32_t(p_data, &idx);
  chassis_dest_port_ = GetUint16_t(p_data, &idx);
  // Chassis data protocol
  GetUint8_t(p_data, &idx);
  // IPC remote control IP/port
  GetUint32_t(p_data, &idx);
  GetUint16_t(p_data, &idx);
  // IPC remote control protocol
  GetUint8_t(p_data, &idx);
  rc_pkt_period_sec_ = (double)GetUint16_t(p_data, &idx) / 1000;
  rc_fb_pkt_period_sec_ = (double)GetUint16_t(p_data, &idx) / 1000;
  rc_fb_pkt_period_sec_ = (rc_fb_pkt_period_sec_ < 0.02) ? 0.02 : rc_fb_pkt_period_sec_;
  // Error code
  GetUint8_t(p_data, &idx);

  return true;
}

bool tmcComponent::CtrlRespPktAssemble(uint8_t *buf, int32_t *p_len)
{
  if (buf == NULL) {
    AERROR << "Invalid buffer pointer";
    return false;
  }
  // Add head
  int32_t idx = 0;
  int32_t len = *p_len;
  if (len < 31) {
    AERROR << "Insufficient buffer lengh";
    return false;
  }
  len -= 31;
  SetUint8_t(proto_ver_, buf, &idx);
  SetUint8_t(rcvd_ctrl_pkt_type_ + 1, buf, &idx);
  SetUint16_t(rc_ctrl_id_, buf, &idx);
  SetUint16_t(rc_fb_id_, buf, &idx);
  SetUint32_t(ntohl(inet_addr("10.28.63.36")), buf, &idx);
  //SetUint32_t(ntohl(inet_addr("10.28.76.231")), buf, &idx);
  SetUint16_t(10002, buf, &idx);
  SetUint32_t(chassis_dest_ip_, buf, &idx);
  SetUint16_t(chassis_dest_port_, buf, &idx);
  SetUint8_t(17, buf, &idx);
  SetUint32_t(ntohl(inet_addr("10.28.63.36")), buf, &idx);
  //SetUint32_t(ntohl(inet_addr("10.28.76.231")), buf, &idx);
  SetUint16_t(tmc_conf_.rc_udp_port(), buf, &idx);
  SetUint8_t(17, buf, &idx);
  SetUint16_t(rc_pkt_period_sec_ * 1000, buf, &idx);
  SetUint16_t(rc_fb_pkt_period_sec_ * 1000, buf, &idx);
  SetUint8_t(0, buf, &idx);

  *p_len = idx;
  return true;
}

void tmcComponent::OnRcvdCtrlStream()
{
  uint8_t tx_buf[1024];
  int32_t tx_len = sizeof(tx_buf);
  // 0: Remote control start request
  // 1: Remote control start response
  // 2: Heartbeat request
  // 3: Heartbeat response
  // 4: Remote control stop request
  // 5: Remote control stop response
  if (rcvd_ctrl_pkt_type_ == 0) {
    rc_enable_ = true;
    rc_rcvd_first_pkt_ = false;
    rc_fb_seq_ = 0;
    msgs_cnt_last_ = 0;
  } else if (rcvd_ctrl_pkt_type_ == 4) {
    rc_enable_ = false;
    msgs_cnt_last_ = 3;
    ad_status_ = 0;
    ad_dtc_ = 0;
  }

  CtrlRespPktAssemble(tx_buf, &tx_len);
  send(ctrl_csock_fd_, tx_buf, tx_len, 0);
}

void tmcComponent::CtrlStreamDaemon()
{
  struct sockaddr_in addr;
  socklen_t addr_len = sizeof(addr);
  if (ctrl_lsock_fd_ < 0) {
    AERROR << "Socket fd invalid";
    return;
  }
  while (1) {
    ctrl_csock_fd_ = accept(ctrl_lsock_fd_, (struct sockaddr *)&addr, &addr_len);
    if (ctrl_csock_fd_ < 0) {
      AERROR << "Accept socket failed";
      return;
    }
    
    uint8_t rx_buf[1024];
    int32_t rx_len;

    while (1) {
      rx_len = read(ctrl_csock_fd_, rx_buf, sizeof(rx_buf));
      
      if (rx_len > 0) {
        //HexDump(buf, len);
        if (ParseCtrlPkt(rx_buf, rx_len)) {
          OnRcvdCtrlStream();
        }
      } else {
        rc_enable_ = false;
        msgs_cnt_last_ = 3;
        AERROR << "Connection was broken, reconnect needed";
        break;
      }
    }
  }
}

bool tmcComponent::StartCtrlSocket()
{
  
  ctrl_lsock_fd_ = socket(AF_INET, SOCK_STREAM, 0);
    if (ctrl_lsock_fd_ < 0) {
      AERROR << "Socket creation failed";
      return false;
  }

  int32_t opt = 1;
  if (setsockopt(ctrl_lsock_fd_, SOL_SOCKET, SO_REUSEADDR | SO_REUSEPORT, &opt, sizeof(opt))) { 
    AERROR << "Set socket failed";
    return false;
  }

  struct sockaddr_in server_addr;
  server_addr.sin_family      = AF_INET;
  server_addr.sin_addr.s_addr = INADDR_ANY;
  server_addr.sin_port        = htons(tmc_conf_.ctrl_tcp_port());
  if (bind(ctrl_lsock_fd_, (struct sockaddr *)&server_addr, sizeof(server_addr)) < 0) {
    AERROR << "Bind failed";
    ctrl_lsock_fd_ = -1;
    return false;
  }

  if (listen(ctrl_lsock_fd_, 1) < 0) {
    AERROR << "Listen socket failed";
    return false;
  }

  ctrl_thread_ptr_.reset(new std::thread(&tmcComponent::CtrlStreamDaemon, this));
  return true;
}

void tmcComponent::OnChassis(const std::shared_ptr<Chassis> &chassis)
{
  ADEBUG << "Received chassis data: run chassis callback.";
  std::lock_guard<std::mutex> lock(mutex_);
  latest_chassis_.CopyFrom(*chassis);
}

void tmcComponent::OnMWradar(const std::shared_ptr<ContiRadar> &mwradar)
{
  ADEBUG << "Received mwradar data: run mwradar callback.";
  std::lock_guard<std::mutex> lock(mutex_);
  latest_mwradar_[mwradar->radar_id()].CopyFrom(*mwradar);
  //rcvd_mwradar_msg_ = true;
}
void tmcComponent::OnUTradar(const std::shared_ptr<Ultrasonic> &utradar)
{
  ADEBUG << "Received mwradar data: run mwradar callback.";
  std::lock_guard<std::mutex> lock(mutex_);
  latest_utradar_.CopyFrom(*utradar);
}

void tmcComponent::OnGuardian(const std::shared_ptr<guardian::GuardianCommand> &guardian)
{
  ADEBUG << "Received guardian data: run guardian callback.";
  std::lock_guard<std::mutex> lock(mutex_);
  latest_guardian_.CopyFrom(*guardian);
}

void tmcComponent::PublishMsg()
{
  tmcMsg msg;
  common::util::FillHeader(node_->Name(), &msg);

  bool signal_lost = true;
  if (rc_enable_ && rc_rcvd_first_pkt_) {
    if (Clock::NowInSeconds() - last_rc_pkt_rcvd_time_ < rc_pkt_timeout_sec_) {
      signal_lost = false;
    }
  } else if (!rc_enable_) {
    signal_lost = false;
  }

  if (signal_lost) {
    msg.mutable_control_command()->set_throttle(0);
    msg.mutable_control_command()->set_brake(30);
    msg.mutable_control_command()->mutable_signal()->set_warning_lamp(true);
  } else {
    msg.mutable_control_command()->set_throttle(ad_torque_req_);
    msg.mutable_control_command()->set_brake(ad_brake_percent_);
    msg.mutable_control_command()->mutable_signal()->set_warning_lamp(false);
  }

  msg.mutable_control_command()->set_adstatusreq(ad_status_req_);

  msg.mutable_control_command()->set_gear_location(ad_gear_req_);
  msg.mutable_control_command()->set_steering_target(ad_turn_angle_);
  msg.mutable_control_command()->set_steering_rate(ad_wheel_speed_);
  msg.mutable_control_command()->set_error_level(ad_status_);
  msg.mutable_control_command()->set_error_dtc(ad_dtc_);
  msg.mutable_control_command()->mutable_signal()->set_turn_signal(ad_turn_signal_);
  msg.mutable_control_command()->mutable_signal()->set_high_beam(ad_full_beam_lamp_req_);
  msg.mutable_control_command()->mutable_signal()->set_low_beam(ad_dipped_lamp_req_);

  if (rc_enable_ && rc_rcvd_first_pkt_) {
    if(rc_status_ == 0x01)
    {
      msgs_cnt_last_ = 3;
       msg.mutable_control_command()->mutable_signal()->set_position_lamp(true);
       msg.set_flag(true);
    }
    else
    {
       if(ad_brake_req_ == 0x01 && ad_brake_percent_ > 10)
       {
	  msgs_cnt_last_ = 3;
          msg.mutable_control_command()->mutable_signal()->set_position_lamp(true);
          msg.set_flag(true);
       }
       else
       {
          msg.mutable_control_command()->mutable_signal()->set_position_lamp(false);
         msg.set_flag(false);
       }
    }
  } else {
    msg.mutable_control_command()->mutable_signal()->set_position_lamp(false);
    msg.set_flag(false);
  }

  if (msgs_cnt_last_ > 0) {
    msgs_cnt_last_--;
    msg.set_flag(true);
  }

  if (ad_status_ != 0 || ad_dtc_ != 0 || ad_status_req_ == 2) {
    msg.set_flag(true);
  }

  msg.set_aebsflag(enable_aebs_);

  tmc_writer_->Write(msg);
}

bool tmcComponent::Proc()
{
  chassis_reader_->Observe();
  const auto &chassis_msg = chassis_reader_->GetLatestObserved();
  if (chassis_msg != nullptr) {
    OnChassis(chassis_msg);
  }
  mwradar_reader_->Observe();
  const auto &mwradar_msg = mwradar_reader_->GetLatestObserved();
  if (mwradar_msg != nullptr) {
    OnMWradar(mwradar_msg);
  }
  utradar_reader_->Observe();
  const auto &ut_radar_msg = utradar_reader_->GetLatestObserved();
  if (ut_radar_msg != nullptr) {
    OnUTradar(ut_radar_msg);
  }
  guardian_reader_->Observe();
  const auto &guardian_msg = guardian_reader_->GetLatestObserved();
  if (guardian_msg != nullptr) {
    OnGuardian(guardian_msg);
  }

  PublishMsg();

  return true;
}

}  // namespace tmc
}  // namespace apollo
