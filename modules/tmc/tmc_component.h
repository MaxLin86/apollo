#include <thread>
#include "cyber/cyber.h"
#include "cyber/component/timer_component.h"
#include "modules/tmc/proto/tmc.pb.h"
#include "modules/tmc/proto/tmc_conf.pb.h"

#include "modules/guardian/proto/guardian.pb.h"
#include "modules/canbus/proto/chassis.pb.h"
#include "modules/drivers/proto/conti_radar.pb.h"
#include "modules/drivers/proto/ultrasonic_radar.pb.h"

#include "modules/manuctrl/proto/manuctrl.pb.h"
#include "modules/urgencystop/proto/aeb_cmd.pb.h"
#include "modules/drivers/proto/conti_radar.pb.h"
#include "modules/control/proto/control_cmd.pb.h"
#include "modules/drivers/gnss/proto/gnss_best_pose.pb.h"
#include "modules/planning/proto/planning.pb.h"
#include "modules/localization/proto/localization.pb.h"
#include "modules/map/relative_map/proto/navigation.pb.h"
#include "modules/perception/proto/perception_obstacle.pb.h"
#include "modules/drivers/proto/sensor_image.pb.h"


using apollo::cyber::Component;

namespace apollo{
namespace tmc{

#define MAX_MWRADAR_OBS 10
  struct mwradar_obs_s {
    // Unit of 0.1m
    int8_t pos_x;
    int8_t pos_y;
    int8_t pos_z;
    // Unit of 1m/s
    int8_t vel_x;
    int8_t vel_y;
  } mwradar_obs_list_[MAX_MWRADAR_OBS];

#define MAX_UTRADAR_OBS 8
  struct utradar_obs_s {
    uint8_t nID;
    uint8_t nDistance;
  } utradar_obs_list_[MAX_UTRADAR_OBS];

class tmcComponent : public apollo::cyber::TimerComponent {
 public:
  ~tmcComponent() {}
  bool Init() override;
  bool Proc() override;
  void InitChannel();
  void UpdateModulesStatus();

 private:

  apollo::guardian::GuardianCommand guardian_cmd_;

        std::mutex mutex_;
        TmcConf tmc_conf_;
        manuctrl::manuctrlMsg                          info_Manuctrl_;
        urgency::AEBCommand                          info_Aebs_;
        drivers::ContiRadar                                   info_Radar_wm_raw_;
        control::ControlCommand                    info_Control_;
        drivers::gnss::GnssBestPose                  info_Gps_;
        planning::ADCTrajectory                         info_Planning_;
        localization::LocalizationEstimate      info_Location_;
        relative_map::MapMsg                             info_Map_;
        perception::PerceptionObstacles        info_Perception_;
        drivers::Image                                              info_Camera_1_;
        drivers::Image                                              info_Camera_3_;

  std::shared_ptr<apollo::cyber::Writer<tmcMsg>> tmc_writer_;
  std::shared_ptr<apollo::cyber::Writer<apollo::control::ControlCommand>> control_cmd_writer_;

  std::shared_ptr<apollo::cyber::Reader<apollo::canbus::Chassis>> chassis_reader_;
  std::shared_ptr<apollo::cyber::Reader<apollo::drivers::ContiRadar>> mwradar_reader_;
  std::shared_ptr<apollo::cyber::Reader<apollo::drivers::Ultrasonic>> utradar_reader_;
  std::shared_ptr<apollo::cyber::Reader<apollo::guardian::GuardianCommand>> guardian_reader_;

        std::shared_ptr<apollo::cyber::Reader<apollo::manuctrl::manuctrlMsg>> reader_Manuctrl_;
        std::shared_ptr<apollo::cyber::Reader<apollo::urgency::AEBCommand>> reader_Aebs_;
        std::shared_ptr<apollo::cyber::Reader<apollo::drivers::ContiRadar>> reader_Radar_wm_raw_;
        std::shared_ptr<apollo::cyber::Reader<apollo::control::ControlCommand>> reader_Control_;
        std::shared_ptr<apollo::cyber::Reader<apollo::drivers::gnss::GnssBestPose>> reader_Gps_;
        std::shared_ptr<apollo::cyber::Reader<apollo::planning::ADCTrajectory>> reader_Planning_;
        std::shared_ptr<apollo::cyber::Reader<apollo::localization::LocalizationEstimate>> reader_Location_;
        std::shared_ptr<apollo::cyber::Reader<apollo::relative_map::MapMsg>> reader_Map_;
        std::shared_ptr<apollo::cyber::Reader<apollo::perception::PerceptionObstacles>> reader_Perception_;
        std::shared_ptr<apollo::cyber::Reader<apollo::drivers::Image>> reader_Camera_1_;
        std::shared_ptr<apollo::cyber::Reader<apollo::drivers::Image>> reader_Camera_3_;

  

  void OnChassis(const std::shared_ptr<apollo::canbus::Chassis> &chassis);
  canbus::Chassis latest_chassis_;
  void OnMWradar(const std::shared_ptr<apollo::drivers::ContiRadar> &mwradar);
  #define MWRADAR_NUM 5
  drivers::ContiRadar latest_mwradar_[MWRADAR_NUM];
  bool rcvd_mwradar_msg_ = false;
  void OnUTradar(const std::shared_ptr<apollo::drivers::Ultrasonic> &utradar);
  drivers::Ultrasonic latest_utradar_;
  void OnGuardian(const std::shared_ptr<apollo::guardian::GuardianCommand> &guardian);
  guardian::GuardianCommand latest_guardian_;
  

  uint8_t GetUint8_t(const uint8_t *p_data, int32_t *idx);
  uint16_t GetUint16_t(const uint8_t *p_data, int32_t *idx);
  int16_t GetInt16_t(const uint8_t *p_data, int32_t *idx);
  uint32_t GetUint32_t(const uint8_t *p_data, int32_t *idx);
  void SetUint8_t(uint8_t value, uint8_t *p_data, int32_t *idx);
  void SetUint16_t(uint16_t value, uint8_t *p_data, int32_t *idx);
  void SetUint32_t(uint32_t value, uint8_t *p_data, int32_t *idx);

  std::unique_ptr<std::thread> rc_rxthread_ptr_;
  std::unique_ptr<std::thread> rc_txthread_ptr_;
  int32_t rc_sock_fd_;
  double rc_pkt_interval_sec_;
  uint16_t rc_fb_seq_ = 0;
  struct sockaddr_in rc_client_addr_;
  socklen_t rc_client_addr_len_;
  bool rc_rcvd_first_pkt_ = false;
  // TODO: random gen needed
  uint16_t rc_fb_id_ = 0x1234;
  uint16_t rc_ctrl_id_;
  double rc_pkt_timeout_sec_;
  double last_rc_pkt_rcvd_time_;
  double rc_pkt_period_sec_ = 0.05;
  double rc_fb_pkt_period_sec_ = 0.2;
  int32_t msgs_cnt_last_ = 0;

  bool StartRcSocket();
  void RcReadStream();
  void RcFbStream();
  bool StopRcSocket();
  bool ParseRcStream(const uint8_t *p_data, int32_t len);
  bool RcFbPktAssemble(uint8_t *buf, int32_t *p_len);
  void RcFbTx();

  std::unique_ptr<std::thread> ctrl_thread_ptr_;
  int32_t ctrl_lsock_fd_;
  int32_t ctrl_csock_fd_;
  uint16_t last_ctrl_seq_;
  uint8_t rcvd_ctrl_pkt_type_;
  bool rc_enable_ = false;
  bool StartCtrlSocket();
  void CtrlStreamDaemon();
  void OnRcvdCtrlStream();
  bool StopCtrlSocket();
  void HexDump(uint8_t *buf, int32_t len);
  bool ParseCtrlPkt(const uint8_t *p_data, int32_t len);
  bool CtrlRespPktAssemble(uint8_t nDataType, uint8_t *buf, int32_t *p_len);

  bool tmc_connected_ = false;
  uint8_t proto_ver_ = 1;
  uint8_t rc_status_ = 0;
  uint8_t rc_pre_status_ = 0;
  uint8_t rc_id_ = 0;
  uint8_t enable_aebs_ = 1;
  uint8_t cpu_util_percent_ = 50;
  uint8_t mem_util_percent_ = 60;
  uint8_t cpu_temp_ = 70;
  uint8_t dtc_ = 0x0;
  uint8_t status_;
  fd_set  fdRcRead_;

  uint8_t ad_status_req_;
  canbus::Chassis::GearPosition ad_gear_req_ = apollo::canbus::Chassis::GEAR_PARKING;
  canbus::Chassis::GearPosition ad_gear_pre_req_ = apollo::canbus::Chassis::GEAR_PARKING;
  uint8_t ad_torque_req_ = 0;
  uint8_t ad_max_speed_limit_;
  uint8_t ad_vcu_sw_req_;
  uint8_t ad_brake_req_;
  uint8_t ad_brake_percent_ = 30;
  uint8_t ad_epb_;
  uint8_t ad_status_ = 0;
  uint8_t ad_dtc_ = 0;

  double ad_turn_angle_ = 0;
  uint8_t ad_work_mode_;
  uint8_t ad_adu_status_;
  uint8_t ad_turn_cmd_valid_;
  uint8_t ad_turn_added_torque_;
  double ad_wheel_speed_ = 0;

  bool ad_warning_lamp_req_ = false;
  bool ad_left_turn_lamp_req_ = false;
  bool ad_right_turn_lamp_req_ = false;
  bool ad_position_lamp_req_ = false;
  bool ad_dipped_lamp_req_ = false;
  bool ad_full_beam_lamp_req_ = false;
  bool ad_front_fog_lamp_req_ = false;
  bool ad_rear_fog_lamp_req_ = false;

  uint32_t chassis_dest_ip_;
  uint16_t chassis_dest_port_;

  common::VehicleSignal::TurnSignal ad_turn_signal_ = common::VehicleSignal::TURN_NONE;
  int32_t mwradar_obs_cnt_ = 0;
  int32_t utradar_obs_cnt_ = 0;

  int8_t  nStatusRadarLeft_ = 0;
  int8_t  nStatusRadarLeftFront_ = 0;
  int8_t  nStatusRadarFront_ = 0;
  int8_t  nStatusRadarRightFront_ = 0;
  int8_t  nStatusRadarRight_ = 0;
  int8_t  nStatusRadarTop_ = 0;
  int8_t  nStatusCanbus_ = 0;
  int8_t  nStatusGuardian_ = 0;
  int8_t  nStatusRemoteManage_ = 0;
  int8_t  nStatusManuctrl_ = 0;
  int8_t  nStatusAebs_ = 0;
  int8_t  nStatusControl_ = 0;
  int8_t  nStatusGps_ = 0;
  int8_t  nStatusPlanning_ = 0;
  int8_t  nStatusLocation_ = 0;
  int8_t  nStatusMap_ = 0;
  int8_t  nStatusPrediction_ = 0;
  int8_t  nStatusPerception_ = 0;
  int8_t  nStatusCamera1_ = 0;
  int8_t  nStatusCamera3_ = 0;

  double dbCanbusLastTimeHeader_ = 0.0;
  double dbGuardianLastTimeHeader_ = 0.0;
  double dbRMLastTimeHeader_ = 0.0;
  double dbManuctrlLastTimeHeader_ = 0.0;
  double dbAebsLastTimeHeader_ = 0.0;
  double dbControlLastTimeHeader_ = 0.0;
  double dbGpsLastTimeHeader_ = 0.0;
  double dbPlanningLastTimeHeader_ = 0.0;
  double dbLocationLastTimeHeader_ = 0.0;
  double dbMapLastTimeHeader_ = 0.0;
  double dbPredictionLastTimeHeader_ = 0.0;
  double dbPerceptionLastTimeHeader_ = 0.0;
  double dbCamera1LastTimeHeader_ = 0.0;
  double dbCamera3LastTimeHeader_ = 0.0;

  double dbCanbusLastTimeRead_ = 0.0;
  double dbGuardianLastTimeRead_ = 0.0;
  double dbRMLastTimeRead_ = 0.0;
  double dbManuctrlLastTimeRead_ = 0.0;
  double dbAebsLastTimeRead_ = 0.0;
  double dbControlLastTimeRead_ = 0.0;
  double dbGpsLastTimeRead_ = 0.0;
  double dbPlanningLastTimeRead_ = 0.0;
  double dbLocationLastTimeRead_ = 0.0;
  double dbMapLastTimeRead_ = 0.0;
  double dbPredictionLastTimeRead_ = 0.0;
  double dbPerceptionLastTimeRead_ = 0.0;
  double dbCamera1LastTimeRead_ = 0.0;
  double dbCamera3LastTimeRead_ = 0.0;

  template <typename T>
  bool IsModuleNormal(T &type, double &dbPreTimeHeader, double &dbPreTimeRead, float dfInternalTime);

  void PublishMsg();
  void AssembleMWObs(uint8_t *p_data, int32_t *idx, int32_t *buf_len);
  void AssembleUTObs(uint8_t *p_data, int32_t *idx, int32_t *buf_len);
  bool AbstractMWObs();
  bool AbstractUTObs();
};

CYBER_REGISTER_COMPONENT(tmcComponent)

}  // namespace tmc
}  // namespace apollo
