#include <thread>
#include "cyber/cyber.h"
#include "cyber/component/timer_component.h"
#include "modules/manuctrl/proto/manuctrl.pb.h"
#include "modules/canbus/proto/chassis.pb.h"
#include "modules/manuctrl/proto/manuctrl_conf.pb.h"
#include "modules/drivers/joystick/stream/stream.h"

using apollo::cyber::Component;

namespace apollo{
namespace manuctrl{

class manuctrlComponent : public apollo::cyber::TimerComponent {
 public:
  ~manuctrlComponent() {}
  bool Init() override;
  bool Proc() override;
 private:
  #define SLIP_BYTE_END      0300    /* indicates end of packet */
  #define SLIP_BYTE_ESC      0333    /* indicates byte stuffing */
  #define SLIP_BYTE_ESC_END  0334    /* ESC ESC_END means END data byte */
  #define SLIP_BYTE_ESC_ESC  0335    /* ESC ESC_ESC means ESC data byte */
  typedef enum {
    SLIP_STATE_DECODING, //!< Ready to receive the next byte.
    SLIP_STATE_ESC_RECEIVED, //!< An ESC byte has been received and the next byte must be decoded differently.
    SLIP_STATE_CLEARING_INVALID_PACKET //!< The received data is invalid and transfer must be restarted.
  } slip_read_state_t;

  typedef struct {
    slip_read_state_t state; //!< Current state of the packet (see @ref slip_read_state_t).
    uint8_t          *p_buf; //!< Decoded data.
    uint32_t          cur_idx; //!< Current length of the packet that has been received.
    uint32_t          buf_len; //!< Size of the buffer that is available.
  } slip_t;

  bool CreateStream(const ManuctrlConf &conf);
  void ReadStream();
  void StartStream();
  bool slip_decode_add_byte(slip_t * p_slip, uint8_t c);

  ManuctrlConf manuctrl_conf_;
  static constexpr size_t DATA_BUFFER_SIZE = 2048;
  uint8_t data_buffer_[DATA_BUFFER_SIZE] = {0};
  uint8_t slip_buffer_[DATA_BUFFER_SIZE] = {0};
  slip_t slip = {
    .state    = SLIP_STATE_DECODING, 
    .p_buf    = slip_buffer_,
    .cur_idx  = 0,
    .buf_len  = sizeof(slip_buffer_),
  };

  std::shared_ptr<apollo::cyber::Writer<manuctrlMsg>> manuctrl_writer_;
  std::shared_ptr<apollo::cyber::Writer<apollo::control::ControlCommand>> control_cmd_writer_;
  std::shared_ptr<apollo::cyber::Reader<apollo::canbus::Chassis>> chassis_reader_;
  std::shared_ptr<apollo::drivers::joystick::Stream> data_stream_;
  std::unique_ptr<std::thread> data_thread_ptr_;

  typedef struct {
    uint16_t x      :1;
    uint16_t a      :1;
    uint16_t b      :1;
    uint16_t y      :1;
    uint16_t rb     :1;
    uint16_t rt     :1;
    uint16_t rr     :1;
    uint16_t lr     :1;
    uint16_t lt     :1;
    uint16_t lb     :1;
    uint16_t up     :1;
    uint16_t left   :1;
    uint16_t down   :1;
    uint16_t right  :1;
    uint16_t select :1;
    uint16_t start  :1;
  } rc_key_status_t;

  typedef struct {
    uint8_t  lx;
    uint8_t  ly;
    uint8_t  rx;
    uint8_t  ry;
  } rc_rocker_status_t;

  typedef struct {
    rc_key_status_t    key;
    rc_rocker_status_t rocker;
    uint16_t           voltage;
  } rc_status_t;

  bool rc_status_parse(rc_status_t *p_status, uint8_t const *p_raw_data, size_t len);

  #define CTRL_PKT_RATE_HZ     33U
  #define CTRL_TORQUE_DEFAULT  0U
  #define CTRL_TORQUE_MAX      100U
  #define CTRL_TORQUE_SLOPE  ((float)5 / CTRL_PKT_RATE_HZ)
  #define CTRL_BRAKE_DEFAULT   100U
  #define CTRL_BRAKE_MAX       100U
  #define CTRL_BRAKE_SLOPE   ((float)3300 / CTRL_PKT_RATE_HZ)

  #define CTRL_ROCKER_MID        50U
  #define CTRL_ROCKER_TRIG_TH    15U
  #define CTRL_ROCKER_RANGE      50U
  #define CTRL_STEER_ANGLE_MAX   100
  #define CTRL_STEER_SPEED_MIN   0U
  #define CTRL_STEER_SPEED_MAX   100U
  #define CTRL_STEER_SPEED_CAL(offset) ((CTRL_STEER_SPEED_MAX - CTRL_STEER_SPEED_MIN) * (offset) / (CTRL_ROCKER_RANGE - CTRL_ROCKER_TRIG_TH) + CTRL_STEER_SPEED_MIN)
    
  typedef enum {
    VEHICLE_STATUS_IGNITION,
    VEHICLE_STATUS_AD,
    VEHICLE_STATUS_PWROFF,
  } vehicle_status_t;

  typedef enum {
    VEHICLE_GEAR_NEUTRAL,
    VEHICLE_GEAR_DRIVE,
    VEHICLE_GEAR_REVERSE,
    VEHICLE_GEAR_PARK,
  } vehicle_gear_t;

  typedef enum {
    ADU_NO_FAILURE,
    ADU_LEVEL1_FAULT,
    ADU_LEVEL2_FAULT,
    ADU_LEVEL3_FAULT,
    ADU_FAULT_OTHERS,
  } adu_fault_level_t;

  typedef struct {
    vehicle_status_t  vehicle_status;
    rc_status_t       rc_status;
    adu_fault_level_t fault_level;
    int32_t           dtc;
    rc_key_status_t   rc_key_pre_status;
    bool              rc_status_valid;
    vehicle_gear_t    gear;
    float             torque;
    float             brake;
    double            steer_angle;
    double            steer_speed;
    bool              manual_ch;
    bool              full_beam_head_lamp;
    bool              dipped_head_lamp;
    common::VehicleSignal::TurnSignal turn_signal;
  } ctrl_t;

  ctrl_t ctrl;

  bool ctrl_signal_proc();
  bool ctrl_vehicle_ctrl();
  void ctrl_vehicle_brake_chg();
  void ctrl_vehicle_torque_chg();
  void OnChassis(const std::shared_ptr<apollo::canbus::Chassis> &chassis);
  canbus::Chassis latest_chassis_;
  std::mutex mutex_;
  double radio_timeout_sec_;
  double last_rcved_time_;
  double exit_ad_delay_sec_;
  double exit_ad_start_time_;
  bool   exiting_ad_;
};

CYBER_REGISTER_COMPONENT(manuctrlComponent)

}  // namespace manuctrl
}  // namespace apollo
