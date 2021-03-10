 

#pragma once

#include "modules/drivers/canbus/can_comm/protocol_data.h"
#include "modules/canbus/proto/chassis_detail.pb.h"

namespace apollo {
namespace canbus {
namespace ch {

class Ascsteeringinfo198fff113 : public ::apollo::drivers::canbus::ProtocolData<
                    ::apollo::canbus::ChassisDetail> {
 public:
  static const int32_t ID;
  Ascsteeringinfo198fff113();
  void Parse(const std::uint8_t* bytes, int32_t length,
                     ChassisDetail* chassis) const override;

 private:
  double asc_column_torque(const std::uint8_t* bytes,
                                                     int32_t length) const;
  double asc_column_velocity(
      const std::uint8_t* bytes, int32_t length) const;

  double asc_pull_compensation(
      const std::uint8_t* bytes, int32_t length) const;

  int asc_column_torque_qf(const std::uint8_t* bytes,
                                                     int32_t length) const;

  int asc_column_velocity_qf(
      const std::uint8_t* bytes, int32_t length) const;

  int asc_ignition_state(const std::uint8_t* bytes,
                                                   int32_t length) const;
  int asc_hands_off_detected(
      const std::uint8_t* bytes, int32_t length) const;

  int asc_hands_on_detected(const std::uint8_t* bytes,
                                                      int32_t length) const;
  int asc_message_counter(const std::uint8_t* bytes,
                                                    int32_t length) const;
  int asc_message_checksum(const std::uint8_t* bytes,
                                                     int32_t length) const;

  int asc_message_check(const std::uint8_t* bytes,
                                                     int32_t length) const;

  /* ZF steer */
  double asc_get_sign_value_2bytes(const std::uint8_t* bytes, 
                            int32_t idx_low_byte,
                            int32_t idx_high_byte,
                            double factor) const;
};

}  // namespace ch
}  // namespace canbus
}  // namespace apollo


