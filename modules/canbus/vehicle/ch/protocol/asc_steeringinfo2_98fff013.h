 

#pragma once

#include "modules/drivers/canbus/can_comm/protocol_data.h"
#include "modules/canbus/proto/chassis_detail.pb.h"

namespace apollo {
namespace canbus {
namespace ch {

class Ascsteeringinfo298fff013 : public ::apollo::drivers::canbus::ProtocolData<
                    ::apollo::canbus::ChassisDetail> {
 public:
  static const int32_t ID;
  Ascsteeringinfo298fff013();
  void Parse(const std::uint8_t* bytes, int32_t length,
                     ChassisDetail* chassis) const override;

 private:
  double asc_absolute_steering_position(
      const std::uint8_t* bytes, int32_t length) const;
  double asc_external_input_command_echo(
      const std::uint8_t* bytes, int32_t length) const;

  int asc_status_error(const std::uint8_t* bytes,
                                                 int32_t length) const;

  Asc_steeringinfo2_98fff013::Asc_actual_mode asc_actual_mode(const std::uint8_t* bytes,
                                                    int32_t length) const;
  int asc_request_mode(const std::uint8_t* bytes,
                                                 int32_t length) const;
  int asc_absolute_steering_position_qf(
      const std::uint8_t* bytes, int32_t length) const;

  int asc_message_checksum(const std::uint8_t* bytes,
                                                     int32_t length) const;

  int asc_message_counter(const std::uint8_t* bytes,
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


