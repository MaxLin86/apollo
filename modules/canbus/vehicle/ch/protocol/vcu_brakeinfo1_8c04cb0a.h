 

#pragma once

#include "modules/drivers/canbus/can_comm/protocol_data.h"
#include "modules/canbus/proto/chassis_detail.pb.h"

namespace apollo {
namespace canbus {
namespace ch {

class Vcubrakeinfo18c04cb0a : public ::apollo::drivers::canbus::ProtocolData<
                    ::apollo::canbus::ChassisDetail> {
 public:
  static const int32_t ID;
  Vcubrakeinfo18c04cb0a();
  void Parse(const std::uint8_t* bytes, int32_t length,
                     ChassisDetail* chassis) const override;

 private:
  int vcu_brake1_air_pressure(
      const std::uint8_t* bytes, int32_t length) const;
};

}  // namespace ch
}  // namespace canbus
}  // namespace apollo


