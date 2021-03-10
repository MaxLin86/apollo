
#pragma once

#include "modules/drivers/canbus/can_comm/protocol_data.h"
#include "modules/canbus/proto/chassis_detail.pb.h"

namespace apollo {
namespace canbus {
namespace ch {

class Aduautosteering98ffefe8 : public ::apollo::drivers::canbus::ProtocolData<
                    ::apollo::canbus::ChassisDetail> {
 public:
    static const int32_t ID;

    Aduautosteering98ffefe8();

    uint32_t GetPeriod() const override;

    void UpdateData(uint8_t* data) override;

    void Reset() override;

    Aduautosteering98ffefe8* set_ad_desiredoperatingmode(
        Adu_autosteering_98ffefe8::Ad_desiredoperatingmode
            ad_desiredoperatingmode){
      ad_desiredoperatingmode_ = ad_desiredoperatingmode;
      return this;
    }

    Aduautosteering98ffefe8* set_ad_command_from_external_controller(double  ad_command_from_external_controller){
      ad_command_from_external_controller_ = ad_command_from_external_controller;
      return this;
    }
    Aduautosteering98ffefe8* set_ad_message_counter(int  ad_message_counter){
      ad_message_counter_ = ad_message_counter;
      return this;
    }
    Aduautosteering98ffefe8* set_ad_message_checksum(int  ad_message_checksum){
      ad_message_checksum_ = ad_message_checksum;
      return this;
    }

   private:
    void set_p_ad_desiredoperatingmode(uint8_t* data, Adu_autosteering_98ffefe8::Ad_desiredoperatingmode ad_desiredoperatingmode);
    void set_p_ad_command_from_external_controller(uint8_t* data, double  ad_command_from_external_controller);
    void set_p_ad_message_counter(uint8_t* data, int ad_message_counter);
    void set_p_ad_message_checksum(uint8_t* data, int ad_message_checksum); 
    void asc_set_sign_value_2bytes(uint8_t* data, 
                                double value,
                                double min,
                                double max,
                                int32_t idx_low_byte,
                                int32_t idx_high_byte,
                                double factor);

   private:
    Adu_autosteering_98ffefe8::Ad_desiredoperatingmode
        ad_desiredoperatingmode_;
    double ad_command_from_external_controller_;
    int ad_message_counter_;
    int ad_message_checksum_;
};

}  // namespace ch
}  // namespace canbus
}  // namespace apollo