#pragma once
#include "cyber/cyber.h"
#include "modules/drivers/rs485/stream/stream.h"
#include "modules/roadguide/slip.h"
#include "modules/roadguide/rxboard_def.h"

#define TRANSER_SRC_ADDR (0xFE)

typedef struct {
  uint8_t cnt;
  uint16_t id;
  uint32_t mag;
} transfer_meas_ant_t;

typedef struct {
  transfer_meas_ant_t ant[RXBOARD_ANTS];
} transfer_meas_t;

using namespace apollo::drivers::rs485;

namespace apollo{
namespace roadguide{

class Transfer {
 public:
  Transfer(const char* device_name, uint32_t baud_rate);
  ~Transfer() = default;

  bool FetchMeas(uint8_t dest_addr, transfer_meas_t &meas);
  bool CreateStream();

 private:
  std::shared_ptr<Stream> data_stream_;
  std::string device_name_;
  uint32_t baud_rate_;
  uint8_t *p_rcv_buf_;
  size_t rcv_len_;

  Slip slip_;
  uint8_t local_addr_;
  bool SendData2Rs485(uint8_t dest_addr, const uint8_t *p_data, size_t len);
  bool RcvDataFromRs485(uint8_t dest_addr);
  bool DecodeMeas(transfer_meas_t& meas);
};

}  // namespace roadguide
}  // namespace apollo
