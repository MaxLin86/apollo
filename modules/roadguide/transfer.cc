#include "modules/roadguide/transfer.h"
#include "modules/roadguide/CRC.h"
#include "modules/roadguide/util.h"
#include "modules/drivers/rs485/stream/stream.h"
#include "modules/roadguide/rxboard_def.h"

typedef enum
{
  TRANSFER_CMD_ID_GET_MEAS_REQ      = 0x00,
  TRANSFER_CMD_ID_GET_MEAS_RESP     = 0x01,
  TRANSFER_CMD_ID_SET_ADDR_REQ      = 0x02,
  TRANSFER_CMD_ID_SET_ADDR_RESP     = 0x03,
  TRANSFER_CMD_ID_GET_ADDR_REQ      = 0x04,
  TRANSFER_CMD_ID_GET_ADDR_RESP     = 0x05,
  TRANSFER_CMD_ID_SET_REG_REQ       = 0x06,
  TRANSFER_CMD_ID_SET_REG_RESP      = 0x07,
  TRANSFER_CMD_ID_GET_REG_REQ       = 0x08,
  TRANSFER_CMD_ID_GET_REG_RESP      = 0x09,
  TRANSFER_CMD_ID_SET_ID_REQ        = 0x0A,
  TRANSFER_CMD_ID_SET_ID_RESP       = 0x0B,
  TRANSFER_CMD_ID_GET_ID_REQ        = 0x0C,
  TRANSFER_CMD_ID_GET_ID_RESP       = 0x0D,
  TRANSFER_CMD_ID_SET_CAL_PARA_REQ  = 0x0E,
  TRANSFER_CMD_ID_SET_CAL_PARA_RESP = 0x0F,
  TRANSFER_CMD_ID_GET_CAL_PARA_REQ  = 0x10,
  TRANSFER_CMD_ID_GET_CAL_PARA_RESP = 0x11,
  TRANSFER_CMD_ID_GET_SW_VER_REQ    = 0x12,
  TRANSFER_CMD_ID_GET_SW_VER_RESP   = 0x13,
  TRANSFER_CMD_ID_UNSUPPORTED       = 0xFF,
} cmd_id_t;

using namespace apollo::drivers::rs485;

namespace apollo{
namespace roadguide{

bool Transfer::CreateStream()
{
  Stream *s = nullptr;

  s = Stream::create_serial(device_name_.c_str(), baud_rate_);

  if (s == nullptr) {
    AERROR << "Failed to create data stream.";
    return false;
  }

  data_stream_.reset(s);

  return true;
}

Transfer::Transfer(const char* device_name, uint32_t baud_rate)
  : device_name_(device_name), baud_rate_(baud_rate)
{
  local_addr_ = TRANSER_SRC_ADDR;
}

bool Transfer::SendData2Rs485(uint8_t dest_addr, const uint8_t *p_data, size_t len)
{
  uint8_t crc;
  size_t written_len;

  if (p_data == nullptr) {
    AERROR << "Invalid pointer: p_data";
    return false;
  }

  CRC::Table<std::uint8_t, 8> table(CRC::CRC_8());

  slip_.Encode(&dest_addr, sizeof(dest_addr));
  crc = CRC::Calculate(&dest_addr, sizeof(dest_addr), table);

  slip_.EncodeAppend(&local_addr_, sizeof(local_addr_));
  crc = CRC::Calculate(&local_addr_, sizeof(local_addr_), table, crc);

  slip_.EncodeAppend(p_data, len);
  crc = CRC::Calculate(p_data, len, table, crc);

  slip_.EncodeAppend(&crc, sizeof(crc));
  
  written_len = data_stream_->write(slip_.encode_buf, slip_.encode_len);

  return written_len == slip_.encode_len ? true : false;
}

bool Transfer::RcvDataFromRs485(uint8_t dest_addr)
{
  size_t rx_len, single_rx_len;
  uint8_t rx_buf[100];
  uint8_t crc, crc_cal;

  CRC::Table<std::uint8_t, 8> table(CRC::CRC_8());

  rx_len = 0;
  do {
    single_rx_len = data_stream_->read(&rx_buf[rx_len], sizeof(rx_buf) - rx_len);
    rx_len += single_rx_len;
  } while (single_rx_len > 0);

  if (!slip_.Decode(rx_buf, rx_len)) {
    return false;
  }

  if (slip_.decode_len <= 2 + 1) {
    AERROR << "Invalid packet";
    // Util_Hexdump(slip_.decode_buf, slip_.decode_len);
    // usleep(5000 * 1000);
    return false;
  }

  crc = slip_.decode_buf[slip_.decode_len - 1];
  crc_cal = CRC::Calculate(slip_.decode_buf, slip_.decode_len - 1, table);

  if (crc != crc_cal) {
    AERROR << "CRC error";
    // Util_Hexdump(slip_.decode_buf, slip_.decode_len);
    // usleep(5000 * 1000);
    return false;
  }

  if (slip_.decode_buf[0] != local_addr_) {
    AERROR << "Destination address unmatch";
    return false;
  }

  if (slip_.decode_buf[1] != dest_addr) {
    AERROR << "Source address unmatch";
    return false;
  }
  
  p_rcv_buf_ = &slip_.decode_buf[2];
  rcv_len_ = slip_.decode_len - 3;
  
  return true;
}

bool Transfer::DecodeMeas(transfer_meas_t &meas)
{
  uint8_t *p_buf;
  uint8_t cmd, ant_num;

  // Frame format: 
  //  cmd (uint8_t) + ant_num (uint8_t) 
  //  + ant1_cnt (uint8_t) + ant1_id (uint16_t) + ant1_mag (uint32_t)
  //  + ant2_cnt (uint8_t) + ant2_id (uint16_t) + ant2_mag (uint32_t)
  // 16 Bytes total
  if (rcv_len_ != 16) {
    AERROR << "Invalid packet len: " << static_cast<int>(rcv_len_);
    return false;
  }

  p_buf = p_rcv_buf_;

  cmd = *p_buf++;
  if (cmd != TRANSFER_CMD_ID_GET_MEAS_RESP) {
    AERROR << "Unexpected cmd id" << static_cast<int>(cmd);
    return false;
  }

  ant_num = *p_buf++;
  if (ant_num != RXBOARD_ANTS) {
    AERROR << "Unexpected ant number";
    return false;
  }

  for (uint32_t i = 0; i < RXBOARD_ANTS; i++) {
    meas.ant[i].cnt = *p_buf++;
    memcpy(&meas.ant[i].id, p_buf, 2);
    p_buf += 2;
    memcpy(&meas.ant[i].mag, p_buf, 4);
    p_buf += 4;
  }

  return true;
}

bool Transfer::FetchMeas(uint8_t dest_addr, transfer_meas_t& meas)
{ 
  uint8_t cmd = TRANSFER_CMD_ID_GET_MEAS_REQ;
  // TODO: flush rx buffer
  if (SendData2Rs485(dest_addr, &cmd, sizeof(cmd))) {
    usleep(2 * 1000);
    if (RcvDataFromRs485(dest_addr)) {
      return DecodeMeas(meas);
    }
  }
  return false;
}

}  // namespace roadguide
}  // namespace apollo
