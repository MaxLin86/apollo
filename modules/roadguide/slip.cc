#include "modules/roadguide/slip.h"

#define SLIP_BYTE_END      0300    /* indicates end of packet */
#define SLIP_BYTE_ESC      0333    /* indicates byte stuffing */
#define SLIP_BYTE_ESC_END  0334    /* ESC ESC_END means END data byte */
#define SLIP_BYTE_ESC_ESC  0335    /* ESC ESC_ESC means ESC data byte */

namespace apollo{
namespace roadguide{

bool Slip::EncodeImpl(const uint8_t *p_data, size_t len)
{
  if (p_data == nullptr) {
    AERROR << "Invalid pointer: p_data";
    return false;
  }

  for (size_t i = 0; i < len; i++) {
    if (encode_len == sizeof(encode_buf))
    {
      AERROR << "Insufficient buf len";
      return false;
    }
    switch (p_data[i]) {
      case SLIP_BYTE_END:
        encode_buf[encode_len++] = SLIP_BYTE_ESC;
        encode_buf[encode_len++] = SLIP_BYTE_ESC_END;
        break;

      case SLIP_BYTE_ESC:
        encode_buf[encode_len++] = SLIP_BYTE_ESC;
        encode_buf[encode_len++] = SLIP_BYTE_ESC_ESC;
        break;

      default:
        encode_buf[encode_len++] = p_data[i];
    }
  }
  encode_buf[encode_len++] = SLIP_BYTE_END;

  return true;
}

bool Slip::Encode(const uint8_t *p_data, size_t len)
{
  encode_len = 0;
  
  return EncodeImpl(p_data, len);
}

bool Slip::EncodeAppend(const uint8_t *p_data, size_t len)
{
  if (encode_len > 0)
  {
    encode_len--;
  }
  
  return EncodeImpl(p_data, len);
}

void Slip::ResetDecodeState()
{
  decode_state_ = SLIP_DECODE_STATE_DECODING;
  decode_idx_ = 0;
}

bool Slip::DecodeByte(uint8_t c)
{
  if (decode_idx_ >= sizeof(decode_buf)) {
    AERROR << "Slip decode buffer full.";
    decode_state_ = SLIP_DECODE_STATE_INVALID_PACKET;
    return false;
  }

  switch (decode_state_) {
    case SLIP_DECODE_STATE_DECODING:
      switch (c) {
        case SLIP_BYTE_END:
          decode_len = decode_idx_;
          decode_idx_ = 0;
          return true;

        case SLIP_BYTE_ESC:
          decode_state_ = SLIP_DECODE_STATE_ESC_RECEIVED;
          break;

        default:
          decode_buf[decode_idx_++] = c;
          break;
      }
        break;

    case SLIP_DECODE_STATE_ESC_RECEIVED:
      switch (c) {
        case SLIP_BYTE_ESC_END:
          decode_buf[decode_idx_++] = SLIP_BYTE_END;
          decode_state_ = SLIP_DECODE_STATE_DECODING;
          break;

        case SLIP_BYTE_ESC_ESC:
          decode_buf[decode_idx_++] = SLIP_BYTE_ESC;
          decode_state_ = SLIP_DECODE_STATE_DECODING;
          break;

        default:
          decode_state_ = SLIP_DECODE_STATE_INVALID_PACKET;
          return false;
      }
        break;

    case SLIP_DECODE_STATE_INVALID_PACKET:
      if (c == SLIP_BYTE_END) {
        decode_state_ = SLIP_DECODE_STATE_DECODING;
        decode_idx_ = 0;
      }
      break;
  }

  return false;
}

bool Slip::Decode(const uint8_t *p_data, size_t len)
{
  if (p_data == nullptr) {
    AERROR << "Invalid pointer: p_data";
    return false;
  }

  ResetDecodeState();

  for (size_t i = 0; i < len; i++) {
    if (DecodeByte(p_data[i])) {
      return true;
    }
  }

  return false;
}

}  // namespace roadguide
}  // namespace apollo
