#pragma once
#include "cyber/cyber.h"

namespace apollo{
namespace roadguide{

typedef enum {
  SLIP_DECODE_STATE_DECODING,
  SLIP_DECODE_STATE_ESC_RECEIVED,
  SLIP_DECODE_STATE_INVALID_PACKET
} slip_decode_state_t;

class Slip {
 public:
  ~Slip() {}
  uint8_t encode_buf[100];
  size_t  encode_len;
  bool Encode(const uint8_t *p_data, size_t len);
  bool EncodeAppend(const uint8_t *p_data, size_t len);

  uint8_t decode_buf[100];
  size_t decode_len;
  bool Decode(const uint8_t *p_data, size_t len);

 private:
  bool EncodeImpl(const uint8_t *p_data, size_t len);
  size_t decode_idx_;
  slip_decode_state_t decode_state_;
  void ResetDecodeState();
  bool DecodeByte(uint8_t c);
};

}  // namespace roadguide
}  // namespace apollo
