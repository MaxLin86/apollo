#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <stdbool.h>
#include <string.h>
#include <cstdint>
#include <sys/types.h>

#include "modules/roadguide/util.h"

namespace apollo {
namespace roadguide {

void Util_Hexdump(const uint8_t *pBuf, size_t len)
{
  uint8_t bytePerLine = 16;
  char print_buf[6 + 3 + bytePerLine * 3 + 1];
  size_t print_buf_len = sizeof(print_buf);
  uint32_t idx = 0;

  if (pBuf == nullptr) {
    AERROR << "Null pointer: pBuf";
    return;
  }

  for (size_t i = 0; i < len; i++) {
    if (i % bytePerLine == 0) {
      idx += snprintf(&print_buf[idx], print_buf_len - idx, "0x%04X: ", (uint32_t)i);
    }
    idx += snprintf(&print_buf[idx], print_buf_len - idx, " %02X", *pBuf++);
    if ((i + 1) % bytePerLine == 0) {
      print_buf[idx] = 0x00;
      AERROR << print_buf;
      idx = 0;
    } else if((i + 1) % 8 == 0) {
      idx += snprintf(&print_buf[idx], print_buf_len - idx, " ");
    }
  }
  if (len % bytePerLine != 0) {
      print_buf[idx] = 0x00;
      AERROR << print_buf;
  }
}

}  // namespace roadguide
}  // namespace apollo
