/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#include "modules/drivers/radar/ultrasonic_radar/protocol/ultra401.h"

#include "modules/drivers/canbus/common/byte.h"

namespace apollo {
namespace drivers {
namespace ultrasonic_radar {

using apollo::drivers::Ultrasonic;
using apollo::drivers::canbus::Byte;

const uint32_t Ultra401::ID = 0x401;

Ultra401::Ultra401() {}
Ultra401::~Ultra401() {}

uint32_t Ultra401::GetPeriod() const {
  static const uint32_t PERIOD = 100 * 1000;
  return PERIOD;
}

/**
 * @brief update the data
 * @param data a pointer to the data to be updated
 */
void Ultra401::UpdateData(uint8_t* data) {
  Byte Byte0Frame(data);
  Byte Byte1Frame(data + 1);
  Byte Byte2Frame(data + 2); 

      //   Byte0Frame.set_value(0x20);
      // Byte1Frame.set_value(0x11);
      // Byte2Frame.set_value(0x08);

  // static int nSwitch = 0;
  // AERROR << "NSWITCH : " << nSwitch;
  // switch(nSwitch) {
  //   case 0: {
  //     Byte0Frame.set_value(0x12);
  //     Byte1Frame.set_value(0x04);
  //     Byte2Frame.set_value(0x11);
  //     break;
  //   }
  //   case 1: {
  //     Byte0Frame.set_value(0x09);
  //     Byte1Frame.set_value(0x12);
  //     Byte2Frame.set_value(0x02);
  //     break;
  //   }
  //   case 2: {
  //     Byte0Frame.set_value(0x20);
  //     Byte1Frame.set_value(0x11);
  //     Byte2Frame.set_value(0x08);
  //     break;
  //   }
  //   case 3: {
  //     Byte0Frame.set_value(0x14);
  //     Byte1Frame.set_value(0x08);
  //     Byte2Frame.set_value(0x04);
  //     break;
  //   }
  //   default: {
  //     Byte0Frame.set_value(0x12);
  //     Byte1Frame.set_value(0x04);
  //     Byte2Frame.set_value(0x11);
  //     break;
  //   }
  // }
  
  // nSwitch = (nSwitch < 3)?(nSwitch+1):0;

    static int nSwitch = 0;
  AERROR << "NSWITCH : " << nSwitch;
  switch(nSwitch) {
    case 0:
    case 1: {
      Byte0Frame.set_value(0x12);
      Byte1Frame.set_value(0x04);
      Byte2Frame.set_value(0x11);
      break;
    }
    case 2:
    case 3: {
      Byte0Frame.set_value(0x09);
      Byte1Frame.set_value(0x12);
      Byte2Frame.set_value(0x02);
      break;
    }
    case 4:
    case 5: {
      Byte0Frame.set_value(0x20);
      Byte1Frame.set_value(0x11);
      Byte2Frame.set_value(0x08);
      break;
    }
    case 6:
    case 7: {
      Byte0Frame.set_value(0x14);
      Byte1Frame.set_value(0x08);
      Byte2Frame.set_value(0x04);
      break;
    }
    default: {
      Byte0Frame.set_value(0x12);
      Byte1Frame.set_value(0x04);
      Byte2Frame.set_value(0x11);
      break;
    }
  }
  
  nSwitch = (nSwitch < 7)?(nSwitch+1):0;

}

/**
 * @brief reset the private variables
 */
void Ultra401::Reset() { }


}  // namespace conti_radar
}  // namespace drivers
}  // namespace apollo
