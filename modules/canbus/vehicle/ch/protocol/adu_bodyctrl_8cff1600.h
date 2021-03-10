/******************************************************************************
 * Copyright 2019 The Apollo Authors. All Rights Reserved.
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

#pragma once

#include "modules/drivers/canbus/can_comm/protocol_data.h"
#include "modules/canbus/proto/chassis_detail.pb.h"

namespace apollo {
namespace canbus {
namespace ch {

class Adubodyctrl8cff1600 : public ::apollo::drivers::canbus::ProtocolData<
                    ::apollo::canbus::ChassisDetail> {
 public:
  static const int32_t ID;

  Adubodyctrl8cff1600();

  uint32_t GetPeriod() const override;

  void UpdateData(uint8_t* data) override;

  void Reset() override;

  Adubodyctrl8cff1600* set_ad_ledscreenreq(Adu_bodyctrl_8cff1600::AD_LEDscreenreqType ad_ledscreenreq);
  Adubodyctrl8cff1600* set_ad_hornreq(Adu_bodyctrl_8cff1600::AD_hornreqType ad_hornreq);
  Adubodyctrl8cff1600* set_ad_topwarninglampreq(Adu_bodyctrl_8cff1600::AD_topwarninglampreqType ad_topwarninglampreq);
  // config detail: {'name': 'AD_RearFogLampReq', 'enum': {0: 'AD_REARFOGLAMPREQ_CLOSE', 1: 'AD_REARFOGLAMPREQ_OPEN', 2: 'AD_REARFOGLAMPREQ_RESERVE1', 3: 'AD_REARFOGLAMPREQ_RESERVE2'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 26, 'type': 'enum', 'order': 'intel', 'physical_unit': ''}
  Adubodyctrl8cff1600* set_ad_rearfoglampreq(Adu_bodyctrl_8cff1600::Ad_rearfoglampreqType ad_rearfoglampreq);

  // config detail: {'name': 'AD_FrontFogLampReq', 'enum': {0: 'AD_FRONTFOGLAMPREQ_CLOSE', 1: 'AD_FRONTFOGLAMPREQ_OPEN', 2: 'AD_FRONTFOGLAMPREQ_RESERVE1', 3: 'AD_FRONTFOGLAMPREQ_RESERVE2'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 24, 'type': 'enum', 'order': 'intel', 'physical_unit': ''}
  Adubodyctrl8cff1600* set_ad_frontfoglampreq(Adu_bodyctrl_8cff1600::Ad_frontfoglampreqType ad_frontfoglampreq);


  Adubodyctrl8cff1600* set_ad_headlampreq(Adu_bodyctrl_8cff1600::Ad_headlampreqType ad_headlampreq);
  // config detail: {'name': 'AD_HeadLightFullBeamReq', 'enum': {0: 'AD_HEADLIGHTFULLBEAMREQ_CLOSE', 1: 'AD_HEADLIGHTFULLBEAMREQ_OPEN', 2: 'AD_HEADLIGHTFULLBEAMREQ_RESERVE1', 3: 'AD_HEADLIGHTFULLBEAMREQ_RESERVE2'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 22, 'type': 'enum', 'order': 'intel', 'physical_unit': ''}
  Adubodyctrl8cff1600* set_ad_headlightfullbeamreq(Adu_bodyctrl_8cff1600::Ad_headlightfullbeamreqType ad_headlightfullbeamreq);
  // config detail: {'name': 'AD_DippedHeadLampReq', 'enum': {0: 'AD_DIPPEDHEADLAMPREQ_CLOSE', 1: 'AD_DIPPEDHEADLAMPREQ_OPEN', 2: 'AD_DIPPEDHEADLAMPREQ_RESERVE1', 3: 'AD_DIPPEDHEADLAMPREQ_RESERVE2'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 20, 'type': 'enum', 'order': 'intel', 'physical_unit': ''}
  Adubodyctrl8cff1600* set_ad_dippedheadlampreq(Adu_bodyctrl_8cff1600::Ad_dippedheadlampreqType ad_dippedheadlampreq);

  // config detail: {'name': 'AD_PositionLampReq', 'enum': {0: 'AD_POSITIONLAMPREQ_CLOSE', 1: 'AD_POSITIONLAMPREQ_OPEN', 2: 'AD_POSITIONLAMPREQ_RESERVE1', 3: 'AD_POSITIONLAMPREQ_RESERVE2'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 18, 'type': 'enum', 'order': 'intel', 'physical_unit': ''}
  Adubodyctrl8cff1600* set_ad_positionlampreq(Adu_bodyctrl_8cff1600::Ad_positionlampreqType ad_positionlampreq);

  // config detail: {'name': 'AD_RightTurnLampReq', 'enum': {0: 'AD_RIGHTTURNLAMPREQ_CLOSE', 1: 'AD_RIGHTTURNLAMPREQ_OPEN', 2: 'AD_RIGHTTURNLAMPREQ_RESERVE1', 3: 'AD_RIGHTTURNLAMPREQ_RESERVE2'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 12, 'type': 'enum', 'order': 'intel', 'physical_unit': ''}
  Adubodyctrl8cff1600* set_ad_rightturnlampreq(Adu_bodyctrl_8cff1600::Ad_rightturnlampreqType ad_rightturnlampreq);

  // config detail: {'name': 'AD_LeftTurnLampReq', 'enum': {0: 'AD_LEFTTURNLAMPREQ_CLOSE', 1: 'AD_LEFTTURNLAMPREQ_OPEN', 2: 'AD_LEFTTURNLAMPREQ_RESERVE1', 3: 'AD_LEFTTURNLAMPREQ_RESERVE2'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 10, 'type': 'enum', 'order': 'intel', 'physical_unit': ''}
  Adubodyctrl8cff1600* set_ad_leftturnlampreq(Adu_bodyctrl_8cff1600::Ad_leftturnlampreqType ad_leftturnlampreq);

  // config detail: {'name': 'AD_WarningLampReq', 'enum': {0: 'AD_WARNINGLAMPREQ_CLOSE', 1: 'AD_WARNINGLAMPREQ_OPEN', 2: 'AD_WARNINGLAMPREQ_RESERVE1', 3: 'AD_WARNINGLAMPREQ_RESERVE2'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 8, 'type': 'enum', 'order': 'intel', 'physical_unit': ''}
  Adubodyctrl8cff1600* set_ad_warninglampreq(Adu_bodyctrl_8cff1600::Ad_warninglampreqType ad_warninglampreq);

 private:

  void set_p_ad_ledscreenreq(uint8_t* data, Adu_bodyctrl_8cff1600::AD_LEDscreenreqType ad_ledscreenreq);
  void set_p_ad_hornreq(uint8_t* data, Adu_bodyctrl_8cff1600::AD_hornreqType ad_hornreq);
  void set_p_ad_topwarninglampreq(uint8_t* data, Adu_bodyctrl_8cff1600::AD_topwarninglampreqType ad_topwarninglampreq);
  // config detail: {'name': 'AD_RearFogLampReq', 'enum': {0: 'AD_REARFOGLAMPREQ_CLOSE', 1: 'AD_REARFOGLAMPREQ_OPEN', 2: 'AD_REARFOGLAMPREQ_RESERVE1', 3: 'AD_REARFOGLAMPREQ_RESERVE2'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 26, 'type': 'enum', 'order': 'intel', 'physical_unit': ''}
  void set_p_ad_rearfoglampreq(uint8_t* data, Adu_bodyctrl_8cff1600::Ad_rearfoglampreqType ad_rearfoglampreq);

  // config detail: {'name': 'AD_FrontFogLampReq', 'enum': {0: 'AD_FRONTFOGLAMPREQ_CLOSE', 1: 'AD_FRONTFOGLAMPREQ_OPEN', 2: 'AD_FRONTFOGLAMPREQ_RESERVE1', 3: 'AD_FRONTFOGLAMPREQ_RESERVE2'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 24, 'type': 'enum', 'order': 'intel', 'physical_unit': ''}
  void set_p_ad_frontfoglampreq(uint8_t* data, Adu_bodyctrl_8cff1600::Ad_frontfoglampreqType ad_frontfoglampreq);

  void set_p_ad_headlampreq(uint8_t* data, Adu_bodyctrl_8cff1600::Ad_headlampreqType ad_headlampreq);
  // config detail: {'name': 'AD_HeadLightFullBeamReq', 'enum': {0: 'AD_HEADLIGHTFULLBEAMREQ_CLOSE', 1: 'AD_HEADLIGHTFULLBEAMREQ_OPEN', 2: 'AD_HEADLIGHTFULLBEAMREQ_RESERVE1', 3: 'AD_HEADLIGHTFULLBEAMREQ_RESERVE2'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 22, 'type': 'enum', 'order': 'intel', 'physical_unit': ''}
  void set_p_ad_headlightfullbeamreq(uint8_t* data, Adu_bodyctrl_8cff1600::Ad_headlightfullbeamreqType ad_headlightfullbeamreq);
  // config detail: {'name': 'AD_DippedHeadLampReq', 'enum': {0: 'AD_DIPPEDHEADLAMPREQ_CLOSE', 1: 'AD_DIPPEDHEADLAMPREQ_OPEN', 2: 'AD_DIPPEDHEADLAMPREQ_RESERVE1', 3: 'AD_DIPPEDHEADLAMPREQ_RESERVE2'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 20, 'type': 'enum', 'order': 'intel', 'physical_unit': ''}
  void set_p_ad_dippedheadlampreq(uint8_t* data, Adu_bodyctrl_8cff1600::Ad_dippedheadlampreqType ad_dippedheadlampreq);

  // config detail: {'name': 'AD_PositionLampReq', 'enum': {0: 'AD_POSITIONLAMPREQ_CLOSE', 1: 'AD_POSITIONLAMPREQ_OPEN', 2: 'AD_POSITIONLAMPREQ_RESERVE1', 3: 'AD_POSITIONLAMPREQ_RESERVE2'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 18, 'type': 'enum', 'order': 'intel', 'physical_unit': ''}
  void set_p_ad_positionlampreq(uint8_t* data, Adu_bodyctrl_8cff1600::Ad_positionlampreqType ad_positionlampreq);

  // config detail: {'name': 'AD_RightTurnLampReq', 'enum': {0: 'AD_RIGHTTURNLAMPREQ_CLOSE', 1: 'AD_RIGHTTURNLAMPREQ_OPEN', 2: 'AD_RIGHTTURNLAMPREQ_RESERVE1', 3: 'AD_RIGHTTURNLAMPREQ_RESERVE2'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 12, 'type': 'enum', 'order': 'intel', 'physical_unit': ''}
  void set_p_ad_rightturnlampreq(uint8_t* data, Adu_bodyctrl_8cff1600::Ad_rightturnlampreqType ad_rightturnlampreq);

  // config detail: {'name': 'AD_LeftTurnLampReq', 'enum': {0: 'AD_LEFTTURNLAMPREQ_CLOSE', 1: 'AD_LEFTTURNLAMPREQ_OPEN', 2: 'AD_LEFTTURNLAMPREQ_RESERVE1', 3: 'AD_LEFTTURNLAMPREQ_RESERVE2'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 10, 'type': 'enum', 'order': 'intel', 'physical_unit': ''}
  void set_p_ad_leftturnlampreq(uint8_t* data, Adu_bodyctrl_8cff1600::Ad_leftturnlampreqType ad_leftturnlampreq);

  // config detail: {'name': 'AD_WarningLampReq', 'enum': {0: 'AD_WARNINGLAMPREQ_CLOSE', 1: 'AD_WARNINGLAMPREQ_OPEN', 2: 'AD_WARNINGLAMPREQ_RESERVE1', 3: 'AD_WARNINGLAMPREQ_RESERVE2'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 8, 'type': 'enum', 'order': 'intel', 'physical_unit': ''}
  void set_p_ad_warninglampreq(uint8_t* data, Adu_bodyctrl_8cff1600::Ad_warninglampreqType ad_warninglampreq);

 private:
  Adu_bodyctrl_8cff1600::AD_LEDscreenreqType ad_ledscreenreq_;
  Adu_bodyctrl_8cff1600::AD_hornreqType ad_hornreq_;
  Adu_bodyctrl_8cff1600::AD_topwarninglampreqType ad_topwarninglampreq_;
  Adu_bodyctrl_8cff1600::Ad_rearfoglampreqType ad_rearfoglampreq_;
  Adu_bodyctrl_8cff1600::Ad_frontfoglampreqType ad_frontfoglampreq_;
  Adu_bodyctrl_8cff1600::Ad_headlampreqType ad_headlampreq_;
  Adu_bodyctrl_8cff1600::Ad_headlightfullbeamreqType ad_headlightfullbeamreq_;
  Adu_bodyctrl_8cff1600::Ad_dippedheadlampreqType ad_dippedheadlampreq_;
  Adu_bodyctrl_8cff1600::Ad_positionlampreqType ad_positionlampreq_;
  Adu_bodyctrl_8cff1600::Ad_rightturnlampreqType ad_rightturnlampreq_;
  Adu_bodyctrl_8cff1600::Ad_leftturnlampreqType ad_leftturnlampreq_;
  Adu_bodyctrl_8cff1600::Ad_warninglampreqType ad_warninglampreq_;
};

}  // namespace ch
}  // namespace canbus
}  // namespace apollo


