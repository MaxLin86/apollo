// Copyright (C) Uhnder, Inc. All rights reserved. Confidential and Proprietary - under NDA.
// Refer to SOFTWARE_LICENSE file for details
#ifndef SRS_HDR_DEFAULT_PRESETS_H
#define SRS_HDR_DEFAULT_PRESETS_H 1

#include "modules/drivers/radar/rocket_radar/driver/system-radar-software/env-reference/coredefs/uhnder-common.h"
#include "modules/drivers/radar/rocket_radar/driver/system-radar-software/engine/common/eng-api/rhal_out.h"

SRS_DECLARE_NAMESPACE()

static const FLOAT                 CAL_KEY_DO_NOT_USE = 1e7F;
static const FLOAT                 DEF_TEMP = 43.0F; // default temperature for cals
static const FLOAT                 DEF_RF   = 76.6F; // default carrier frequency
static const RHAL_TxBWPresets      DEF_TX_BW   = Tx_BW_700MHz_900MHz;
static const RHAL_RxBWPresets      DEF_RX_BW   = Rx_BW_720MHz_Tandem_2_2_2;
static const RHAL_TxGainPresets    CP_TX_GAIN  = Tx_Gain_Efficient_7_4_4;
static const RHAL_TxGainPresets    VP_TX_GAIN  = Tx_Gain_Maximum_7_4_44;
static const RHAL_RxVGAGainPresets CP_RX_GAIN  = Rx_Gain_28dB_CP_3_1_0_1_1;
static const RHAL_RxVGAGainPresets VP_RX_GAIN  = Rx_Gain_41dB_VP_7_0_0_3_3;

SRS_CLOSE_NAMESPACE()

#endif // SRS_HDR_DEFAULT_PRESETS_H
