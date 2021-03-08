#ifndef SRS_HDR_DEFAULT_PRESETS_H
#define SRS_HDR_DEFAULT_PRESETS_H 1
// START_SOFTWARE_LICENSE_NOTICE
// -------------------------------------------------------------------------------------------------------------------
// Copyright (C) 2016-2019 Uhnder, Inc. All rights reserved.
// This Software is the property of Uhnder, Inc. (Uhnder) and is Proprietary and Confidential.  It has been provided
// under license for solely use in evaluating and/or developing code for Uhnder products.  Any use of the Software to
// develop code for a product not manufactured by or for Uhnder is prohibited.  Unauthorized use of this Software is
// strictly prohibited.
// Restricted Rights Legend:  Use, Duplication, or Disclosure by the Government is Subject to Restrictions as Set
// Forth in Paragraph (c)(1)(ii) of the Rights in Technical Data and Computer Software Clause at DFARS 252.227-7013.
// THIS PROGRAM IS PROVIDED UNDER THE TERMS OF THE UHNDER END-USER LICENSE AGREEMENT (EULA). THE PROGRAM MAY ONLY
// BE USED IN A MANNER EXPLICITLY SPECIFIED IN THE EULA, WHICH INCLUDES LIMITATIONS ON COPYING, MODIFYING,
// REDISTRIBUTION AND WARRANTIES. PROVIDING AFFIRMATIVE CLICK-THROUGH CONSENT TO THE EULA IS A REQUIRED PRECONDITION
// TO YOUR USE OF THE PROGRAM. YOU MAY OBTAIN A COPY OF THE EULA FROM WWW.UHNDER.COM. UNAUTHORIZED USE OF THIS
// PROGRAM IS STRICTLY PROHIBITED.
// THIS SOFTWARE IS PROVIDED "AS IS".  NO WARRANTIES ARE GIVEN, WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING
// WARRANTIES OR MERCHANTABILITY OR FITNESS FOR A PARTICULAR PURPOSE, NONINFRINGEMENT AND TITLE.  RECIPIENT SHALL HAVE
// THE SOLE RESPONSIBILITY FOR THE ADEQUATE PROTECTION AND BACK-UP OF ITS DATA USED IN CONNECTION WITH THIS SOFTWARE.
// IN NO EVENT WILL UHNDER BE LIABLE FOR ANY CONSEQUENTIAL DAMAGES WHATSOEVER, INCLUDING LOSS OF DATA OR USE, LOST
// PROFITS OR ANY INCIDENTAL OR SPECIAL DAMAGES, ARISING OUT OF OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS
// SOFTWARE, WHETHER IN ACTION OF CONTRACT OR TORT, INCLUDING NEGLIGENCE.  UHNDER FURTHER DISCLAIMS ANY LIABILITY
// WHATSOEVER FOR INFRINGEMENT OF ANY INTELLECTUAL PROPERTY RIGHTS OF ANY THIRD PARTY.
// -------------------------------------------------------------------------------------------------------------------
// END_SOFTWARE_LICENSE_NOTICE

#include "modules/drivers/radar/rocket_radar/driver/system-radar-software/env-uhnder/coredefs/uhnder-common.h"
#include "modules/drivers/radar/rocket_radar/driver/system-radar-software/engine/common/eng-api/rhal_out.h"

SRS_DECLARE_NAMESPACE()

static const FLOAT                 CAL_KEY_DO_NOT_USE = 1e7F;
static const FLOAT                 DEF_TEMP = 43.0F; // default temperature for cals

#if SABINE_A
static const FLOAT                 DEF_RF   = 77.5F; // default carrier frequency
static const RHAL_TxBWPresets      DEF_TX_BW   = Tx_BW_28_28_12_12_02_02;
static const RHAL_RxBWPresets      DEF_RX_BW   = Rx_BW_c23091314140101_f00000000000000;
static const RHAL_TxGainPresets    CP_TX_GAIN  = Tx_Gain_1_3;
static const RHAL_TxGainPresets    VP_TX_GAIN  = Tx_Gain_4_1;
#elif SABINE_B
static const FLOAT                 DEF_RF   = 76.6F; // default carrier frequency
static const RHAL_TxBWPresets      DEF_TX_BW   = Tx_BW_700MHz_900MHz;
static const RHAL_RxBWPresets      DEF_RX_BW   = Rx_BW_720MHz_Tandem_2_2_2;
static const RHAL_TxGainPresets    CP_TX_GAIN  = Tx_Gain_Efficient_7_4_4;
static const RHAL_TxGainPresets    VP_TX_GAIN  = Tx_Gain_Maximum_7_4_44;
static const RHAL_RxVGAGainPresets CP_RX_GAIN  = Rx_Gain_28dB_CP_3_1_0_1_1;
static const RHAL_RxVGAGainPresets VP_RX_GAIN  = Rx_Gain_41dB_VP_7_0_0_3_3;
#else
#error
#endif

SRS_CLOSE_NAMESPACE()

#endif // SRS_HDR_DEFAULT_PRESETS_H
