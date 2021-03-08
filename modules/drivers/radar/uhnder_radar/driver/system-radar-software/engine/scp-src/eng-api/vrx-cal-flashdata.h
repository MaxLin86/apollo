#ifndef SRS_HDR_VRX_CAL_FLASHDATA_H
#define SRS_HDR_VRX_CAL_FLASHDATA_H 1
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

#include "modules/drivers/radar/uhnder_radar/driver/system-radar-software/env-uhnder/coredefs/uhnder-common.h"
#include "modules/drivers/radar/uhnder_radar/driver/system-radar-software/engine/common/eng-api/rhal_out.h"
#include "modules/drivers/radar/uhnder_radar/driver/system-radar-software/engine/scp-src/eng-api/default-presets.h"

SRS_DECLARE_NAMESPACE()

struct VrxAlignCalData
{
    FLOAT Rx_Offset[NUM_RX_BANK][NUM_RX_PER_BANK]; // Rx offset
    FLOAT Tx_Offset[NUM_TX]; // Tx offset
    uint8_t Ref_Tx;
    uint8_t Ref_Rx;

    static const uint16_t NOT_RANGE_CALIBRATED = 0xFFFF;

    // the interpolated range bin at which the spillover must be located in CP1a
    // scans in order for targets to appear at the correct range. RX/TX kicking
    // and rx_delay offsets must be used at boot time to move the spillover here
    uint16_t spillover_range; // 4 integer unsigned bits, 12 fractional bits

    void uhprint(bool all = false) const
    {
        for (uint32_t i = 0; i < NUM_RX_BANK; i++)
        {
            UHPRINTF("Rx_Offset[bank%d] %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f\n", i,
                     Rx_Offset[i][0], Rx_Offset[i][1], Rx_Offset[i][2], Rx_Offset[i][3],
                     Rx_Offset[i][4], Rx_Offset[i][5], Rx_Offset[i][6], Rx_Offset[i][7]);
            if (!all) { break; }
        }

        UHPRINTF("Tx_Offset %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f\n",
                 Tx_Offset[0], Tx_Offset[1], Tx_Offset[2], Tx_Offset[3], Tx_Offset[4],
                 Tx_Offset[5], Tx_Offset[6], Tx_Offset[7], Tx_Offset[8], Tx_Offset[9],
                 Tx_Offset[10], Tx_Offset[11]);
        UHPRINTF("Ref Tx %d, Ref Rx %d\n", Ref_Tx, Ref_Rx);

        if (spillover_range == NOT_RANGE_CALIBRATED)
        {
            UHPRINTF("ERROR: Not calibrated for range!\n");
        }
        else
        {
            FLOAT sprange = FLOAT(spillover_range) / (1 << 12);
            UHPRINTF("Spillover range %f in CP1a bins\n", sprange);
        }
    }
};

struct VrxAlignKey
{
    FLOAT                     temp;         // Chip temperature C at which this cal was performed
    uint32_t                  reserved_tx;  // formerly bitmaps
    uint32_t                  reserved_rx;  // formerly bitmaps
    RHAL_TxBWPresets          tx_bw_enum;
    RHAL_RxBWPresets          rx_bw_enum;
    RHAL_RxVGAGainPresets     rx_vga_gain_enum;
    RHAL_TxGainPresets        tx_gain_enum;
    RHAL_LOMode               LO_mode;      // Int LO / OBLO / Ext LO
    FLOAT                     carrier_freq;

    uint32_t                  reserved_0;
    uint32_t                  reserved_1;
    uint32_t                  reserved_2;
    uint32_t                  reserved_3;
    uint32_t                  reserved_4;
    uint32_t                  reserved_5;
    uint32_t                  reserved_6;

    enum { KEY_VERSION = 3 };

    VrxAlignKey()
    {
        memset(this, 0, sizeof(*this));
    }


    VrxAlignKey(FLOAT                 _temp,
                RHAL_TxBWPresets      _tx_bw_enum,
                RHAL_RxBWPresets      _rx_bw_enum,
                RHAL_RxVGAGainPresets _rx_vga_gain_enum,
                RHAL_TxGainPresets    _tx_gain_enum,
                RHAL_LOMode           _LO_mode)
        : reserved_tx(0)
        , reserved_rx(0)
        , reserved_0(0)
        , reserved_1(0)
        , reserved_2(0)
        , reserved_3(0)
        , reserved_4(0)
        , reserved_5(0)
        , reserved_6(0)
    {
        temp = _temp;
        tx_bw_enum = _tx_bw_enum;
        rx_bw_enum = _rx_bw_enum;
        rx_vga_gain_enum = _rx_vga_gain_enum;
        tx_gain_enum = _tx_gain_enum;
        LO_mode = _LO_mode;
        carrier_freq = DEF_RF;
    }


    void uhprint() const
    {
        UHPRINTF("(VRX) Temperature: %f\n", temp);
        UHPRINTF("tx_bw_enum: %d %s\n", tx_bw_enum, tx_bw_names[tx_bw_enum]);
        UHPRINTF("rx_bw_enum: %d %s\n", rx_bw_enum, rx_bw_names[rx_bw_enum]);
        UHPRINTF("rx_vga_gain_enum: %d %s\n", rx_vga_gain_enum, rx_gain_names[rx_vga_gain_enum]);
        UHPRINTF("tx_gain_enum: %d %s\n", tx_gain_enum, tx_gain_names[tx_gain_enum]);
        UHPRINTF("LO mode: %d\n", LO_mode);
        UHPRINTF("carrier_freq: %.2f\n", carrier_freq);
    }


    void upgrade_from_version(uint32_t old_key_version, VrxAlignCalData& data)
    {
        if (old_key_version < 3)
        {
            data.spillover_range = VrxAlignCalData::NOT_RANGE_CALIBRATED;
        }
        if (old_key_version < 2)
        {
            carrier_freq = DEF_RF;
        }
    }


    //!! This is the all-important key-search algorithm !!
    bool compare(const VrxAlignKey& other) const
    {
        return (//uh_fabsf(temp - other.temp) < 1.5F &&
                (uh_fabsf(carrier_freq - other.carrier_freq) <= 0.2F) &&
                tx_bw_enum  == other.tx_bw_enum &&
                rx_bw_enum  == other.rx_bw_enum &&
                rx_vga_gain_enum == other.rx_vga_gain_enum &&
                LO_mode == other.LO_mode &&
                tx_gain_enum == other.tx_gain_enum);
    }


    // find the 'nearest-match' by picking the key with the shortest disance
    // from the reference key
    FLOAT distance(const VrxAlignKey& other) const
    {
        FLOAT dist = uh_fabsf(temp - other.temp) + uh_fabsf(carrier_freq - other.carrier_freq) * 5;
        INT diffs = !(tx_bw_enum == other.tx_bw_enum);

        diffs += !(rx_bw_enum == other.rx_bw_enum);
        diffs += !(rx_vga_gain_enum == other.rx_vga_gain_enum);
        diffs += !(LO_mode == other.LO_mode);
        diffs += !(tx_gain_enum == other.tx_gain_enum);
        return dist + CAL_KEY_DO_NOT_USE * diffs;
    }
};

SRS_CLOSE_NAMESPACE()

#endif // SRS_HDR_VRX_CAL_FLASHDATA_H
