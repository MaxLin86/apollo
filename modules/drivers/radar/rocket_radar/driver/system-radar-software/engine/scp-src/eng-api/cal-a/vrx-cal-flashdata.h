// Copyright (C) Uhnder, Inc. All rights reserved. Confidential and Proprietary - under NDA.
// Refer to SOFTWARE_LICENSE file for details
#ifndef SRS_HDR_VRX_CAL_FLASHDATA_H
#define SRS_HDR_VRX_CAL_FLASHDATA_H 1

#include "modules/drivers/radar/rocket_radar/driver/system-radar-software/env-reference/coredefs/uhnder-common.h"
#include "modules/drivers/radar/rocket_radar/driver/system-radar-software/engine/common/eng-api/rhal_out.h"
#include "rdc-common.h"
#include "modules/drivers/radar/rocket_radar/driver/system-radar-software/engine/scp-src/eng-api/default-presets.h"

SRS_DECLARE_NAMESPACE()

struct VrxAlignCalData
{
    FLOAT Rx_Offset[NUM_RX_BANK][NUM_RX_PER_BANK]; // Rx offset
    FLOAT Tx_Offset[NUM_TX]; // Tx offset
    uint8_t Ref_Tx;
    uint8_t Ref_Rx;

    static const uint16_t NOT_RANGE_CALIBRATED = 0xFFFFU;

    // the interpolated range bin at which the spillover must be located in CP1a
    // scans in order for targets to appear at the correct range. RX/TX kicking
    // and rx_delay offsets must be used at boot time to move the spillover here
    uint16_t spillover_range; // 4 integer unsigned bits, 12 fractional bits

    void set_defaults()
    {
        memset(this, 0, sizeof(*this));
    }

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
    RHAL_TxBWPresets          reserved_tx_bw_enum;
    RHAL_RxBWPresets          reserved_rx_bw_enum;
    RHAL_RxVGAGainPresets     reserved_rx_vga_gain_enum;
    RHAL_TxGainPresets        reserved_tx_gain_enum;
    uint32_t                  reserved_LO_mode;
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

    void uhprint() const
    {
        UHPRINTF("(VRX) Temperature: %f\n", temp);
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

    void upgrade_from_version(uint32_t, struct VrxAlignCalBData&) {}

    bool compare(const VrxAlignKey& other) const
    {
        return (uh_fabsf(carrier_freq - other.carrier_freq) <= 0.2F);
    }

    // find the 'nearest-match' by picking the key with the shortest distance
    // from the reference key
    FLOAT distance(const VrxAlignKey& other) const
    {
        return uh_fabsf(temp - other.temp) + uh_fabsf(carrier_freq - other.carrier_freq) * 5;
    }
};

SRS_CLOSE_NAMESPACE()

#endif // SRS_HDR_VRX_CAL_FLASHDATA_H
