// Copyright (C) Uhnder, Inc. All rights reserved. Confidential and Proprietary - under NDA.
// Refer to SOFTWARE_LICENSE file for details
#ifndef SRS_HDR_TXBW_CALB_FLASHDATA_H
#define SRS_HDR_TXBW_CALB_FLASHDATA_H 1

#include "modules/drivers/radar/rocket_radar/driver/system-radar-software/env-reference/coredefs/uhnder-common.h"
#include "modules/drivers/radar/rocket_radar/driver/system-radar-software/engine/scp-src/eng-api/uhmath.h"
#include "modules/drivers/radar/rocket_radar/driver/system-radar-software/engine/common/eng-api/rhal_out.h"
#include "modules/drivers/radar/rocket_radar/driver/system-radar-software/engine/scp-src/eng-api/default-presets.h"

SRS_DECLARE_NAMESPACE()

struct TxBwCalData
{
    uint32_t bbgm1[NUM_TX];
    uint32_t bbgm2[NUM_TX];
    uint32_t bbc1i[NUM_TX];
    uint32_t bbc1q[NUM_TX];
    uint32_t bbc2i[NUM_TX];
    uint32_t bbc2q[NUM_TX];

    uint32_t reserved0[NUM_TX];
    uint32_t reserved1[NUM_TX];

    void set_defaults()
    {
        memset(this, 0U, sizeof(*this));
    }

    void uhprint(bool all=true) const
    {
        for (INT tx = 0; tx < NUM_TX; tx++)
        {
            UHPRINTF("TX%d: GM1 %u GM2 %u CL1 %u,%u CL2 %u,%u\n",
                     tx, bbgm1[tx], bbgm2[tx], bbc1i[tx], bbc1q[tx],
                     bbc2i[tx], bbc2q[tx]);

            if (!all)
            {
                break;
            }
        }
    }

    bool apply_preset(RHAL_TxBWPresets tx_bw_preset)
    {
        bool valid_preset = false;

        switch (tx_bw_preset)
        {
        case Tx_BW_1200MHz_1700MHz:
            bbgm1[0] = 2;
            bbgm2[0] = 3;
            bbc1i[0] = 3;
            bbc1q[0] = 3;
            bbc2i[0] = 3;
            bbc2q[0] = 3;
            valid_preset = true;
            break;

        case Tx_BW_700MHz_900MHz:
            bbgm1[0] = 2;
            bbgm2[0] = 3;
            bbc1i[0] = 11;
            bbc1q[0] = 11;
            bbc2i[0] = 11;
            bbc2q[0] = 11;
            valid_preset = true;
            break;

        case Tx_BW_500MHz_600MHz:
            bbgm1[0] = 2;
            bbgm2[0] = 3;
            bbc1i[0] = 19;
            bbc1q[0] = 19;
            bbc2i[0] = 19;
            bbc2q[0] = 19;
            valid_preset = true;
            break;

        case Tx_BW_300MHz_400MHz:
            bbgm1[0] = 2;
            bbgm2[0] = 3;
            bbc1i[0] = 27;
            bbc1q[0] = 27;
            bbc2i[0] = 27;
            bbc2q[0] = 27;
            valid_preset = true;
            break;

        default:
            break;
        }

        if (valid_preset)
        {
            for (INT tx = 1; tx < NUM_TX; tx++)
            {
                bbgm1[tx] = bbgm1[0];
                bbgm2[tx] = bbgm2[0];
                bbc1i[tx] = bbc1i[0];
                bbc1q[tx] = bbc1q[0];
                bbc2i[tx] = bbc2i[0];
                bbc2q[tx] = bbc2q[0];
            }
        }

        return valid_preset;
    }
};


struct TxBwCalBKey
{
    FLOAT                     temperature;      // Chip temperature C at which this cal was performed
    uint32_t                  tx_bw_preset;     // RHAL_TxBWPresets

    uint32_t                  reserved_0;
    uint32_t                  reserved_1;

    enum { KEY_VERSION = 1 };

    TxBwCalBKey() { memset(this, 0, sizeof(*this)); }

    void uhprint() const
    {
        UHPRINTF("(TXBW) Temperature: %f\n", temperature);
        UHPRINTF("      TX Bw Preset: %d (%s)\n", tx_bw_preset,
                 RHAL_TxBWPresets(tx_bw_preset) < NUM_TXBW_PRESETS ?
                 tx_bw_names[tx_bw_preset] : "N/A");
    }

    void upgrade_from_version(uint32_t, TxBwCalData&) { }

    bool compare(const TxBwCalBKey& other) const
    {
        return other.tx_bw_preset == tx_bw_preset;
    }

    FLOAT distance(const TxBwCalBKey& other) const
    {
        return compare(other) ? uh_fabsf(temperature - other.temperature) : CAL_KEY_DO_NOT_USE;
    }
};

SRS_CLOSE_NAMESPACE()

#endif
