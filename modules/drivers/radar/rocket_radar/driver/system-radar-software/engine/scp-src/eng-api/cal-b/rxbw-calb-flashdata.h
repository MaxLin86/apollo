// Copyright (C) Uhnder, Inc. All rights reserved. Confidential and Proprietary - under NDA.
// Refer to SOFTWARE_LICENSE file for details
#ifndef SRS_HDR_RXBW_CALB_FLASHDATA_H
#define SRS_HDR_RXBW_CALB_FLASHDATA_H 1

#include "modules/drivers/radar/rocket_radar/driver/system-radar-software/env-reference/coredefs/uhnder-common.h"
#include "modules/drivers/radar/rocket_radar/driver/system-radar-software/engine/scp-src/eng-api/uhmath.h"
#include "modules/drivers/radar/rocket_radar/driver/system-radar-software/engine/common/eng-api/rhal_out.h"
#include "modules/drivers/radar/rocket_radar/driver/system-radar-software/engine/scp-src/eng-api/default-presets.h"

SRS_DECLARE_NAMESPACE()

struct RxBwCalData
{
    uint32_t bw1[NUM_RX_PER_BANK];
    uint32_t bw2[NUM_RX_PER_BANK];
    uint32_t bw3[NUM_RX_PER_BANK];
    uint32_t byp_vga[NUM_RX_PER_BANK];
    uint32_t swp2_byp[NUM_RX_PER_BANK];
    uint32_t swp3_byp[NUM_RX_PER_BANK];
    uint32_t cmh1[NUM_RX_PER_BANK];
    uint32_t cmh2[NUM_RX_PER_BANK];
    uint32_t cmh3[NUM_RX_PER_BANK];

    uint32_t reserved0[NUM_RX_PER_BANK];
    uint32_t reserved1[NUM_RX_PER_BANK];

    void set_defaults()
    {
        memset(this, 0U, sizeof(*this));
    }

    void uhprint(bool all=true) const
    {
        for (INT rx = 0; rx < NUM_RX_PER_BANK; rx++)
        {
            UHPRINTF("RX%d: BW: %u %u %u BYP: VGA %u SWP2 %u SWP3 %u CMH: %u %u %u\n", rx,
                    bw1[rx], bw2[rx], bw3[rx], byp_vga[rx], swp2_byp[rx], swp3_byp[rx],
                    cmh1[rx], cmh2[rx], cmh3[rx]);

            if (!all)
            {
                break;
            }
        }
    }

    bool apply_preset(RHAL_RxBWPresets rx_bw_preset)
    {
        bool valid_preset = false;

        switch (rx_bw_preset)
        {
        case Rx_BW_2000MHz_Tandem_3_3_3:
            bw1[0] = 3;
            bw2[0] = 3;
            bw3[0] = 3;

            // Below settings are not defined for this mode. Re-using the definitions for 1GHz here for now
            byp_vga[0] = 1;
            swp2_byp[0] = 15;
            swp3_byp[0] = 15;
            cmh1[0] = 1;
            cmh2[0] = 7;
            cmh3[0] = 7;
            valid_preset = true;
            break;

        case Rx_BW_720MHz_Tandem_2_2_2:
            bw1[0] = 2;
            bw2[0] = 2;
            bw3[0] = 2;
            byp_vga[0] = 1;
            swp2_byp[0] = 15;
            swp3_byp[0] = 15;
            cmh1[0] = 1;
            cmh2[0] = 7;
            cmh3[0] = 7;
            valid_preset = true;
            break;

        default:
            break;
        }

        if (valid_preset)
        {
            for (INT rx = 1; rx < NUM_RX_PER_BANK; rx++)
            {
                bw1[rx]         = bw1[0];
                bw2[rx]         = bw2[0];
                bw3[rx]         = bw3[0];
                byp_vga[rx]     = byp_vga[0];
                swp2_byp[rx]    = swp2_byp[0];
                swp3_byp[rx]    = swp3_byp[0];
                cmh1[rx]        = cmh1[0];
                cmh2[rx]        = cmh2[0];
                cmh3[rx]        = cmh3[0];
            }
        }

        return valid_preset;
    }
};


struct RxBwCalBKey
{
    FLOAT                     temperature;      // Chip temperature C at which this cal was performed
    uint32_t                  rx_bw_preset;     // RHAL_RxBWPresets

    uint32_t                  reserved_0;
    uint32_t                  reserved_1;

    enum { KEY_VERSION = 1 };

    RxBwCalBKey() { memset(this, 0, sizeof(*this)); }

    void uhprint() const
    {
        UHPRINTF("(RXBW) Temperature: %f\n", temperature);
        UHPRINTF("      RX Bw Preset: %d (%s)\n", rx_bw_preset,
                 RHAL_RxBWPresets(rx_bw_preset) < NUM_RXBW_PRESETS ?
                 rx_bw_names[rx_bw_preset] : "N/A");
    }

    void upgrade_from_version(uint32_t, RxBwCalData&) { }

    bool compare(const RxBwCalBKey& other) const
    {
        return other.rx_bw_preset == rx_bw_preset;
    }

    FLOAT distance(const RxBwCalBKey& other) const
    {
        return compare(other) ? uh_fabsf(temperature - other.temperature) : CAL_KEY_DO_NOT_USE;
    }
};

SRS_CLOSE_NAMESPACE()

#endif
