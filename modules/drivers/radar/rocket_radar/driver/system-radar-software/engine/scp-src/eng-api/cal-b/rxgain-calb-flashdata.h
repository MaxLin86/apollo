// Copyright (C) Uhnder, Inc. All rights reserved. Confidential and Proprietary - under NDA.
// Refer to SOFTWARE_LICENSE file for details
#ifndef SRS_HDR_RXGAIN_CALB_FLASHDATA_H
#define SRS_HDR_RXGAIN_CALB_FLASHDATA_H 1

#include "modules/drivers/radar/rocket_radar/driver/system-radar-software/env-reference/coredefs/uhnder-common.h"
#include "modules/drivers/radar/rocket_radar/driver/system-radar-software/engine/scp-src/eng-api/uhmath.h"
#include "modules/drivers/radar/rocket_radar/driver/system-radar-software/engine/common/eng-api/rhal_out.h"
#include "modules/drivers/radar/rocket_radar/driver/system-radar-software/engine/scp-src/eng-api/default-presets.h"

SRS_DECLARE_NAMESPACE()

struct RxGainCalData
{
    uint32_t gain1[NUM_RX_PER_BANK];
    uint32_t gain2[NUM_RX_PER_BANK];
    uint32_t gain3[NUM_RX_PER_BANK];
    uint32_t lna2_gain_at_mask_m[NUM_RX_PER_BANK];
    uint32_t lna2_gain_mt_mask_m[NUM_RX_PER_BANK];
    uint32_t rx_mx_i_bc[NUM_RX_PER_BANK];
    uint32_t rx_mx_q_bc[NUM_RX_PER_BANK];
    uint32_t rx_mx_br[NUM_RX_PER_BANK];

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
            UHPRINTF("RX%d: GAIN: %u %u %u LNA2 AT %u MT %u MIX: %u,%u BR %u\n", rx,
                    gain1[rx], gain2[rx], gain3[rx],
                    lna2_gain_at_mask_m[rx], lna2_gain_mt_mask_m[rx],
                    rx_mx_i_bc[rx], rx_mx_q_bc[rx], rx_mx_br[rx]);

            if (!all)
            {
                break;
            }
        }
    }

    bool apply_preset(RHAL_RxVGAGainPresets rx_gain_preset)
    {
        bool valid_preset = false;

        switch (rx_gain_preset)
        {
        case Rx_Gain_51dB_VP_7_1_3_3_3:
            lna2_gain_at_mask_m[0] = 7;
            lna2_gain_mt_mask_m[0] = 7;
            rx_mx_i_bc[0]          = 22;
            rx_mx_q_bc[0]          = 22;
            rx_mx_br[0]            = 3;
            gain1[0]               = 3;
            gain2[0]               = 3;
            gain3[0]               = 3;
            valid_preset = true;
            break;

        case Rx_Gain_41dB_VP_7_0_0_3_3:
            lna2_gain_at_mask_m[0]  = 7;
            lna2_gain_mt_mask_m[0]  = 7;
            rx_mx_i_bc[0]           = 4;
            rx_mx_q_bc[0]           = 4;
            rx_mx_br[0]             = 3;
            gain1[0]                = 0;
            gain2[0]                = 3;
            gain3[0]                = 3;
            valid_preset = true;
            break;

        case Rx_Gain_35dB_VP_5_0_1_1_3:
            lna2_gain_at_mask_m[0]  = 5;
            lna2_gain_mt_mask_m[0]  = 5;
            rx_mx_i_bc[0]           = 4;
            rx_mx_q_bc[0]           = 4;
            rx_mx_br[0]             = 3;
            gain1[0]                = 1;
            gain2[0]                = 1;
            gain3[0]                = 3;
            valid_preset = true;
            break;

        case Rx_Gain_31dB_CP_3_0_0_3_1:
            lna2_gain_at_mask_m[0]   = 3;
            lna2_gain_mt_mask_m[0]   = 3;
            rx_mx_i_bc[0]            = 4;
            rx_mx_q_bc[0]            = 4;
            rx_mx_br[0]              = 3;
            gain1[0]                 = 0;
            gain2[0]                 = 3;
            gain3[0]                 = 1;
            valid_preset = true;
            break;

        case Rx_Gain_28dB_CP_3_1_0_1_1:
            lna2_gain_at_mask_m[0]    = 3;
            lna2_gain_mt_mask_m[0]    = 3;
            rx_mx_i_bc[0]             = 22;
            rx_mx_q_bc[0]             = 22;
            rx_mx_br[0]               = 3;
            gain1[0]                  = 0;
            gain2[0]                  = 1;
            gain3[0]                  = 1;
            valid_preset = true;
            break;

        case Rx_Gain_38dB_CP_3_1_2_3_3:
            lna2_gain_at_mask_m[0]    = 3;
            lna2_gain_mt_mask_m[0]    = 3;
            rx_mx_i_bc[0]             = 22;
            rx_mx_q_bc[0]             = 22;
            rx_mx_br[0]               = 3;
            gain1[0]                  = 2;
            gain2[0]                  = 3;
            gain3[0]                  = 3;
            valid_preset = true;
            break;

        default:
            break;
        }

        if (valid_preset)
        {
            for (INT rx = 1; rx < NUM_RX_PER_BANK; rx++)
            {
                lna2_gain_at_mask_m[rx] = lna2_gain_at_mask_m[0];
                lna2_gain_mt_mask_m[rx] = lna2_gain_mt_mask_m[0];
                rx_mx_i_bc[rx]          = rx_mx_i_bc[0];
                rx_mx_q_bc[rx]          = rx_mx_q_bc[0];
                rx_mx_br[rx]            = rx_mx_br[0];
                gain1[rx]               = gain1[0];
                gain2[rx]               = gain2[0];
                gain3[rx]               = gain3[0];
            }
        }

        return valid_preset;
    }
};


struct RxGainCalBKey
{
    FLOAT                     temperature;      // Chip temperature C at which this cal was performed
    uint32_t                  rx_gain_preset;   // RHAL_RxVGAGainPresets

    uint32_t                  reserved_0;
    uint32_t                  reserved_1;

    enum { KEY_VERSION = 1 };

    RxGainCalBKey() { memset(this, 0, sizeof(*this)); }

    void uhprint() const
    {
        UHPRINTF("(RXGAIN) Temperature: %f\n", temperature);
        UHPRINTF("      RX Gain Preset: %d (%s)\n", rx_gain_preset,
                 RHAL_RxVGAGainPresets(rx_gain_preset) < NUM_VGAGAIN_PRESETS ?
                 rx_gain_names[rx_gain_preset] : "N/A");
    }

    void upgrade_from_version(uint32_t, RxGainCalData&) { }

    bool compare(const RxGainCalBKey& other) const
    {
        return other.rx_gain_preset == rx_gain_preset;
    }

    FLOAT distance(const RxGainCalBKey& other) const
    {
        return compare(other) ? uh_fabsf(temperature - other.temperature) : CAL_KEY_DO_NOT_USE;
    }
};

SRS_CLOSE_NAMESPACE()

#endif
