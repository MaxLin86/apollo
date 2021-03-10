// Copyright (C) Uhnder, Inc. All rights reserved. Confidential and Proprietary - under NDA.
// Refer to SOFTWARE_LICENSE file for details
#ifndef SRS_HDR_TXGAIN_CALB_FLASHDATA_H
#define SRS_HDR_TXGAIN_CALB_FLASHDATA_H 1

#include "modules/drivers/radar/rocket_radar/driver/system-radar-software/env-reference/coredefs/uhnder-common.h"
#include "modules/drivers/radar/rocket_radar/driver/system-radar-software/engine/scp-src/eng-api/uhmath.h"
#include "modules/drivers/radar/rocket_radar/driver/system-radar-software/engine/common/eng-api/rhal_out.h"
#include "modules/drivers/radar/rocket_radar/driver/system-radar-software/engine/scp-src/eng-api/default-presets.h"

SRS_DECLARE_NAMESPACE()

struct TxGainCalData
{
    uint32_t daci_ramp[NUM_TX]; // Sets the LSB size and range of the DAC
    uint32_t dacq_ramp[NUM_TX]; // Sets the LSB size and range of the DAC
    uint32_t bbca_gain[NUM_TX]; // BBCA gain
    uint32_t ldo_padac[NUM_TX]; // PA120 ryldo_lv100 reference current Iref

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
            UHPRINTF("TX%d: DAC ramp I,Q %u/%u BBCA gain %u VTR PADAC %u\n",
                     tx, daci_ramp[tx], dacq_ramp[tx], bbca_gain[tx], ldo_padac[tx]);

            if (!all)
            {
                break;
            }
        }
    }

    bool apply_preset(RHAL_TxGainPresets tx_gain_preset)
    {
        bool valid_preset = false;

        switch (tx_gain_preset)
        {
        case Tx_Gain_Maximum_7_4_44:
            daci_ramp[0] = 7;
            dacq_ramp[0] = 7;
            bbca_gain[0] = 4;
            ldo_padac[0] = 44;
            valid_preset = true;
            break;

        case Tx_Gain_Efficient_7_4_4:
            daci_ramp[0] = 7;
            dacq_ramp[0] = 7;
            bbca_gain[0] = 4;
            ldo_padac[0] = 4;
            valid_preset = true;
            break;

        case Tx_Gain_Backoff20_0_4_4:
            daci_ramp[0] = 0;
            dacq_ramp[0] = 0;
            bbca_gain[0] = 4;
            ldo_padac[0] = 4;
            valid_preset = true;
            break;

        default:
            break;
        }

        if (valid_preset)
        {
            for (INT tx = 1; tx < NUM_TX; tx++)
            {
                daci_ramp[tx] = daci_ramp[0];
                dacq_ramp[tx] = dacq_ramp[0];
                bbca_gain[tx] = bbca_gain[0];
                ldo_padac[tx] = ldo_padac[0];
            }
        }

        return valid_preset;
    }
};


struct TxGainCalBKey
{
    FLOAT                     temperature;     // Chip temperature C at which this cal was performed
    uint32_t                  tx_gain_preset;  // RHAL_TxGainPresets

    uint32_t                  reserved_0;
    uint32_t                  reserved_1;

    enum { KEY_VERSION = 1 };

    TxGainCalBKey() { memset(this, 0, sizeof(*this)); }

    void uhprint() const
    {
        UHPRINTF("(TXGAIN) Temperature: %f\n", temperature);
        UHPRINTF("      TX Gain Preset: %d (%s)\n", tx_gain_preset,
                 RHAL_TxGainPresets(tx_gain_preset) < NUM_TxGAIN_PRESETS ?
                 tx_gain_names[tx_gain_preset] : "N/A");
    }

    void upgrade_from_version(uint32_t, TxGainCalData&) { }

    bool compare(const TxGainCalBKey& other) const
    {
        return other.tx_gain_preset == tx_gain_preset;
    }

    FLOAT distance(const TxGainCalBKey& other) const
    {
        return compare(other) ? uh_fabsf(temperature - other.temperature) : CAL_KEY_DO_NOT_USE;
    }
};

SRS_CLOSE_NAMESPACE()

#endif
