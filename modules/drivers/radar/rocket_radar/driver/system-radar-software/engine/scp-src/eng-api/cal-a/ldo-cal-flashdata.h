// Copyright (C) Uhnder, Inc. All rights reserved. Confidential and Proprietary - under NDA.
// Refer to SOFTWARE_LICENSE file for details
#ifndef SRS_HDR_LDO_CAL_FLASHDATA_H
#define SRS_HDR_LDO_CAL_FLASHDATA_H 1

#include "modules/drivers/radar/rocket_radar/driver/system-radar-software/env-reference/coredefs/uhnder-common.h"
#include "modules/drivers/radar/rocket_radar/driver/system-radar-software/env-reference/coredefs/uhmathtypes.h"
#include "modules/drivers/radar/rocket_radar/driver/system-radar-software/engine/scp-src/eng-api/default-presets.h"

SRS_DECLARE_NAMESPACE()

struct LDOCalKey
{
    uint32_t tx_power_mode;
    FLOAT    temperature;
    FLOAT    carrier_freq;
    uint32_t reserved_0;

    enum { KEY_VERSION = 1 };

    bool compare(const LDOCalKey& other) const
    {
        return tx_power_mode == other.tx_power_mode;
    }

    FLOAT distance(const LDOCalKey& other) const
    {
        // ignoring carrier_freq
        return compare(other) ? uh_fabsf(temperature - other.temperature) : CAL_KEY_DO_NOT_USE;
    }

    void upgrade_from_version(uint32_t old_version, struct LDOCalData&)
    {
        (void)old_version;
    }

    void upgrade_from_version(uint32_t old_version, struct LDOCalBData&)
    {
        (void)old_version;
    }

    void uhprint() const
    {
        UHPRINTF("(LDO Cal) tx_power_mode: %d\n", tx_power_mode);
        UHPRINTF("            temperature: %.1f\n", temperature);
        UHPRINTF("           carrier_freq: %.3f\n", carrier_freq);
    }
};

struct LDOCalData
{
    uint8_t tx0_sdac_diag_curr_dac1;
    uint8_t tx0_sdac_diag_curr_dac0;
    uint8_t tx0_qilocfg_diag_curr_dac;
    uint8_t tx0_qilocfg_diag_v1p5_ldo_dac;
    uint8_t tx1_sdac_diag_curr_dac1;
    uint8_t tx1_sdac_diag_curr_dac0;
    uint8_t tx1_qilocfg_diag_curr_dac;
    uint8_t tx1_qilocfg_diag_v1p5_ldo_dac;
    uint8_t tx2_sdac_diag_curr_dac1;
    uint8_t tx2_sdac_diag_curr_dac0;
    uint8_t tx2_qilocfg_diag_curr_dac;
    uint8_t tx2_qilocfg_diag_v1p5_ldo_dac;
    uint8_t tx3_sdac_diag_curr_dac1;
    uint8_t tx3_sdac_diag_curr_dac0;
    uint8_t tx3_qilocfg_diag_curr_dac;
    uint8_t tx3_qilocfg_diag_v1p5_ldo_dac;
    uint8_t tx4_sdac_diag_curr_dac1;
    uint8_t tx4_sdac_diag_curr_dac0;
    uint8_t tx4_qilocfg_diag_curr_dac;
    uint8_t tx4_qilocfg_diag_v1p5_ldo_dac;
    uint8_t tx5_sdac_diag_curr_dac1;
    uint8_t tx5_sdac_diag_curr_dac0;
    uint8_t tx5_qilocfg_diag_curr_dac;
    uint8_t tx5_qilocfg_diag_v1p5_ldo_dac;
    uint8_t tx6_sdac_diag_curr_dac1;
    uint8_t tx6_sdac_diag_curr_dac0;
    uint8_t tx6_qilocfg_diag_curr_dac;
    uint8_t tx6_qilocfg_diag_v1p5_ldo_dac;
    uint8_t tx7_sdac_diag_curr_dac1;
    uint8_t tx7_sdac_diag_curr_dac0;
    uint8_t tx7_qilocfg_diag_curr_dac;
    uint8_t tx7_qilocfg_diag_v1p5_ldo_dac;
    uint8_t tx8_sdac_diag_curr_dac1;
    uint8_t tx8_sdac_diag_curr_dac0;
    uint8_t tx8_qilocfg_diag_curr_dac;
    uint8_t tx8_qilocfg_diag_v1p5_ldo_dac;
    uint8_t tx9_sdac_diag_curr_dac1;
    uint8_t tx9_sdac_diag_curr_dac0;
    uint8_t tx9_qilocfg_diag_curr_dac;
    uint8_t tx9_qilocfg_diag_v1p5_ldo_dac;
    uint8_t tx10_sdac_diag_curr_dac1;
    uint8_t tx10_sdac_diag_curr_dac0;
    uint8_t tx10_qilocfg_diag_curr_dac;
    uint8_t tx10_qilocfg_diag_v1p5_ldo_dac;
    uint8_t tx11_sdac_diag_curr_dac1;
    uint8_t tx11_sdac_diag_curr_dac0;
    uint8_t tx11_qilocfg_diag_curr_dac;
    uint8_t tx11_qilocfg_diag_v1p5_ldo_dac;
    uint8_t rx0_sdac_diag_curr_dac1;
    uint8_t rx0_sdac_diag_curr_dac0;
    uint8_t rx0_qilocfg_diag_cur_dac;
    uint8_t rx0_qilocfg_diag_v1p5_ldo_dac;
    uint8_t rx1_sdac_diag_curr_dac1;
    uint8_t rx1_sdac_diag_curr_dac0;
    uint8_t rx1_qilocfg_diag_cur_dac;
    uint8_t rx1_qilocfg_diag_v1p5_ldo_dac;
    uint8_t rx2_sdac_diag_curr_dac1;
    uint8_t rx2_sdac_diag_curr_dac0;
    uint8_t rx2_qilocfg_diag_cur_dac;
    uint8_t rx2_qilocfg_diag_v1p5_ldo_dac;
    uint8_t rx3_sdac_diag_curr_dac1;
    uint8_t rx3_sdac_diag_curr_dac0;
    uint8_t rx3_qilocfg_diag_cur_dac;
    uint8_t rx3_qilocfg_diag_v1p5_ldo_dac;
    uint8_t rx4_sdac_diag_curr_dac1;
    uint8_t rx4_sdac_diag_curr_dac0;
    uint8_t rx4_qilocfg_diag_cur_dac;
    uint8_t rx4_qilocfg_diag_v1p5_ldo_dac;
    uint8_t rx5_sdac_diag_curr_dac1;
    uint8_t rx5_sdac_diag_curr_dac0;
    uint8_t rx5_qilocfg_diag_cur_dac;
    uint8_t rx5_qilocfg_diag_v1p5_ldo_dac;
    uint8_t rx6_sdac_diag_curr_dac1;
    uint8_t rx6_sdac_diag_curr_dac0;
    uint8_t rx6_qilocfg_diag_cur_dac;
    uint8_t rx6_qilocfg_diag_v1p5_ldo_dac;
    uint8_t rx7_sdac_diag_curr_dac1;
    uint8_t rx7_sdac_diag_curr_dac0;
    uint8_t rx7_qilocfg_diag_cur_dac;
    uint8_t rx7_qilocfg_diag_v1p5_ldo_dac;
    uint8_t sh_lo_generic_ldo_cur_dac;
    uint8_t reserved[47]; // pad to 128 bytes

    void set_defaults()
    {
        memset(this, 0xFFU, sizeof(*this));
    }

    void uhprint() const
    {
        UHPRINTF("      dac1 dac0 qilo v1p5\n");
        UHPRINTF("      ==== ==== ==== ====\n");
        const uint8_t *vals = reinterpret_cast<const uint8_t*>(this);
        CHAR type = 'T';
        INT channel = 0;
        for (size_t i = 0; i < sizeof(*this); i += 4)
        {
            UHPRINTF("%cX%02d: %02d   %02d   %02d   %02d\n", type, channel,
                    vals[i + 0], vals[i + 1], vals[i + 2], vals[i + 3]);
            channel++;
            if (channel == 12 && type == 'T')
            {
                type = 'R';
                channel = 0;
            }
            else if (channel == 8 && type == 'R')
            {
                UHPRINTF("sh_lo_generic_ldo_cur_dac %02d\n", sh_lo_generic_ldo_cur_dac);
                break;
            }
        }
    }
};

SRS_CLOSE_NAMESPACE()

#endif
