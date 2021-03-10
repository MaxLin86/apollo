// Copyright (C) Uhnder, Inc. All rights reserved. Confidential and Proprietary - under NDA.
// Refer to SOFTWARE_LICENSE file for details
#ifndef SRS_HDR_VTR_CAL_FLASHDATA_H
#define SRS_HDR_VTR_CAL_FLASHDATA_H 1

#include "modules/drivers/radar/rocket_radar/driver/system-radar-software/env-reference/coredefs/uhnder-common.h"
#include "modules/drivers/radar/rocket_radar/driver/system-radar-software/engine/scp-src/eng-api/uhmath.h"

SRS_DECLARE_NAMESPACE()

struct VTRCalData
{
    uint8_t tx0_sdac_config_vtr_code;
    uint8_t tx1_sdac_config_vtr_code;
    uint8_t tx2_sdac_config_vtr_code;
    uint8_t tx3_sdac_config_vtr_code;
    uint8_t tx4_sdac_config_vtr_code;
    uint8_t tx5_sdac_config_vtr_code;
    uint8_t tx6_sdac_config_vtr_code;
    uint8_t tx7_sdac_config_vtr_code;
    uint8_t tx8_sdac_config_vtr_code;
    uint8_t tx9_sdac_config_vtr_code;
    uint8_t tx10_sdac_config_vtr_code;
    uint8_t tx11_sdac_config_vtr_code;

    uint8_t tx0_bias_diag_tx_vtr_code;
    uint8_t tx1_bias_diag_tx_vtr_code;
    uint8_t tx2_bias_diag_tx_vtr_code;
    uint8_t tx3_bias_diag_tx_vtr_code;
    uint8_t tx4_bias_diag_tx_vtr_code;
    uint8_t tx5_bias_diag_tx_vtr_code;
    uint8_t tx6_bias_diag_tx_vtr_code;
    uint8_t tx7_bias_diag_tx_vtr_code;
    uint8_t tx8_bias_diag_tx_vtr_code;
    uint8_t tx9_bias_diag_tx_vtr_code;
    uint8_t tx10_bias_diag_tx_vtr_code;
    uint8_t tx11_bias_diag_tx_vtr_code;

    uint8_t tx0_qilocfg_config_vtr_temp; // incorrectly named register, actually controls vtr_code
    uint8_t tx1_qilocfg_config_vtr_temp;
    uint8_t tx2_qilocfg_config_vtr_temp;
    uint8_t tx3_qilocfg_config_vtr_temp;
    uint8_t tx4_qilocfg_config_vtr_temp;
    uint8_t tx5_qilocfg_config_vtr_temp;
    uint8_t tx6_qilocfg_config_vtr_temp;
    uint8_t tx7_qilocfg_config_vtr_temp;
    uint8_t tx8_qilocfg_config_vtr_temp;
    uint8_t tx9_qilocfg_config_vtr_temp;
    uint8_t tx10_qilocfg_config_vtr_temp;
    uint8_t tx11_qilocfg_config_vtr_temp;

    uint8_t rx0_vga1_config_vt_code;
    uint8_t rx1_vga1_config_vt_code;
    uint8_t rx2_vga1_config_vt_code;
    uint8_t rx3_vga1_config_vt_code;
    uint8_t rx4_vga1_config_vt_code;
    uint8_t rx5_vga1_config_vt_code;
    uint8_t rx6_vga1_config_vt_code;
    uint8_t rx7_vga1_config_vt_code;

    uint8_t rx0_sdac_config_vtr_code;
    uint8_t rx1_sdac_config_vtr_code;
    uint8_t rx2_sdac_config_vtr_code;
    uint8_t rx3_sdac_config_vtr_code;
    uint8_t rx4_sdac_config_vtr_code;
    uint8_t rx5_sdac_config_vtr_code;
    uint8_t rx6_sdac_config_vtr_code;
    uint8_t rx7_sdac_config_vtr_code;

    uint8_t rx0_qilocfg_config_vtr_temp;
    uint8_t rx1_qilocfg_config_vtr_temp;
    uint8_t rx2_qilocfg_config_vtr_temp;
    uint8_t rx3_qilocfg_config_vtr_temp;
    uint8_t rx4_qilocfg_config_vtr_temp;
    uint8_t rx5_qilocfg_config_vtr_temp;
    uint8_t rx6_qilocfg_config_vtr_temp;
    uint8_t rx7_qilocfg_config_vtr_temp;

    uint8_t rx0_adc_ldo_control_0_rx0_adc_ldo_control_0_vtr_code;
    uint8_t rx1_adc_ldo_control_0_rx1_adc_ldo_control_0_vtr_code;
    uint8_t rx2_adc_ldo_control_0_rx2_adc_ldo_control_0_vtr_code;
    uint8_t rx3_adc_ldo_control_0_rx3_adc_ldo_control_0_vtr_code;
    uint8_t rx4_adc_ldo_control_0_rx4_adc_ldo_control_0_vtr_code;
    uint8_t rx5_adc_ldo_control_0_rx5_adc_ldo_control_0_vtr_code;
    uint8_t rx6_adc_ldo_control_0_rx6_adc_ldo_control_0_vtr_code;
    uint8_t rx7_adc_ldo_control_0_rx7_adc_ldo_control_0_vtr_code;

    uint8_t rx0_rxfe1_config_lna_stg12_vtr_vtr_code;
    uint8_t rx1_rxfe1_config_lna_stg12_vtr_vtr_code;
    uint8_t rx2_rxfe1_config_lna_stg12_vtr_vtr_code;
    uint8_t rx3_rxfe1_config_lna_stg12_vtr_vtr_code;
    uint8_t rx4_rxfe1_config_lna_stg12_vtr_vtr_code;
    uint8_t rx5_rxfe1_config_lna_stg12_vtr_vtr_code;
    uint8_t rx6_rxfe1_config_lna_stg12_vtr_vtr_code;
    uint8_t rx7_rxfe1_config_lna_stg12_vtr_vtr_code;

    uint8_t sh_synth2_diag_vtr_code;
    uint8_t sh_lo_generic_vtr_code;
    uint8_t sh_clkdiv1_generic_vtr_code;
    uint8_t sh_bias_config_vtr_code;

    uint8_t reserved[48]; // pad to 128 bytes

    void set_defaults()
    {
        memset(this, 0xFFU, sizeof(*this));
    }

    void uhprint() const
    {
        UHPRINTF("      sdac\tbias\tqilo\n");
        UHPRINTF("===== ====\t====\t====\n");
        const uint8_t* txval = reinterpret_cast<const uint8_t*>(&tx0_sdac_config_vtr_code);
        for (INT t = 0; t < 12; t++)
        {
            CHAR line[80];
            INT size = sprintf(line, "TX %02d ", t);
            for (INT f = 0; f < 3; f++)
            {
                size += sprintf(line + size, "%02d\t", txval[12 * f + t]);
            }
            UHPRINTF("%s\n", line);
        }

        UHPRINTF("      vga1\tsdac\tqilo\tadc\tlna\n");
        UHPRINTF("===== ====\t====\t====\t====\t====\n");
        const uint8_t* rxval = reinterpret_cast<const uint8_t*>(&rx0_vga1_config_vt_code);
        for (INT r = 0; r < 8; r++)
        {
            CHAR line[80];
            INT size = sprintf(line, "RX %02d ", r);
            for (INT f = 0; f < 5; f++)
            {
                size += sprintf(line + size, "%0d\t", rxval[8 * f + r]);
            }
            UHPRINTF("%s\n", line);
        }

        UHPRINTF("SH: synth2_diag_vtr %02d lo_generic_vtr %02d clkdiv1_generic_vtr %02d bias_config_vtr %02d\n",
                 sh_synth2_diag_vtr_code, sh_lo_generic_vtr_code, sh_clkdiv1_generic_vtr_code, sh_bias_config_vtr_code);
    }
};

struct VTRCalKey
{
    FLOAT    chip_temperature; // Celcius
    FLOAT    carrier_freq;     // GHz
    FLOAT    analog_high;      // volts
    FLOAT    analog_low;       // volts
    uint32_t reserved[4];

    enum { KEY_VERSION = 1 };

    bool compare(const VTRCalKey&) const
    {
        return true;
    }

    FLOAT distance(const VTRCalKey& other) const
    {
        // try to scale the units to equivalent relevance
        const FLOAT voltage_weight = 0.05f;
        const FLOAT temp_C_weight  = 20.0F;
        const FLOAT rf_weight      = 1.0F;

        return uh_fabsf(other.analog_low       - analog_low)       / voltage_weight +
               uh_fabsf(other.analog_high      - analog_high)      / voltage_weight +
               uh_fabsf(other.carrier_freq     - carrier_freq)     / rf_weight +
               uh_fabsf(other.chip_temperature - chip_temperature) / temp_C_weight;
    }

    void upgrade_from_version(uint32_t, struct VTRCalData&)
    {
    }

    void uhprint() const
    {
        UHPRINTF("(VTR Cal) analog_high: %.2f", analog_high);
        UHPRINTF("           analog_low: %.2f", analog_low);
        UHPRINTF("     chip_temperature: %.1f", chip_temperature);
        UHPRINTF("         carrier_freq: %.3f", carrier_freq);
    }
};

SRS_CLOSE_NAMESPACE()

#endif
