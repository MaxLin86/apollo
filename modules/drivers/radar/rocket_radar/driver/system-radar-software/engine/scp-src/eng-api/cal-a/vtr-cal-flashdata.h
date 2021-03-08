#ifndef SRS_HDR_VTR_CAL_FLASHDATA_H
#define SRS_HDR_VTR_CAL_FLASHDATA_H 1
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
