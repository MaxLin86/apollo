#ifndef SRS_HDR_LDO_CAL_FLASHDATA_H
#define SRS_HDR_LDO_CAL_FLASHDATA_H 1
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

#include "modules/drivers/radar/quill_radar/driver/system-radar-software/env-uhnder/coredefs/uhnder-common.h"
#include "modules/drivers/radar/quill_radar/driver/system-radar-software/env-uhnder/coredefs/uhmathtypes.h"
#include "modules/drivers/radar/quill_radar/driver/system-radar-software/engine/scp-src/eng-api/default-presets.h"

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
