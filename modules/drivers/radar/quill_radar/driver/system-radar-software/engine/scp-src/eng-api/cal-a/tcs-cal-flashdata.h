#ifndef SRS_HDR_TCS_CAL_FLASHDATA_H
#define SRS_HDR_TCS_CAL_FLASHDATA_H 1
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
#include "modules/drivers/radar/quill_radar/driver/system-radar-software/engine/common/eng-api/rhal_out.h"
#include "modules/drivers/radar/quill_radar/driver/system-radar-software/engine/scp-src/eng-api/default-presets.h"

SRS_DECLARE_NAMESPACE()


struct CarrSupData
{
    uint8_t             itrim_p_en_hv_i; // tx0_idac_generic : itrim_p_en_hv
    uint8_t             itrim_n_en_hv_i; // tx0_idac_generic : itrim_n_en_hv
    uint8_t             itrim_p_en_hv_q; // tx0_qdac_generic : itrim_p_en_hv
    uint8_t             itrim_n_en_hv_q; // tx0_qdac_generic : itrim_n_en_hv
    int8_t              dc_offset_i;     // dc_offset_tx00 : dc_offset_correct_i
    int8_t              dc_offset_q;     // dc_offset_tx00 : dc_offset_correct_q
};

struct TCSCalData
{
    CarrSupData tx_carrier_sup_data[NUM_TX];

    void uhprint(bool all=false) const
    {
        for (uint32_t i = 0; i < NUM_TX; i++)
        {
            UHPRINTF("TX[%d] itrim P %u %u N %u %u DC %d %d\n", i,
                    tx_carrier_sup_data[i].itrim_p_en_hv_i,
                    tx_carrier_sup_data[i].itrim_p_en_hv_q,
                    tx_carrier_sup_data[i].itrim_n_en_hv_i,
                    tx_carrier_sup_data[i].itrim_n_en_hv_q,
                    tx_carrier_sup_data[i].dc_offset_i,
                    tx_carrier_sup_data[i].dc_offset_q);
            if (!all) break;
        }
    }

    void set_defaults()
    {
        for (uint32_t i = 0; i < NUM_TX; i++)
        {
            tx_carrier_sup_data[i].itrim_p_en_hv_i = 77; // Sabine-B defaults
            tx_carrier_sup_data[i].itrim_p_en_hv_q = 77;
            tx_carrier_sup_data[i].itrim_n_en_hv_i = 77;
            tx_carrier_sup_data[i].itrim_n_en_hv_q = 77;
            tx_carrier_sup_data[i].dc_offset_i = 0;
            tx_carrier_sup_data[i].dc_offset_q = 0;
        }
    }
};


struct TCSKey
{
    FLOAT                     temp;         // Chip temperature C at which this cal was performed
    uint32_t                  reserved_tx_ant_bmap;  // Out of 1 Txs (RevA)
    uint32_t                  reserved_rx_ant_bmap;  // Out of 16 Rxs (RevA)
    RHAL_TxBWPresets          tx_bw_enum;
    RHAL_RxBWPresets          reserved_rx_bw_enum;
    RHAL_RxVGAGainPresets     reserved_rx_vga_gain_enum;
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

    enum { KEY_VERSION = 1 };

    TCSKey() { memset(this, 0, sizeof(*this)); }

    void upgrade_from_version(uint32_t old_version, TCSCalData&)
    {
        if (old_version < 1)
        {
            carrier_freq = DEF_RF;
        }
    }

    void uhprint() const
    {
        UHPRINTF("(TCS) temp: %f\n", temp);
        UHPRINTF("tx_bw_enum: %d %s\n", tx_bw_enum, tx_bw_names[tx_bw_enum]);
        UHPRINTF("tx_gain_enum: %d %s\n", tx_gain_enum, tx_gain_names[tx_gain_enum]);
        UHPRINTF("Lo_mode: %d\n", LO_mode);
        UHPRINTF("carrier_freq: %.2f\n", carrier_freq);
    }

    //!! This is the all-important key-search algorithm !!
    bool compare(const TCSKey& other) const
    {
        return (//uh_fabsf(temp - other.temp) < 1.5F &&
                (uh_fabsf(carrier_freq - other.carrier_freq) <= 0.2F) &&
                tx_bw_enum == other.tx_bw_enum &&
                LO_mode == other.LO_mode &&
                tx_gain_enum == other.tx_gain_enum);
    }

    // find the 'nearest-match' by picking the key with the shortest disance
    // from the reference key
    FLOAT distance(const TCSKey& other) const
    {
        FLOAT dist = uh_fabsf(temp - other.temp) + uh_fabsf(carrier_freq - other.carrier_freq) * 5;
        INT diffs = !(tx_bw_enum == other.tx_bw_enum);
        diffs += !(LO_mode == other.LO_mode);
        diffs += !(tx_gain_enum == other.tx_gain_enum);
        return dist + CAL_KEY_DO_NOT_USE * diffs;
    }
};


SRS_CLOSE_NAMESPACE()

#endif // SRS_HDR_TCS_CAL_FLASHDATA_H
