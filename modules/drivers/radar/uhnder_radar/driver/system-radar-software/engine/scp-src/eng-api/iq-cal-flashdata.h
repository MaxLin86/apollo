#ifndef SRS_HDR_IQ_CAL_FLASHDATA_H
#define SRS_HDR_IQ_CAL_FLASHDATA_H 1
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

#include "modules/drivers/radar/uhnder_radar/driver/system-radar-software/env-uhnder/coredefs/uhnder-common.h"
#include "modules/drivers/radar/uhnder_radar/driver/system-radar-software/engine/common/eng-api/rhal_out.h"
#include "modules/drivers/radar/uhnder_radar/driver/system-radar-software/engine/scp-src/eng-api/default-presets.h"

SRS_DECLARE_NAMESPACE()

struct IQCalRxData
{
    FLOAT                       mat_cd[2];      // The bottom row of the 2x2 IQ correction matrix (multiplies Q sample)
    uint8_t                     qilo_xc;        // QILO cross-coupling strength ('rx%d_qilocfg_generic'%(ch), 'cdac_quad')
    uint8_t                     reserved_1;
    uint8_t                     reserved_2;
    uint8_t                     reserved_3;
};

struct IQCalData
{
    IQCalRxData                 rx[NUM_RX_PER_BANK];    // Per-Rx channel corrections

    void uhprint() const
    {
        for (INT i = 0; i < NUM_RX_PER_BANK; i++)
        {
            UHPRINTF("rx[%d]:   mat_cd:[%6.4f,%6.4f], qilo_xc:%d\n", i, rx[i].mat_cd[0], rx[i].mat_cd[1], rx[i].qilo_xc);
        }
    }
};

struct IqCalKey
{
    FLOAT                       temperature;      // Internal chip temperature (deg C) at which this cal was performed
    FLOAT                       carrier_freq;     // Carrier frequency in GHz at which this cal was performed
    RHAL_RxBWPresets            rx_bw_enum;       // Rx bandwidths
    RHAL_RxVGAGainPresets       rx_vga_gain_enum; // Rx VGA gains
    RHAL_LOMode                 LO_mode;          // Internal LO / On-Board LO (OBLO) / External LO
    // TODO: Might need Tx side stuff?  (tx_ant_bmap, RHAL_TxBWPresets)
    // TODO: Might need LNA 0/1 (rx_ant_bmap)

    uint32_t                    reserved_0;
    uint32_t                    reserved_1;
    uint32_t                    reserved_2;
    uint32_t                    reserved_3;
    uint32_t                    reserved_4;
    uint32_t                    reserved_5;
    uint32_t                    reserved_6;
    uint32_t                    reserved_7;

    enum { KEY_VERSION = 0 };

    IqCalKey() { memset(this, 0, sizeof(*this)); }
    IqCalKey(FLOAT                  _temperature,
             FLOAT                  _carrier_freq,
             RHAL_RxBWPresets       _rx_bw_enum,
             RHAL_RxVGAGainPresets  _rx_vga_gain_enum,
             RHAL_LOMode            _LO_mode)
    {
        memset(this, 0, sizeof(*this));
        temperature = _temperature;
        carrier_freq = _carrier_freq;
        rx_bw_enum = _rx_bw_enum;
        rx_vga_gain_enum = _rx_vga_gain_enum;
        LO_mode = _LO_mode;
    }

    void uhprint() const
    {
        UHPRINTF("(IQ) temperature: %f\n", temperature);
        UHPRINTF("carrier_freq: %f\n", carrier_freq);
        UHPRINTF("rx_bw_enum: %d %s\n", rx_bw_enum, rx_bw_names[rx_bw_enum]);
        UHPRINTF("rx_vga_gain_enum: %d %s\n", rx_vga_gain_enum, rx_gain_names[rx_vga_gain_enum]);
        UHPRINTF("Lo_mode: %d\n", LO_mode);
    }

    void upgrade_from_version(uint32_t, IQCalData&)
    {
        // TODO: Upgrade an old key to the current key version, initialize
        // new key fields to sane default values
    }

    //!! This is the all-important key-search algorithm !!
    bool compare(const IqCalKey& other) const
    {
        return (//(uh_fabsf(temperature - other.temperature) <= 10.0F) &&     // deg C
                (uh_fabsf(carrier_freq - other.carrier_freq) <= 0.2F) &&    // GHz
                (rx_bw_enum  == other.rx_bw_enum) &&
                (rx_vga_gain_enum == other.rx_vga_gain_enum) &&
                (LO_mode == other.LO_mode));
    }

    // find the 'nearest-match' by picking the key with the shortest disance
    // from the reference key
    FLOAT distance(const IqCalKey& other) const
    {
        FLOAT dist = uh_fabsf(temperature - other.temperature) +
                     uh_fabsf(carrier_freq - other.carrier_freq) * 5;
        INT diffs = !(rx_bw_enum == other.rx_bw_enum);
        diffs += !(rx_vga_gain_enum == other.rx_vga_gain_enum);
        diffs += !(LO_mode == other.LO_mode);
        return dist + CAL_KEY_DO_NOT_USE * diffs;
    }
};


SRS_CLOSE_NAMESPACE()

#endif // SRS_HDR_IQ_CAL_FLASHDATA_H
