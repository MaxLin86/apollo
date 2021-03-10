// Copyright (C) Uhnder, Inc. All rights reserved. Confidential and Proprietary - under NDA.
// Refer to SOFTWARE_LICENSE file for details
#ifndef SRS_HDR_IQ_CAL_FLASHDATA_H
#define SRS_HDR_IQ_CAL_FLASHDATA_H 1

#include "modules/drivers/radar/rocket_radar/driver/system-radar-software/env-reference/coredefs/uhnder-common.h"
#include "modules/drivers/radar/rocket_radar/driver/system-radar-software/engine/common/eng-api/rhal_out.h"
#include "modules/drivers/radar/rocket_radar/driver/system-radar-software/engine/scp-src/eng-api/default-presets.h"

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
    void set_defaults()
    {
        for (INT i = 0; i < NUM_RX_PER_BANK; i++)
        {
            rx[i].mat_cd[0] = 0.0F;
            rx[i].mat_cd[1] = 1.0F;
            rx[i].qilo_xc = 15;
        }
    }
};

struct IqCalKey
{
    FLOAT                       temperature;      // Internal chip temperature (deg C) at which this cal was performed
    FLOAT                       carrier_freq;     // Carrier frequency in GHz at which this cal was performed
    RHAL_RxBWPresets            rx_bw_enum;       // Rx bandwidths
    RHAL_RxVGAGainPresets       rx_vga_gain_enum; // Rx VGA gains
    // TODO: Might need Tx side stuff?  (tx_ant_bmap, RHAL_TxBWPresets)
    // TODO: Might need LNA 0/1 (rx_ant_bmap)

    uint32_t                    reserved_LO_mode;
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

    void uhprint() const
    {
        UHPRINTF("(IQ) temperature: %f\n", temperature);
        UHPRINTF("carrier_freq: %f\n", carrier_freq);
        UHPRINTF("rx_bw_enum: %d %s\n", rx_bw_enum, rx_bw_enum < NUM_RXBW_PRESETS ? rx_bw_names[rx_bw_enum] : "N/A");
        UHPRINTF("rx_vga_gain_enum: %d %s\n", rx_vga_gain_enum, rx_vga_gain_enum < NUM_VGAGAIN_PRESETS ? rx_gain_names[rx_vga_gain_enum] : "N/A");
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
                (rx_vga_gain_enum == other.rx_vga_gain_enum));
    }

    // find the 'nearest-match' by picking the key with the shortest disance
    // from the reference key
    FLOAT distance(const IqCalKey& other) const
    {
        FLOAT dist = uh_fabsf(temperature - other.temperature) +
                     uh_fabsf(carrier_freq - other.carrier_freq) * 5;
        INT diffs = !(rx_bw_enum == other.rx_bw_enum);
        diffs += !(rx_vga_gain_enum == other.rx_vga_gain_enum);
        return dist + CAL_KEY_DO_NOT_USE * float(diffs);
    }
};


SRS_CLOSE_NAMESPACE()

#endif // SRS_HDR_IQ_CAL_FLASHDATA_H
