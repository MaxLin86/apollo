// Copyright (C) Uhnder, Inc. All rights reserved. Confidential and Proprietary - under NDA.
// Refer to SOFTWARE_LICENSE file for details
#ifndef SRS_HDR_QILO_CALB_FLASHDATA_H
#define SRS_HDR_QILO_CALB_FLASHDATA_H 1

#include "modules/drivers/radar/rocket_radar/driver/system-radar-software/env-reference/coredefs/uhnder-common.h"
#include "modules/drivers/radar/rocket_radar/driver/system-radar-software/engine/common/eng-api/rhal_out.h"

SRS_DECLARE_NAMESPACE()

struct QiloCalBDataRaw
{
    /* temperature and carrier dependent */
    uint8_t dcap_hv[NUM_RX_PER_BANK];
    uint8_t qilo_dcaps[NUM_TX];

    /* obsolete and unused (redundant with DC cal) */
    uint8_t reserved_vga2_config__bn_i[NUM_RX_PER_BANK];
    uint8_t reserved_vga2_config__bn_q[NUM_RX_PER_BANK];
    uint8_t reserved_vga2_config__bp_i[NUM_RX_PER_BANK];
    uint8_t reserved_vga2_config__bp_q[NUM_RX_PER_BANK];
    uint8_t reserved_rxfe3_config__rx_mx_i_bc[NUM_RX_PER_BANK];
    uint8_t reserved_rxfe3_generic__rx_mx_q_bc[NUM_RX_PER_BANK];

    /* boot calibration - these values are known to get track and hold
     * circuit above 0.95V and thus biased. They really don't belong here
     * long-term */
    uint8_t vga0_generic__bias4_i[NUM_RX_PER_BANK];
    uint8_t vga2_generic__bias4_q[NUM_RX_PER_BANK];

    void set_reg_defaults();

    void uhprint() const
    {
        UHPRINTF("RX dcap_hv: %u, %u, %u, %u, %u, %u, %u, %u\n",
                dcap_hv[0], dcap_hv[1], dcap_hv[2], dcap_hv[3],
                dcap_hv[4], dcap_hv[5], dcap_hv[6], dcap_hv[7]);
        UHPRINTF("TX dcaps  : %u, %u, %u, %u, %u, %u, %u, %u, %u, %u, %u, %u\n",
                qilo_dcaps[0], qilo_dcaps[1], qilo_dcaps[2], qilo_dcaps[3],
                qilo_dcaps[4], qilo_dcaps[5], qilo_dcaps[6], qilo_dcaps[7],
                qilo_dcaps[8], qilo_dcaps[9], qilo_dcaps[10], qilo_dcaps[11]);
    }
};


struct QiloCalBData : public QiloCalBDataRaw
{
    uint8_t padding[256 - sizeof(QiloCalBDataRaw)];

    void set_defaults()
    {
        set_reg_defaults();
        memset(&padding[0], 0xFFU, sizeof(padding));
    }
};

SRS_CLOSE_NAMESPACE()

#endif
