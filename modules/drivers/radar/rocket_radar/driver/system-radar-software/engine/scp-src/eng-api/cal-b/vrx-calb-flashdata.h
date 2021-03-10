// Copyright (C) Uhnder, Inc. All rights reserved. Confidential and Proprietary - under NDA.
// Refer to SOFTWARE_LICENSE file for details
#ifndef SRS_HDR_VRX_CALB_FLASHDATA_H
#define SRS_HDR_VRX_CALB_FLASHDATA_H 1

#include "modules/drivers/radar/rocket_radar/driver/system-radar-software/env-reference/coredefs/uhnder-common.h"
#include "modules/drivers/radar/rocket_radar/driver/system-radar-software/engine/common/eng-api/rhal_out.h"
#include "rdc-common.h"

SRS_DECLARE_NAMESPACE()

struct VrxAlignCalBData
{
    //! relative TX delay in units of picoseconds, +ve moves target away
    //! all delay_tx values must be greater than or equal to zero
    FLOAT     delay_tx[NUM_TX];

    //! relative RX delay in units of picoseconds, +ve moves target closer
    //! can be negative or positive
    FLOAT     delay_rx[NUM_RX_BANKS][NUM_RX_PER_BANK];

    //! measured ADI delay_computed as seen with the above correction applied
    //! NOT USED
    uint8_t   delay_computed[NUM_RX_BANKS][NUM_RX_PER_BANK];

    // The net effect of applying both the RX and TX delays should be that
    // targets appear at the correct range, and all virtual receivers are
    // aligned in range (and thus in time)

    uint32_t  padding[2 * (NUM_TX + (NUM_RX_BANKS * NUM_RX_PER_BANK)) - 4];

    void uhprint() const
    {
        for (uint32_t i = 0; i < NUM_TX; i++)
        {
            if (i < NUM_RX_PER_BANK)
            {
                UHPRINTF("TX[%d] delay: %5.1f  RX[%d] delay: Bank0 %5.1f (%d) Bank1 %5.1f (%d)\n",
                        i, delay_tx[i],
                        i, delay_rx[0][i], delay_computed[0][i],
                           delay_rx[1][i], delay_computed[1][i]);
            }
            else
            {
                UHPRINTF("TX[%d] delay: %5.1f\n", i, delay_tx[i]);
            }
        }
    }

    void set_defaults()
    {
        memset(this, 0, sizeof(*this));
    }
};

SRS_CLOSE_NAMESPACE()

#endif // SRS_HDR_VRX_CALB_FLASHDATA_H
