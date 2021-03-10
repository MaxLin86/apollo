// Copyright (C) Uhnder, Inc. All rights reserved. Confidential and Proprietary - under NDA.
// Refer to SOFTWARE_LICENSE file for details
#ifndef SRS_HDR_DC_CALB_FLASHDATA_H
#define SRS_HDR_DC_CALB_FLASHDATA_H 1

#include "modules/drivers/radar/rocket_radar/driver/system-radar-software/env-reference/coredefs/uhnder-common.h"
#include "modules/drivers/radar/rocket_radar/driver/system-radar-software/engine/common/eng-api/rhal_out.h"
#include "rdc-common.h"

SRS_DECLARE_NAMESPACE()

struct DCCalBData074
{
    struct VgaDcData
    {
        uint8_t vga0_generic__bf_i, vga1_config__offset1_i, vga1_config__offset2_i, vga1_config__offset3_i;
        uint8_t vga2_generic__bf_q, vga1_generic__offset1_q, vga1_generic__offset2_q, vga1_generic__offset3_q;
        uint8_t vga0_generic__bias4_i, vga2_generic__bias4_q;
    };

    VgaDcData   vga[NUM_RX_PER_BANK];
    cint16      color_gain[NUM_RX_PER_BANK][NUM_ADC_LANE]; // note - could be cuint8
    cint16      color_dc[NUM_RX_PER_BANK][NUM_ADC_LANE];
    uint32_t    algorithm_version;
};


struct DCCalBData
{
    struct VgaDcData
    {
        uint8_t vga0_generic__bf_i, vga1_config__offset1_i, vga1_config__offset2_i, vga1_config__offset3_i;
        uint8_t vga2_generic__bf_q, vga1_generic__offset1_q, vga1_generic__offset2_q, vga1_generic__offset3_q;
        uint8_t vga0_generic__bias4_i, vga2_generic__bias4_q;
    };

    VgaDcData   vga[NUM_RX_PER_BANK];
    cint16      color_gain[NUM_RX_PER_BANK][NUM_ADC_LANE]; // note - could be cuint8
    cint16      color_dc[NUM_RX_PER_BANK][NUM_ADC_LANE];

    uint8_t     vga2_config__bn_i[NUM_RX_PER_BANK];
    uint8_t     vga2_config__bn_q[NUM_RX_PER_BANK];
    uint8_t     vga2_config__bp_i[NUM_RX_PER_BANK];
    uint8_t     vga2_config__bp_q[NUM_RX_PER_BANK];

    uint32_t    algorithm_version;
    uint32_t    reserved[32];

    void uhprint(bool all=false) const
    {
        for (uint32_t rx = 0; rx < NUM_RX_PER_BANK; rx++)
        {
            UHPRINTF("RX[%d] VGA IOFF: %u, %u, %u QOFF %u %u %u BQF %ui, %uq BIAS %ui %uq\n", rx,
                    vga[rx].vga1_config__offset1_i,  vga[rx].vga1_config__offset2_i, vga[rx].vga1_config__offset3_i,
                    vga[rx].vga1_generic__offset1_q, vga[rx].vga1_generic__offset2_q, vga[rx].vga1_generic__offset3_q,
                    vga[rx].vga0_generic__bf_i,      vga[rx].vga2_generic__bf_q,
                    vga[rx].vga0_generic__bias4_i,   vga[rx].vga2_generic__bias4_q);
            UHPRINTF("RX[%d] Color Gain IOFF: %u, %u, %u, %u, %u, %u, %u, %u, %u\n", rx,
                    color_gain[rx][0].i, color_gain[rx][1].i, color_gain[rx][2].i,
                    color_gain[rx][3].i, color_gain[rx][4].i, color_gain[rx][5].i,
                    color_gain[rx][6].i, color_gain[rx][7].i, color_gain[rx][8].i);
            UHPRINTF("RX[%d] Color Gain QOFF: %u, %u, %u, %u, %u, %u, %u, %u, %u\n", rx,
                    color_gain[rx][0].q, color_gain[rx][1].q, color_gain[rx][2].q,
                    color_gain[rx][3].q, color_gain[rx][4].q, color_gain[rx][5].q,
                    color_gain[rx][6].q, color_gain[rx][7].q, color_gain[rx][8].q);
            UHPRINTF("RX[%d] Color DC IOFF: %d, %d, %d, %d, %d, %d, %d, %d, %d\n", rx,
                    color_dc[rx][0].i, color_dc[rx][1].i, color_dc[rx][2].i,
                    color_dc[rx][3].i, color_dc[rx][4].i, color_dc[rx][5].i,
                    color_dc[rx][6].i, color_dc[rx][7].i, color_dc[rx][8].i);
            UHPRINTF("RX[%d] Color DC QOFF: %d, %d, %d, %d, %d, %d, %d, %d, %d\n", rx,
                    color_dc[rx][0].q, color_dc[rx][1].q, color_dc[rx][2].q,
                    color_dc[rx][3].q, color_dc[rx][4].q, color_dc[rx][5].q,
                    color_dc[rx][6].q, color_dc[rx][7].q, color_dc[rx][8].q);
            UHPRINTF("RX[%d] BNi: %u BPi: %u, BNq: %u BPq: %u\n", rx,
                    vga2_config__bn_i[rx], vga2_config__bp_i[rx], vga2_config__bn_q[rx], vga2_config__bp_q[rx]);
            if (!all)
                break;
        }
    }

    void set_defaults();
};

SRS_CLOSE_NAMESPACE()

#endif // SRS_HDR_DC_CALB_FLASHDATA_H
