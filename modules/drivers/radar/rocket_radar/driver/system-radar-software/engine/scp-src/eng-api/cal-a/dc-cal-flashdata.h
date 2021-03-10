// Copyright (C) Uhnder, Inc. All rights reserved. Confidential and Proprietary - under NDA.
// Refer to SOFTWARE_LICENSE file for details
#ifndef SRS_HDR_DC_CAL_FLASHDATA_H
#define SRS_HDR_DC_CAL_FLASHDATA_H 1

#include "modules/drivers/radar/rocket_radar/driver/system-radar-software/env-reference/coredefs/uhnder-common.h"
#include "modules/drivers/radar/rocket_radar/driver/system-radar-software/engine/common/eng-api/rhal_out.h"
#include "rdc-common.h"
#include "modules/drivers/radar/rocket_radar/driver/system-radar-software/engine/scp-src/eng-api/default-presets.h"
#if __SCP__
#include "antennas.h"
#endif

SRS_DECLARE_NAMESPACE()

enum { NUM_DAC_STG = 4 };
enum { MAX_NUM_ACCURACIES = 8 };

struct VgaDcData
{
    uint8_t             ioff_dac_stg[NUM_DAC_STG];
    uint8_t             qoff_dac_stg[NUM_DAC_STG];
    // rx0_vga2_config  : ioff_dac_stg1
    // rx0_vga2_diag    : ioff_dac_stg2_72
    // rx0_vga2_generic : ioff_dac_stg3_72
    // rx0_vga2_config  : ioff_dac_stg4_41

    // rx0_vga3_config  : qoff_dac_stg1
    // rx0_vga3_diag    : qoff_dac_stg2_72
    // rx0_vga3_generic : qoff_dac_stg3_72
    // rx0_vga3_config  : qoff_dac_stg4_41

    uint8_t             ioff_dac_bqf;       // rx0_vga2_config  : ioff_dac_bqf_74
    uint8_t             qoff_dac_bqf;       // rx0_vga3_config  : qoff_dac_bqf_74
};


struct AdcCData
{
    uint8_t             i_cdata_0[NUM_ADC_LANE];  // rx0_adc_cal_cdata_i0_0  : rx0_adc_cal_cdata_i0_0_cdata_15_0
    // rx0_adc_cal_cdata_i1_0  : rx0_adc_cal_cdata_i1_0_cdata_15_0
    // rx0_adc_cal_cdata_i2_0  : rx0_adc_cal_cdata_i2_0_cdata_15_0
    // rx0_adc_cal_cdata_i3_0  : rx0_adc_cal_cdata_i3_0_cdata_15_0
    // rx0_adc_cal_cdata_i4_0  : rx0_adc_cal_cdata_i4_0_cdata_15_0
    // rx0_adc_cal_cdata_i5_0  : rx0_adc_cal_cdata_i5_0_cdata_15_0
    // rx0_adc_cal_cdata_i6_0  : rx0_adc_cal_cdata_i6_0_cdata_15_0
    // rx0_adc_cal_cdata_i7_0  : rx0_adc_cal_cdata_i7_0_cdata_15_0
    // rx0_adc_cal_cdata_i8_0  : rx0_adc_cal_cdata_i8_0_cdata_15_0
    uint8_t             q_cdata_0[NUM_ADC_LANE];  // rx0_adc_cal_cdata_q0_0  : rx0_adc_cal_cdata_q0_0_cdata_15_0
    // rx0_adc_cal_cdata_q1_0  : rx0_adc_cal_cdata_q1_0_cdata_15_0
    // rx0_adc_cal_cdata_q2_0  : rx0_adc_cal_cdata_q2_0_cdata_15_0
    // rx0_adc_cal_cdata_q3_0  : rx0_adc_cal_cdata_q3_0_cdata_15_0
    // rx0_adc_cal_cdata_q4_0  : rx0_adc_cal_cdata_q4_0_cdata_15_0
    // rx0_adc_cal_cdata_q5_0  : rx0_adc_cal_cdata_q5_0_cdata_15_0
    // rx0_adc_cal_cdata_q6_0  : rx0_adc_cal_cdata_q6_0_cdata_15_0
    // rx0_adc_cal_cdata_q7_0  : rx0_adc_cal_cdata_q7_0_cdata_15_0
    // rx0_adc_cal_cdata_q8_0  : rx0_adc_cal_cdata_q8_0_cdata_15_0
};

struct ColorDCData
{
    int8_t             i_dcc_lane[NUM_ADC_LANE]; // rx0_i_dccx4_config_0  : rx0_i_dcc_lane0
    // rx0_i_dccx4_config_0  : rx0_i_dcc_lane1
    // rx0_i_dccx4_config_0  : rx0_i_dcc_lane2
    // rx0_i_dccx4_config_0  : rx0_i_dcc_lane3
    // rx0_i_dccx4_config_1  : rx0_i_dcc_lane4
    // rx0_i_dccx4_config_1  : rx0_i_dcc_lane5
    // rx0_i_dccx4_config_1  : rx0_i_dcc_lane6
    // rx0_i_dccx4_config_1  : rx0_i_dcc_lane7
    // rx0_i_dccx1_gainx3_config : rx0_i_dcc_lane8
    int8_t             q_dcc_lane[NUM_ADC_LANE]; // rx0_q_dccx4_config_0  : rx0_q_dcc_lane0
    // rx0_q_dccx4_config_0  : rx0_q_dcc_lane1
    // rx0_q_dccx4_config_0  : rx0_q_dcc_lane2
    // rx0_q_dccx4_config_0  : rx0_q_dcc_lane3
    // rx0_q_dccx4_config_1  : rx0_q_dcc_lane4
    // rx0_q_dccx4_config_1  : rx0_q_dcc_lane5
    // rx0_q_dccx4_config_1  : rx0_q_dcc_lane6
    // rx0_q_dccx4_config_1  : rx0_q_dcc_lane7
    // rx0_q_dccx1_gainx3_config : rx0_q_dcc_lane8
};


struct DCCalData
{
    VgaDcData   rx[NUM_RX_PER_BANK];
    AdcCData    rx_adc_cdata[NUM_RX_PER_BANK];
    ColorDCData rx_color_dc[NUM_RX_PER_BANK];
    uint8_t     ADC_iters[NUM_RX_PER_BANK][MAX_NUM_ACCURACIES];
    uint8_t     VGA_iters[NUM_RX_PER_BANK][NUM_DAC_STG][MAX_NUM_ACCURACIES];
    uint32_t    algorithm_version;

    void uhprint(bool all=false) const
    {
        for (uint32_t i = 0; i < NUM_RX_PER_BANK; i++)
        {
            UHPRINTF("RX[%d] VGA IOFF: %u, %u, %u, %u QOFF %u %u %u %u BQF %ui, %uq\n", i,
                    rx[i].ioff_dac_stg[0], rx[i].ioff_dac_stg[1], rx[i].ioff_dac_stg[2], rx[i].ioff_dac_stg[3],
                    rx[i].qoff_dac_stg[0], rx[i].qoff_dac_stg[1], rx[i].qoff_dac_stg[2], rx[i].qoff_dac_stg[3],
                    rx[i].ioff_dac_bqf, rx[i].qoff_dac_bqf);
            UHPRINTF("RX[%d] ADC IOFF: %u, %u, %u, %u, %u, %u, %u, %u, %u\n", i,
                    rx_adc_cdata[i].i_cdata_0[0], rx_adc_cdata[i].i_cdata_0[1], rx_adc_cdata[i].i_cdata_0[2],
                    rx_adc_cdata[i].i_cdata_0[3], rx_adc_cdata[i].i_cdata_0[4], rx_adc_cdata[i].i_cdata_0[5],
                    rx_adc_cdata[i].i_cdata_0[6], rx_adc_cdata[i].i_cdata_0[7], rx_adc_cdata[i].i_cdata_0[8]);
            UHPRINTF("RX[%d] ADC QOFF: %u, %u, %u, %u, %u, %u, %u, %u, %u\n", i,
                    rx_adc_cdata[i].q_cdata_0[0], rx_adc_cdata[i].q_cdata_0[1], rx_adc_cdata[i].q_cdata_0[2],
                    rx_adc_cdata[i].q_cdata_0[3], rx_adc_cdata[i].q_cdata_0[4], rx_adc_cdata[i].q_cdata_0[5],
                    rx_adc_cdata[i].q_cdata_0[6], rx_adc_cdata[i].q_cdata_0[7], rx_adc_cdata[i].q_cdata_0[8]);
            UHPRINTF("RX[%d] Color IOFF: %d, %d, %d, %d, %d, %d, %d, %d, %d\n", i,
                    rx_color_dc[i].i_dcc_lane[0], rx_color_dc[i].i_dcc_lane[1], rx_color_dc[i].i_dcc_lane[2],
                    rx_color_dc[i].i_dcc_lane[3], rx_color_dc[i].i_dcc_lane[4], rx_color_dc[i].i_dcc_lane[5],
                    rx_color_dc[i].i_dcc_lane[6], rx_color_dc[i].i_dcc_lane[7], rx_color_dc[i].i_dcc_lane[8]);
            UHPRINTF("RX[%d] Color QOFF: %d, %d, %d, %d, %d, %d, %d, %d, %d\n", i,
                    rx_color_dc[i].q_dcc_lane[0], rx_color_dc[i].q_dcc_lane[1], rx_color_dc[i].q_dcc_lane[2],
                    rx_color_dc[i].q_dcc_lane[3], rx_color_dc[i].q_dcc_lane[4], rx_color_dc[i].q_dcc_lane[5],
                    rx_color_dc[i].q_dcc_lane[6], rx_color_dc[i].q_dcc_lane[7], rx_color_dc[i].q_dcc_lane[8]);
            if (!all)
                break;
        }
    }

    void set_defaults()
    {
        // TODO
    }
};

struct DCBiasKey
{
    FLOAT                     temp;         // Chip temperature C at which this cal was performed
    uint32_t                  tx_ant_bmap;  // 8 Out of 12 Txs (RevA)
    uint32_t                  rx_ant_bmap;  // 8 Out of 16 Rxs (RevA)
    RHAL_TxBWPresets          tx_bw_enum;
    RHAL_RxBWPresets          rx_bw_enum;
    RHAL_RxVGAGainPresets     rx_vga_gain_enum;
    RHAL_TxGainPresets        tx_gain_enum;
    uint32_t                  reserved_lomode;
    INT                       reserved_tx;
    INT                       reserved_rx;
    FLOAT                     carrier_freq;
    uint32_t                  antenna_config_id;

    uint32_t                  reserved_0;
    uint32_t                  reserved_1;
    uint32_t                  reserved_2;
    uint32_t                  reserved_3;

    enum { KEY_VERSION = 3 };

    DCBiasKey() { memset(this, 0, sizeof(*this)); }

    void uhprint() const
    {
        UHPRINTF("(DC) Temperature: %f\n", temp);
        UHPRINTF("antenna config ID: %d\n", antenna_config_id);
        UHPRINTF("tx: 0x%03x (%d)\n", tx_ant_bmap, reserved_tx);
        UHPRINTF("rx: 0x%04x (%d)\n", rx_ant_bmap, reserved_rx);
        UHPRINTF("tx_bw_enum: %d %s\n", tx_bw_enum, tx_bw_enum < NUM_TXBW_PRESETS ? tx_bw_names[tx_bw_enum] : "");
        UHPRINTF("rx_bw_enum: %d %s\n", rx_bw_enum, rx_bw_enum < NUM_RXBW_PRESETS ? rx_bw_names[rx_bw_enum] : "");
        UHPRINTF("rx_vga_gain_enum: %d %s\n", rx_vga_gain_enum, rx_vga_gain_enum < NUM_VGAGAIN_PRESETS ? rx_gain_names[rx_vga_gain_enum] : "");
        UHPRINTF("tx_gain_enum: %d %s\n", tx_gain_enum, tx_gain_enum < NUM_TxGAIN_PRESETS ? tx_gain_names[tx_gain_enum] : "");
        UHPRINTF("carrier freq: %.2f\n", carrier_freq);
    }

#if __SCP__
    void upgrade_from_version(uint32_t, DCCalData&) { }
    void upgrade_from_version(uint32_t, struct DCCalBData&) { }
#endif

    //!! This is the all-important key-search algorithm, find an 'exact-match' !!
    bool compare(const DCBiasKey& other) const
    {
        return ( // uh_fabsf(temp - other.temp) < 1.5F &&   // TODO: This check messes up minical when using real temps
                (uh_fabsf(carrier_freq - other.carrier_freq) <= 0.2F) &&
                tx_ant_bmap == other.tx_ant_bmap &&
                rx_ant_bmap == other.rx_ant_bmap &&
                tx_bw_enum  == other.tx_bw_enum &&
                rx_bw_enum  == other.rx_bw_enum &&
                rx_vga_gain_enum == other.rx_vga_gain_enum &&
                tx_gain_enum == other.tx_gain_enum);
    }

    // find the 'nearest-match' by picking the key with the shortest disance
    // from the reference key
    FLOAT distance(const DCBiasKey& other) const
    {
        FLOAT dist = uh_fabsf(temp - other.temp) + uh_fabsf(carrier_freq - other.carrier_freq) * 5;
        INT diffs = !(tx_ant_bmap == other.tx_ant_bmap);
        diffs += !(rx_ant_bmap == other.rx_ant_bmap);
        diffs += !(tx_bw_enum == other.tx_bw_enum);
        diffs += !(rx_bw_enum == other.rx_bw_enum);
        diffs += !(rx_vga_gain_enum == other.rx_vga_gain_enum);
        diffs += !(tx_gain_enum == other.tx_gain_enum);
        return dist + CAL_KEY_DO_NOT_USE * float(diffs);
    }
};

SRS_CLOSE_NAMESPACE()

#endif // SRS_HDR_DC_CAL_FLASHDATA_H
