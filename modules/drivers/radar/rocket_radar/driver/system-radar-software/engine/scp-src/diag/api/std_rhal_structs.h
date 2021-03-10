// Copyright (C) Uhnder, Inc. All rights reserved. Confidential and Proprietary - under NDA.
// Refer to SOFTWARE_LICENSE file for details
#ifndef SRS_HDR_STD_RHAL_STRUCTS_H
#define SRS_HDR_STD_RHAL_STRUCTS_H 1

#include "modules/drivers/radar/rocket_radar/driver/system-radar-software/env-reference/coredefs/uhnder-common.h"

SRS_DECLARE_NAMESPACE()

struct StdRhal_set_proconlyhook_params_input
{
    //! Diag Input structure for "std_rhal::set_proconlyhook_params"
    CHAR file_name[64];
    uint32_t start_r_bin;
    uint32_t end_r_bin;
    uint32_t rdc3_uthr;
    uint8_t  mce_bypass;
};

//! Diag Output structure for "std_rhal::set_proc_crc_params"
struct StdRhal_proc_crc_params_input
{
    //! FEU expected CRC
    uint32_t feu_crc;
    //! SS2 expected CRC
    uint32_t ss2_crc;
};

struct StdRhal_set_adc_capture_id_input
{
    //! Diag Input structure for "std_rhal::set_adc_capture_id"
    uint8_t  adc_capture_test_id;
};

struct StdRhal_set_mceonlyhook_params_input
{
    //! Diag Input structure for "std_rhal::set_proconlyhook_params"
    CHAR data_file_name[64];
    CHAR sum_file_name[64];
    uint32_t n_skewers;
    uint32_t n_chan_iter;
    uint32_t n_act_vrx;
};

//! Diag Output structure for "std_rhal::set_srdc3_capture_params"
struct StdRhal_srdc3_capture_params_input
{
    //! sRDC3 range min
    uint16_t range_min;

    //! sRDC3 range max
    uint16_t range_max;

    //! sRDC3 doppler min
    uint16_t doppler_min;

    //! sRDC3 doppler max
    uint16_t doppler_max;
};

struct RLLD_scan_hang_timeouts
{
    uint32_t scan_timeout;
    uint32_t proc_timeout;
};

struct RLLD_adc_pattern
{
    uint32_t adc_pattern_mode;
};

struct RLLD_proc_only_config
{
    uint32_t input_rdc1;
    uint32_t set_scale_factors;
    uint8_t feu_wgt0_mult_scale;
    uint8_t feu_wgt1_mult_scale;
    uint8_t feu_wgt2_mult_scale;
    uint8_t feu_wgt3_mult_scale;
    uint8_t rau_mat_norm;
    uint32_t get_stv;
    uint32_t get_threshold;
};

SRS_CLOSE_NAMESPACE()

#endif // SRS_HDR_STD_RHAL_STRUCTS_H
