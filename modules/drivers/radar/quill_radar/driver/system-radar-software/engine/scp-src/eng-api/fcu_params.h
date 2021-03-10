#ifndef SRS_HDR_FCU_PARAMS_H
#define SRS_HDR_FCU_PARAMS_H 1

#include "modules/drivers/radar/quill_radar/driver/system-radar-software/env-uhnder/coredefs/uhnder-common.h"

SRS_DECLARE_NAMESPACE()

//! Structure for holding RMS zero threshold structure
struct FCU_Params_rms_zth
{
    uint16_t rx0;
    uint16_t rx1;
    uint16_t rx2;
    uint16_t rx3;
    uint16_t rx4;
    uint16_t rx5;
    uint16_t rx6;
    uint16_t rx7;
};

//! Structure for holding Adaptation coefficients
struct FCU_Params_coef
{
    uint16_t fcu_coef0_re;
    uint16_t fcu_coef0_im;
    uint16_t fcu_coef1_re;
    uint16_t fcu_coef1_im;
    uint16_t fcu_coef2_re;
    uint16_t fcu_coef2_im;
    uint16_t fcu_coef3_re;
    uint16_t fcu_coef3_im;
    uint16_t fcu_coef4_re;
    uint16_t fcu_coef4_im;
    uint16_t fcu_coef5_re;
    uint16_t fcu_coef5_im;
    uint16_t fcu_coef6_re;
    uint16_t fcu_coef6_im;
    uint16_t fcu_coef7_re;
    uint16_t fcu_coef7_im;
    uint16_t fcu_coef8_re;
    uint16_t fcu_coef8_im;
    uint16_t fcu_coef9_re;
    uint16_t fcu_coef9_im;
    uint16_t fcu_coef10_re;
    uint16_t fcu_coef10_im;
    uint16_t fcu_coef11_re;
    uint16_t fcu_coef11_im;
    uint16_t fcu_coef12_re;
    uint16_t fcu_coef12_im;
    uint16_t fcu_coef13_re;
    uint16_t fcu_coef13_im;
    uint16_t fcu_coef14_re;
    uint16_t fcu_coef14_im;
};

//! Master FCU configs structure
struct FCU_Params
{
    // FCU Configs
    uint8_t  mu;
    uint8_t  leak;
    uint8_t  reverse_length;
    uint8_t  error_sat_sel;
    uint8_t  reverse_delay_load;
    uint16_t gradient_limit_thresh;
    uint8_t  gradient_limit_time;
    // Zero configs
    uint16_t rms_threshold_rx0;
    uint16_t rms_threshold_rx1;
    uint16_t rms_threshold_rx2;
    uint16_t rms_threshold_rx3;
    uint16_t rms_threshold_rx4;
    uint16_t rms_threshold_rx5;
    uint16_t rms_threshold_rx6;
    uint16_t rms_threshold_rx7;
    uint8_t  rms_zero_xy_factor_rxX;
    uint8_t  rms_zero_integrator_coef_rxX;
    // RMS ctrl configs
    uint32_t rms_duration_rxX;
    // RMS Measurement configs
    uint8_t  rms_inp_sel;
    uint8_t  rms_outp_sel;
    uint8_t  rms_inp_coef;
    uint8_t  rms_outp_coef;
    // Tap configs
    uint16_t fcu_tap_load;
    uint16_t fcu_tap_static;
    uint16_t fcu_tap_auto_zero_start;
    uint16_t fcu_tap_auto_zero_ping;
    // Coeffs
    uint16_t fcu_coef0_re;
    uint16_t fcu_coef0_im;
    uint16_t fcu_coef1_re;
    uint16_t fcu_coef1_im;
    uint16_t fcu_coef2_re;
    uint16_t fcu_coef2_im;
    uint16_t fcu_coef3_re;
    uint16_t fcu_coef3_im;
    uint16_t fcu_coef4_re;
    uint16_t fcu_coef4_im;
    uint16_t fcu_coef5_re;
    uint16_t fcu_coef5_im;
    uint16_t fcu_coef6_re;
    uint16_t fcu_coef6_im;
    uint16_t fcu_coef7_re;
    uint16_t fcu_coef7_im;
    uint16_t fcu_coef8_re;
    uint16_t fcu_coef8_im;
    uint16_t fcu_coef9_re;
    uint16_t fcu_coef9_im;
    uint16_t fcu_coef10_re;
    uint16_t fcu_coef10_im;
    uint16_t fcu_coef11_re;
    uint16_t fcu_coef11_im;
    uint16_t fcu_coef12_re;
    uint16_t fcu_coef12_im;
    uint16_t fcu_coef13_re;
    uint16_t fcu_coef13_im;
    uint16_t fcu_coef14_re;
    uint16_t fcu_coef14_im;

    void set_defaults()
    {
        mu   = 9;
        leak = 7;
        reverse_length = 0;
        error_sat_sel = 0;
        reverse_delay_load = 0;
        gradient_limit_thresh = 0;
        gradient_limit_time = 0;
        rms_threshold_rx0 = 0;
        rms_threshold_rx1 = 0;
        rms_threshold_rx2 = 0;
        rms_threshold_rx3 = 0;
        rms_threshold_rx4 = 0;
        rms_threshold_rx5 = 0;
        rms_threshold_rx6 = 0;
        rms_threshold_rx7 = 0;
        rms_zero_xy_factor_rxX = 0;
        rms_zero_integrator_coef_rxX = 0;
        rms_duration_rxX = 0;
        rms_inp_sel = 0;
        rms_outp_sel = 0;
        rms_inp_coef = 0;
        rms_outp_coef = 0;
        fcu_tap_load = 0;
        fcu_tap_static = 0;
        fcu_tap_auto_zero_start = 0;
        fcu_tap_auto_zero_ping = 0;
        fcu_coef0_re = 0;
        fcu_coef0_im = 0;
        fcu_coef1_re = 0;
        fcu_coef1_im = 0;
        fcu_coef2_re = 0;
        fcu_coef2_im = 0;
        fcu_coef3_re = 0;
        fcu_coef3_im = 0;
        fcu_coef4_re = 0;
        fcu_coef4_im = 0;
        fcu_coef5_re = 0;
        fcu_coef5_im = 0;
        fcu_coef6_re = 0;
        fcu_coef6_im = 0;
        fcu_coef7_re = 0;
        fcu_coef7_im = 0;
        fcu_coef8_re = 0;
        fcu_coef8_im = 0;
        fcu_coef9_re = 0;
        fcu_coef9_im = 0;
        fcu_coef10_re = 0;
        fcu_coef10_im = 0;
        fcu_coef11_re = 0;
        fcu_coef11_im = 0;
        fcu_coef12_re = 0;
        fcu_coef12_im = 0;
        fcu_coef13_re = 0;
        fcu_coef13_im = 0;
        fcu_coef14_re = 0;
        fcu_coef14_im = 0;
    }
};

SRS_CLOSE_NAMESPACE()

#endif // SRS_HDR_FCU_PARAMS_H
