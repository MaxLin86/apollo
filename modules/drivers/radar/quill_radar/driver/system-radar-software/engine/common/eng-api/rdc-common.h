#ifndef SRS_HDR_RDC_COMMON_H
#define SRS_HDR_RDC_COMMON_H 1
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
/*! \file */

#include "modules/drivers/radar/quill_radar/driver/system-radar-software/env-uhnder/coredefs/uhnder-common.h"
#include "modules/drivers/radar/quill_radar/driver/system-radar-software/engine/common/eng-api/event-enums.h"
#include "modules/drivers/radar/quill_radar/driver/system-radar-software/engine/common/eng-api/uhtypes.h"

SRS_DECLARE_NAMESPACE()

// RDC structures that are exposed in the public engine API for use in
// wire-protocols or file formats. These are the only RDC structs that the RHAL
// Layer is allowed to know about.

enum RDC_AnalogPowerMode
{
    //! default power state configured by the radar
    APM_DEFAULT,

    //! Power on the analog just prior to microcal, power off after the scan
    APM_OFF_BETWEEN_SCANS,

    //! Power on the analog just prior to microcal, power off TX between
    //! microcal and the scan start, power off all after the scan
    APM_OFF_BETWEEN_SCANS_TX_OFF_MICROCAL,

    //! Same as APS_OFF_BETWEEN_SCANS but leave RX turned on so long as more
    //! scans are scheduled to run
    APM_RX_ON_BETWEEN_SCANS,

    //! Power on the analog just prior to microcal, leave on until frame config
    //! released. NOT COMPATIBLE WITH DET-LOOP
    APM_PERSISTENT_ON,
};


enum PeakDetector
{
    PD_CAL_DC,
    PD_PA120_CAL,
    PD_PA60_CAL,
    PD_PA30_CAL,
    PD_RFMX_CAL,
    PD_LOIOPK,
    PD_LOQOPK,
    PD_RFLPBK,
    PD_NUM_DETECTORS
};


enum
{
    DPROBE_MEAS_COUNT = 4,
    MAX_SCAN_APROBES = 5
};

//
//! D-Probe configuration structure for use with RDC_ScanConfig::request_dprobes()
//
struct RDC_DProbe_corr_config
{
    uint32_t corr_cov_length;                   //!< dprobe_corr_len

    uint16_t dc_init_dc_i_init;                 //!< dprobe_dc_i_init
    uint16_t dc_init_dc_q_init;                 //!< dprobe_dc_q_init

    uint8_t  corr_start;                        //!< dprobe_corr_start
    uint8_t  corr_ctrl_mod_ab_sel;              //!< Select dprobe_corr_mod_ab_select
    uint8_t  corr_ctrl_tx_data_sel;             //!< dprobe_corr_tx_data_sel
    uint8_t  corr_ctrl_1b_sel;                  //!< dprobe_corr_1b_sel

    uint8_t  corr_ctrl_ref_sel;                 //!< dprobe_corr_ref_sel
    uint8_t  corr_ctrl_xcor_sel;                //!< dprobe_corr_xcor_sel
    uint8_t  corr_ctrl_cov_sel;                 //!< dprobe_corr_cov_sel
    uint8_t  corr_ctrl_prbs_delay;              //!< dprobe_corr_prbs_delay

    uint8_t  corr_ctrl_tx_data_delay;           //!< dprobe_corr_tx_data_delay
    uint8_t  corr_ctrl_1b_data_delay;           //!< dprobe_corr_1b_data_delay
    uint8_t  corr_ctrl_1b_ref_delay;            //!< dprobe_corr_1b_ref_delay
    uint8_t  corr_ctrl_umsk_subsymbol_delay;    //!< dprobe_corr_mod_subsymbol_delay

    uint8_t  dc_control_dc_mode;                //!< dprobe_dc_mode
    uint8_t  dc_control_dc_clear;               //!< dprobe_dc_clear
    uint8_t  dc_control_dc_coef;                //!< dprobe_dc_coef
    uint8_t  reserved;
};

//
//! D-Probe configuration structure for use with RDC_ScanConfig::request_dprobes()
//
struct RDC_DProbe_rms_config
{
    uint32_t rms_length;                    //!< dprobe_rms_len
    uint8_t  rms_start;                     //!< dprobe_rms_start
    uint8_t  rms_ctrl_mode;                 //!< bit 0 dprobe_rms_mode, bit 1 dprobe_fir_bypass
    uint8_t  rms_ctrl_clear;                //!< dprobe_rms_clear
    uint8_t  rms_ctrl_coef;                 //!< dprobe_rms_coef
    uint8_t  rms_drive_0_i;                 //!< if 1, force 0 on the I output of channel
    uint8_t  rms_drive_0_q;                 //!< if 1, force 0 on the Q output of channel
    uint8_t  rms_hold_lane_enable;          //!< if 1, multiplexor holds output to a single lane
    uint8_t  rms_hold_lane;                 //!< if rms_hold_lane_enable, the ADC lane selection 0..8
};

//
//! D-Probe configuration structure for use with RDC_ScanConfig::request_dprobes()
//
struct RDC_DProbe_iq_config
{
    uint32_t iq_control_iq_prbs_load;       //!< dprobe_iq_prbs_load
    uint32_t iq_length;                     //!< dprobe_iq_len

    uint16_t iq_init_iq_pha_init;           //!< dprobe_iq_pha_init
    uint16_t iq_init_iq_mag_init;           //!< dprobe_iq_mag_init

    uint8_t  iq_start;                      //!< dprobe_iq_start
    uint8_t  iq_control_iq_mode;            //!< dprobe_iq_mode
    uint8_t  iq_control_iq_clear;           //!< dprobe_iq_clear
    uint8_t  iq_control_iq_rx_gain;         //!< dprobe_iq_rx_gain
    uint8_t  iq_control_iq_mag_coef;        //!< dprobe_iq_mag_coef
    uint8_t  iq_control_iq_pha_coef;        //!< dprobe_iq_pha_coef

    uint8_t  reserved_0;
    uint8_t  reserved_1;
};

//
//! ADI DC configuration structure for use with RDC_ScanConfig::request_dc_measure()
//
struct RDC_DCMeasure_config
{
    //! dc measure duration
    uint32_t  dc_measure_duration;

    //! bitmap for 8 Rx antenna; When 0, DC measurement is before correction. When 1, DC measurement is after correction.
    uint8_t   dc_measurement_location;

    //! bitmap for 8 Rx antenna; When dc_measurement is configured, and
    //! this bit is 1, dc_measurement accumulation and duration counter are gated
    //! by dc_measure_on input.  Else, no gating.
    uint8_t   dc_measurement_mode;

    //!This  is used to configure one lane per rx for DC measurement.
    //!The format is as follows rx_lane[0] = 8 represents rx0 and lane number 8.
    //!In case no lane selection is to be made make {9,9,9,9,9,9,9}
    void set_defaults()
    {
        dc_measure_duration = 4096;
        dc_measurement_location = 0;
        dc_measurement_mode = 255;
    }
};


//
//! Enumeration of aprobe priorities and scheduling requirements
//
enum RDC_AprobePriorities
{
    RDC_AP_HIGH_PRIORITY,                                       //!< as soon as possible, scan or not
    RDC_AP_SCAN_IDLE,                                           //!< wait for gap between scans to measure
    RDC_AP_BEST_EFFORT,                                         //!< as soon as aprobe is otherwise idle
};

//
//! A-Probe configuration structure for use with RDC_ScanConfig::request_aprobe()
//! and with RDC_Layer::request_aprobe()
//
struct RDC_AprobeRequest
{
    INT   bus;
    INT   channel;
    INT   probe;
    INT   num_samples;
    FLOAT measured_value;

    void set_defaults()
    {
        bus = 0;
        channel = 0;
        probe = 0;
        num_samples = 32768;
        measured_value = -1e9f;
    }
};

//
//! Peak Det configuration structure for use with RDC_ScanConfig::request_peakdet()
//! and with RDC_Layer::request_peakdet()
//
struct RDC_PeakDetRequest
{
    INT   channel;
    INT   measuring_point;
    INT   num_samples;
    FLOAT measured_value;

    void set_defaults()
    {
        channel = 0;
        measuring_point = 0;
        num_samples = 32768;
        measured_value = -1e9f;
    }
};

//
//! Rotation matrix for RX as configured in MIQ
//
struct IQCorrMatrix
{
    uint32_t coeffA;
    uint32_t coeffB;
    uint32_t coeffC;
    uint32_t coeffD;
};


//
//! TODO
//
struct RDC_RDsummary
{
    uint16_t                D_bin;          //!< Doppler bin
    uint16_t                R_bin_offset;   //!< Range bin offset from RHAL_RDC3_output.rb_start
    uint16_t                max_val;        //!< Magnitude
    int8_t                  exponent;       //!< Exponent:   value = sample * 2^exponent
    uint8_t                 reserved;       //!< Reserved
};

//
//! Range bin exponent structure in Static Slice
//
struct RDC_SSRsummary
{
    int8_t                  exponent;       //!< Exponent:   value = sample * 2^exponent
    uint8_t                 reserved_1;     //!< Reserved
    uint16_t                R_bin;          //!< Range bin
    uint32_t                reserved_2;     //!< Reserved
};

//
//! TODO
//
struct RDC_Histogram
{
    enum { NUM_HISTOGRAM_BINS = 96 };

    uint16_t                bins[NUM_HISTOGRAM_BINS]; //!< count of angles for 1 - 96 DB bins
    int8_t                  exponent;                 //!< signed 8-bit hist exponent
    uint8_t                 max_bin_index;            //!< unsigned 7-bit index value of max bin, 0 to 95
    uint16_t                reserved;                 //!< reserved bits
    uint32_t                crc;                      //!< hist crc
};

//
//! TODO
//
struct RDC_Dig_Loopback
{
    uint8_t test_0_tx_num;
    uint8_t test_0_DAC_lane_num;
    uint8_t test_1_tx_num;
    uint8_t test_1_DAC_lane_num;
    uint32_t rx_test_input_bitmap;
    FLOAT    tgt_in_m; // tgt_in_m
};

//
//! TODO
//
enum RDC_Loopback_Rotation
{
    LPBK_250 = 0,
    LPBK_125 = 2,
    LPBK_62P5 = 3,
    LPBK_BYPASS = 4,
};

//
//! TODO
//
enum RDC_Doppler_Shift_Update_Mode
{
    DSUM_ABSOLUTE_OFFSET = 1,
    DSUM_EGO_FRACTION = 2,
    DSUM_EGO_OFFSET = 3,
};

//
//! TODO
//
enum RDC_Doppler_Shift_RDC_Mode
{
    DSRM_NO_SHIFT = 1,               // No shift in RDC2/3 Doppler:  Center bin is true zero Doppler
    DSRM_SHIFT = 2,                  // RDC2/3 is shifted by some amount, as returned by get_RDC_zero_Doppler_bin_index()
};

//
//! TODO
//
struct RDC_Analog_Loopback
{
    uint8_t tx_num;
    uint8_t rx_num;
    RDC_Loopback_Rotation lpbk_rotation;
};

//
//! TODO
//
struct RDC_DProbe_Output
{
    int32_t dprobe_corr_xcorr_i;
    int32_t dprobe_corr_xcorr_q;
    int32_t dprobe_corr_cov_i;
    int32_t dprobe_corr_cov_q;

    int16_t dprobe_iq_mag_meas;
    int16_t dprobe_iq_pha_meas;

    int16_t dprobe_dc_i_meas;
    int16_t dprobe_dc_q_meas;

    int16_t dprobe_rms_meas;

    enum {
        DPROBE_IQ_DONE = 1,
        DPROBE_DC_DONE = 2,
        DPROBE_RMS_DONE = 4,
        DPROBE_CORR_DONE = 8
    };

    int16_t dprobe_completion_flags;
};

//! Structure to hold the QDU patter and size provided from capture command
struct RDC_QDU_Pattern_Override
{

    //! 8 bit QDU size value.
    uint8_t   qdu_size;

    //! 32 bit QDU pattern
    uint32_t   qdu_pattern;

    void set_defaults()
    {
        qdu_size = 0;
        qdu_pattern = 0;

    }
};

//! Structure to hold the doppler rotator params provided from capture command
struct RDC_Dop_Rotator_Config
{

    FLOAT                           ego_vel_shift_param;       //!< offste of ego-velocity to shift Doppler using rotator
    RDC_Doppler_Shift_Update_Mode   doppler_shift_update_mode; //!< doppler shift 1. absolute shift 2. fraction of ego vel shift 3. ego+offset
    RDC_Doppler_Shift_RDC_Mode      doppler_shift_rdc_mode;    //!< rdc shift mode 1: no shift 2 : RDC shift

    void set_defaults()
    {
        ego_vel_shift_param = 0;
        doppler_shift_update_mode = DSUM_ABSOLUTE_OFFSET;
        doppler_shift_rdc_mode = DSRM_NO_SHIFT;

    }
};

//
//! Aprobe (MADC) output structure which is passed to the completion trigger
//! event handler.
//
struct MeasurementRequest
{
    uint8_t   bus;          //!< 0 - 2  bus index
    uint8_t   channel;      //!< 0 - 11 channel index
#if SABINE_A
    uint16_t  probe;        //!< 0 - ?? probe enumeration
    int16_t   method;       //!< 0 -    measurement enumeration
    EventEnum user_event_tag;
#elif SABINE_B
    uint32_t  hash;         //!< unsigned 32-bit measurement hash
    uint32_t  wait_clocks;  //!< unsigned 32-bit sampling wait clocks
#endif
    uint32_t  state;        //!< sampling system state
    EventEnum event_num;    //!< measurement request return event
    uint16_t  num_samples;  //!< current default 32768
    bool      blocking;
    EventEnum event_tag;
    FLOAT     value;        //!< output measured value
    void*     user_pointer;
    FLOAT     std_dev;

    //! Release an instance of MeasurementRequest
    //
    //! release() must be called on MeasurementRequest passed to aprobe callback
    //! event trigger function, to recycle the resources consumed by this aprobe
    //! request
    void release();
};


//
//! Number of Sub-ADC lanes
//
enum { NUM_ADC_LANE = 9 };

//
//! TODO
//
struct RDC_adi_measured_dc
{
    int32_t rx0_measured_dc_i[NUM_ADC_LANE];
    int32_t rx0_measured_dc_q[NUM_ADC_LANE];
    int32_t rx1_measured_dc_i[NUM_ADC_LANE];
    int32_t rx1_measured_dc_q[NUM_ADC_LANE];
    int32_t rx2_measured_dc_i[NUM_ADC_LANE];
    int32_t rx2_measured_dc_q[NUM_ADC_LANE];
    int32_t rx3_measured_dc_i[NUM_ADC_LANE];
    int32_t rx3_measured_dc_q[NUM_ADC_LANE];
    int32_t rx4_measured_dc_i[NUM_ADC_LANE];
    int32_t rx4_measured_dc_q[NUM_ADC_LANE];
    int32_t rx5_measured_dc_i[NUM_ADC_LANE];
    int32_t rx5_measured_dc_q[NUM_ADC_LANE];
    int32_t rx6_measured_dc_i[NUM_ADC_LANE];
    int32_t rx6_measured_dc_q[NUM_ADC_LANE];
    int32_t rx7_measured_dc_i[NUM_ADC_LANE];
    int32_t rx7_measured_dc_q[NUM_ADC_LANE];
};

//
//! TODO
//
enum RDC_ADC_CaptureMode
{
    NO_ADC_CAPTURE = 0,
    ADC_DATA_SPRAY_MODE = 1,
    ADC_RAW_DATA_CAPTURE_MODE = 2,
    ADC_ADIE_CAPTURE_MODE = 3
};

//
//! TODO
//
struct RDC_ADC_Params
{
    RDC_ADC_CaptureMode adc_capture_mode;
    uint16_t    adc_capture_channel_bitmap;      //!< bitmap of 8-bits indicating which Rxs(and how many) are needed for ADC Capture
    uint16_t    sel_rx_tx;                       //!< select rx or tx for ADC capture 0- Rx 1 - Tx
    uint16_t    sample_bit_width;                //!< ADC samples bit width
    uint32_t    samples_per_channel;             //!< number of samples to capture

    void set_defaults()
    {
        adc_capture_mode = ADC_DATA_SPRAY_MODE;
        adc_capture_channel_bitmap = 1U;
        sel_rx_tx = 0;
        sample_bit_width = 8;
        samples_per_channel = 1024 * 1024;
    }

    uint32_t num_selected_channels() const
    {
        uint32_t selected = 0;
        INT limit = sel_rx_tx ? 12 : 8;
        for (INT i = 0; i < limit; i++)
        {
            if (adc_capture_channel_bitmap & (1U << i))
            {
                selected++;
            }
        }
        return selected;
    }

    bool validate() const
    {
        bool ok = true;

        ok &= (adc_capture_mode <= ADC_ADIE_CAPTURE_MODE);
        ok &= (sample_bit_width == 8 || sample_bit_width == 16);
        ok &= (samples_per_channel > 0 && ((samples_per_channel & 15) == 0));
        if (adc_capture_mode == ADC_DATA_SPRAY_MODE)
        {
            ok &= (num_selected_channels() <= 2);
            ok &= (sel_rx_tx == 0 || sel_rx_tx == 1);
        }
        else if (adc_capture_mode == ADC_RAW_DATA_CAPTURE_MODE || adc_capture_mode == ADC_ADIE_CAPTURE_MODE)
        {
            ok &= (num_selected_channels() <= 8);
            ok &= (sel_rx_tx == 0 );
        }

        return ok;
    }
};

//
//! TODO
//
enum RDC_Fdac_IF_Ratio
{
    DAC_LOIF_0  =  0,
    DAC_LOIF_64 = 64,
    DAC_LOIF_48 = 48,
    DAC_LOIF_32 = 32,
    DAC_LOIF_24 = 24,
    DAC_LOIF_16 = 16,
    DAC_LOIF_12 = 12,
    DAC_LOIF_8  = 8,
    DAC_LOIF_6  = 6,
    DAC_LOIF_4  = 4,
    DAC_LOIF_3  = 3,
    DAC_LOIF_2  = 2,
    NUM_LOIF_RATIOS = 12
};

//
//! TODO
//
enum RDC_Sampling_rate
{
    FSAMP_2G = 2000,
    FSAMP_1G = 1000,
    FSAMP_500M = 500,
    FSAMP_250M = 250,
    NUM_SAMPLING_RATES = 4
};

//
//! TODO
//
enum RDC_Symbol_rate
{
    FSYMB_2G = 2000,
    FSYMB_1G = 1000,
    FSYMB_500M = 500,
    FSYMB_250M = 250,
    NUM_SYMBOL_RATES = 4
};

//
//! TODO
//
enum RDC_DAC_Rate
{
    DAC_8G = 8000,    //!< 8GHz DAC rate
    DAC_4G = 4000,    //!< 4GHz DAC rate
    DAC_2G = 2000,    //!< 2GHz DAC rate
    DAC_1G = 1000,    //!< 1GHz DAC rate
    NUM_DAC_RATES = 4 //!< Count of DAC rates
};

//
//! TODO
//
enum RDC_Fsym_Fchip_Ratio
{
    FSYMB_FCHIP_ONE = 1,
    FSYMB_FCHIP_TWO = 2,
    FSYMB_FCHIP_THREE = 3,
    FSYMB_FCHIP_FOUR = 4,
    FSYMB_FCHIP_FIVE = 5,
    FSYMB_FCHIP_SIX = 6,
    FSYMB_FCHIP_SEVEN = 7,
    FSYMB_FCHIP_EIGHT = 8,
};


SRS_CLOSE_NAMESPACE()

#endif // SRS_HDR_RDC_COMMON_H
