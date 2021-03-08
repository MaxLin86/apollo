#ifndef SRS_HDR_STD_LAB_STRUCTS_H
#define SRS_HDR_STD_LAB_STRUCTS_H 1

#include "modules/drivers/radar/rocket_radar/driver/system-radar-software/env-uhnder/coredefs/uhnder-common.h"

SRS_DECLARE_NAMESPACE()

//! Diag Input structure for "std_rdc::lab_DProbe"
struct StdLab_DProbe_input
{
    //! Selection of RX (0 ... 7) input for calibration measurement
    uint8_t  cal_control_reg__rx_sel;

    //! Correlation measurement unit selection of 0 of 11 PRN data from TXU
    uint8_t  corr_ctrl_1__tx_data_sel;

    //! Reference side correlation measurement unit selection. 0-UMSK, 1-PRBS
    uint8_t  corr_ctrl_1__ref_sel;

    //! Correlation measurement unit selection of 1b data for correlators.
    //! 0 for 1+j0
    //! 1 for 0+j1
    //! 2 for PRBS+j0
    //! 3 for 0+j*PRBS
    uint8_t  corr_ctrl_1__1b_sel;

    //! UMSK table selection between A & B
    //! Only Table-A is supported
    uint8_t  corr_ctrl_1__mod_ab_select;

    //! UMSK internal RAM access type 0-Read, 1-Write
    // TODO: Exporting out the read table
    uint8_t  corr_mod_access__mod_mem_op;

    //! Select either sample/hold PRBS data across 4x oversampling (0),
    //! or upsample by inserting 3 zeros after data (1)
    //! CX PRBS Generator
    uint8_t  prbs_ctrl__prbs_mode_0;

    //! Setting to 1 loads the seed for PRBS generator
    //! CX PRBS Generator
    uint8_t  prbs_ctrl__prbs_load_0;

    //! Sets the delay of PRBS in increments of 1/4 symbol
    //! CX PRBS Generator
    uint8_t  prbs_ctrl__prbs_delay_0;

    //! Correlation measurement unit symbol dalay of the PRBS signal from analog CX unit
    uint8_t  corr_ctrl_0__prbs_delay;

    //! Correlation measurement unit symbol dalay of the PRN codes from TXU
    uint8_t  corr_ctrl_0__tx_data_delay;

    //! Correlation measurement unit symbol dalay of the 1b reference (PRBS)
    uint8_t  corr_ctrl_0__1b_ref_delay;

    //! Correlation measurement unit symbol dalay of the 1b data (PRBS)
    uint8_t  corr_ctrl_0__1b_data_delay;

    //! Correlation measurement unit UMSK sub-symbol dalay
    uint8_t  corr_ctrl_0__mod_subsym_delay;

    //! Enables calibration-bypass color correction & pass ADC data directly to instrument
    uint8_t  adi_cal_config__adi_cal;

    //! Force i-channel ADC data to zero during calibration
    //! 0 for ADC data through
    //! 1 for forcing zero on i-channel instrument input
    uint8_t  adi_cal_config__force_i_zero;

    //! Force i-channel ADC data to zero during calibration
    //! 0 for ADC data through
    //! 1 for forcing zero on q-channel instrument input
    uint8_t  adi_cal_config__force_q_zero;

    uint8_t  rsvd0; // This is to keep the word/sub-word alignment

    //! Selection of sub-adc lane held for i-channel calibration
    //! This is a bit-map of 9-bit one-hot signal to indicate which lane is held
    //! for i-channel calibration. Applies to both even and odd muxes
    uint16_t  adi_cal_config__adi_hold_x_i;
    //! Selection of sub-adc lane held for q-channel calibration
    //! This is a bit-map of 9-bit one-hot signal to indicate which lane is held
    //! for q-channel calibration. Applies to both even and odd muxes
    uint16_t  adi_cal_config__adi_hold_x_q;

    //! Low-power mode bit for RX ADI Level Shifters (LS)
    //! 0 for Low power mode (LS off)
    //! 1 for LS active
    uint8_t  adi_cal_config__adi_ls_nsleep;

    uint8_t  rsvd1; // This is to keep the word/sub-word alignment

    //! The seed for PRBS generator in analog CX module
    uint32_t prbs_seed_0__prbs_seed_0;

    //! The taps for PRBS generator in analog CX module
    uint32_t prbs_taps_0__prbs_taps_0;

    //! Selects the measurement mode
    //! 0 for measuring with finite length
    //! 1 for infinite measurement
    uint8_t  iq_control__iq_mode;

    //! Coefficient for the IQ magnitude measurement loop
    uint8_t  iq_control__iq_mag_coef;

    //! Coefficient for the IQ phase measurement loop
    uint8_t  iq_control__iq_pha_coef;

    //! Specifies gain in powers of 2 for the RX signal
    uint8_t  iq_control__iq_rx_gain;

    //! Sets the initial value for the IQ magnitude measurement
    uint16_t iq_init__iq_mag_init;

    //! Sets the initial value for the IQ phase measurement
    uint16_t iq_init__iq_pha_init;

    //! Seed for the IQ input selection PRBS generator
    uint32_t iq_prbs_seed;

    //! Tpas for the IQ input selection PRBS generator
    uint32_t iq_prbs_taps;

    //! Sets the initial value for dc i-channel correction/measurement
    uint16_t dc_init__dc_i_init;

    //! Sets the initial value for dc q-channel correction/measurement
    uint16_t dc_init__dc_q_init;

    //! Selects the correction mode.
    //! 0 for continuous correction (notch filter)
    //! 1 for static correction using dc_i/q_init values
    uint8_t  dc_control__dc_mode;

    //! Coefficient for the DC correction/measurement loop
    uint8_t  dc_control__dc_coef;

    //! Selects the measurement mode
    //! 0 for measuring with finite length
    //! 1 for infinite measurement
    uint8_t  rms_mode;

    //! Coefficient for the RMS measurement loop
    uint8_t  rms_coef;

    //! Specifies the length of correlation measurement in samples, Should be less than (Lc * Symb_Chip_Rate)/2
    uint32_t corr_cov_length;

    //! Specifies the length of IQ measurement
    uint32_t iq_length;

    //! Specifies the length of RMS measurement
    uint32_t rms_length;

    //! Clears the dc measurement/correction when set to 1
    uint8_t  dc_control__dc_clear;

    //! Clears the measurement when set to 1
    uint8_t  iq_control__iq_clear;

    //! Clears the measurement when set to 1
    uint8_t  rms_clear;

    //! Indicate the current test is a regression one
    //! User should set this to zero
    uint8_t  is_regression;

    //! Name of the File that contains the UMSK table
    CHAR     umsk_table_file_name[16];
};

//! Diag Input structure for "std_rdc::lab_AProbe"
struct StdLab_AProbe_input
{
    //! Specifies the tx channel being measured
    uint8_t tx_num;

    //! Specifies the type of measurement to make (-1 to use default)
    int8_t tx_measurement;

    //! Specifies which probe point to measure
    int16_t tx_probe;

    //! Specifies the number of accumulations to make
    uint32_t tx_num_acc;

    //! Specifies the rx channel being measured
    uint8_t rx_num;

    //! Specifies the type of measurement to make (-1 for default)
    int8_t rx_measurement;

    //! Specifies which probe point to measure
    int16_t rx_probe;

    //! Specifies the number of accumulations to make
    uint32_t rx_num_acc;

    //! Specifies which probe point to measure
    int16_t sh_probe;

    //! Specifies the type of measurement to make (-1 for default)
    int8_t sh_measurement;
    uint8_t fill1;

    //! Specifies the number of accumulations to make
    uint32_t sh_num_acc;
};

//! Diag Input structure for "std_rdc::lab_AProbePin"
struct StdLab_AProbePin_input
{
    //! Specifies the source of the monitoring pin
    uint8_t pin_mode;
    uint8_t dummy1;
    uint8_t dummy2;
    uint8_t dummy3;
};

//! Diag Input structure for "std_rdc::lab_scope"
struct StdLab_scope_input
{
    //! ADC data rate
    uint8_t rate;

    //! Specifies the mode in which ADC data has to be captured
    //! 1 for spray mode
    //! 2 for raw capture mode
    uint8_t mode;

    //! Specifies the RX to be used in spray mode
    uint8_t rx_num;

    //! Indicate the current test is a regression one
    uint8_t  is_regression;

    //! Specifies the number of samples to be captured
    //! It is fixed to 1MB
    uint32_t num_samples;
};

//! Enum to specify the signal type for Signal generator diag test
enum sig_gen_signal_type
{
    //! Generate SINE wave
    STD_LAB_SIG_GEN_SIG_TYPE_SINE = 0,
    //! Generate SQUARE Wave
    STD_LAB_SIG_GEN_SIG_TYPE_SQUARE,
    //! GMSK/UMSK mode where user provides the UMSK table
    STD_LAB_SIG_GEN_SIG_TYPE_UMSK,
    //! Same as GMSK mode but with SCAN(default scan is configured)
    STD_LAB_SIG_GEN_SIG_TYPE_UMSK_SCAN,
    //! Only for CX path, generates the PRBS sequence
    STD_LAB_SIG_GEN_SIG_TYPE_CX_PRBS,
    STD_LAB_SIG_GEN_SIG_TYPE_END,
};

//! Signal generator output type(applicable only for SINE and SQUARE waves
enum sig_gen_output_type
{
    //! Generates real only(configures only real part of UMSK table
    STD_LAB_SIG_GEN_OUTPUT_TYPE_REAL = 1,
    //! Generates Imaginary only
    STD_LAB_SIG_GEN_OUTPUT_TYPE_IMAGINARY = 2,
    //! Generates complex output
    STD_LAB_SIG_GEN_OUTPUT_TYPE_COMPLEX = 3,
};

//! Input data for signal generator diag test
struct StdLab_sig_gen_input
{
    //! Sets symbol rate in MHz.
    //! For 1GHZ adc sample rate, it should be 1000(MHz), 500(MHz), 250(MHz)
    //! For 2GHz adc sample rate, it should be 2000(MHz), 1000(MH), 500(MHz), 250(MHz)
    uint32_t symbol_rate;
    //! TX channels to be enabled( 1bit per each TX)
    uint32_t tx_sel;
    uint32_t table_size;
    //! Sets SINE frequency incase sig_type = STD_LAB_SIG_GEN_SIG_TYPE_SINE
    FLOAT sine_freq;
    //! Signal type to be generated(refer to enum sig_gen_signal_type)
    uint8_t sig_type; // sine, square, gmsk
    //! number of ON cycles if sig_type= STD_LAB_SIG_GEN_SIG_TYPE_SQUARE
    uint8_t sq_num_on_cycles;
    //! number of OFF cycles if sig_type= STD_LAB_SIG_GEN_SIG_TYPE_SQUARE
    uint8_t sq_num_off_cycles;
    //! Output signal type to be generated by signal generator
    uint8_t output_type; //1->real only, 2-> imag only, 3 -> complx
    //! Set cx_enable = 1 to enable CX path
    uint8_t cx_enable;
    //! Do regression test for siggen
    uint8_t regression;
    //! Reserved to align to 32 bit boundary
    uint8_t reserved[2];
    //! PRBS seed for CX path PRBS generator
    uint32_t prbs_seed;
    //! PRBS tap for CX path PRBS generator
    uint32_t prbs_taps;
    //! TX UMSK table file name, user needs to upload this file to TFS before running the diag
    int8_t tx_umsk_table_file_name[16];
    //! CX UMSK table file name, user needs to upload this file to TFS before running the diag
    int8_t cx_umsk_table_file_name[16];
    //! set expected correlation peek pos for siggen + d-probe
    uint32_t exp_correl_pos;
    //! selects either I (0) or Q (1)
    uint32_t prbs_iq_sel;
    //! Set cx_tx_sel bitmap respect to tx select
    uint32_t cx_tx_sel;
    //! Select tiny or long scan 0-tiny once scan, 1-tiny continuous, 2-long once, 3-long continuous scan
    uint32_t scan_type;
    //! Select the Chip count for siggen
    uint32_t Lc;
    //! Select the symbol delay value before TXU modulator, can be 0-31
    uint32_t txu_symb_delay;
};

//! Diag output structure for "std_rdc::lab_DProbe"
struct StdLab_DProbe_output
{
    //! Cross-correlator's i-channel output
    int32_t xcorr_i_result[7];

    //! Cross-correlator's q-channel output
    int32_t xcorr_q_result[7];

    //! Covariance-correlator's i-channel output
    int32_t cov_i_result[7];

    //! Covariance-correlator's q-channel output
    int32_t cov_q_result[7];

    //! IQ magnitude imbalance measurement result
    int32_t iq_mag_result;

    //! IQ phase imbalance measurement result
    int32_t iq_pha_result;

    //! DC offset measurement result
    int32_t dc_i_result;

    //! DC offset measurement result
    int32_t dc_q_result;

    //! RMS measurement result
    int32_t rms_result;
};

//! Diag Output structure for "std_rdc::lab_scope"
struct StdLab_scope_output
{
    //! DDR location at which ADC data is captured
    void *scope_out;

    //! Output file that contains the ADC captured data
    CHAR  adc_output_file[16];

    //! Specifies the ADC output data size
    uint32_t output_size;
};

struct StdLab_AProbe_output
{
    FLOAT tx_output;
    FLOAT rx_output;
    FLOAT sh_output;
};

SRS_CLOSE_NAMESPACE()

#endif // SRS_HDR_STD_LAB_STRUCTS_H
