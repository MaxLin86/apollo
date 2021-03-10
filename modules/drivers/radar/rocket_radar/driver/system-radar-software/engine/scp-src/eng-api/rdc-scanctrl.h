// Copyright (C) Uhnder, Inc. All rights reserved. Confidential and Proprietary - under NDA.
// Refer to SOFTWARE_LICENSE file for details
#ifndef SRS_HDR_RDC_SCAN_CTRL_H
#define SRS_HDR_RDC_SCAN_CTRL_H 1
/*! \file */

#include "modules/drivers/radar/rocket_radar/driver/system-radar-software/env-reference/coredefs/uhnder-common.h"
#include "modules/drivers/radar/rocket_radar/driver/system-radar-software/engine/common/eng-api/rdc-structs.h"
#include "modules/drivers/radar/rocket_radar/driver/system-radar-software/engine/common/eng-api/rdc-common.h"
#include "modules/drivers/radar/rocket_radar/driver/system-radar-software/engine/scp-src/eng-api/uhmath.h"
#include "modules/drivers/radar/rocket_radar/driver/system-radar-software/engine/common/eng-api/rhal_out.h"
#include "window.h"
#if __SCP__
#include "uhnder-helpers.h"
#include "antennas.h"
#include "modules/drivers/radar/rocket_radar/driver/system-radar-software/engine/scp-src/eng-api/default-presets.h"
#endif

SRS_DECLARE_NAMESPACE()

extern const CHAR* rdc_preset_descriptions[];

enum RDC_ScanPresetEnum
{
    VP101, // VP1a with 256 range bins
    VP201, // VP1a with 256 range bins, 2GHz Sampling rate
    VP102, // City
    VP202, // City, 2GHz Sampling rate
    VP103, // Highway
    VP203, // Highway, 2GHz Sampling rate
    VP104, // Highway-2
    VP105, // City-1
    VP211, // 1 Giga chip rate variable mode scan
    CP101, // CP1a with 256 range bins
    CP201, // CP1a with 256 range bins, 2GHz Sampling rate
    CP102, // CpCity
    CP202, // CpCity, 2GHz Sampling rate
    CP103, // Highway, CP Mode - cooked up scan, for first bringup using digital loopback
    CP203, // Highway, CP Mode - cooked up scan, for first bringup using digital loopback, 2GHz Sampling rate
    CP104, // Basic scan to run @ 1G Sampling rate
    CP210, // 1 Giga chip rate continuous power mode scan
    VP114, // Like VP101, but fewer pings
    VP114U, // Like VP114, but correlating all pings
    VP104N, // VP104 scan with nuking targets
    VP105N, // VP105 scan with nuking targets
    VP204, // Highway-2, 2GHz Sampling Rate
    VP205, // City-1, 2GHz Sampling Rate

    // Add new presets HERE ^^^^.
    // Whenever a new preset is added, the following steps should be taken to expose
    // the new preset throughout SRS and RRA:
    //      1. Add the preset to the strings table rdc_preset_descriptions[] in engine/scp-src/eng-api/string-tables.h
    //      2. Add the preset in RRA:  radar-remote-api/python/sip/rdc_scancontrol.sip
    NUM_PRESETS
};

enum RDC_ScanCtrlFlags
{
    RDCSCF_FCU_ENABLE = 0,              //!< Enable or Disable FMCW interference cancellation
    RDCSCF_TCU_ENABLE = 1,              //!< Enable or Disable larget target cancellation
    RDCSCF_DISABLE_SS_CURVATURE = 2,    //!< disable static slice curvature for egovel
    RDCSCF_COMPLEX_SS = 3,              //!< complex RDC3 static slice
    RDCSCF_SIGGEN_ENABLE = 4,           //!< force an output signal pattern (sin, square, etc)
    RDCSCF_COMPLEX_ACT = 5,             //!< complex RDC3 activations
    RDCSCF_PRN_CONTINUOUS = 6,          //!< if set, PRN codes are not reset at scan starts
    RDCSCF_ADC_RSU1_ENABLE = 7,         //!< Enable RSU1 when capturing ADC samples
    RDCSCF_ADC_RSU2_ENABLE = 8,         //!< Enable RSU2 when capturing ADC samples
    RDCSCF_FCU_AFTER_RSU2 = 9,          //!< Apply FMCW cancellation after resampling
    RDCSCF_ANALOG_LOOPBACK = 10,        //!< loopback analog TX directly to RX
    RDCSCF_NO_RDC1 = 11,                //!< disable write of RDC1, implies SS1ONLY
    RDCSCF_DIGITAL_LOOPBACK = 12,       //!< Enable digital loopback
    RDCSCF_DISABLE_IQ_CORRECTION = 13,  //!< Disable application of calibrated IQ corrections
    RDCSCF_FORCE_SW_SVA = 14,           //!< Enable or Disable force SW-SVA option
    RDCSCF_SS1ONLY = 15,                //!< If set, completely disable PROC (FEU + ss2)
    RDCSCF_ADC_CDATA_CAL = 16,          //!< If set, trigger hardware ADC CDATA calibration
    RDCSCF_NO_OVERLAP_SCAN_PROC = 17,   //!< If set, do not allow proc to overlap with scans (for entire frame)
    RDCSCF_DISABLE_VRX_CORRECTION = 18, //!< Disable application of calibrated IQ corrections
    RDCSCF_BACK_TO_BACK_IN_FRAME = 19,  //!< If set, run all scans (FE) in one frame back to back, before any procs (BE)
    RDCSCF_QDU_OVERRIDE = 20,           //!< If set, override the QDU pattern and size configuration
    RDCSCF_DOP_ROTATOR = 21,            //!< If set, override doppler rotator configurations
    RDCSCF_PGU_TIMER_MODE = 22,         //!< If set, use PGU timer mode for scan start, else use immediate start
    RDCSCF_RDC1_NO_STRIDE = 23,         //!< Do not allow additional stride in RDC1, for RDC1 replay only
    RDCSCF_REDUCE_NUM_VRX = 24,         //!< Try to reduce number of VRx by reducing PRN codes assigned to Transmitters
    //
    // 32bits in scan_control_flags
    //
    // Whenever a flag is added here, the following steps should be taken to expose
    // the new flag throughout RRA:
    //      1. Add the flag in radar-remote-api/python/sip/rra/rdc_scancontrol.sip
    //      2. In radar-remote-api/python/rra/lib/scans.py, add the flag to
    //          a.  from ..structs import (
    //          b.  def make_argparser(
    //          c.  def make_scandesc(
    //      3. Rebuild RRA:  "make"
};

/*! UHDP_TYPE_SCAN_CONTROL messages have an RDC_ScanControl header, and then one
 * or more RDC_ScanDescriptor instances (as specified by scan_loop_count),
 * describing all of the scan types to run in a deterministic loop.
 * scan_loop_count == 1 is the equivalent of scan mode SINGLE. If scan_count or
 * scan_loop_count are 0, all current scanning is stopped. Note that
 * RDC_ScanControl is only used in UhDP messaging, while RDC_ScanDescriptor is
 * used by all radar software which runs scans. */
struct RDC_ScanControl
{
    int32_t                  scan_count;        //!< number of scans or -1 for indefinite
    uint32_t                 frame_interval_us; //!< target interval between scan frame starts, in microseconds (0 = continuous)
    uint32_t                 scan_loop_count;   //!< number of scan configurations, 1 ~ single, >1 ~ deterministic loop

    RDC_clutter_image_format az_only_ci_format;
    RDC_clutter_image_format az_el_ci_format;

    //! This is a mechanism for avoiding the overhead of re-configuring a scan
    //! each time you run a scan. When persistent_frame_config is false
    //! (default), the radar data agent automatically releases the
    //! RDC_FrameConfig once the 'scan_count' scans are completed.  When
    //! persistent_frame_config is true, the frame config is not
    //! released when scan_count is reached, allowing more scans to be run again
    //! without re-creating the frame configuration. Once a persistent scan
    //! config has been created, an RDC_ScanControl with scan_loop_count=0 and
    //! scan_count>0 can be sent to the radar data agent to trigger additional
    //! scans. An RDC_ScanControl with both scan_loop_count=0 and scan_count=0
    //! will release the persisent RDC_ScanControl. While a persistent scan
    //! config is held by the radar data agent, no other frame config can be
    //! created.
    uint32_t                 persistent_frame_config;

    RDC_AnalogPowerMode      analog_power_mode;

    RHAL_Ego_Velocity_Mode   ego_velocity_mode;             //!< Mode selector for internal ego-velocity estimation

    uint32_t                 future_use_u32[7];

    void defaults()
    {
        memset(this, 0, sizeof(*this));

        scan_count        = -1;
        frame_interval_us = 0;
        scan_loop_count   = 1;

        az_only_ci_format = CIMG_M16P_D7P;
        az_el_ci_format   = CIMG_M16PP_D7PP;
        persistent_frame_config = false;
        analog_power_mode = APM_DEFAULT;
        ego_velocity_mode = EV_MODE_DISABLED;
    }
};

/*! RDC_ScanDescriptor describes a single scan configuration. When configuring
 * deterministic loop, there must be a contiguous array of these descriptors */
struct RDC_ScanDescriptor
{
    /* Describe scan preset configuration, all set by apply_preset(RDC_ScanPresetEnum) */
    uint32_t              N_bins;               //!< number of pulses, or doppler bins
    uint16_t              R_bins;               //!< number of range bins in RDC1
    uint16_t              rangebin_start;       //!< chip distance of first range bin in VP mode
    uint8_t               chan_ratio;           //!< ratio of doppler bins to channelizer bins - 1 if disabled
    uint8_t               rb_combine;           //!< combine every N consecutive range bins together
    bool                  variable_power_mode;  //!< bool use variable power mode
    uint8_t               scan_reserved_u8;
    RDC_MIMOType          mimo_type_enum;
    uint32_t              hadamard_M;
    RDC_rdc1_sharing      rdc1_shared;          //!< indicates how SIs are to be re-used by RADAR-HAL layer
    RDC_PeriodicCodeType  code_type_enum;
    RDC_Symbol_rate       symbol_rate_enum;
    RDC_DAC_Rate          dac_rate_enum;        //!< always 4x symbol rate
    RDC_Sampling_rate     adc_sample_rate_enum;
    RDC_Fsym_Fchip_Ratio  symb_chip_ratio;      //!< Symbol Rate to Chip Rate Ratio
    RDC_Fdac_IF_Ratio     Fdac_LoIF_ratio;      //!< DAC Rate to Low Intermediate Frequency ratio
    uint32_t              chips_per_ping;       //!< VP: number of chips per ping; CP: Must be set to 1
    uint16_t              rx_chips_per_ping;    //!< No. of chips in RX side per ping; CP: Must be set to 1
    uint16_t              tx_chips_per_ping;    //!< No. of chips in RX side per ping; CP: Must be set to 1
    uint32_t              pings_per_pri;        //!< No. of correlated pings in one PRI
    uint32_t              ss1_min_pri_chips;    //!< TODO: Delete me once calculated in SRS.
                                                //!< No. of total chips in a PRI (including discarded) needed to ensure SS1 BE completes in one PRI time
    uint32_t              Lc_min;               //!< Minimum PRI time specified as number of chips, used to achieve a particular doppler resolution

    bool                  icu_d_enable;        //!< Enable or Disable ICU for cancelling very close range reflection
    int8_t                mu_coeff;            //!< Coeffiecient for ICU-D adaptation
    int8_t                sym_bulk_delay;      //!< Number of clocks to delay icu data
    int8_t                icu_d_scale;         //!< Scale factor for ICU-D

    // NB: inlined struct RDC_PumpConfig starts here
    uint16_t              pump_inp_range_bin;
    uint16_t              reserved_u16;
    uint16_t              kill_rangebin_p1;    //!< number of range bins to nuke
    uint16_t              start_rangebin_p1;   //!< chip distance of first range bin
    uint16_t              end_rangebin_p1;     //!< last chip number in current pump
    uint16_t              kill_rangebin_p2;    //!< number of range bins to nuke
    uint16_t              start_rangebin_p2;   //!< chip distance of first range bin
    uint16_t              end_rangebin_p2;     //!< last chip number in current pump
    uint16_t              kill_rangebin_p3;    //!< number of range bins to nuke
    uint16_t              start_rangebin_p3;   //!< chip distance of first range bin
    uint16_t              end_rangebin_p3;     //!< last chip number in current pump
    uint16_t              kill_rangebin_p4;    //!< number of range bins to nuke
    uint16_t              start_rangebin_p4;   //!< chip distance of first range bin
    uint16_t              end_rangebin_p4;     //!< last chip number in current pump
    // NB: inlined struct RDC_PumpConfig ends here

    uint8_t               preset_applied;      //!< set automatically by set_defaults() and apply_preset()
    uint8_t               scan_future_use[3];

    uint32_t              scan_future_use_u32[8];

    /* End of scan preset configuration */

    FLOAT                 carrier_frequency;   //!< MHz, any frequency out-of-range will result in system default RF
    uint32_t              antenna_config_id;   //!< index into AntennaConfig's enumerated antenna configurations

    RDC_WindowType        angle_window;        //!< Suggested window type (or SVA) for beamforming
    RDC_InterpolationType interpolation_enum;
    RHAL_RxVGAGainPresets rx_gain_enum;        //!< Preset for Rx VGA, BQF gains
    RHAL_TxGainPresets    tx_gain_enum;        //!< Preset for Tx Gain and Power mode
    RHAL_RxBWPresets      rx_bw_enum;          //!< Preset for Receiver bandwidth
    RHAL_TxBWPresets      tx_bw_enum;          //!< Preset for Transmitter bandwidth
    RDC_WindowType        doppler_window;      //!< Dopper window type
    uint32_t              ss_doppler;          //!< Number of static slice doppler layers

    bool                  umsk_mode;           //!< Enable or Disable UMSK modulation (vs BPSK modulation)

    uint8_t               loopback_rx_input;   //!< digital lpbk bitmap of target receivers for looped TX output
    uint8_t               loopback_tx0_dac;    //!< digital lpbk 6 bits TX ID, 2 bits DAC lane ID
    uint8_t               loopback_tx1_dac;    //!< digital lpbk 6 bits TX ID, 2 bits DAC lane ID
    FLOAT                 tgt_in_m;            //!< Target location (in meters) for digital loopback scan

    /* applicable only for variable power scans */
    int16_t               rx_delay;            //!< RX data delay, -1 for default
    int16_t               rx_early_unsquelch;  //!< analog Rx unsquelch ahead of Rx-On part of ping in the units of number of chips
    uint16_t              tx_power_mask;       //!< Mask of transmitters to (analog) power off, in "HW12" order
    uint16_t              tx_dsquelch_mask;    //!< Mask of transmitters to (digitally) squelch, in "HW12" order

    uint32_t              scan_control_flags;  //!< RDC_ScanCtrlFlags bits
    FLOAT                 ego_vel_shift_param; //!< offste of ego-velocity to shift Doppler using rotator

    uint8_t               fcu_mu;              //!< Sets LMS adaptation speed. Selects values as 2^-k (5 bits)
    uint8_t               fcu_leak;            //!< Sets coefficient adaptation leakage (5 bits)
    uint32_t              fcu_tap_disable;     //!< When bits are set to 1, disables coefficients from updating (20 bits)

    uint8_t               num_si;              //!< Number of SI to allocate, or 0 to use default number
    uint8_t               num_pi;              //!< Number of PI to allocate, or 0 to use default number

    uint8_t               loopback_tx_num;     //!< analog (RF) loopback source TX
    uint8_t               loopback_rx_num;     //!< analog (RF) loopback dest RX
    RDC_Loopback_Rotation loopback_rotation;   //!< analog (RF) loopback rotation

    uint32_t              qdu_pattern_override_size;      //!< QDU override params
    uint32_t              qdu_pattern_override_pattern;   //!< QDU override enable

    RDC_Doppler_Shift_Update_Mode   doppler_shift_update_mode; //!< doppler shift 1. absolute shift 2. fraction of ego vel shift 3. ego+offset
    RDC_Doppler_Shift_RDC_Mode      doppler_shift_rdc_mode;    //!< rdc shift mode 1: no shift 2 : RDC shift

    uint16_t              spl_doppler_bins;    //!< Number of doppler bins specified for Magnitude RDC3 act through special queue
    bool                  use_custom_ciu_mode; //!< Flag for custom ciu correlation mode
    bool                  complex_ptcloud;

    RHAL_Memory_Type      mem_store_static_slice;           //!< Select which memory to store HW Static Slice output
    RHAL_Memory_Type      mem_store_activations_and_dlcr;   //!< Select which memory to store HW Lower/Upper Activations and DLCR Histograms outputs
    RHAL_Memory_Type      mem_store_special_activations;    //!< Select which memory to store HW Special Activations output
    RHAL_Memory_Type      mem_store_channelizer;            //!< Select which memory to store HW Channelizer output

    uint32_t              user_data;           //!< User defined identifier for the RDC_ScanConfig that will be created from this descriptor

    uint32_t              future_use_u32[23];

    void get_pump_config(RDC_PumpConfig& pump_config)
    {
        uh_memcpy(&pump_config.rb_cfg_per_pump[0], &kill_rangebin_p1, sizeof(pump_config.rb_cfg_per_pump));
        pump_config.inp_range_bin = pump_inp_range_bin;
    }

    void set_pump_config(const RDC_PumpConfig& pump_config)
    {
        pump_inp_range_bin = pump_config.inp_range_bin;
        uh_memcpy(&kill_rangebin_p1, &pump_config.rb_cfg_per_pump[0], sizeof(pump_config.rb_cfg_per_pump));
    }

    // All units in meters and seconds

    FLOAT UHINLINE get_chip_rate_per_sec() const
    {
        FLOAT Fcarr = carrier_frequency * 1e9f;
        FLOAT chip_rate = (FLOAT)symbol_rate_enum/(FLOAT)symb_chip_ratio;
        return (chip_rate * Fcarr) / 80.0e3f; // Rc Chip rate (chips/sec)
    }

    FLOAT UHINLINE get_pulse_rate_interval_sec() const
    {
        FLOAT chip_time = 1.0F / get_chip_rate_per_sec();

        uint32_t pulse_chips = (Lc_min > chips_per_ping*pings_per_pri) ? Lc_min : chips_per_ping*pings_per_pri;
        pulse_chips = (pulse_chips > ss1_min_pri_chips) ? pulse_chips : ss1_min_pri_chips;
        return FLOAT(pulse_chips) * chip_time; // pulses per second
    }

    FLOAT UHINLINE get_dwell_time_sec() const
    {
        return get_pulse_rate_interval_sec() * N_bins;
    }

    FLOAT UHINLINE get_doppler_bin_width_mps() const
    {
        uint32_t pulse_chips = (Lc_min > chips_per_ping*pings_per_pri) ? Lc_min : chips_per_ping*pings_per_pri;
        pulse_chips = (pulse_chips > ss1_min_pri_chips) ? pulse_chips : ss1_min_pri_chips;
        FLOAT PRF = get_chip_rate_per_sec() / FLOAT(pulse_chips); // pulses per second
        FLOAT doppler_bin_hz = PRF / N_bins;                      // Doppler bin size (Hz)
        FLOAT Fcarr = carrier_frequency * 1e9f;                   // RF Hz
        FLOAT lambda = SPEED_OF_LIGHT / Fcarr;
        return doppler_bin_hz * lambda / 2.0F;   // Doppler bin size (m/s)
    }

    FLOAT UHINLINE get_max_unambiguous_velocity_mps() const
    {
        return get_doppler_bin_width_mps() * FLOAT(N_bins) / 2.0F;
    }

    FLOAT UHINLINE get_range_bin_width_m() const
    {
        FLOAT chip_time = 1.0F / get_chip_rate_per_sec();
        return (rb_combine * chip_time) * SPEED_OF_LIGHT / 2.0F;
    }

    // the actual scan range can be smaller in variable power scans, because of discarded range bins
    FLOAT UHINLINE get_nominal_max_range_m() const
    {
        return get_range_bin_width_m() * R_bins;
    }

    FLOAT UHINLINE rfa_to_snr_dB(FLOAT rate_of_false_alarm, uint32_t num_rough_angles) const
    {
        uint32_t num_cells = R_bins * N_bins * num_rough_angles;
        return rfa_to_db(rate_of_false_alarm, num_cells);
    }

    void UHINLINE set_scan_control_flag(const RDC_ScanCtrlFlags f, bool enabled)
    {
        if (enabled)
        {
            scan_control_flags |= uint32_t(1U << f);
        }
        else
        {
            scan_control_flags &= ~uint32_t(1U << f);
        }
    }

    bool UHINLINE get_scan_control_flag(const RDC_ScanCtrlFlags f) const
    {
        return (scan_control_flags & uint32_t(1U << f)) != 0U;
    }

#if __SCP__
    // return false if there are no differences in the scan settings of the two
    // descriptors
    bool check_preset_diffs(const RDC_ScanDescriptor& other)
    {
        return !!memcmp(this, &other, uhoffsetof(&RDC_ScanDescriptor::preset_applied));
    }

    // return false if there are no differences in the non-scan settings of the
    // two descriptors
    bool check_non_preset_diffs(const RDC_ScanDescriptor& other)
    {
        // ignore changes to antenna_config_id
        bool diff = !!memcmp(&angle_window, &other.angle_window, sizeof(*this) - uhoffsetof(&RDC_ScanDescriptor::angle_window));
        diff |= carrier_frequency != other.carrier_frequency;
        return diff;
    }
#endif

    void set_defaults(RDC_ScanPresetEnum p)
    {
        memset(this, 0, sizeof(*this));

#if __SCP__
        antenna_config_id = AntennaConfig::instance().get_basic_antenna_config_id(BAC_TRADITIONAL_1D);
#endif

        apply_preset(p);

        apply_non_preset_defaults();
    }

    void apply_non_preset_defaults()
    {
        doppler_window      = WINDOW_TAYLOR_55;
        angle_window        = WINDOW_SVA;
        interpolation_enum  = RDC_INTERP_QUADRATIC;

        rx_gain_enum        = VGAGain_DEFAULT;
        tx_gain_enum        = Tx_Gain_DEFAULT;
        rx_bw_enum          = Rx_BW_DEFAULT;
        tx_bw_enum          = Tx_BW_DEFAULT;

        /* The ICU-D should be enabled for constant power scans, disabled for
         * variable power scans */
        mu_coeff            = -1;   // use default
        sym_bulk_delay      = -1;   // use default
        icu_d_scale         = -1;   // use default

        rx_delay            = -1;   // use default
        tx_power_mask       = 0U;   // no TX power masking
        tx_dsquelch_mask    = 0U;   // no TX digital squelch
        loopback_rx_input   = 0U;
        loopback_tx0_dac    = 0xFFU;
        loopback_tx1_dac    = 0xFFU;
        tgt_in_m            = 0.0F;
        loopback_tx_num     = 0U;
        loopback_rx_num     = 0U;
        loopback_rotation   = LPBK_BYPASS;
        Fdac_LoIF_ratio     = DAC_LOIF_0;
        umsk_mode           = true;
        num_si              = 0;
        num_pi              = 0;
        spl_doppler_bins    = 8;
        complex_ptcloud     = 0;

        mem_store_static_slice          = RHAL_MEM_DEFAULT;
        mem_store_activations_and_dlcr  = RHAL_MEM_DEFAULT;
        mem_store_special_activations   = RHAL_MEM_DEFAULT;
        mem_store_channelizer           = RHAL_MEM_DEFAULT;
    }

#if __SCP__
    // convert 'default' settings into the presets defined by the radar software
    void apply_radar_default_gains()
    {
        if (rx_gain_enum >= NUM_VGAGAIN_PRESETS)
        {
            if (variable_power_mode)
            {
                rx_gain_enum = AntennaConfig::instance().get_variable_power_vga_gain();
            }
            else
            {
                rx_gain_enum = AntennaConfig::instance().get_constant_power_vga_gain();
            }
        }

        if (rx_bw_enum >= NUM_RXBW_PRESETS)
        {
            rx_bw_enum = DEF_RX_BW;
        }

        if (tx_gain_enum >= NUM_TxGAIN_PRESETS)
        {
            if (variable_power_mode)
            {
                tx_gain_enum = VP_TX_GAIN;
            }
            else
            {
                tx_gain_enum = CP_TX_GAIN;
            }
        }

        if (tx_bw_enum >= NUM_TXBW_PRESETS)
        {
            tx_bw_enum = DEF_TX_BW;
        }
    }
#endif

    void apply_preset(RDC_ScanPresetEnum p)
    {
        preset_applied = uint8_t(p);

        // this flag is treated as a preset field, it must be set or reset by
        // apply_preset()
        set_scan_control_flag(RDCSCF_NO_OVERLAP_SCAN_PROC, false);
        set_scan_control_flag(RDCSCF_REDUCE_NUM_VRX, false);

        if (apply_preset_revb(p))
        {
            ss_doppler = 7;
        }
        else
        {
#if __SCP__
            UHASSERT(!"UNKNOWN_SCAN_PRESET");
#endif
        }
    }

    bool apply_preset_revb(RDC_ScanPresetEnum p)
    {
        RDC_PumpConfig pump_config;
        pump_config.set_defaults();

        bool val = true;
        switch (p)
        {
        case VP101: // City
            setup_scan(true,                 // Variable Power
                        76.6F,               // Carrier Freq
                        FSAMP_1G,            // Sampling Rate
                        DAC_4G,              // DAC Rate
                        FSYMB_500M,          // Symbol Rate
                        FSYMB_FCHIP_ONE,     // Chip Rate
                        512,                 // Chips Per Ping
                        254,                 // Tx Chips Per Ping
                        256,                 // Rx Chips per ping
                        0,                   // Rx early unsquelch
                        16,                  // Pings correlated per PRI
                        1008,                // Num PRIs (N)
                        1,                   // Channelizer Ratio
                        256,                 // Num Range bins
                        1,                   // Range bin combine
                        multi_buffer         // rdc1 shared
                        );
            ss1_min_pri_chips = 20*512;
            use_custom_ciu_mode = 0;
            break;

        case VP114: // City, like VP101, but with fewer total and correlated pings
            setup_scan(true,                 // Variable Power
                        76.6F,               // Carrier Freq
                        FSAMP_1G,            // Sampling Rate
                        DAC_4G,              // DAC Rate
                        FSYMB_500M,          // Symbol Rate
                        FSYMB_FCHIP_ONE,     // Chip Rate
                        512,                 // Chips Per Ping
                        254,                 // Tx Chips Per Ping
                        256,                 // Rx Chips per ping
                        0,                   // Rx early unsquelch
                        11,                  // Pings correlated per PRI
                        630,                 // Num PRIs (N)
                        1,                   // Channelizer Ratio
                        256,                 // Num Range bins
                        1,                   // Range bin combine
                        multi_buffer         // rdc1 shared
                        );
            ss1_min_pri_chips = 15*512;
            use_custom_ciu_mode = 0;
            set_scan_control_flag(RDCSCF_NO_OVERLAP_SCAN_PROC, true);
            break;

        case VP114U:
            setup_scan(true,                 // Variable Power
                        76.6F,               // Carrier Freq
                        FSAMP_1G,            // Sampling Rate
                        DAC_4G,              // DAC Rate
                        FSYMB_500M,          // Symbol Rate
                        FSYMB_FCHIP_ONE,     // Chip Rate
                        512,                 // Chips Per Ping
                        254,                 // Tx Chips Per Ping
                        256,                 // Rx Chips per ping
                        0,                   // Rx early unsquelch
                        15,                  // Pings correlated per PRI
                        630,                 // Num PRIs (N)
                        1,                   // Channelizer Ratio
                        256,                 // Num Range bins
                        1,                   // Range bin combine
                        multi_buffer         // rdc1 shared
                        );
            ss1_min_pri_chips = 15*512;
            use_custom_ciu_mode = 1;
            set_scan_control_flag(RDCSCF_NO_OVERLAP_SCAN_PROC, true);
            break;

        case VP201: // City
            setup_scan(true,                 // Variable Power
                        76.6F,               // Carrier Freq
                        FSAMP_2G,            // Sampling Rate
                        DAC_8G,              // DAC Rate
                        FSYMB_500M,          // Symbol Rate
                        FSYMB_FCHIP_ONE,     // Chip Rate
                        512,                 // Chips Per Ping
                        254,                 // Tx Chips Per Ping
                        256,                 // Rx Chips per ping
                        0,                   // Rx early unsquelch
                        20,                  // Pings correlated per PRI
                        1008,                // Num PRIs (N)
                        1,                   // Channelizer Ratio
                        256,                 // Num Range bins
                        1,                   // Range bin combine
                        multi_buffer         // rdc1 shared
                        );
            use_custom_ciu_mode = 0;
            break;

        case VP102: // City
            setup_scan(true,                 // Variable Power
                        76.6F,               // Carrier Freq
                        FSAMP_1G,            // Sampling Rate
                        DAC_4G,              // DAC Rate
                        FSYMB_500M,          // Symbol Rate
                        FSYMB_FCHIP_ONE,     // Chip Rate
                        700,                 // Chips Per Ping
                        348,                 // Tx Chips Per Ping
                        350,                 // Rx Chips per ping
                        0,                   // Rx early unsquelch
                        8,                   // Pings correlated per PRI
                        420,                 // Num PRIs (N)
                        1,                   // Channelizer Ratio
                        512,                 // Num Range bins
                        1,                   // Range bin combine
                        multi_buffer         // rdc1 shared
                        );
            ss1_min_pri_chips = 16*700;
            use_custom_ciu_mode = 0;
            break;

        case VP202: // City
            setup_scan(true,                 // Variable Power
                        76.6F,               // Carrier Freq
                        FSAMP_2G,            // Sampling Rate
                        DAC_8G,              // DAC Rate
                        FSYMB_500M,          // Symbol Rate
                        FSYMB_FCHIP_ONE,     // Chip Rate
                        700,                 // Chips Per Ping
                        348,                 // Tx Chips Per Ping
                        350,                 // Rx Chips per ping
                        0,                   // Rx early unsquelch
                        8,                   // Pings correlated per PRI
                        420,                 // Num PRIs (N)
                        1,                   // Channelizer Ratio
                        512,                 // Num Range bins
                        1,                   // Range bin combine
                        multi_buffer         // rdc1 shared
                        );
            ss1_min_pri_chips = 16*700;
            use_custom_ciu_mode = 0;
            break;

        case VP103: // Highway
            setup_scan(true,                 // Variable Power
                        76.6F,               // Carrier Freq
                        FSAMP_1G,            // Sampling Rate
                        DAC_4G,              // DAC Rate
                        FSYMB_500M,          // Symbol Rate
                        FSYMB_FCHIP_FOUR,    // Chip Rate
                        300,                 // Chips Per Ping
                        148,                 // Tx Chips Per Ping
                        150,                 // Rx Chips per ping
                        0,                   // Rx early unsquelch
                        6,                   // Pings correlated per PRI
                        504,                 // Num PRIs (N)
                        1,                   // Channelizer Ratio
                        256,                 // Num Range bins
                        1,                   // Range bin combine
                        multi_buffer         // rdc1 shared
                        );
            ss1_min_pri_chips = 7*300;
            use_custom_ciu_mode = 0;
            break;

        case VP203: // Highway
            setup_scan(true,                 // Variable Power
                        76.6F,               // Carrier Freq
                        FSAMP_2G,            // Sampling Rate
                        DAC_8G,              // DAC Rate
                        FSYMB_500M,          // Symbol Rate
                        FSYMB_FCHIP_FOUR,    // Chip Rate
                        300,                 // Chips Per Ping
                        148,                 // Tx Chips Per Ping
                        150,                 // Rx Chips per ping
                        0,                   // Rx early unsquelch
                        6,                   // Pings correlated per PRI
                        504,                 // Num PRIs (N)
                        1,                   // Channelizer Ratio
                        256,                 // Num Range bins
                        1,                   // Range bin combine
                        multi_buffer         // rdc1 shared
                        );
            ss1_min_pri_chips = 7*300;
            use_custom_ciu_mode = 0;
            break;

        case VP104: // Highway
            setup_scan(true,                 // Variable Power
                        76.6F,               // Carrier Freq
                        FSAMP_1G,            // Sampling Rate
                        DAC_4G,              // DAC Rate
                        FSYMB_500M,          // Symbol Rate
                        FSYMB_FCHIP_FOUR,    // Chip Rate
                        300,                 // Chips Per Ping
                        148,                 // Tx Chips Per Ping
                        150,                 // Rx Chips per ping
                        0,                   // Rx early unsquelch
                        6,                   // Pings correlated per PRI
                        630,                 // Num PRIs (N)
                        1,                   // Channelizer Ratio
                        256,                 // Num Range bins
                        1,                   // Range bin combine
                        multi_buffer         // rdc1 shared
                        );
            ss1_min_pri_chips = 7*300;
            use_custom_ciu_mode = 0;
            set_scan_control_flag(RDCSCF_NO_OVERLAP_SCAN_PROC, true);
            break;

        case VP104N: // Highway with nuking
            setup_scan(true,                 // Variable Power
                        76.6F,               // Carrier Freq
                        FSAMP_1G,            // Sampling Rate
                        DAC_4G,              // DAC Rate
                        FSYMB_500M,          // Symbol Rate
                        FSYMB_FCHIP_FOUR,    // Chip Rate
                        300,                 // Chips Per Ping
                        148,                 // Tx Chips Per Ping
                        150,                 // Rx Chips per ping
                        0,                   // Rx early unsquelch
                        6,                   // Pings correlated per PRI
                        630,                 // Num PRIs (N)
                        1,                   // Channelizer Ratio
                        256,                 // Num Range bins
                        1,                   // Range bin combine
                        multi_buffer         // rdc1 shared
                        );
            ss1_min_pri_chips = 8*300;
            use_custom_ciu_mode = 0;
            pump_config.inp_range_bin = 512;

            pump_config.rb_cfg_per_pump[0].kill_rangebin = 0;
            pump_config.rb_cfg_per_pump[0].start_rangebin = 0;
            pump_config.rb_cfg_per_pump[0].end_rangebin = 49;

            pump_config.rb_cfg_per_pump[1].kill_rangebin  = 32;
            pump_config.rb_cfg_per_pump[1].start_rangebin = 50;
            pump_config.rb_cfg_per_pump[1].end_rangebin  = 255;

            set_scan_control_flag(RDCSCF_NO_OVERLAP_SCAN_PROC, true);
            break;

        case VP105: // City
            setup_scan(true,                 // Variable Power
                        76.6F,               // Carrier Freq
                        FSAMP_1G,            // Sampling Rate
                        DAC_4G,              // DAC Rate
                        FSYMB_500M,          // Symbol Rate
                        FSYMB_FCHIP_ONE,     // Chip Rate
                        1024,                // Chips Per Ping
                        510,                 // Tx Chips Per Ping
                        512,                 // Rx Chips per ping
                        0,                   // Rx early unsquelch
                        12,                  // Pings correlated per PRI
                        360,                 // Num PRIs (N)
                        1,                   // Channelizer Ratio
                        512,                 // Num Range bins
                        1,                   // Range bin combine
                        multi_buffer         // rdc1 shared
                        );
            ss1_min_pri_chips = 17408;
            use_custom_ciu_mode = 0;
            set_scan_control_flag(RDCSCF_NO_OVERLAP_SCAN_PROC, true);
            break;

        case VP105N: // City with nuking
            setup_scan(true,                 // Variable Power
                        76.6F,               // Carrier Freq
                        FSAMP_1G,            // Sampling Rate
                        DAC_4G,              // DAC Rate
                        FSYMB_500M,          // Symbol Rate
                        FSYMB_FCHIP_ONE,     // Chip Rate
                        1024,                // Chips Per Ping
                        510,                 // Tx Chips Per Ping
                        512,                 // Rx Chips per ping
                        0,                   // Rx early unsquelch
                        12,                  // Pings correlated per PRI
                        360,                 // Num PRIs (N)
                        1,                   // Channelizer Ratio
                        512,                 // Num Range bins
                        1,                   // Range bin combine
                        multi_buffer         // rdc1 shared
                        );
            ss1_min_pri_chips = 17408;
            use_custom_ciu_mode = 0;
            pump_config.inp_range_bin = 512;

            pump_config.rb_cfg_per_pump[0].kill_rangebin  = 0;
            pump_config.rb_cfg_per_pump[0].start_rangebin = 0;
            pump_config.rb_cfg_per_pump[0].end_rangebin  = 255;

            pump_config.rb_cfg_per_pump[1].kill_rangebin  = 70;
            pump_config.rb_cfg_per_pump[1].start_rangebin = 0;
            pump_config.rb_cfg_per_pump[1].end_rangebin  = 511;

            set_scan_control_flag(RDCSCF_NO_OVERLAP_SCAN_PROC, true);
            break;

         case VP211: // City
            setup_scan(true,                 // Variable Power
                        76.6F,               // Carrier Freq
                        FSAMP_2G,            // Sampling Rate
                        DAC_8G,              // DAC Rate
                        FSYMB_1G,            // Symbol Rate
                        FSYMB_FCHIP_ONE,     // Chip Rate
                        1024,                // Chips Per Ping
                        510,                 // Tx Chips Per Ping
                        512,                 // Rx Chips per ping
                        0,                   // Rx early unsquelch
                        18,                  // Pings correlated per PRI
                        360,                 // Num PRIs (N)
                        1,                   // Channelizer Ratio
                        512,                 // Num Range bins
                        1,                   // Range bin combine
                        multi_buffer         // rdc1 shared
                        );
            ss1_min_pri_chips = 30*1024;
            use_custom_ciu_mode = 0;
            set_scan_control_flag(RDCSCF_NO_OVERLAP_SCAN_PROC, true);
            break;

        case VP204: // Highway
            setup_scan(true,                 // Variable Power
                        76.6F,               // Carrier Freq
                        FSAMP_2G,            // Sampling Rate
                        DAC_8G,              // DAC Rate
                        FSYMB_1G,            // Symbol Rate
                        FSYMB_FCHIP_EIGHT,   // Chip Rate
                        300,                 // Chips Per Ping
                        148,                 // Tx Chips Per Ping
                        150,                 // Rx Chips per ping
                        0,                   // Rx early unsquelch
                        6,                   // Pings correlated per PRI
                        630,                 // Num PRIs (N)
                        1,                   // Channelizer Ratio
                        256,                 // Num Range bins
                        1,                   // Range bin combine
                        multi_buffer         // rdc1 shared
                        );
            ss1_min_pri_chips = 7*300;
            set_scan_control_flag(RDCSCF_NO_OVERLAP_SCAN_PROC, true);
            break;

        case VP205: // City
            setup_scan(true,                 // Variable Power
                        76.6F,               // Carrier Freq
                        FSAMP_2G,            // Sampling Rate
                        DAC_8G,              // DAC Rate
                        FSYMB_1G,            // Symbol Rate
                        FSYMB_FCHIP_TWO,     // Chip Rate
                        1024,                // Chips Per Ping
                        510,                 // Tx Chips Per Ping
                        512,                 // Rx Chips per ping
                        0,                   // Rx early unsquelch
                        12,                  // Pings correlated per PRI
                        360,                 // Num PRIs (N)
                        1,                   // Channelizer Ratio
                        512,                 // Num Range bins
                        1,                   // Range bin combine
                        multi_buffer         // rdc1 shared
                        );
            ss1_min_pri_chips = 17408;
            set_scan_control_flag(RDCSCF_NO_OVERLAP_SCAN_PROC, true);
            break;

        case CP101: // CP1a with 256 range bins
            setup_scan(false,                // Variable Power
                        76.6F,               // Carrier Freq
                        FSAMP_1G,            // Sampling Rate
                        DAC_4G,              // DAC Rate
                        FSYMB_500M,          // Symbol Rate
                        FSYMB_FCHIP_ONE,     // Chip Rate
                        1,                   // Chips Per Ping
                        1,                   // Tx Chips Per Ping
                        1,                   // Rx Chips per ping
                        0,                   // Rx early unsquelch
                        6144,                // Pings correlated per PRI
                        1008,                // Num PRIs (N)
                        1,                   // Channelizer Ratio
                        256,                 // Num Range bins
                        1,                   // Range bin combine
                        multi_buffer         // rdc1 shared
                        );
            ss1_min_pri_chips = 8192;
            use_custom_ciu_mode = 0;
            break;

        case CP201: // CP1a with 256 range bins
            setup_scan(false,                // Variable Power
                        76.6F,               // Carrier Freq
                        FSAMP_2G,            // Sampling Rate
                        DAC_8G,              // DAC Rate
                        FSYMB_500M,          // Symbol Rate
                        FSYMB_FCHIP_ONE,     // Chip Rate
                        1,                   // Chips Per Ping
                        1,                   // Tx Chips Per Ping
                        1,                   // Rx Chips per ping
                        0,                   // Rx early unsquelch
                        8192,                // Pings correlated per PRI
                        1008,                // Num PRIs (N)
                        1,                   // Channelizer Ratio
                        256,                 // Num Range bins
                        1,                   // Range bin combine
                        multi_buffer         // rdc1 shared
                        );
            ss1_min_pri_chips = 8192;
            use_custom_ciu_mode = 0;
            break;

        case CP102: // CpCity
            setup_scan(false,                // Variable Power
                        76.6F,               // Carrier Freq
                        FSAMP_1G,            // Sampling Rate
                        DAC_4G,              // DAC Rate
                        FSYMB_1G,            // Symbol Rate
                        FSYMB_FCHIP_TWO,     // Chip Rate
                        1,                   // Chips Per Ping
                        1,                   // Tx Chips Per Ping
                        1,                   // Rx Chips per ping
                        0,                   // Rx early unsquelch
                        4096,                // Pings correlated per PRI
                        630,                 // Num PRIs (N)
                        1,                   // Channelizer Ratio
                        512,                 // Num Range bins
                        1,                   // Range bin combine
                        multi_buffer         // rdc1 shared
                        );
            ss1_min_pri_chips = 11200;
            use_custom_ciu_mode = 0;
            break;

        case CP202: // CpCity
            setup_scan(false,                // Variable Power
                        76.6F,               // Carrier Freq
                        FSAMP_2G,            // Sampling Rate
                        DAC_8G,              // DAC Rate
                        FSYMB_1G,            // Symbol Rate
                        FSYMB_FCHIP_TWO,     // Chip Rate
                        1,                   // Chips Per Ping
                        1,                   // Tx Chips Per Ping
                        1,                   // Rx Chips per ping
                        0,                   // Rx early unsquelch
                        7000,                // Pings correlated per PRI
                        630,                 // Num PRIs (N)
                        1,                   // Channelizer Ratio
                        512,                 // Num Range bins
                        1,                   // Range bin combine
                        multi_buffer         // rdc1 shared
                        );
            ss1_min_pri_chips = 11200;
            use_custom_ciu_mode = 0;
            break;

        case CP103: // Highway, CP mode
            setup_scan(false,                // Variable Power
                        76.6F,               // Carrier Freq
                        FSAMP_1G,            // Sampling Rate
                        DAC_4G,              // DAC Rate
                        FSYMB_500M,          // Symbol Rate
                        FSYMB_FCHIP_ONE,     // Chip Rate
                        1,                   // Chips Per Ping
                        1,                   // Tx Chips Per Ping
                        1,                   // Rx Chips per ping
                        0,                   // Rx early unsquelch
                        6144,                // Pings correlated per PRI
                        504,                 // Num PRIs (N)
                        1,                   // Channelizer Ratio
                        256,                 // Num Range bins
                        1,                   // Range bin combine
                        multi_buffer         // rdc1 shared
                        );
            ss1_min_pri_chips = 8192;
            use_custom_ciu_mode = 0;
            break;

        case CP203: // Highway, CP mode
            setup_scan(false,                // Variable Power
                        76.6F,               // Carrier Freq
                        FSAMP_2G,            // Sampling Rate
                        DAC_8G,              // DAC Rate
                        FSYMB_500M,          // Symbol Rate
                        FSYMB_FCHIP_ONE,     // Chip Rate
                        1,                   // Chips Per Ping
                        1,                   // Tx Chips Per Ping
                        1,                   // Rx Chips per ping
                        0,                   // Rx early unsquelch
                        8192,                // Pings correlated per PRI
                        504,                 // Num PRIs (N)
                        1,                   // Channelizer Ratio
                        256,                 // Num Range bins
                        1,                   // Range bin combine
                        multi_buffer         // rdc1 shared
                        );
            ss1_min_pri_chips = 8192;
            use_custom_ciu_mode = 0;
            break;

        case CP104: // Basic scan @ 1G Sampling rate
            setup_scan(false,                // Variable Power
                        76.6F,               // Carrier Freq
                        FSAMP_1G,            // Sampling Rate
                        DAC_4G,              // DAC Rate
                        FSYMB_500M,          // Symbol Rate
                        FSYMB_FCHIP_TWO,     // Chip Rate
                        1,                   // Chips Per Ping
                        1,                   // Tx Chips Per Ping
                        1,                   // Rx Chips per ping
                        0,                   // Rx early unsquelch
                        8192,                // Pings correlated per PRI
                        630,                 // Num PRIs (N)
                        1,                   // Channelizer Ratio
                        256,                 // Num Range bins
                        1,                   // Range bin combine
                        multi_buffer         // rdc1 shared
                        );
            ss1_min_pri_chips = 8192;
            use_custom_ciu_mode = 0;
            break;

        case CP210:  // Basic scan @ 1G Chip rate
            setup_scan(false,                // Variable Power
                76.6F,               // Carrier Freq
                FSAMP_2G,            // Sampling Rate
                DAC_8G,              // DAC Rate
                FSYMB_1G,            // Symbol Rate
                FSYMB_FCHIP_ONE,     // Chip Rate
                1,                   // Chips Per Ping
                1,                   // Tx Chips Per Ping
                1,                   // Rx Chips per ping
                0,                   // Rx early unsquelch
                10240,                // Pings correlated per PRI
                504,                 // Num PRIs (N)
                1,                   // Channelizer Ratio
                256,                 // Num Range bins
                1,                   // Range bin combine
                multi_buffer         // rdc1 shared
            );
            ss1_min_pri_chips = 13312;
            use_custom_ciu_mode = 0;
            break;

        default:
            val = false;
            break;
        }

        set_pump_config(pump_config);
        return val;
    }

private:

    void setup_scan(bool                    variable_power,

                    FLOAT                   Fc,
                    RDC_Sampling_rate       Fs,
                    RDC_DAC_Rate            dac_rate,
                    RDC_Symbol_rate         Fsymb,
                    RDC_Fsym_Fchip_Ratio    Symb_CR_ratio,

                    uint16_t                ping_chips,                 // = 1 for CP mode
                    uint16_t                tx_ping_chips,              // = 1 for CP mode
                    uint16_t                rx_ping_chips,              // = 1 for CP mode
                    uint8_t                 rx_early_unsquelch_chips,   // Don't care for CP mode

                    uint32_t                corr_pings,                 // =Corr length for CP mode

                    uint32_t                N,
                    uint8_t                 chan,

                    uint16_t                R,
                    uint8_t                 rbc,
                    RDC_rdc1_sharing        share)
    {
        variable_power_mode = variable_power;

        if (variable_power_mode == false)
        {
            icu_d_enable        = true;
        }
        else
        {
            icu_d_enable        = false;
            rx_early_unsquelch  = rx_early_unsquelch_chips;
        }

        carrier_frequency       = Fc;
        R_bins                  = R;
        N_bins                  = N;
        chan_ratio              = chan;
        rb_combine              = rbc;
        rdc1_shared             = share;
        chips_per_ping          = ping_chips;
        mimo_type_enum          = MIMO_PRBS;
        code_type_enum          = PRN_MSEQUENCE;
        rangebin_start          = 0;
        adc_sample_rate_enum    = Fs;
        symbol_rate_enum        = Fsymb;
        dac_rate_enum           = dac_rate;
        hadamard_M              = 1;
        rx_chips_per_ping       = rx_ping_chips;
        tx_chips_per_ping       = tx_ping_chips;
        pings_per_pri           = corr_pings;
        symb_chip_ratio         = Symb_CR_ratio;
    }
};

SRS_CLOSE_NAMESPACE()

#endif // SRS_HDR_RDC_SCAN_CTRL_H
