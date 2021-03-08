#ifndef SRS_HDR_RDC_SCAN_CTRL_H
#define SRS_HDR_RDC_SCAN_CTRL_H 1
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


#include "modules/drivers/radar/uhnder_radar/driver/system-radar-software/env-uhnder/coredefs/uhnder-common.h"
#include "modules/drivers/radar/uhnder_radar/driver/system-radar-software/engine/common/eng-api/rdc-structs.h"
#include "modules/drivers/radar/uhnder_radar/driver/system-radar-software/engine/common/eng-api/rhal_out.h"
#include "window.h"
#if __SCP__
#include "antennas.h"
#endif

SRS_DECLARE_NAMESPACE()


extern const CHAR* rdc_preset_descriptions[];
enum RDC_ScanPresetEnum
{
    VP1a,
    VP1as,
    VP1b,
    VP1bb,
    VP1c,
    VP4,
    VP8,
    VP9,
    VP9f,
    VP11,
    VP12,
    VP13,
    VP14,
    VP14f,
    VP14ff,
    VP15f,
    VP15m,
    VP15s,
    VP16,
    CP1a,
    CP1as,
    CP1b,
    CP1bb,
    CP1c,
    CP2,

    NUM_PRESETS
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
    int32_t                  scan_count;       //!< number of scans or -1 for indefinite
    uint32_t                 scan_interval_us; //!< target interval between scan starts, in microseconds (0 = continuous)
    uint32_t                 scan_loop_count;  //!< number of scan configurations, 1 ~ single, >1 ~ deterministic loop

    RDC_clutter_image_format az_only_ci_format;
    RDC_clutter_image_format az_el_ci_format;

    //! When persistent_scan_config is false (default), the radar data agent
    //! automatically releases the RDC_ScanConfig once the 'scan_count' scans are
    //! completed.  When persistent_scan_config is true, the scan config is not
    //! released when scan_count is reached, allowing more scans to be run again
    //! without re-creating the scan configuration. Once a persistent scan
    //! config has been created, an RDC_ScanControl with scan_loop_count=0 and
    //! scan_count>0 can be sent to the radar data agent to trigger additional
    //! scans. An RDC_ScanControl with both scan_loop_count=0 and scan_count=0
    //! will release the persisent RDC_ScanControl. While a persistent scan
    //! config is held by the radar data agent, no other scan config can be
    //! created.
    bool                     persistent_scan_config;

    uint32_t                 future_use_u32[3];

    void defaults()
    {
        memset(this, 0, sizeof(*this));

        scan_count        = -1;
        scan_interval_us  = 0;
        scan_loop_count   = 1;

        az_only_ci_format = CIMG_M16P_D7P;
        az_el_ci_format   = CIMG_M16PP_D7PP;
        persistent_scan_config = false;
    }
};


/*! RDC_ScanDescriptor describes a single scan configuration. When configuring
 * deterministic loop, there must be a contiguous array of descriptors */
struct RDC_ScanDescriptor
{
    /* Describe scan configuration, all set by apply_preset(RDC_ScanPresetEnum) */
    uint32_t              pulse_chips;         //!< number of chips in each pulse (Lc)
    uint32_t              N_bins;              //!< number of pulses, or doppler bins
    uint16_t              chip_rate;           //!< chip rate in Mcps
    uint16_t              R_bins;              //!< number of range bins in RDC1
    uint16_t              rangebin_start;      //!< chip distance of first range bin in VP mode
    uint8_t               chan_ratio;          //!< ratio of doppler bins to channelizer bins - 1 if disabled
    uint8_t               rb_combine;          //!< combine every N consecutive range bins together
    bool                  variable_power_mode; //!< bool use variable power mode
    uint8_t               scan_reserved_u8[3];
    uint32_t              corr_length;         //!< (VP) Correlation Length
    uint32_t              chips_per_ping;      //!< (VP) number of chips per ping (512 or 256)
    RDC_rdc1_sharing      rdc1_shared;         //!< indicates how SIs are to be re-used by RADAR-HAL layer
    RHAL_Symbol_rate      symbol_rate_enum;
    RDC_MIMOType          mimo_type_enum;
    RDC_PeriodicCodeType  code_type_enum;
    uint32_t              hadamard_M;
    uint32_t              scan_future_use_u32[8];
    /* End of scan configuration */

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
    bool                  fcu_enable;          //!< Enable or Disable FMCW interference cancellation
    bool                  tcu_enable;          //!< Enable or Disable larget target cancellation

    /* applicable only for constant power scans */
    bool                  icu_d_enable;        //!< Enable or Disable ICU for cancelling very close range reflection
    int8_t                mu_coeff;            //!< Coeffiecient for ICU-D adaptation
    int8_t                sym_bulk_delay;      //!< Number of clocks to delay icu data
    int8_t                icu_d_scale;         //!< Scale factor for ICU-D

    uint8_t               disable_ss_curvature; //!< disable curvature of static slice based on ego velocity

    /* applicable only for variable power scans */
    int16_t               rx_delay;            //!< RX data delay, -1 for default
    int16_t               rx_early_unsquelch;  //!< analog Rx unsquelch ahead of Rx-On part of ping in the units of number of chips
    int16_t               tx_early_squelch;    //!< Variable power mode Tx Squelch ahead in RBs, -1 for default
    uint16_t              tx_power_mask;       //!< Mask of transmitters to (analog) power off, in "HW12" order
    uint16_t              tx_dsquelch_mask;    //!< Mask of transmitters to (digitally) squelch, in "HW12" order
    uint16_t              reserved_2;

    FLOAT                 doppler_shift_fraction;

    uint32_t              extra_future_use_u32[6];

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
        ss_doppler          = 19;

        rx_gain_enum        = VGAGain_DEFAULT;
        tx_gain_enum        = Tx_Gain_DEFAULT;
        rx_bw_enum          = Rx_BW_DEFAULT;
        tx_bw_enum          = Tx_BW_DEFAULT;

        umsk_mode           = true;
        fcu_enable          = false;
        tcu_enable          = false;

        /* The ICU-D should be enabled for constant power scans, disabled for
         * variable power scans */
        mu_coeff            = -1;   // use default
        sym_bulk_delay      = -1;   // use default
        icu_d_scale         = -1;   // use default

        rx_early_unsquelch  = 1024; // use default
        tx_early_squelch    = 1024; // use default
        rx_delay            = -1;   // use default
        tx_power_mask       = 0;    // no TX power masking
        tx_dsquelch_mask    = 0;    // no TX digital squelch
        disable_ss_curvature = 0;   // always allow static slice to curve based on ego vel
    }

    /// Modify this descriptor to use one of the predefined scan presets
    void apply_preset(RDC_ScanPresetEnum p)
    {
        switch (p)
        {
        case VP1a:
            //       R    N    chan Lc     MHz  corr  cpb rbc rdc1_shared
            setup_vp(128, 1008, 14, 11264, 500, 5120, 512, 1, double_buffer);
            break;

        case VP1as:
            //       R    N    chan Lc     MHz  corr  cpb rbc rdc1_shared
            setup_vp(128, 1680, 14, 11264, 500, 5120, 512, 1, double_buffer);
            break;

        case VP1b:
            //       R    N    chan Lc     MHz  corr  cpb rbc rdc1_shared
            setup_vp(128, 1008, 14, 11264, 250, 5120, 512, 1, double_buffer);
            break;

        case VP1bb:
            //       R    N    chan Lc     MHz  corr  cpb rbc rdc1_shared
            setup_vp(128, 1008, 14, 11264, 167, 5120, 512, 1, double_buffer);
            break;

        case VP1c:
            //       R    N    chan Lc     MHz  corr  cpb rbc rdc1_shared
            setup_vp(128, 1008, 14, 11264, 125, 5120, 512, 1, double_buffer);
            break;

        case VP4:
            //       R    N    chan Lc     MHz  corr  cpb  rbc rdc1_shared
            setup_vp(256, 1008, 14, 11264, 250, 5120, 1024, 1, double_buffer);
            break;

        case VP8:
            //       R    N    chan Lc     MHz  corr  cpb  rbc rdc1_shared
            setup_vp(512, 840,   7, 11264, 250, 5120, 1024, 1, single_buffer);
            break;

        case VP9:
            //       R    N    chan Lc     MHz  corr  cpb  rbc rdc1_shared
            setup_vp(512, 840,   7, 21504, 500, 5120, 1024, 1, single_buffer);
            break;

        case VP9f:
            //       R    N    chan Lc     MHz  corr  cpb  rbc rdc1_shared
            setup_vp(512, 720,   1, 21504, 500, 5120, 1024, 1, double_buffer);
            break;

        case VP11:
            //       R    N    chan Lc     MHz  corr  cpb  rbc rdc1_shared
            setup_vp(324, 1260, 10, 4096,  167, 2048, 1024, 1, single_buffer);
            break;

        case VP12:
            //       R    N    chan Lc     MHz  corr  cpb  rbc rdc1_shared
            setup_vp(256, 720,  10, 10240, 167, 5120, 1024, 1, double_buffer);
            break;

        case VP13:
            //       R    N    chan Lc     MHz  corr  cpb  rbc rdc1_shared
            setup_vp(256, 720,  10, 11264, 250, 5120, 1024, 1, double_buffer);
            break;

        case VP14:
            //       R    N    chan Lc     MHz  corr  cpb  rbc rdc1_shared
            setup_vp(256, 1260, 10,  4352, 250, 2048, 1024, 2, double_buffer);
            break;

        case VP14f:
            //       R    N    chan Lc     MHz  corr  cpb  rbc rdc1_shared
            setup_vp(192, 1260, 10,  4352, 250, 2048, 1024, 2, double_buffer);
            break;

        case VP14ff:
            //       R    N    chan Lc     MHz  corr  cpb  rbc rdc1_shared
            setup_vp(192, 1008,  7,  4352, 250, 2048, 1024, 2, double_buffer);
            break;

        case VP15f:
            //       R    N    chan Lc   MHz  corr  cpb  rbc rdc1_shared
            setup_vp(128, 840, 7,  4096, 167, 2048, 512, 1, double_buffer);
            break;

        case VP15m:
            //       R    N    chan  Lc    MHz  corr  cpb  rbc rdc1_shared
            setup_vp(128, 1008, 14,  4096, 167, 2048, 512, 1, double_buffer);
            break;

        case VP15s:
            //       R    N    chan  Lc    MHz  corr  cpb  rbc rdc1_shared
            setup_vp(128, 1260, 10,  4096, 167, 2048, 512, 1, double_buffer);
            break;

        case VP16:
            //       R    N    chan  Lc    MHz  corr  cpb  rbc rdc1_shared
            setup_vp(128, 1008, 14,  6144, 125, 3072, 512, 1, double_buffer);
            break;

        case CP1a:
            //       R    N    chan Lc     MHz  rbc rdc1_shared
            setup_cp(128, 1008, 14, 11264, 500, 1, double_buffer);
            break;

        case CP1as:
            //       R    N    chan Lc     MHz rbc rdc1_shared
            setup_cp(128, 1680, 14, 11264, 500, 1, double_buffer);
            break;

        case CP1b:
            //       R    N    chan Lc     MHz rbc rdc1_shared
            setup_cp(128, 1008, 14, 11264, 250, 1, double_buffer);
            break;

        case CP1bb:
            //       R    N    chan Lc     MHz rbc rdc1_shared
            setup_cp(128, 1008, 14, 11264, 167, 1, double_buffer);
            break;

        case CP1c:
            //       R    N    chan Lc     MHz rbc rdc1_shared
            setup_cp(128, 1008, 14, 11264, 125, 1, double_buffer);
            break;

        case CP2:
            //       R    N    chan  Lc   MHz rbc rdc1_shared
            setup_cp(128, 2520, 1, 11264, 500, 1, double_buffer);
            break;

        default:
            break;
        }
    }

    /// bulk configure variable power scan
    void setup_vp(uint16_t R,
                  uint32_t N,
                  uint8_t  chan,
                  uint32_t Lc,
                  uint16_t CR,
                  uint32_t corr,
                  uint32_t cpb,
                  uint8_t  rbc,
                  RDC_rdc1_sharing share)
    {
        variable_power_mode = true;
        icu_d_enable        = false;
        R_bins              = R;
        N_bins              = N;
        chan_ratio          = chan;
        pulse_chips         = Lc;
        chip_rate           = CR;
        corr_length         = corr;
        chips_per_ping      = cpb;
        rb_combine          = rbc;
        rdc1_shared         = share;
        mimo_type_enum      = MIMO_PRBS;
        code_type_enum      = PRN_MSEQUENCE;
        rangebin_start      = 0;
        symbol_rate_enum    = FSYMB_500M;
    }

    /// bulk configure variable power scan
    void setup_cp(uint16_t R,
                  uint32_t N,
                  uint8_t  chan,
                  uint32_t Lc,
                  uint16_t CR,
                  uint8_t  rbc,
                  RDC_rdc1_sharing share)
    {
        variable_power_mode = false;
        icu_d_enable        = true;
        R_bins              = R;
        N_bins              = N;
        chan_ratio          = chan;
        pulse_chips         = Lc;
        chip_rate           = CR;
        rb_combine          = rbc;
        rdc1_shared         = share;
        corr_length         = Lc;
        chips_per_ping      = 0;
        mimo_type_enum      = MIMO_PRBS;
        code_type_enum      = PRN_MSEQUENCE;
        rangebin_start      = 0;
        symbol_rate_enum    = FSYMB_500M;
    }
};

SRS_CLOSE_NAMESPACE()

#endif // SRS_HDR_RDC_SCAN_CTRL_H
