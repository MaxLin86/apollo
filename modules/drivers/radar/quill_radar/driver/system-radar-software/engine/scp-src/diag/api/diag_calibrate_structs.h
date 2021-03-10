#ifndef SRS_HDR_DIAG_CALIBRATE_STRUCTS_H
#define SRS_HDR_DIAG_CALIBRATE_STRUCTS_H 1
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


#include "modules/drivers/radar/quill_radar/driver/system-radar-software/env-uhnder/coredefs/uhnder-common.h"

SRS_DECLARE_NAMESPACE()

//! test output structure for "calibrate::cal_scan_setup"
struct Calibrate_scan_setup_output
{
    INT                 az_uniform_vrx;
    INT                 el_uniform_vrx;             // Number of VRx in uniformly filled section (az/el)
    INT                 az_vrx_spacing;             // Spacing (1/2 * az_vrx_spacing_factor Lambda units ) between elements, in azimuth (Y)
    INT                 el_vrx_spacing;             // Spacing (1/2 * az_vrx_spacing_factor Lambda units ) between elements, in elevation (Z)
    FLOAT               fov_az_min;
    FLOAT               fov_az_max;                 // field of view (min,max radians) in azimuth
    FLOAT               fov_el_min;
    FLOAT               fov_el_max;                 // field of view (min,max radians) in elevation
    FLOAT               az_vrx_spacing_factor;      // Set to 1.0 for 1/2 Lambda spacing (default) or scale appropriately
                                                    // This variable modifies the effect of az_vrx_spacing
    FLOAT               el_vrx_spacing_factor;      // Set to 1.0 for 1/2 Lambda spacing (default) or scale appropriately
                                                    // This variable modifies the effect of az_vrx_spacing
    bool                nyquist_angle_bins;         // use nyquist sized angle bins (rather than uniform sized)
    uint8_t             reserved[3];

    // Followed by this data:
    //uint8_t             active_vrx[MAX_VRX / 8];
    //uint8_t             dup_vrx[MAX_VRX / 8];
};


//! Test input structure for "calibrate::cal_beamforming_step"
struct Calibrate_cal_beamforming_step_input
{
    //! Target azimuth angle (degrees, + is right)
    FLOAT azimuth_angle;
    //! Target elevation angle (degrees, + is down)
    FLOAT elevation_angle;
};

//! Test input structure for "calibrate::cal_beamforming_init"
struct Calibrate_cal_beamforming_init_input
{
    //! Range to target (meters)
    FLOAT target_range;
    //! Number of azimuth steps
    int16_t num_az_angles;
    //! Number of elevation steps
    int16_t num_el_angles;
    //! Dry run only - don't save changes
    int16_t dryrun;
};


//! SabineB Test input structure for "calibrate::cal_vrx_align_init"
struct Calibrate_vrx_align_init_input
{
    FLOAT   initial_target_range_m;

    FLOAT   physical_target_range_m;
};


//! Test input structure for "calibrate::cal_vrx_align_control"
struct Calibrate_cal_vrx_align_control_input
{
    //! Range bin (center) where spillover/target is expected
    FLOAT target_range;
    uint16_t align_mode;    // TODO REMOVE ME, I'M NOT USED ANYMORE
    //! Num of Range bins of data needed around spillover/target
    uint16_t rb_halfwidth;
    //! flip threshold
    uint16_t flip_threshold;
    //! rx flip time
    uint16_t rx_flip_time;
    //!tx extra threshold
    uint16_t tx_extra_threshold;
    //! tx flip time
    uint16_t tx_flip_time;
    //! tx nflip
    uint16_t tx_nflip;
    //! max iterations for vrx alignment
    uint16_t max_iterations;
    //! enable debug logs
    uint8_t  verbose;
    //! factory mode
    uint8_t  factory_mode;
    //! enable umsk_mode
    uint8_t umsk_mode;
    //! save RDC1 zerod file
    uint8_t save_rdc1;
};

//! Test input structure for "calibrate::cal_dc_megacal"
struct Calibrate_dc_megacal_input
{
    uint32_t  cx_code;
    uint32_t  verbose;
};

//! Test input structure for "calibrate::cal_dc_minical"
struct Calibrate_dc_minical_input
{
    uint32_t  minical_mode;
};

//! Test input structure for "calibrate::transfer tfs to flash"
struct Transfer_tfs_flash_input
{
    CHAR   fname[64];
};

//! Test input structure for "calibrate::apply the cali data to register"
struct Apply_Cal_data_input
{
    uint8_t   calib_type;
};

//! Diag Input structure for "std_rhal::tx_suppression"
struct Calibrate_tx_suppression_input
{
    //! Specifies whether to run in Admin-mode or user-mode
    //! 0. Admin-mode: In this mode, all regulators of a given group will be run for all channels
    //! 1. User-mode: In this mode, a given regulator only run for choosen channel
    uint16_t op_mode;

    //! Rx channel which gets used for measuring DC
    uint16_t rx_ch_num;

    //! Tx channel map to suppress
    uint16_t tx_ch_map;

    //! Enable/Disable LDO cal in Tx Carrier Suppression
    uint8_t en_ldo_cal;

    //! Debug prints
    uint8_t verbose;
};

//! Diag Input structure for "std_rhal::analog_cal_regulator"
struct Calibrate_ldo_regulator_input
{
    //! Specifies whether to run in Admin-mode or user-mode
    //! 0. Admin-mode: In this mode, all regulators of a given group will be run for all channels
    //! 1. User-mode: In this mode, a given regulator only run for choosen channel
    uint8_t op_mode;

    //! Specifies the register group
    //! 0 for Aprobe_TX
    //! 1 for Aprobe_SH
    //! 2 for Aprobe_RX
    int8_t group;

    //! Below table details on the supply selection, for a given value
    //! and choosen group (using above "group" param)
    //! ---------------------------------------------------------------------------------
    //! | Val |        TX_supply      |      SH_supply      |          RX_supply        |
    //! ---------------------------------------------------------------------------------
    //! |  0  | TX_IQDAC_VREG_0P9     | SH_CLKDIV_0P9       |        RX_LNA1_0P9        |
    //! |  1  | TX_IQDAC_VREG_1P5     | SH_LO_0P9           |        RX_LNA2_0P9        |
    //! |  2  | TX_QILO_VREG_ILO_0P9  | SH_SYNTH_0P9        |        RX_MIX_1P5         |
    //! |  3  | TX_QILO_VREG_INV_0P9  |                     |        RX_QILO_INV_0P9    |
    //! |  4  | TX_PA3060_0P9         |                     |        RX_QILO_ILO_0P9    |
    //! |  5  | TX_PA3060_1P5         |                     |        RX_IQDAC_0P9       |
    //! |  6  | TX_PA120_0P9          |                     |        RX_IQDAC_1P5       |
    //! |  7  |                       |                     |                           |
    //! |  8  |                       |                     |                           |
    //! |  9  |                       |                     |                           |
    //! | 10  |                       |                     |                           |
    //! ---------------------------------------------------------------------------------
    int8_t supply_id;

    //! Specifies the TX to be used, supported values are 0,1,2,----,11
    int8_t channel_num;

    //! Specifies volatage setting
    FLOAT   voltage;
};

SRS_CLOSE_NAMESPACE()

#endif // SRS_HDR_DIAG_CALIBRATE_STRUCTS_H
