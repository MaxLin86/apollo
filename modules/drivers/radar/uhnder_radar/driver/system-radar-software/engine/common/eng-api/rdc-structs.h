#ifndef SRS_HDR_RDC_STRUCTS_H
#define SRS_HDR_RDC_STRUCTS_H 1
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
#include "modules/drivers/radar/uhnder_radar/driver/system-radar-software/engine/common/eng-api/uhtypes.h"
#include "modules/drivers/radar/uhnder_radar/driver/system-radar-software/engine/common/eng-api/rhal_out.h"

SRS_DECLARE_NAMESPACE()

// All antenna configurations (module types) must support these basic antenna
// configurations, in order for the engine to operate correctly
enum BasicAntennaConfigEnum
{
    BAC_TRADITIONAL_1D,
    BAC_TRADITIONAL_2D,
    BAC_SPANNING_SET1,
    BAC_SPANNING_SET2,
    BAC_IQ_CAL,

    NUM_BASIC_ANTENNA_CONFIGS
};

enum RDC_InterpolationType
{
    RDC_INTERP_QUADRATIC,
    RDC_INTERP_SINC,
};

enum RDC_rdc1_sharing
{
    multi_buffer = 0,
    single_buffer = 1,
    double_buffer = 2
};

enum RDC_MIMOType
{
    MIMO_PRBS,
    MIMO_HADAMARD,
    MIMO_RANGE_DOMAIN,
};

enum RDC_PeriodicCodeType
{
    PRN_APAS,
    PRN_MSEQUENCE,
    PRN_GOLAY,
};

// Note that the abosolute limit of scan_loop_count is RDC_MAX_SCAN_LOOP,
// but the practical limit might be less than this, depending on the memory
// requirements of each constituent scan
enum { RDC_MAX_SCAN_LOOP = 3 };

//! enumeration of the calibration flash files
enum RDC_CalFlashEnum
{
    RDC_CAL_DC,
    RDC_CAL_TCS,
    RDC_CAL_VRX,
    RDC_CAL_IQ,
    RDC_CAL_TEMP,
    RDC_CAL_ANGLE,
    RDC_CAL_DIAGONAL,
    RDC_CAL_STEERING,
    RDC_CAL_QILO,
    RDC_CAL_LDO,
    RDC_CAL_VTR,

    NUM_RDC_CAL_FLASH
};

enum RDC_antenna_type
{
    ANTENNA_UNDEFINED = 0,      //!< Reserved
    ANTENNA_TX,                 //!< Transmitter
    ANTENNA_RX,                 //!< Receiver
};

enum RDC_antenna_pattern
{
    RDC_ANTENNA_GAIN_3DB = 0,                   //!< angle from boresight (azimuth or elevation) at the -3dB gain
    RDC_ANTENNA_GAIN_6DB,                       //!< . . . at -6dB gain
    RDC_ANTENNA_GAIN_9DB,                       //!< . . . at -9dB gain
    RDC_ANTENNA_GAIN_12DB,                      //!< . . . at -12dB gain
    RDC_ANTENNA_GAIN_16DB,                      //!< . . . at -16dB gain
    RDC_ANTENNA_GAIN_20DB,                      //!< . . . at -20dB gain
    RDC_ANTENNA_GAIN_NUM                        //!< number of gain points
};

enum RDC_detection_type
{
    DET_STATIC,
    DET_DYNAMIC,
    DET_ALL
};

enum RDC_detection_sort
{
    SORT_RANGE,
    SORT_MAGNITUDE
};

enum RDC_detection_flags
{
    RDC_DET_FLAG_STATIC = (1 << 0),
    RDC_DET_FILTERED = (1 << 1),
};

extern const CHAR *clutter_image_enum_names[];
enum RDC_clutter_image_format
{
    // Format:  M#[CP]{H#[CP]}{D#[CP]}
    // #: Number of bits
    // M: Magnitude
    // H: Height image
    // D: Doppler image
    // C: Cartesian (x,y)
    // Z: Cartesian (x,y,z)  "Stereo mode"
    // P: Polar Azimuth (r,az)
    // PP: Polar Azimuth And Elevation (r,az,el)

    // PS: DO NOT REARRANGE THESE ENUMS
    CIMG_DEFAULT            = 0,        // allow RDC layer to use defaults

    // For Azimuth-only scans:
    CIMG_M8C                = 1,        //                                                                          (Unsupported)
    CIMG_M8P                = 2,        //                                                                          (Unsupported)
    CIMG_M16C               = 3,        // 16-bit Magnitude Cartesian 2D image (X,Y)                                (OLD)
    CIMG_M16P               = 4,        // 16-bit Magnitude Polar     2D image (R,A)
    CIMG_M16C_D8C           = 5,        // 16-bit Magnitude Cartesian 2D image (X,Y), with Cartesian Doppler image
    CIMG_M16C_D7P           = 6,        // 16-bit Magnitude Cartesian 2D image (X,Y), with Polar     Doppler image
    CIMG_M16P_D7P           = 7,        // 16-bit Magnitude Polar     2D image (R,A), with Polar     Doppler image

    // For Azimuth+Elevation scans:
    CIMG_M16Z               = 21,       // 16-bit Magnitude Cartesian 3D voxel cuboid                               (OLD)
    CIMG_M16Z_D7P           = 22,       // 16-bit Magnitude Cartesian 3D voxel cuboid, with Polar Doppler image     (Unsupported)
    CIMG_M16C_H8C_D8C       = 23,       // 16-bit Magnitude Cartesian, with 8-bit Height and Doppler Polar images
    CIMG_M16C_H7P_D7P       = 24,       // 16-bit Magnitude Cartesian, with 7-bit Height and Doppler Polar images   (NEW)
    CIMG_M16P_H7P_D7P       = 25,       // 16-bit Magnitude Polar,     with 7-bit Height and Doppler Polar images
    CIMG_M16PP_D7PP         = 26,       // 16-bit Magnitude Polar Azimuth and Polar Elevation, with 7-bit Doppler Polar/Polar image

    MAX_CIMG_FORMAT
};

static UHINLINE bool cifmt_has_height(RDC_clutter_image_format fmt)
{
    if ((fmt == CIMG_M16P_H7P_D7P) || (fmt == CIMG_M16C_H8C_D8C) || (fmt == CIMG_M16C_H7P_D7P))
    {
        return true;
    }
    else
    {
        return false;
    }
}


static UHINLINE bool cifmt_has_doppler(RDC_clutter_image_format fmt)
{
    if ((fmt == CIMG_M16C_D7P) || (fmt == CIMG_M16C_H7P_D7P) || (fmt == CIMG_M16P_D7P) ||
        (fmt == CIMG_M16Z_D7P) || (fmt == CIMG_M16C_H8C_D8C) || (fmt == CIMG_M16C_D8C) ||
        (fmt == CIMG_M16P_H7P_D7P) || (fmt == CIMG_M16PP_D7PP))
    {
        return true;
    }
    else
    {
        return false;
    }
}


static UHINLINE bool cifmt_has_height_or_doppler(RDC_clutter_image_format fmt)
{
    // currently this is equivalent to..
    return cifmt_has_doppler(fmt);
}


struct RDC_detection_region
{
    FLOAT                   range_min;                          //!< minimum range (meters, closest)
    FLOAT                   range_max;                          //!< maximum range (meters, farthest)
    FLOAT                   doppler_min;                        //!< minimum Doppler (meters/second, most outbound)
    FLOAT                   doppler_max;                        //!< maximum Doppler (meters/second, most inbound)
    FLOAT                   azimuth_min;                        //!< minimum azimuth angle (radians, leftmost)
    FLOAT                   azimuth_max;                        //!< maximum azimuth angle (radians, rightmost)
    FLOAT                   elevation_min;                      //!< minimum elevation angle (radians, uppermost)
    FLOAT                   elevation_max;                      //!< maximum elevation angle (radians, lowermost)
};

// also used by UHDP_TYPE_DETECTIONS - do not change without changing UHDP_VERSION
struct DetectionData                    // Instance of a detection
{
    FLOAT                   range;      // range (meters)
    FLOAT                   azimuth;    // azimuth angle (radians)
    FLOAT                   elevation;  // elevation angle (radians)
    FLOAT                   doppler;    // Doppler velocity (meters/second)
    FLOAT                   magnitude;  // signal power (dBFS)
    FLOAT                   snr;        // signal to noise ratio (dB)
    FLOAT                   rcs;        // RCS (dBsm)
    FLOAT                   pos_x;      // position (meters), radar relative SAE coord. system
    FLOAT                   pos_y;      // position (meters), radar relative SAE coord. system
    FLOAT                   pos_z;      // position (meters), radar relative SAE coord. system
    uint32_t                flags;      // enum RDC_detection_flags
    // TODO: add statistics (accuracy, resolution, ..)
};

//! struct used by UHDP_TYPE_POINT_CLOUD - do not change without changing UHDP_VERSION
struct PointCloudData
{
    enum { AZIMUTH_FRAC_BITS = 8 };

    enum { ELEVATION_FRAC_BITS = 8 };

    enum { SNR_FRAC_BITS = 8 };

    uint16_t                range;          //!< distance in units of range bin widths
    uint16_t                azimuth_fbin;   //!< interpolated fractional azimuth bin ID
    uint16_t                elevation_fbin; //!< interpolated fractional elevation bin ID
    uint16_t                doppler_bin;    //!< doppler bin
    uint16_t                snr_dB;         //!< snr in decibels with fractional bits
    uint8_t                 flags;
    uint8_t                 future_use_0;
};

//! struct used by ENV_UHNDER_EGO_VEL_DATA
struct EgoVelocityData // Instance of an ego velocity points
{
    FLOAT                  azimuth;     // azimuth angle (radians)
    FLOAT                  elevation;   // elevation angle (radians)
    FLOAT                  doppler;     // Doppler velocity (meters/second)
    FLOAT                  magnitude;   // signal power (linear)
};

//! struct used by UHDP_TYPE_MUSIC_CONTROL - do not change without changing UHDP_VERSION
struct RDC_music_request                    // Request for MUSIC processing
{
    FLOAT                  range;               //!< Range (meters) - center of region of interest
    FLOAT                  range_half_width;    //!< half-width of range region of interest
    FLOAT                  doppler;             //!< doppler velocity (meters/second) - center of region
    FLOAT                  doppler_half_width;  //!< half-width of doppler region of interest
    FLOAT                  azimuth_min;         //!< Minimum azimuth angle (radians)
    FLOAT                  azimuth_max;         //!< Maximum azimuth angle (radians)
    FLOAT                  elevation_min;       //!< Minimum elevation angle (radians) (0 for Azimuth-only scan)
    FLOAT                  elevation_max;       //!< Maximum elevation angle (radians) (0 for Azimuth-only scan)
    FLOAT                  ns_thresh_dB;        //!< when 0, an autodetection heuristic is used
    uint32_t               peak_threshold;

    void set_defaults()
    {
        range = 7.3F;
        range_half_width = 0.0F;
        doppler = 0.0F;
        doppler_half_width = 0.0F;
        azimuth_min = -0.087266462599716F;      // -5 degrees (in radians)
        azimuth_max =  0.087266462599716F;      // +5 degrees (in radians)
        elevation_min = 0.0F;
        elevation_max = 0.0F;
        ns_thresh_dB = 0;
        peak_threshold = 100000;
    }
};

enum { NUM_MUSIC_1D_SAMPLES = 64 };

enum { NUM_MUSIC_2D_AZ_SAMPLES = 8 };

enum { NUM_MUSIC_2D_EL_SAMPLES = 8 };

// also used by UHDP_TYPE_MUSIC_SAMPLES - do not change without changing UHDP_VERSION
// must be allocated by RDC layer from DRAM so DSP can write into it
struct  RDC_MusicSampleData
{
    /* Filled in by RSI::request_music from RDC_music_request */
    uint32_t umask_low;
    uint32_t umask_high;
    uint32_t num_smoothing_iters;
    uint32_t thresh_S;
    uint32_t music_peak_threshold;
    FLOAT    Ns;
    FLOAT    range;
    FLOAT    doppler;
    FLOAT    azimuth_min;     // Minimum azimuth angle (radians)
    FLOAT    azimuth_max;     // Maximum azimuth angle (radians)
    FLOAT    elevation_min;   // Minimum elevation angle (radians) (0 for Azimuth-only scan)
    FLOAT    elevation_max;   // Maximum elevation angle (radians) (0 for Azimuth-only scan)
    uint16_t num_azimuth_steps;
    uint16_t num_elevation_steps;
    /* Filled in by DSP */
    uint32_t inv_magnitude[NUM_MUSIC_1D_SAMPLES];
};

SRS_CLOSE_NAMESPACE()

#endif // SRS_HDR_RDC_STRUCTS_H
