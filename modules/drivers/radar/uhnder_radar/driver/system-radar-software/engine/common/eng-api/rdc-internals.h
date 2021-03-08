#ifndef SRS_HDR_RDC_INTERNALS_H
#define SRS_HDR_RDC_INTERNALS_H 1
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
#include "modules/drivers/radar/uhnder_radar/driver/system-radar-software/engine/common/eng-api/rhal_out.h"
#include "modules/drivers/radar/uhnder_radar/driver/system-radar-software/engine/common/eng-api/rdc-structs.h"
#include "modules/drivers/radar/uhnder_radar/driver/src/devptr.h"

SRS_DECLARE_NAMESPACE()


class RDC_ScanInstance_Impl;
class RDC_ScanConfig_Impl;
class RHAL_SvdInstance;
struct RDC_ScanMgrParams;

struct RDC_ScanInstParams;

struct RDC_Static_Image_Config;

enum { RANGE_FRAC_BITS = 7 };               //  7 bits of fraction,  9 bits of mantissa:  maximum range bins =    512

enum { AZIMUTH_FRAC_BITS = 7 };             //  7 bits of fraction,  9 bits of mantissa:  maximum azimuth bins =  512

enum { ELEVATION_FRAC_BITS = 11 };          // 11 bits of fraction,  5 bits of mantissa:  maximum elevation bins = 32

enum { DOPPLER_FRAC_BITS = 4 };             //  4 bits of fraction, 12 bits of mantissa:  maximum Doppler bins = 4096

enum { MAG_FRAC_BITS = 20 };

enum { NOISE_FLOOR_FRAC_BITS = 7 };

enum { MAP_DEC_EL_BITS = 2 };

enum { MAP_DEC_DOP_BITS = 2 };

enum { RD_ADDRESS_TABLE_SIZE = 8192 };

enum { RD_ADDRESS_TABLE_PRIME = 3697 };

enum { STATIC_DETECTION_BASE_INDEX  = 384 };  // offset of the static detections in the dsp detection buffer

enum { MAX_DETECTIONS =    768 };   // Maximum nuber of detections per scan

enum { MAX_RD_SPLITS =      32 };   // Maximum number of groups of RD bins that RDC3 can be split into.
#if SABINE_A
enum { MAX_RD_PER_SPLIT =  128 };   // Max size of a group of RD bins.  (128 RD bins requires 64KB of P5 Data RAM)
#elif SABINE_B
enum { MAX_RD_PER_SPLIT =  256 };   // Max size of a group of RD bins.  (256 RD bins requires 64KB of P5 Data RAM)
#endif
enum { MAX_MUSIC_PEAKS =     8 };   // Maximum nuber of MUSIC peaks per [R,D] bin

enum { MAX_MUSIC_RDBINS =    8 };   // Maximum nuber of MUSIC [R,D] bins in flight for processing on P5

enum { MAX_NYQ_FACTOR =     16 };   // Maximum nyquist oversample factor

enum { MAX_RDC2_VEC_SIZE =  20 };

enum { MAX_MUSIC_STV =     600 };   // Maximum nuber of pre-calculated MUSIC Steering Vectors

enum { CLUTTER_IMAGE_SIZE = 256 };   // Number of pixels in Static Image (square)

enum { CLUTTER_IMAGE_Z =      8 };   // Number of voxels vertically in Static image (must be a factor of STATIC_IMAGE_SIZE)

enum { MAX_EGO_VEL_LIST_SIZE = 256 }; // Maximum nuber of max_ego_velocity list size

enum { MAX_EGO_VEL_BINS = 512 - 2 };  // Maximum nuber ego velocity bins.

enum { RBIN_PIXELS_FRAC_BITS = 8 };

enum { MIN_RIDGE_DETECTION_ACTIVATION_NUM = 24};

enum { REFILTER_TARGET_ACTIVATIONS = 512 };

enum { RDC3_UPPER_THRESH_BASE_SNR = 25 };   // dB SNR for Upper Activation Threshold

//
// Structure for tracking RDC3 Moving Activations output from the hardware.
// This structure is a member of RDC_ScanInstance_Impl.
//
//
struct RDC_RDC3_raw_data
{
    uint16_t                num_range_bin;          // Number of range bins
    uint16_t                num_doppler_bin;        // Number of range bins
    uint16_t                num_angle_bin;          // Number of angle bins (Azimuth & Elevation combined)
    uint16_t                num_azimuth_bin;        // Number of angle bins in Azimuth dimension
    uint16_t                num_elevation_bin;      // Number of angle bins in Elevation dimension

    // Raw RDC3 data from RHAL
    uint16_t                rd_num_total;
    uint32_t                rd_num[NUM_RD_BUF];
    RHAL_RDsummary         *rd_info[NUM_RD_BUF];    // Pointer to RHAL data:  RDC3 (R,D) summary structures
    uint16_t               *rd_bins[NUM_RD_BUF];    // Pointer to RHAL data:  RDC3 (R,D) magnitude data

    uint16_t                num_splits;                                 // Number of groups of RD bins

    enum { ACT_IDX_BITS = 14 };                     // 14 bits means up to 16K activations are supported in each of the NUM_RD_BUF buckets
    enum { MAX_SEGMENTS_PER_SPLIT = 64 };

#if SABINE_B
    enum { MAX_ACTIVATIONS_PER_SPLIT = 256 };
#elif SABINE_A
    enum { MAX_ACTIVATIONS_PER_SPLIT = 128 };
#endif

    enum { MAX_ACT_HASH_SIZE = 4357 };              // Prime number at least 50% larger than max activations

    static const uint16_t UNUSED_HASH_KEY = 0xFFFFU; // Value of rd_hash[] entries when empty

    uint16_t                rd_hash[MAX_ACT_HASH_SIZE];
#if SABINE_B
    enum { MAX_CLUSTERS = 128 };
#else
    enum { MAX_CLUSTERS = 256 };
#endif
    enum { MAX_CLUSTER_ELEMS = 256 };
    RHAL_RDsummary         *clusters[MAX_CLUSTERS][MAX_CLUSTER_ELEMS]; // Array of cluster for range doppler activations.
    uint16_t                clusters_length[MAX_CLUSTERS];
    uint16_t                num_clusters;

    //  0                   1
    //  0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5
    // +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
    // |BKT|   Activation Index        |
    // +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
    //
    // The top two bits of each refilter_ID is the bucket ID, the remaining 14
    // bits is the index into that bucket

    uint16_t                num_segments[MAX_RD_SPLITS];
    uint16_t                refilter_ID[MAX_RD_SPLITS][MAX_SEGMENTS_PER_SPLIT];
    uint16_t                refilter_segment_len[MAX_RD_SPLITS][MAX_SEGMENTS_PER_SPLIT]; // number of activations in the segment

    uint32_t count_activations()
    {
        uint32_t count = 0;

        for (uint32_t i = 0; i < num_splits; i++)
        {
            for (uint32_t s = 0; s < num_segments[i]; s++)
            {
                count += refilter_segment_len[i][s];
            }
        }

        return count;
    }
};

struct  RDC_DetectionData
{
    uint16_t                range_bin[MAX_DETECTIONS];     // FLOAT value = (double)range_bin / 2^RANGE_FRAC_BITS
    uint16_t                azimuth_bin[MAX_DETECTIONS];   // FLOAT value = (double)azimuth_bin / 2^AZIMUTH_FRAC_BITS
    uint16_t                elevation_bin[MAX_DETECTIONS]; // FLOAT value = (double)elevation_bin / 2^ELEVATION_FRAC_BITS
    uint16_t                doppler_bin[MAX_DETECTIONS];   // FLOAT value = (double)doppler_bin / 2^DOPPLER_FRAC_BITS
    uint32_t                magnitude_raw[MAX_DETECTIONS]; // stored in dB, without SW exponent, FLOAT value = (double)magnitude_raw / 2^MAG_FRAC_BITS
    uint32_t                det_flags[MAX_DETECTIONS];     // enum RDC_detection_flags
};

struct RDC_EgoVelocityData
{
    uint16_t               ego_vel_histogram[MAX_EGO_VEL_BINS]; // histogram for the ego velocity (output)
    uint32_t               ego_vel_mag_list[MAX_EGO_VEL_LIST_SIZE]; // magnitude list for the top few points.
    uint16_t               ego_az_dop_list[MAX_EGO_VEL_LIST_SIZE]; // angle/doppler list for the top few points.(7:9), angle is el*az_bins+az
    uint16_t               ego_vel_list_size; // size of the ego velocity list.
    uint16_t               ego_bins; // size of ego_vel_histogram;
};

struct  RDC_MusicPeakData
{
    uint32_t                angle_index;    // Steering vector (angle) index of peak
    uint32_t                inv_magnitude;  // TODO: integer or fixed point?
};

//
// RPC Payload structures
//

struct RDC_RPC_Payload_Detections
{
    DEVPTR(RDC_ScanInstance_Impl) rdc_si;         // Not to be accessed by DSP
    DEVPTR(RDC_RDC3_raw_data)     rdc3;           // Input data:   sparsified RDC3 from HW (in DCU or DRAM)
    DEVPTR(RDC_ScanMgrParams)     scan_params;    // Parameters from RDC_ScanMgr
    DEVPTR(RDC_ScanInstParams)    inst_params;    // Parameters from RDC_ScanInstance_Impl
};

struct RDC_RPC_Payload_Sva                  // Only used for testing
{
    uint16_t               *data;           // Not to be accessed by DSP
    void                   *param;          // User defined parameters
    uint16_t                num_angles;     // Number of sva groups
    uint16_t                nyq_factor;     // SVA Nyquist factor
};

struct RDC_RPC_Payload_Virus
{
    uint16_t                num_loops;      // Number of virus loop
    uint16_t                nyq_factor;     // SVA Nyquist factor
};

struct RDC_RPC_Payload_Histograms
{
    DEVPTR(RDC_ScanInstance_Impl)   rdc_si;         // Not to be accessed by DSP
    DEVPTR(RHAL_Histogram)          histograms;     // Input data:  Array of histograms from HW (in DCU or DRAM)
    DEVPTR(uint16_t)                noise_floor;    // Output data: Array of noise floor estimates (one per Range Bin)
    uint16_t                        num_hist;       // Number of input histograms (up to number of range bins)
    int16_t                         exponent;       // Output exponent
    uint16_t                        cfar_rgbinsgroup; // noisefloor group number in range bins
    uint16_t                        nu;             //
    uint32_t                        cfar_pfa_log10; // cfar parameter.

    // ridge detection histogram
    DEVPTR(RDC_RDC3_raw_data)       rdc3;
    DEVPTR(int32_t)                 angle_ridge_thesh_db;
    DEVPTR(RDC_ScanMgrParams)       scan_params;    // Parameters from RDC_ScanMgr

    // ss
    DEVPTR(uint16_t)                static_slice;
    uint16_t                        ss_doppler_size;
};

struct RDC_RPC_Payload_MUSIC
{
    DEVPTR(RHAL_SvdInstance)    svdi;       // Not to be accessed by DSP
    DEVPTR(RDC_MusicPeakData)   peaks;      // Output array of detections (size = MAX_MUSIC_PEAKS)
    DEVPTR(RDC_MusicSampleData) msd;        // Pointer to RDC_MusicSampleData
    DEVPTR(cint32)              u_mat;      // SVD Left Unitary square matrix (U) (row major)
    DEVPTR(cint16)              stv;        // Pointer to matrix of steering vectors (VRx major) (each one is MAX_VRX long)
    DEVPTR(RDC_ScanMgrParams)   scan_params; // Parameters from RDC_ScanMgr
    DEVPTR(int32_t)             cosaz_cosel;
    DEVPTR(int32_t)             sinaz_cosel;
    DEVPTR(int32_t)             sinel;
    uint32_t                threshold;      // Minimum magnitude threshold for output peaks
    uint64_t                u_mask;         // For selecting invalid columns of U matrix (corresponding to signal subspace)
    uint8_t                 size;           // Size of U (number of VRx minus number of smoothing iterations)
    uint8_t                 Ns;             // Number of bits set in bit_mask (dimension of signal subspace)
    uint8_t                 enable_gradient; // Boolean to enable gradient search (rather than uniform search)
    uint8_t                 num_peaks;      // Number of output peaks
    uint16_t                num_stv;        // Number of steering vectors (number of angle bins) (each one is MAX_VRX long)
    uint16_t                music_index;    // Sequence number of this MUSIC request (Not to be accessed by DSP)
};

struct RDC_RPC_Payload_Ego_Velocity
{
    DEVPTR(RDC_ScanInstance_Impl) rdc_si;         // Not to be accessed by DSP
    DEVPTR(RDC_EgoVelocityData)   ego_vel_data;   // output histogram and top points list.
    DEVPTR(uint16_t)              st_slice;       // Static slice data from HW (input)
    DEVPTR(uint16_t)              combine_cosQ15; // cos(theta) * cos(phi) in Q15 format(input).
    DEVPTR(RDC_ScanMgrParams)     scan_params;    // Parameters from RDC_ScanMgr
    DEVPTR(RDC_ScanInstParams)    inst_params;    // Parameters from RDC_ScanInstance_Impl (for SVA and Adaptive Thresholding)
    uint16_t                      D_bins;         // Number of doppler bins in input
    int32_t                       in_ego_velQ15;  // estimate ego velocity for the center dopper of st_slice(input)
};

struct RDC_RPC_Payload_ClutterImage
{
    DEVPTR(RDC_ScanInstance_Impl) rdc_si;         // Not to be accessed by DSP
    DEVPTR(uint16_t)              st_slice;       // Static slice data from HW (input)
    //     #R * ( ( #D * #A * uint16 ) + RHAL_SSRsummary )
    DEVPTR(uint16_t)              cimage;   // Clutter image, 16-bit pixels (output), the height_dop_map followed,
                                            // 2:7:7 bits for flag, elevation and doppler, flag:1-3 express 1-3 peaks, 0 for 4 or more
    DEVPTR(uint16_t)              image_coeff;    // Interpolation Coeff for the output image.
    DEVPTR(RDC_ScanMgrParams)     scan_params;    // Parameters from RDC_ScanMgr
    DEVPTR(RDC_ScanInstParams)    inst_params;    // Parameters from RDC_ScanInstance_Impl (for SVA and Adaptive Thresholding)
    int16_t                 image_exponent; // exponent for the image (output)
    uint16_t                D_bins;         // Number of doppler bins in input
};

struct RDC_RPC_Payload_ClutterImageCoeff
{
    DEVPTR(RDC_ScanConfig_Impl)   rdc_sc;           // Not to be accessed by DSP
    DEVPTR(uint16_t)              clutter_coeff;  // Coefficients buffer for clutter image (output)
    DEVPTR(int16_t)               azimuth_angles; // Table of azimuth bin angles (radians, in Q14 format)
    DEVPTR(int16_t)               elev_angles;    // Table of elevation bin angles (radians, in Q14 format)
    DEVPTR(RDC_ScanMgrParams)     scan_params;    // Parameters from RDC_ScanMgr
};

enum RDC_ThresholdType
{
    THRESH_NONE,                            //!< no adaptive thresholding (only windowing, sva)
    THRESH_AT,                              //!< original adaptive thresholding (stdv, thresh, dynamic range)
    THRESH_SLR,                             //!< side-lobe and ridge suppression (JT)
};

//
// Other data structures
//

struct RDC_RPC_Payload_Zero_Doppler
{
    DEVPTR(RDC_ScanInstance_Impl)       rdc_si;

    // Output parameters
    DEVPTR(cint64)                      rdc2_zd;

    // Input parameters
    DEVPTR(cint16)                      rdc1;
    DEVPTR(uint16_t)                    rdc1_exponents;
    DEVPTR(int16_t)                     rev_rbin_map;

    uint32_t                            zd_total_size;

    uint32_t                            num_range;
    uint32_t                            num_pri;
    uint32_t                            num_vrx;
    uint32_t                            center;
    uint32_t                            half_width;

    int16_t                             ciu_exponent;
    int8_t                              zd_done;
    uint8_t                             rdc1_stride_en;
};

struct RDC_ScanMgrParams                    // This struct is a member of class RDC_ScanMgr : scan_mgr_params
{
    uint16_t                R_bins;         // Number of range bins in input
    uint16_t                pix_per_R_bin;  // Number of pixels in CI output image per input range bin (Fixed point: RBIN_PIXELS_FRAC_BITS)
    uint16_t                pix_x;          // Number of pixels in CI down-range (forward) direction
    uint16_t                pix_y;          // Number of pixels in CI cross-range (left-to-right) direction
    uint16_t                pix_z;          // Number of pixels in CI vertical (top-to-bottom) direction
    uint16_t                az_bins;        // Number of azimuth angle bins
    uint16_t                el_bins;        // Number of elevation angle bins
    uint16_t                az_nyq_factor;  // Azimuth Nyquist factor for SVA (0 disables SVA)
    uint16_t                el_nyq_factor;  // Elevation Nyquist factor for SVA (0 disables SVA)
    uint16_t                sva_second_pass_factor; // for dual sva
    uint16_t                az_per_voxel;   // Azimuth angle per CI voxel
    uint16_t                el_per_voxel;   // Elevation angle per CI voxel
    RDC_clutter_image_format ci_format;      // Format of clutter image output
    bool                    variable_power_mode; // flag for variable power mode
    bool                    detection_bypass_sva;       // bypass sva when perform detection
    RDC_ThresholdType       detection_thresh_type;      // select adaptive threshold for detection
    bool                    ego_vel_bypass_sva;         // bypass sva when perform ego velocity
    RDC_ThresholdType       ego_vel_thresh_type;        // select adaptive threshold for ego velocity
    bool                    suppress_radials;           // for adapt threshold SLR
    uint16_t                db2mag_table[768];          // magnitude table for each DB, precision 0.125f
    bool                    clutter_image_bypass_sva;   // bypass sva and adapt threshold when perform clutter image.
    RDC_ThresholdType       clutter_image_thresh_type;  // select adaptive threshold when perform clutter image.
    bool                    wrap_azimuth_detections;    // azimuth angles wrap around because of ambiguity
    bool                    wrap_elevation_detections;  // elevation angles wrap around because of ambiguity
    bool                    dsp_clutter_image_detecton; // using DSP to compute the clutter image detection.
    int16_t                 azimuth_anglesQ7_8[MAX_ROUGH_ANGLES]; // Table of azimuth bin angles (radians, in Q7.8 format)
    int16_t                 elev_anglesQ7_8[MAX_ROUGH_ANGLES];    // Table of elevation bin angles (radians, in Q7.8 format)
    int16_t                 rbin_map[MAX_RANGE_BINS];       // map for range bin:  distance_rb_order = rbin_map[RDC_rb_order]
    int16_t                 rev_rbin_map[MAX_RANGE_BINS];   // reverse range map:  RDC_rb_order = rev_rbin_map[distance_rb_order]
    FLOAT                   tx_boresight_phase[NUM_TX]; // For phased array, a vector of phase rotations to apply to the Tx (radians)
    FLOAT                   range_mag2rcs[MAX_RANGE_BINS];  // Offset (dB) from dBFS to RCS (dBsm) vs. range (boresight, zero Doppler)
    FLOAT                   angle_loss[MAX_ROUGH_ANGLES];   // Antenna pattern loss (in dB, negative), relative to boresight vs. AOA
    uint16_t                range_bin_start; // start range number
    FLOAT                   lambda;         // Wavelength for current scan, in meters
    vec3f_t                *vrx_pos;        // [MAX_VRX]  Position (in meters) of each Virtual Receiver (Rx-major)
    int8_t                 *vrx_map;        // [MAX_VRX]  Map from Rx-major index to Y-major
    int8_t                 *rev_map;        // [MAX_VRX]  Map from Y-major index to Rx-major
    uint8_t                 active_vrx[MAX_VRX / 8];  // Bitmap of enabled VRx, in Y-major VRx order
    int32_t                 tx_rx_pos[MAX_VRX * 6]; // Q7.24
    bool                    use_dsp_stv;

    uint16_t                num_range_bins;             // TODO: deprecate, redundant with R_bins?
    INT                     az_uniform_vrx;             // Number of VRx in azimuth in fully filled VRx array
    INT                     el_uniform_vrx;             // Number of VRx in elevation in fully filled VRx array
    FLOAT                   azimuth_angles[MAX_ROUGH_ANGLES];       // Azimuth angles to beamform (in Radians)
    FLOAT                   elev_angles[MAX_ROUGH_ANGLES];          // Elevation angles to beamform (in Radians)
    int16_t                 azimuth_angles_Q14[MAX_ROUGH_ANGLES];   // Azimuth angles, in Q14 format. -PI/2 to PI/2
    int16_t                 elev_angles_Q14[MAX_ROUGH_ANGLES];      // Elevation angles, in Q14 format. -PI/2 to PI/2
    uint16_t               *clutter_image_coeff;         // [CLUTTER_IMAGE_SIZE * CLUTTER_IMAGE_SIZE * 7]   Coeff for the image

    bool                    use_sinc_interp;
};

struct  RDC_ScanInstParams                  // This struct is a member of class RDC_ScanInstance_Impl : scan_inst_params
{
    // Noise floor
    uint16_t                noise_floor[MAX_RANGE_BINS];    // Noise floor calculated from histograms,
                                                            // In dB, UQ.NOISE_FLOOR_FRAC_BITS
    uint32_t                noise_floor_linear[MAX_RANGE_BINS]; // Noise floor calculated from histograms,
                                                                // Converted to linear (Q32.0)
    FLOAT                   noise_floor_max[MAX_RANGE_BINS]; // Peak of the power histogram, interpolated (in dB, with HW exponent applied)

    // Detections
    uint32_t                static_detection_threshold[MAX_RANGE_BINS]; // In linear units, with exponent 'thresold_exponent', indexed by HW RB index
    uint32_t                moving_detection_threshold[MAX_RANGE_BINS]; // In linear units, with exponent 'thresold_exponent', indexed by HW RB index
    uint16_t                ss_zero_bin[MAX_ROUGH_ANGLES];  // Doppler bin index of the 0-velocity bin at time of "proc"

    int16_t                 threshold_exponent;             // implied exponent of detection_threshold[] values
    int16_t                 mag32_exponent;                 // Global exponent for uint32_t magnitudes (0..6)
    uint16_t                detection_doppler_combine;      // combine the detection when the distance of doppler
                                                            // is smaller than it and the other direction is smaller than 1.
    // Dynamic Adaptive Thresholding
    uint16_t                moving_nstdv;                   // Q7.8
    uint16_t                moving_thres_margin;            // Q8.8 (linear)
    uint32_t                moving_dynamic_range;           // Q24.8 (linear)

    // Static (CI) Adaptive Thresholding
    uint16_t                static_nstdv;                   // Q7.8
    uint16_t                static_thres_margin;            // Q8.8 (linear)
    uint32_t                static_dynamic_range;           // Q24.8 (linear)

    uint16_t                slr_static_base_offset;         // linear  Q8.8
    uint16_t                slr_moving_base_offset;         // linear  Q8.8
    uint16_t                notch_width_rad;                // radians Q8.8
    uint16_t                notch_depth;                    // linear  Q8.8
    uint16_t                outer_depth;                    // linear  Q8.8
    uint16_t                ridge_threshold;                // linear  Q8.8

    int32_t                 angle_noise_floorQ8[MAX_ROUGH_ANGLES]; // point to RDCScanInstance angle noise floor. 8 bit frac
    RDC_DetectionData      *detections;                     // pointer to detection list for current scan
    uint16_t                num_static_detections;          // number of static detections.
    uint16_t                num_moving_detections;          // number of moving detections.

    uint16_t                reserved;
};

PACK(
    struct RDC_DSP_Config
{
    // Histogram processing
    uint16_t                num_hist;       // Number of input histograms (up to number of range bins)
    uint16_t                cfar_rgbinsgroup; // noisefloor group number in range bins
    uint16_t                nu;             //
    uint32_t                cfar_pfa_log10; // cfar parameter.

    // RDC3 detections
    uint16_t                num_range_bin;          // Number of range bins
    uint16_t                num_doppler_bin;        // Number of range bins
    uint16_t                num_angle_bin;          // Number of angle bins (Azimuth & Elevation combined)
    uint16_t                num_azimuth_bin;        // Number of angle bins in Azimuth dimension
    uint16_t                num_elevation_bin;      // Number of angle bins in Elevation dimension

    // Raw RDC3 data from RHAL
    uint16_t                rd_num_total;
    uint32_t                rd_num[NUM_RD_BUF];

    uint16_t                num_splits;                                 // Number of groups of RD bins

    enum { MAX_SEGMENTS_PER_SPLIT = 64 };
    uint16_t                num_segments[MAX_RD_SPLITS];
    uint16_t                refilter_ID[MAX_RD_SPLITS][MAX_SEGMENTS_PER_SPLIT];
    uint16_t                refilter_segment_len[MAX_RD_SPLITS][MAX_SEGMENTS_PER_SPLIT]; // number of activations in the segment
    uint16_t                rd_split_start[NUM_RD_BUF][MAX_RD_SPLITS];  // Starting RD bin index of each group
    uint16_t                rd_split_length[NUM_RD_BUF][MAX_RD_SPLITS]; // Length of each group (# RD bins)

    // scan Instant parameters
    // Noise floor
    uint16_t                noise_floor[MAX_RANGE_BINS];    // Noise floor calculated from histograms,
                                                            // In dB, UQ.NOISE_FLOOR_FRAC_BITS
    uint32_t                noise_floor_linear[MAX_RANGE_BINS]; // Noise floor calculated from histograms,
                                                                // Converted to linear (Q32.0)
    FLOAT                   noise_floor_max[MAX_RANGE_BINS]; // Peak of the power histogram (in dB, with HW exponent applied)
                                                             // TODO: interpolate

    // Detections
    uint32_t                static_detection_threshold[MAX_RANGE_BINS]; // In linear units, with exponent 'thresold_exponent'
    uint32_t                moving_detection_threshold[MAX_RANGE_BINS]; // In linear units, with exponent 'thresold_exponent'
    int16_t                 threshold_exponent;             // implied exponent of detection_threshold[] values
    int16_t                 mag32_exponent;                 // Global exponent for uint32_t magnitudes (0..6)
    uint16_t                detection_doppler_combine;      // combine the detection when the distance of doppler
                                                            // is smaller than it and the other direction is smaller than 1.
                                                            // Dynamic Adaptive Thresholding
    uint16_t                moving_nstdv;                   // Q7.8
    uint16_t                moving_thres_margin;            // Q8.8 (linear)
    uint32_t                moving_dynamic_range;           // Q24.8 (linear)

    // Static (CI) Adaptive Thresholding
    uint16_t                static_nstdv;                   // Q7.8
    uint16_t                static_thres_margin;            // Q8.8 (linear)
    uint32_t                static_dynamic_range;           // Q24.8 (linear)

    uint16_t                slr_static_base_offset;         // linear  Q8.8
    uint16_t                slr_moving_base_offset;         // linear  Q8.8
    uint16_t                notch_width_rad;                // radians Q8.8
    uint16_t                notch_depth;                    // linear  Q8.8
    uint16_t                outer_depth;                    // linear  Q8.8
    uint16_t                ridge_threshold;                // linear  Q8.8
    int32_t                 angle_noise_floorQ8[MAX_ROUGH_ANGLES]; // noise floor for each angles.
    uint16_t                ss_zero_bin[MAX_RANGE_BINS];

    // scan config
    uint16_t                R_bins;         // Number of range bins in input
    uint16_t                pix_per_R_bin;  // Number of pixels in CI output image per input range bin (Fixed point: RBIN_PIXELS_FRAC_BITS)
    uint16_t                pix_x;          // Number of pixels in CI down-range (forward) direction
    uint16_t                pix_y;          // Number of pixels in CI cross-range (left-to-right) direction
    uint16_t                pix_z;          // Number of pixels in CI vertical (top-to-bottom) direction
    uint16_t                az_bins;        // Number of azimuth angle bins
    uint16_t                el_bins;        // Number of elevation angle bins
    uint16_t                az_nyq_factor;  // Azimuth Nyquist factor for SVA (0 disables SVA)
    uint16_t                el_nyq_factor;  // Elevation Nyquist factor for SVA (0 disables SVA)
    uint16_t                sva_second_pass_factor;
    uint16_t                az_per_voxel;   // Azimuth angle per CI voxel
    uint16_t                el_per_voxel;   // Elevation angle per CI voxel
    RDC_clutter_image_format ci_format;      // Format of clutter image output
    bool                    detection_bypass_sva;       // bypass sva when perform detection
    RDC_ThresholdType       detection_thresh_type;      // select adaptive threshold for detection
    bool                    ego_vel_bypass_sva;         // bypass sva when perform ego velocity
    RDC_ThresholdType       ego_vel_thresh_type;        // select adaptive threshold for ego velocity
    bool                    suppress_radials;           // for adapt threshold SLR
    uint16_t                db2mag_table[768];          // magnitude table for each DB, precision 0.125f
    bool                    clutter_image_bypass_sva;   // bypass sva and adapt threshold when perform clutter image.
    RDC_ThresholdType       clutter_image_thresh_type;  // select adaptive threshold when perform clutter image.
    bool                    wrap_azimuth_detections;    // azimuth angles wrap around because of ambiguity
    bool                    wrap_elevation_detections;  // elevation angles wrap around because of ambiguity
    bool                    dsp_clutter_image_detecton; // using DSP to compute the clutter image detection.
    int16_t                 azimuth_anglesQ7_8[MAX_ROUGH_ANGLES]; // Table of azimuth bin angles (radians, in Q7.8 format)
    int16_t                 elev_anglesQ7_8[MAX_ROUGH_ANGLES];    // Table of elevation bin angles (radians, in Q7.8 format)
    int16_t                 rbin_map[MAX_RANGE_BINS];   // map for range bin - distance_rb_order = rev_rbin_map[RDC_rb_order]
    int16_t                 rev_rbin_map[MAX_RANGE_BINS];   // Reverse range bin map - RDC_rb_order = rev_rbin_map[distance_rb_order]
    uint16_t                range_bin_start; // start range number

    bool                    use_sinc_interp;

    // static slice processing
    uint16_t                ss_d_bins;         // Number of doppler bins in input
    uint16_t                combine_cosQ15[MAX_ROUGH_ANGLES];
    int32_t                 in_ego_velQ15;
    uint32_t                sequence; // sequence number
});

struct MUSIC_DSP_Config
{
    // music
    RDC_MusicSampleData     msd;
    cint16                  stv[MAX_VRX][NUM_MUSIC_1D_SAMPLES];
    uint32_t                music_threshold;      // Minimum magnitude threshold for output peaks
    uint64_t                u_mask;         // For selecting invalid columns of U matrix (corresponding to signal subspace)
    uint8_t                 size;           // Size of U (number of VRx minus number of smoothing iterations)
    uint8_t                 Ns;             // Number of bits set in bit_mask (dimension of signal subspace)
    uint8_t                 enable_gradient; // Boolean to enable gradient search (rather than uniform search)
    uint8_t                 num_peaks;      // Number of output peaks
    uint16_t                num_stv;        // Number of steering vectors (number of angle bins) (each one is MAX_VRX long)

    // scan config
    uint16_t                music_index; // 4 bytes align
    FLOAT                   lambda;         // Wavelength for current scan, in meters
    vec3f_t                 vrx_pos[MAX_VRX];        // [MAX_VRX]  Position (in meters) of each Virtual Receiver (Rx-major)
    int8_t                  vrx_map[MAX_VRX];        // [MAX_VRX]  Map from Rx-major index to Y-major
    int8_t                  rev_map[MAX_VRX];        // [MAX_VRX]  Map from Y-major index to Rx-major
    uint8_t                 active_vrx[MAX_VRX / 8];  // Bitmap of enabled VRx, in Y-major VRx order

    uint32_t                music_peak_threshold;
    FLOAT                   music_ns_thresh_dB;

    int32_t                 cosaz_cosel[NUM_MUSIC_1D_SAMPLES];
    int32_t                 sinaz_cosel[NUM_MUSIC_1D_SAMPLES];
    int32_t                 sinel[NUM_MUSIC_1D_SAMPLES];
    int32_t                 tx_rx_pos[MAX_VRX * 6]; // Q7.24
    bool                    use_dsp_stv;
    uint32_t                sequence; // sequence number
};

struct RDC_SpecialCaptureData
{
    bool               rdc2_zd_enable;
    int16_t            rdc2_zd_rb_center;
    int16_t            rdc2_zd_rb_halfwidth;

    RHAL_ADC_CaptureMode adc_capture_mode;
    uint8_t            adc_capture_rx_num;
    uint32_t           adc_capture_size;

    // ADC (QDU) playback data
    int8_t            *adc_playback_data;          // ADC data for QDU playback: [#Rx][N][Lc][IQ](signed 8 bits)
    uint32_t           adc_playback_len;           // Length in bytes
    uint32_t          *prn_playback_data;          // PRN data for QDU playback: [#Tx][N][Lc](packed bits)
    uint32_t           prn_playback_len;           // Length in bytes
    bool               repeat_same_pulse;          // For adc_playback_mode, repeat first pulse data for all pulses
    RHAL_DummyScanMode dummy_scan_mode;            // For adc_playback_mode, use synthetic (known) input data for adc playback

    void set_defaults()
    {
        rdc2_zd_enable = false;
        rdc2_zd_rb_center = 0U;
        rdc2_zd_rb_halfwidth = 0U;

        adc_capture_mode = NO_ADC_CAPTURE;
        adc_capture_rx_num = 0U;
        adc_capture_size = 0U;

        adc_playback_len = 0U;
        prn_playback_len = 0U;
        adc_playback_data = NULL;
        prn_playback_data = NULL;
        repeat_same_pulse = false;
        dummy_scan_mode = NO_DUMMY_SCAN;
    }
};


SRS_CLOSE_NAMESPACE()

#endif // SRS_HDR_RDC_INTERNALS_H
