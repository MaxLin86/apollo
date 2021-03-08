#ifndef SRS_HDR_UHDP_MSG_STRUCTS_H
#define SRS_HDR_UHDP_MSG_STRUCTS_H 1
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
#include "modules/drivers/radar/uhnder_radar/driver/system-radar-software/engine/common/eng-api/rdc-structs.h" // DetectionData

SRS_DECLARE_NAMESPACE()

/*! UHDP message header - all remaining message structs presume this header will
 * be prepended */
struct UhdpHeader
{
    uint8_t  version;
    uint8_t  message_type;
    uint16_t total_length;
};

enum { INITIAL_PACKET_WINDOW = 8 };

/*! UHDP message types */
enum UhdpMessageTypeEnum
{
    UHDP_TYPE_HELLO            = 0,   //! to radar
    UHDP_TYPE_CAPTURE_CONTROL  = 1,   //! to radar
    UHDP_TYPE_DISCOVERY        = 2,   //! to data logger (discovery)
    UHDP_TYPE_SCAN_INFORMATION = 3,   //! to data logger
    UHDP_TYPE_SCAN_READY       = 4,   //! to radar
    UHDP_TYPE_PRINT            = 5,   //! to data logger (deprecated)
    UHDP_TYPE_SCAN_CONTROL     = 6,   //! to radar
    UHDP_TYPE_PPA              = 7,   //! to data logger
    UHDP_TYPE_CONTROL_ACK      = 8,   //! to data logger
    UHDP_TYPE_CONN_CLOSE       = 9,   //! to radar

    UHDP_TYPE_ADC              = 10,  //! 0A to data logger
    UHDP_TYPE_RDC1             = 11,  //! 0B to data logger
    UHDP_TYPE_RDC2             = 12,  //! 0C to data logger
    UHDP_TYPE_RDC3             = 13,  //! 0D to data logger
    UHDP_TYPE_STATIC_SLICE     = 14,  //! 0E to data logger
    UHDP_TYPE_STATIC_SLICE_BIN = 15,  //! 0F to data logger
    UHDP_TYPE_HISTOGRAM        = 16,  //! 10 to data logger
    UHDP_TYPE_REG_CORE_DUMP    = 17,  //! 11 to data logger
    UHDP_TYPE_COVARIANCE       = 18,  //! 12 to data logger
    UHDP_TYPE_SVD              = 19,  //! 13 to data logger
    UHDP_TYPE_DETECTIONS       = 20,  //! 14 to data logger
    UHDP_TYPE_CLUTTER_IMAGE    = 21,  //! 15 to data logger
    UHDP_TYPE_MUSIC_SAMPLES    = 22,  //! 16 to data logger
    UHDP_TYPE_ZERO_DOPPLER     = 23,  //! 17 to data logger
    UHDP_TYPE_RANGE_BINS       = 24,  //! 18 to data logger
    UHDP_TYPE_ANGLE_BINS       = 25,  //! 19 to data logger
    UHDP_TYPE_ENV_SCAN_DATA    = 26,  //! 1A to data logger
    UHDP_TYPE_POINT_CLOUD      = 27,  //! 1B to data logger

    UHDP_TYPE_REQUEST_FLUSH    = 33,  //! 21 to radar
    UHDP_TYPE_CLOSE_ACK        = 34,  //! 22 to data logger
    UHDP_TYPE_FLUSH            = 35,  //! 23 to data logger

    UHDP_TYPE_DIAG_DISCOVERY   = 36,  //! 24 to data logger (discovery)
    UHDP_TYPE_DIAG_REQUEST     = 37,  //! 25 to radar
    UHDP_TYPE_DIAG_COMPLETE    = 38,  //! 26 to data logger
    UHDP_TYPE_DIAG_FORCE_STOP  = 39,  //! 27 to radar
    UHDP_TYPE_LOG_PRODUCER_NAME = 40, //! 28 to data logger (discovery)
    UHDP_TYPE_ANTENNA_CONFIG   = 41,  //! 29 to data logger (discovery)

    UHDP_TYPE_LOG_MESSAGE      = 42,  //! 2A to data logger (discovery)
    UHDP_TYPE_LOG_TASK_NAME    = 43,  //! 2B to data logger (discovery)
    UHDP_TYPE_DATA_WINDOW      = 44,  //! 2C to radar
    UHDP_TYPE_STATE_QUERY      = 45,  //! 2D to radar
    UHDP_TYPE_STATE_RESPONSE   = 46,  //! 2E to data logger
    UHDP_TYPE_STATE_PEEK       = 47,  //! 2F bidirectional request/response
    UHDP_TYPE_STATE_POKE       = 48,  //! 30 bidirectional request/response
    UHDP_TYPE_TIME_ALIGN       = 49,  //! 31 bidirectional request/response

    UHDP_TYPE_LOG_AGGR         = 50,  //! 32 New aggregated log/uhprintf format
    UHDP_TYPE_LOG_AGGR_ACK     = 51,  //! 33 ack for each LOG_AGGR message
    UHDP_TYPE_TELEMETRY_DATA   = 52,  //! 34 to radar from data logger
    UHDP_TYPE_MUSIC_CONTROL    = 53,  //! 35 to radar from data logger
    UHDP_TYPE_THRESHOLD        = 54,  //! 36 to radar from data logger

    UHDP_TYPE_BULK_UPLOAD      = 55,  //! 37 to radar from data logger
    UHDP_TYPE_BULK_UPLOAD_ACK  = 56,  //! 38 to data logger from radar
    UHDP_TYPE_LOG_CONTROL      = 57,  //! 39 to radar from data logger
    UHDP_TYPE_LLD_CORE_DUMP    = 58,  //! 3A bidirectional request/response

    UHDP_TYPE_CAN_MSG_RX       = 80,  //! 50 bidirectional request/response
    UHDP_TYPE_CAN_MSG_TX       = 81,  //! 51 bidirectional request/response

    UHDP_TYPE_ENV_CONTROL_DATA = 90,  //! 5A bidirectional environment data
    UHDP_TYPE_ENV_CONTROL_ACK  = 91,  //! 5B bidirectional environment data

    UHDP_TYPE_APPLICATION_DATA = 255, //! FF bidirectional environment data
};

/* Description of non-radar data message types:
 *
 * UHDP_TYPE_PRINT             - uint32_t cpuid, followed by NUL separated printf-like strings
 * UHDP_TYPE_LOG               - uint32_t cpuid, followed by a number of 64byte structs
 * UHDP_TYPE_LOG_PRODUCER_NAME - log producer name, followed by NUL separated subunits
 * UHDP_TYPE_LOG_MESSAGE       - NUL separated log message strings
 * UHDP_TYPE_LOG_TASK_NAME     - NUL separates profile task names
 * UHDP_TYPE_DIAG_DISCOVERY    -
 *     uint16_t class_total_len
 *       uint8_t class_name_str_len
 *       CHAR    class_name[class_name_str_len];
 *       {
 *          uint8_t test_api_version;
 *          uint8_t test_name_str_len;
 *          CHAR    test_name[test_name_str_len];
 *       } repeat until class_total_len is filled
 *
 * UHDP_TYPE_LOG_AGGR          - uint32_t sequence_number, followed by series of
 *                             - uint8_t cpuid, uint8_t len, message string of length len
 *                             - or
 *                             - uint8_t cpuid, 0xFFU, struct LogEvent
 * UHDP_TYPE_LOG_AGGR_ACK      - uint32_t sequence_number
 *
 * UHDP_TYPE_CONN_CLOSE        - no content
 * UHDP_TYPE_FLUSH             - no content
 * UHDP_TYPE_CLOSE_ACK         - no content
 * UHDP_TYPE_STATE_QUERY       - no content
 *
 * UHDP_TYPE_STATE_RESPONSE    - 32bit bitmask
 *
 * UHDP_TYPE_MUSIC_CONTROL     - RDC_music_request
 * UHDP_TYPE_EGO_VELOCITY      - EgoVelocityData
 * UHDP_TYPE_DETECTIONS        - DetectionData
 *
 * UHDP_TYPE_CAN_MSG_RX       - undefined
 * UHDP_TYPE_CAN_MSG_TX       - undefined
 * UHDP_TYPE_APPLICATION_DATA - undefined
 */

PACK(
    struct UhdpHelloMessage /* UHDP_TYPE_HELLO */
{
    uint32_t command_sequence_number;
    uint32_t logger_ip_address;         // in host byte order (not net order)
    uint32_t radar_ip_address;          // unused
    uint8_t  next_hop_ethernet_address[6]; // unused
    uint16_t logger_UDP_port;           // in host byte order (not net order)
    uint8_t  radar_ethernet_address[6]; // unused
});

struct UhdpDiscovery /* UHDP_TYPE_DISCOVERY */
{
    uint32_t command_sequence_number;
    uint32_t discovery_state;
    uint64_t timestamp;

    /* followed by variable length string containing SRS version info */
};

/* Mode tells the radar what logging mode to operate in */
enum DataLogModeEnum
{
    DL_NORMAL     = 0,
    DL_INCIDENT   = 1,
    DL_IDLE       = 2,
};

/* mask for data log enabling/disabling */
enum DataLogMaskEnum
{
    DL_ADC            = 1 << 0,  // ADC
    DL_RDC1           = 1 << 1,  // RDC1
    DL_RDC2           = 1 << 2,  // RDC2
    DL_RDC3           = 1 << 3,  // RDC3
    DL_SS             = 1 << 4,  // Static Slice
    DL_DET            = 1 << 5,  // Detections and/or Point-Cloud
    DL_CI             = 1 << 6,  // Clutter Image
    DL_CAN_RX         = 1 << 7,  // Can Message Rx
    DL_CAN_TX         = 1 << 8,  // Can Message Tx
    DL_APP_DATA       = 1 << 9,  // Application Data
    DL_SCAN_CORE_DUMP = 1 << 10, // Register dumps from digital front end
    DL_PROC_CORE_DUMP = 1 << 11, // Register dumps from digital back end
    DL_COV            = 1 << 12, // Outputs of spatial smoothing and covariance (debug)
    DL_SVD            = 1 << 13, // Outputs of singular value decomposition (debug)
    DL_MUSIC          = 1 << 14, // MUSIC angle sample magnitudes
    DL_ZERO_DOPPLER   = 1 << 15, // RDC2 Zero Doppler (derived from RDC1 coherent sum)
    DL_HISTOGRAM      = 1 << 16, // Per-Range Histograms
};

struct UhdpCaptureControl /* UHDP_TYPE_CAPTURE_CONTROL */
{
    uint32_t   command_sequence_number; //!< will be returned by UHDP_TYPE_CONTROL_ACK
    uint32_t   capture_mode;            //!< enum DataLogModeEnum

    //! (capture_mode == DL_NORMAL), then the below fields specify which radar
    // data outputs are sent to the Data logger, and their frequency. Note that
    // in incident mode (capture_mode == DL_INCIDENT), enable_mask=DL_RDC1 and
    // rdc1_interval=0 are implied
    uint32_t   enable_mask;

    uint8_t    rdc1_interval;           //!< scan_seq_num % (rdc1_interval + 1)
    uint8_t    rdc23ss_interval;        //!< scan_seq_num % (rdc23ss_interval + 1)
    uint8_t    det_ci_interval;         //!< scan_seq_num % (det_ci_interval + 1)
    uint8_t    ss_doppler_0_only;       //!< only xmit the 0-doppler layer of static slice

    // if (enable_mask & DL_RDC1) and (capture_mode == DL_NORMAL), the
    // following fields specify which portions of RDC1 are returned to the Data
    // Logger
    uint16_t   rdc1_first_pulse;
    uint16_t   rdc1_last_pulse;

    // if (enable_mask & DL_ZERO_DOPPLER) and (capture_mode == DL_NORMAL), the
    // following fields specify which range bins of the RDC2 zero-doppler plane
    // are returned to the Data Logger
    int16_t    rdc2_zd_rb_center;
    int16_t    rdc2_zd_rb_halfwidth;

    // if (enable_mask & DL_ADC) and (capture_mode == DL_NORMAL), the following
    // fields specify which ADC samples are captured, and how many
    uint16_t   adc_capture_mode;        //!< data spray(1) or raw(2)
    uint16_t   adc_capture_rx_num;      //!< physical receiver to capture
    uint32_t   adc_capture_size;        //!< number of samples to capture per scan
};

extern const CHAR* threshold_preset_descriptions[];
enum ThresholdPresetEnum
{
    LOW_SENSITIVITY,  //!< indoors
    MID_SENSITIVITY,  //!< parking lot
    HIGH_SENSITIVITY, //!< driving

    NUM_THRESHOLD_PRESETS
};

struct UhdpThresholdControl /* UHDP_TYPE_THRESHOLD */
{
    // Detection thresholds - the final signal to noise thresholds that a target
    // must eclipse in order to trigger a detection

    FLOAT      static_1D_thresh_snr_dB; //!< detection threshold for static slice data in 1D scans
    FLOAT      moving_1D_thresh_snr_dB; //!< detection threshold for activation data in 1D scans

    FLOAT      static_2D_thresh_snr_dB; //!< detection threshold for static slice data in 2D scans
    FLOAT      moving_2D_thresh_snr_dB; //!< detection threshold for activation data in 2D scans

    //! the distance in X meters from the radar at which the detection threshold
    //! scaling begins. Beyond this range, the detetection threshold is flat. Typical value is 25m
    FLOAT      scale_det_thresh_max_range;

    //! the maximum adjustment in detection threshold (in SNR dB) directly in
    //! front of the radar. This detection threshold adjustment starts as
    //! scale_det_thresh_snr_adj dB at the radar and reduces linearly to 0 at
    //! scale_det_thresh_max_range meters. Typical value is 10 dB
    FLOAT      scale_det_thresh_snr_adj;

    // Side-lobe removal (SLR) - we define a baseline threshold level (as SNR
    // dB) below which all samples are considered noise.  Large targets will
    // raise this noise threshold for all the other samples in the same
    // activation, based on a simple notch curve.

    FLOAT      base_static_snr_dB;      //!< base static threshold SNR, all below this threshold is noise (as dB)
    FLOAT      base_moving_snr_dB;      //!< base moving threshold SNR, all below this threshold is noise (as dB)
    FLOAT      notch_width_radians;     //!< the azimuth delta angle at which larger side-lobes appear (radians)
    FLOAT      notch_depth_dB;          //!< the amount of clean SNR from target peak at near-angles (as dB)
    FLOAT      outer_depth_dB;          //!< the amount of clean SNR from target peak at off-angles (as dB)
    FLOAT      ridge_threshold_dB;      //!< minimum SNR above an angle ridge to allow a detection

    FLOAT      phased_array_azimuth;    //!< phased array transmitter array steering azimuth angle (radians)

    void defaults()
    {
        static_1D_thresh_snr_dB    = 15.0F;
        moving_1D_thresh_snr_dB    = 16.0F;
        static_2D_thresh_snr_dB    = 16.0F;
        moving_2D_thresh_snr_dB    = 17.0F;
        scale_det_thresh_max_range = 15.0F;
        scale_det_thresh_snr_adj   = 4.0F;

        notch_width_radians        = 0.47123889803846897f; // 27 degrees
        base_static_snr_dB         = 16.0F;
        base_moving_snr_dB         = 16.0F;
        notch_depth_dB             = 18.0F;
        outer_depth_dB             = 15.0F;
        ridge_threshold_dB         = 9.0F;

        phased_array_azimuth       = 0.0F; // boresight
    }


    void disable_slr()
    {
        base_static_snr_dB = 0.0F;
        base_moving_snr_dB = 0.0F;
        notch_depth_dB     = 0.0F;
        outer_depth_dB     = 0.0F;
        ridge_threshold_dB = 0.0F;
    }

    bool is_slr_enabled()
    {
        return (base_static_snr_dB != 0.0F) ||
               (base_moving_snr_dB != 0.0F) ||
               (notch_depth_dB     != 0.0F) ||
               (outer_depth_dB     != 0.0F) ||
               (ridge_threshold_dB != 0.0F);
    }

    void apply_slr_preset(ThresholdPresetEnum p)
    {
        switch (p)
        {
        case LOW_SENSITIVITY:
            base_static_snr_dB = 12.0F;
            base_moving_snr_dB = 12.0F;
            notch_depth_dB = 10.0F;
            outer_depth_dB = 7.0F;
            ridge_threshold_dB = 9.0F;
            break;

        case MID_SENSITIVITY:
        case HIGH_SENSITIVITY:
            base_static_snr_dB = 6.0F;
            base_moving_snr_dB = 6.0F;
            notch_depth_dB = 18.0F;
            outer_depth_dB = 15.0F;
            ridge_threshold_dB = 5.0F;
            break;

        default:
            break;
        }
    }


    void apply_detection_threshold_preset(ThresholdPresetEnum p)
    {
        switch (p)
        {
        case LOW_SENSITIVITY:
            static_1D_thresh_snr_dB = 18.0F;
            moving_1D_thresh_snr_dB = 19.0F;
            static_2D_thresh_snr_dB = 19.0F;
            moving_2D_thresh_snr_dB = 20.0F;
            scale_det_thresh_max_range = 25.0F;
            scale_det_thresh_snr_adj   = 9.0F;
            break;

        case MID_SENSITIVITY:
        case HIGH_SENSITIVITY:
            static_1D_thresh_snr_dB = 15.0F;
            moving_1D_thresh_snr_dB = 16.0F;
            static_2D_thresh_snr_dB = 16.0F;
            moving_2D_thresh_snr_dB = 17.0F;
            scale_det_thresh_max_range = 15.0F;
            scale_det_thresh_snr_adj   = 4.0F;
            break;

        default:
            break;
        }
    }


    void apply_preset(ThresholdPresetEnum p)
    {
        defaults();
        apply_slr_preset(p);
        apply_detection_threshold_preset(p);
    }
};

// The configuration for adaptive thresholding; there can be multiple such
// configurations for different scan situations or static vs moving
struct RDC_AdaptiveThreshold
{
    FLOAT  adaptive_threshold_num_stdv;
    FLOAT  adaptive_threshold_margin_dB;
    FLOAT  adaptive_threshold_dynamic_range_dB;

    void apply_preset(ThresholdPresetEnum p, bool moving, bool twoD)
    {
        adaptive_threshold_dynamic_range_dB = 35.0F;
        switch (p)
        {
        case LOW_SENSITIVITY:
            if (twoD)
            {
                if (moving)
                {
                    adaptive_threshold_num_stdv = 4.5F;
                    adaptive_threshold_margin_dB = 4.5F;
                }
                else
                {
                    adaptive_threshold_num_stdv = 7.0F;
                    adaptive_threshold_margin_dB = 7.0F;
                }
            }
            else
            {
                if (moving)
                {
                    adaptive_threshold_num_stdv = 4.0F;
                    adaptive_threshold_margin_dB = 4.0F;
                }
                else
                {
                    adaptive_threshold_num_stdv = 6.0F;
                    adaptive_threshold_margin_dB = 6.0F;
                }
            }
            break;
        case MID_SENSITIVITY:
            if (twoD)
            {
                if (moving)
                {
                    adaptive_threshold_num_stdv = 3.0F;
                    adaptive_threshold_margin_dB = 3.0F;
                }
                else
                {
                    adaptive_threshold_num_stdv = 3.4F;
                    adaptive_threshold_margin_dB = 3.4F;
                }
            }
            else
            {
                if (moving)
                {
                    adaptive_threshold_num_stdv = 3.0F;
                    adaptive_threshold_margin_dB = 3.0F;
                }
                else
                {
                    adaptive_threshold_num_stdv = 3.5F;
                    adaptive_threshold_margin_dB = 3.8F;
                }
            }
            break;
        case HIGH_SENSITIVITY:
            if (twoD)
            {
                if (moving)
                {
                    adaptive_threshold_num_stdv = 3.0F;
                    adaptive_threshold_margin_dB = 3.0F;
                }
                else
                {
                    adaptive_threshold_num_stdv = 2.5F;
                    adaptive_threshold_margin_dB = 2.5F;
                }
            }
            else
            {
                if (moving)
                {
                    adaptive_threshold_num_stdv = 2.5F;
                    adaptive_threshold_margin_dB = 2.5F;
                }
                else
                {
                    adaptive_threshold_num_stdv = 2.5F;
                    adaptive_threshold_margin_dB = 2.5F;
                }
            }
            break;
        default:
            break;
        }
    }
};

/* Command tells the Radar to start or stop scanning */
enum DataLogCommandEnum
{
    DL_STOP     = 0,    // Command to tell Radar to stop scanning
    DL_START    = 1,    // Command to tell Radar to start scanning
};

struct UhdpControlAck      /* UHDP_TYPE_CONTROL_ACK */
{
    uint32_t   command_sequence_number;
};

struct UhdpRangeBinInfo    /* UHDP_TYPE_RANGE_BINS */
{
    int16_t  distance_in_bins;        //!< distance of range bin from radar, in units of range_bin_width - negative means invalid
    int16_t  reverse_map;             //!< map distance to range bin (indexed by distance in bins) - negative means not represented
    FLOAT    noise_floor_max_peak_dB; //!< the max peak of the power histogram in this range
    FLOAT    noise_floor_pfa_dB;      //!< the knee of the noise curve, identified by PFA
};

struct UhdpTelemetryData   /* UHDP_TYPE_TELEMETRY_DATA */
{
    vec3f_t  ego_velocity;             //!< meters per second of world relative to radar
    vec3f_t  ego_acceleration;         //!< meters per second per second of world relative to radar
    FLOAT    yaw_rate;                 //!< radians per second
    FLOAT    steering_turn_angle;      //!< radians, 0 at boresight
    FLOAT    braking_pressure;         //!< bars
    FLOAT    external_temp_C;          //!< celcius
    uint32_t telemetry_age_us;         //!< age of telemetry data, in microseconds
    uint32_t future_use_0;             // padding to even multiple of vec3f_t
};

struct UhdpScanInformation /* UHDP_TYPE_SCAN_INFORMATION */
{
    uint64_t   rdc1_full_scale_value;
    uint64_t   rdc2_full_scale_value;
    uint64_t   rdc2ch_full_scale_value;
    uint64_t   rdc3_full_scale_value;

    uint32_t   scan_sequence_number;    // UHDP sequence number
    uint32_t   scan_ID_number;          // RHAL sequence number
    uint32_t   scan_timestamp;          // radar time at scan end
    uint32_t   current_time;            // radar time at info message send

    FLOAT      estimated_ego_velocity_X;
    FLOAT      estimated_ego_velocity_Y;
    FLOAT      estimated_ego_velocity_Z;
    FLOAT      ego_velocity_X;
    FLOAT      ego_velocity_Y;
    FLOAT      ego_velocity_Z;

    uint32_t   device_id;
    FLOAT      scan_time;               // dwell time
    FLOAT      pulse_time;              // PRI time
    FLOAT      chip_time;               // seconds
    FLOAT      sample_rate;             // Hz
    FLOAT      doppler_bin_width;       // mps
    FLOAT      range_bin_width;         // meters
    int32_t    chips_per_pulse;

    uint16_t   num_tx_prn;              // number of tx codes that were active in this scan
    int16_t    vp_scan_mode;            // enum RHAL_ScanModes VP_INVALID(-1) if not variable power
    int16_t    code_type;
    uint16_t   total_vrx;
    uint16_t   num_range_bins;
    uint16_t   num_pulses;
    uint16_t   num_channelizer_iters;
    uint16_t   num_channelizer_doppler_bins;
    uint16_t   num_beamforming_angles;
    uint16_t   num_azimuth_angles;      // num_el = num_beamforming_angles / num_azimuth_angles
    uint8_t    num_angle_groups;        // number of RAU passes on RDC2
    uint8_t    azimuth_nyquist_oversampling_factor;
    uint8_t    elevation_nyquist_oversampling_factor;
    uint8_t    angle_wrap_flags;

    int16_t    rdc1_software_exponent;
    int16_t    rdc2_software_exponent;
    int16_t    rdc3_software_exponent;
    int16_t    system_exponent;

    uint16_t   overflow_underflow_flags;
    uint16_t   num_detections;          // number of output detections
    uint16_t   total_points;            // number of output point cloud points TBD
    uint16_t   num_music_instances;     // number of SVD and COV outputs

    uint16_t   SS_size_R;
    uint16_t   SS_size_A;
    uint16_t   SS_size_D;
    uint16_t   CI_width;
    uint16_t   CI_height;
    uint16_t   CI_format;
    uint16_t   CI_doppler_width;
    uint16_t   CI_doppler_height;
    uint16_t   CI_bytes_per_pixel;
    int16_t    clutter_image_exponent;
    FLOAT      CI_pixel_size_in_meters;

    FLOAT      chip_temp_C;
    FLOAT      chip_LO_temp_C;
    FLOAT      board_T3_temp_C;
    uint16_t   rdc2_zd_rb_center;
    uint16_t   rdc2_zd_rb_halfwidth;

    cfloat     dc_bias[2][8];           // 1D, 2D. For all 8 RX

    FLOAT      activation_filter_snr;  // magnitude of smallest saved activation (dB)
    uint32_t   radar_status_bitmap;
    uint32_t   antenna_config_id;
    uint8_t    rb_combine;
    uint8_t    future_use_u8;
    uint16_t   future_use_u16;

    FLOAT      vrx_position_offset_X;
    FLOAT      vrx_position_offset_Y;
    FLOAT      vrx_position_offset_Z;
    FLOAT      carrier_frequency;

    // Note!  This structure must be a multiple of 64 bits!

    /* Followed by one byte per total_vrx, which is a table that maps virtual
     * receiver indices from RX-major order into +Y-major order.  RDC1 skewers
     * arrive at the data logger in RX-major order, while RDC2 channelizer
     * output arrives in +Y major order
     *
     * uint8_t vrx_map[total_vrx]; */
};

struct UhdpAngleBinInfo /* UHDP_TYPE_ANGLE_BINS */
{
    FLOAT azimuth;
    FLOAT elevation;
}; // [num_angle_groups][num_beamforming_angles];

struct UhdpScanReady  /* UHDP_TYPE_SCAN_READY */
{
    uint32_t   scan_sequence_number;
};

struct UhdpDataWindow /* UHDP_TYPE_DATA_WINDOW */
{
    uint32_t   scan_sequence_number;
    uint32_t   max_packet_counter;
};

struct UhdpADCHeader /* UHDP_TYPE_ADC */
{
    uint32_t   scan_sequence_num;
    uint32_t   packet_counter;
    uint32_t   total_sample_count;
    uint32_t   interleaved;
};

struct UhdpRDC1Header
{
    uint32_t   scan_sequence_num;
    uint32_t   packet_counter;
    uint16_t   cur_pulse;
    uint16_t   cur_rb;
};

struct UhdpRDC2Header /* UHDP_TYPE_RDC2 */
{
    uint32_t   scan_sequence_num;
    uint32_t   packet_counter;
    uint16_t   total_tuple_in_scan;
    uint16_t   per_tuple_size;
};

struct UhdpRDC3Header /* UHDP_TYPE_RDC3 */
{
    uint32_t   scan_sequence_num;
    uint32_t   packet_counter;
    uint16_t   total_tuple_in_bucket;
    uint16_t   per_tuple_size;
    uint8_t    bucket_index;
    uint8_t    angle_group_id;
    uint16_t   reserved;
};

struct UhdpCoreDumpHeader /* UHDP_TYPE_REG_CORE_DUMP */
{
    uint32_t   scan_sequence_num;
    uint32_t   packet_counter;
    uint16_t   total_core_dump_size;
    uint16_t   cur_core_dump_offset;
    uint8_t    proc_or_scan_flag;
    uint8_t    core_packet_counter;
    uint16_t   unit_id;
};

struct UhdpLLDCoreDumpRequest  /* UHDP_TYPE_LLD_CORE_DUMP */
{
    // If hardware_unit_name[0] is "?", the response will be a comma separated
    // list of the names of available hardware units
    CHAR       hardware_unit_name[8];
};

struct UhdpLLDCoreDumpResponse /* UHDP_TYPE_LLD_CORE_DUMP */
{
    CHAR       hardware_unit_name[8];
    uint16_t   cur_tuple_pair;
    uint16_t   total_tuple_pairs;

    // followed by (uint32_t address, uint32_t value) tuple pairs
};

struct UhdpStaticSliceHeader /* UHDP_TYPE_STATIC_SLICE */
{
    uint32_t   scan_sequence_num;
    uint32_t   packet_counter;
    uint8_t    future_use;
    uint8_t    angle_group_id;
    uint16_t   per_tuple_size;
};

struct UhdpStaticSliceBinHeader
{
    uint32_t   scan_sequence_num;
    uint32_t   packet_counter;
    uint32_t   bin_count;

    // followed by: uint16_t ss_zero_doppler_bin[num_bins];
};

struct UhdpCovarianceHeader /* UHDP_TYPE_COVARIANCE */
{
    uint32_t   scan_sequence_num;
    uint32_t   packet_counter;
    uint16_t   doppler_bin_num;
    uint16_t   range_bin_num;
    uint16_t   matrix_size;
    uint16_t   sample_offset;

    /* Data will be matrix_size x matrix_size covariance matrix. Samples are I,Q
     * pairs of signed 64bits */
};

struct UhdpSVDHeader /* UHDP_TYPE_SVD */
{
    uint32_t   scan_sequence_num;
    uint32_t   packet_counter;
    uint16_t   doppler_bin_num;
    uint16_t   range_bin_num;
    uint16_t   matrix_size;
    uint16_t   sample_offset;

    /* Data will be matrix_size x 1 S vector, then matrix_size x matrix_size
     * Unitary matrix. Samples are I,Q pairs of signed 32bits */
};

struct UhdpDataHeader /* UHDP_TYPE_RDC1, UHDP_TYPE_DETECTIONS, UHDP_TYPE_MUSIC_SAMPLES */
{
    uint32_t   scan_sequence_num;
    uint32_t   packet_counter;
};

struct UhdpClutterImageHeader /* UHDP_TYPE_CLUTTER_IMAGE */
{
    uint32_t   scan_sequence_num;
    uint32_t   packet_counter;
    uint32_t   sample_offset;
};

PACK(
    struct UhdpDiagRequest /* UHDP_TYPE_DIAG_REQUEST */
{
    uint32_t   diag_sequence_number;
    uint16_t   class_id;
    uint16_t   test_id;
    uint16_t   reserved; // To align the start of diag input data to 4 bytes
});

PACK(
    struct UhdpDiagComplete /* UHDP_TYPE_DIAG_COMPLETE */
{
    uint32_t   diag_sequence_number;
    uint32_t   return_code;
    int32_t    error_num;
    uint16_t   reserved; // To align the start of diag output data to 4 bytes
});

struct UhdpPeekReq /* UHDP_TYPE_STATE_PEEK to Radar */
{
    uint32_t sequence_id;
    uint32_t type;           //!< 0: memory; 1: register
    uint32_t read_dev_ptr;   //!< memory or reigister address
    union {
        uint32_t byte_count; //!< number of bytes to read
        uint32_t reg_mask;   //!< mask for register peek
    };
};

struct UhdpPeekResp /* UHDP_TYPE_STATE_PEEK to Data Logger */
{
    uint32_t sequence_id;
    uint32_t type;           //!< 0: memory; 1: register
    uint32_t status;         //!< non-zero if unable to read
    uint32_t read_dev_ptr;   //!< memory or reigister address
    union {
        uint32_t byte_count; //!< number of bytes to read
        uint32_t reg_mask;   //!< mask for register peek
    };

    /* followed by memory buffer payload or register value */
};

struct UhdpPokeReq /* UHDP_TYPE_STATE_POKE to Radar*/
{
    uint32_t sequence_id;
    uint32_t type;           //!< 0: memory; 1: register
    uint32_t write_dev_ptr;
    /* for memory, followed by payload; for register, followed by mask & value */
};

struct RegisterInfo /* Used by register poke to parse the UhdpPokeReq packet */
{
    uint32_t mask;
    uint32_t value;
};

struct UhdpPokeResp /* UHDP_TYPE_STATE_POKE to Data Logger */
{
    uint32_t sequence_id;
    uint32_t type;           //!< 0: memory; 1: register
    uint32_t status;         //!< non-zero if unable to write
};

struct UhdpEnvControl /* UHDP_TYPE_ENV_CONTROL_DATA, UHDP_TYPE_ENV_CONTROL_ACK */
{
    uint16_t message_type;
    uint16_t sequence_number;
};

//! When sent from the radar to the data logger, it contains the current radar
//! harware clock with a resolution of 1us, as time since boot.  When sent from
//! the data logger to the radar, it contains an update to the scan cadence base
//! timer (adjusting skew).
struct UhdpTimeAlignment /* UHDP_TYPE_TIME_ALIGN */
{
    uint64_t timestamp64;
    uint32_t timestamp32;
    uint32_t scan_sequence_number;
};

//! each UHDP_TYPE_LOG_CONTROL message to the radar can contain a series of
//! these 16bit tuples
struct UhdpLogLevelControl /* UHDP_TYPE_LOG_CONTROL */
{
    uint16_t producer : 3;
    uint16_t subunit  : 10;
    uint16_t loglevel : 3;
};

SRS_CLOSE_NAMESPACE()

#endif // SRS_HDR_UHDP_MSG_STRUCTS_H
