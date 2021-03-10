// Copyright (C) Uhnder, Inc. All rights reserved. Confidential and Proprietary - under NDA.
// Refer to SOFTWARE_LICENSE file for details
#ifndef SRS_HDR_UHDP_MSG_STRUCTS_H
#define SRS_HDR_UHDP_MSG_STRUCTS_H 1

#include "modules/drivers/radar/rocket_radar/driver/system-radar-software/env-reference/coredefs/uhnder-common.h"
#include "modules/drivers/radar/rocket_radar/driver/system-radar-software/engine/common/eng-api/uhtypes.h"
#include "modules/drivers/radar/rocket_radar/driver/system-radar-software/engine/common/eng-api/rdc-structs.h" // DetectionData

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
    UHDP_TYPE_DPROBE_DC_DATA   = 28,  //! 1C to data logger
    UHDP_TYPE_RDC1_PLAYBACK    = 29,  //! 1D to data logger

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
    UHDP_TYPE_DPROBE_CONFIG    = 59,  //! 3B from data logger to radar
    UHDP_TYPE_DC_MEAS_CONFIG   = 60,  //! 3C from data logger to radar
    UHDP_TYPE_RDC1_DATA_READY  = 61,  //! 3D from data logger to radar
    UHDP_TYPE_TELEMETRY_ASYNC  = 62,  //! 3E to radar from data logger (no connection)

    UHDP_TYPE_CAN_MSG_RX       = 80,  //! 50 bidirectional request/response
    UHDP_TYPE_CAN_MSG_TX       = 81,  //! 51 bidirectional request/response

    UHDP_TYPE_ENV_CONTROL_DATA = 90,  //! 5A bidirectional environment data
    UHDP_TYPE_ENV_CONTROL_ACK  = 91,  //! 5B bidirectional environment data

    UHDP_TYPE_FAST_CAPTURE     = 100, //! 64 from radar to data logger
    UHDP_TYPE_FAST_CAPTURE_ACK = 101, //! 65 from data logger to radar

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
    uint32_t logger_ip_address;         // unused
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
    DL_INCIDENT   = 1,  // No longer supported, will be deleted
    DL_IDLE       = 2,
};

/* mask for data log enabling/disabling */
enum DataLogMaskEnum
{
    DL_ADC            = 1 << 0,  //!< ADC samples
    DL_RDC1           = 1 << 1,  //!< RDC1
    DL_RDC2           = 1 << 2,  //!< RDC2
    DL_RDC3           = 1 << 3,  //!< RDC3 dynamic activations
    DL_SS             = 1 << 4,  //!< Static Slice
    DL_DET            = 1 << 5,  //!< Radar Detections
    DL_CI             = 1 << 6,  //!< Clutter Image
    DL_CAN_RX         = 1 << 7,  //!< Can Message Rx
    DL_CAN_TX         = 1 << 8,  //!< Can Message Tx
    DL_APP_DATA       = 1 << 9,  //!< Application Data
    DL_SCAN_CORE_DUMP = 1 << 10, //!< Register dumps from digital front end
    DL_PROC_CORE_DUMP = 1 << 11, //!< Register dumps from digital back end
    DL_COV            = 1 << 12, //!< Outputs of spatial smoothing and covariance (debug)
    DL_SVD            = 1 << 13, //!< Outputs of singular value decomposition (debug)
    DL_MUSIC          = 1 << 14, //!< MUSIC angle sample magnitudes
    DL_ZERO_DOPPLER   = 1 << 15, //!< RDC2 Zero Doppler (derived from RDC1 coherent sum)
    DL_HISTOGRAM      = 1 << 16, //!< Per-Range Histograms
    DL_DC_MEASURE     = 1 << 17, //!< Measure DC per rx per lane
    DL_POINT_CLOUD    = 1 << 18, //!< Dynamic point cloud
    DL_RDC1_PLAYBACK  = 1 << 19, //!< Raw RDC1 capture (for playback)
    DL_REPLAY_RDC1    = 1 << 20, //!< Replay Raw RDC1 capture (allow RDC1 to be uploaded by tftp)
    DL_DPROBE         = 1 << 21, //!< Enable D-Probes regitered with data agent
    DL_SPARSE_RDC3    = 1 << 22, //!< Capture sparisified RDC3 for rapid replay
    DL_NO_SUMMARY     = 1 << 23, //!< Don't print detections and activations summary to log
};

/*! Structure which controls the scan data which is output from the radar */
struct UhdpCaptureControl /* UHDP_TYPE_CAPTURE_CONTROL */
{
    uint32_t   command_sequence_number; //!< will be returned by UHDP_TYPE_CONTROL_ACK
    uint32_t   capture_mode;            //!< enum DataLogModeEnum DL_NORMAL or DL_IDLE

    //! mask of DataLogMaskEnum bits describing data to be output or features to
    //! enable
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
    uint16_t   adc_capture_mode;               //!< adc capture mode
    uint16_t   adc_capture_channel_bitmap;     //!< bitmap of 16-bits indicating which Rxs(and how many) are needed for ADC Capture
    uint16_t   sel_rx_tx;                      //!< select rx or tx for ADC capture 0- Rx 1 - Tx
    uint16_t   sample_bit_width;               //!< ADC samples bit width
    uint32_t   samples_per_channel;            //!< number of samples to capture

    // if (enable_mask & DL_RDC3) and (capture_mode == DL_NORMAL), the following
    // specifies a region of RDC3 to force capture of activations, and specifies
    // the number of range bins and doppler bins to capture.  if
    // rdc3_num_range_bins or rdc3_num_doppler_bins are 0, no special
    // activations are captured
    FLOAT      rdc3_special_range_m;
    FLOAT      rdc3_special_doppler_mps;
    uint16_t   rdc3_num_range_bins;
    uint16_t   rdc3_num_doppler_bins;

    uint16_t   peak_det_channel_bitmap; // 0 implies no peak detect request
    uint16_t   peak_det_measure_point;  // enum PeakDetector

    uint32_t   reserved_u32[7];

    void set_defaults()
    {
        memset(this, 0, sizeof(*this));
        rdc2_zd_rb_center = -1;
        rdc2_zd_rb_halfwidth = -1;
    }
};

struct UhdpControlAck      /* UHDP_TYPE_CONTROL_ACK */
{
    uint32_t command_sequence_number;
};

struct UhdpRangeBinInfo071 /* UHDP_TYPE_RANGE_BINS */
{
    int16_t  distance_in_bins;        //!< distance of range bin from radar, in units of range_bin_width - negative means invalid
    int16_t  reverse_map;             //!< map distance to range bin (indexed by distance in bins) - negative means not represented
    FLOAT    noise_floor_max_peak_dB; //!< the max peak of the power histogram in this range
    FLOAT    noise_floor_pfa_dB;
};

struct UhdpRangeBinInfo    /* UHDP_TYPE_RANGE_BINS */
{
    int16_t  distance_in_bins;        //!< distance of range bin from radar, in units of range_bin_width - negative means invalid
    int16_t  reverse_map;             //!< map distance to range bin (indexed by distance in bins) - negative means not represented
    FLOAT    noise_floor_max_peak_dB; //!< the max peak of the power histogram in this range
    int16_t  exponent;
    int16_t  reserved;
};

struct UhdpTelemetryData   /* UHDP_TYPE_TELEMETRY_DATA */
{
    vec3f_t  ego_linear_velocity;      //!< meters per second of world relative to radar
    vec3f_t  ego_acceleration;         //!< meters per second per second of world relative to radar
    FLOAT    yaw_rate;                 //!< radians per second - DEPRECATED, NEVER USED
    FLOAT    steering_turn_angle;      //!< radians, 0 at boresight
    FLOAT    braking_pressure;         //!< bars
    FLOAT    external_temp_C;          //!< celcius
    uint32_t telemetry_age_us;         //!< age of telemetry data, in microseconds
    FLOAT    phased_array_azimuth;     //!< phased array transmitter array steering azimuth angle (radians)
    vec3f_t  ego_angular_velocity;     //!< roll, pitch, yaw rates (radians per second)
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

    FLOAT      estimated_ego_velocity_X; // ego linear vel estimated from analyzing RDC3
    FLOAT      estimated_ego_velocity_Y;
    FLOAT      estimated_ego_velocity_Z;
    FLOAT      ego_linear_velocity_X;    // extrapolated external linear velocity at scan time
    FLOAT      ego_linear_velocity_Y;
    FLOAT      ego_linear_velocity_Z;

    uint32_t   device_id;
    FLOAT      scan_time;               // dwell time
    FLOAT      pulse_time;              // PRI time
    FLOAT      chip_time;               // seconds
    FLOAT      sample_rate;             // Hz
    FLOAT      doppler_bin_width;       // mps
    FLOAT      range_bin_width;         // meters
    int32_t    chips_per_pulse;

    uint16_t   num_tx_prn;              // number of tx codes that were active in this scan
    uint8_t    preset_applied;
    uint8_t    preset_diff_flags;       // bit 0 = preset diffs, bit 1 = non-preset diffs
    int16_t    code_type;
    uint16_t   total_vrx;
    uint16_t   num_range_bins;
    uint16_t   num_pulses;
    uint16_t   num_channelizer_iters;
    uint16_t   num_channelizer_doppler_bins;
    uint16_t   num_beamforming_angles;  // Total number of angle-bins including padding (ss2_num_angles)
    uint16_t   num_azimuth_angles;      // In-use azimuth angles
                                        // Total in-use angles (num_angles) = num_azimuth_angles * num_elevation_angles
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

    FLOAT      chip_temp_C;            // SabineA: analog probe measured chip temperature
                                       // SabineB: PVT reported D-Die temperature
    FLOAT      analog_die_temp_C;      // SabineB: analog probe measured A-Die temperature
    FLOAT      board_T3_temp_C;
    uint16_t   rdc2_zd_rb_center;
    uint16_t   rdc2_zd_rb_halfwidth;

    cfloat     dc_bias[2][8];          // 1D, 2D. For all 8 RX

    FLOAT      activation_filter_snr;  // magnitude of smallest saved activation (dB)
    uint32_t   radar_status_bitmap;
    uint32_t   antenna_config_id;
    uint8_t    rb_combine;
    uint8_t    complex_rdc3;           // RDC_rdc3_complex bits
    uint8_t    ss_doppler_0_only;
    uint8_t    num_elevation_angles;

    FLOAT      vrx_position_offset_X;
    FLOAT      vrx_position_offset_Y;
    FLOAT      vrx_position_offset_Z;
    FLOAT      carrier_frequency;

    uint16_t   tx_power_map;           // bitmap of TX powered on (HW12); 1 bit means ON
    uint16_t   tx_prn_map;             // bitmap of TX transmiting (HW12); 1 bit means ON

    FLOAT      peak_detector_output;
    FLOAT      dop_rotator_shift;      // meters per second

    uint8_t    scan_loop_size;         // number of scans in frame (scan loop)
    uint8_t    scan_loop_idx;          // 0 .. scan_loop_size - 1
    uint16_t   num_RD_lower;

    uint16_t   clock_tick_numerator;
    uint16_t   clock_tick_denominator;

    uint16_t   num_RD_upper;
    uint16_t   num_RD_above_cutoff;

    FLOAT      ego_angular_velocity_X;     // extrapolated external angular velocity at scan time
    FLOAT      ego_angular_velocity_Y;
    FLOAT      ego_angular_velocity_Z;

    uint8_t    estimated_ego_flag;
    uint8_t    connection_uhdp_version;
    uint8_t    reserved_u8[2];

    FLOAT      az_vrx_spacing_lambda;
    FLOAT      el_vrx_spacing_lambda;

    uint16_t   az_uniform_vrx;
    uint16_t   el_uniform_vrx;

    FLOAT      extrapolated_ego_velocity_X; // equals to si_linear_velocity, the one used to set static slice
    FLOAT      extrapolated_ego_velocity_Y;
    FLOAT      extrapolated_ego_velocity_Z;

    uint32_t   user_data;                   //!< user_data copied from RDC_ScanDescriptor

    uint32_t   reserved_u32[45];
    // Note!  This structure must be a multiple of 64 bits!

    // followed by:
    // * currently active RDC_ThresholdControl
    // * uint8_t vrx_map[total_vrx]
    // * vec3f_t rx_pos[8];
    // * vec3f_t tx_pos[num_tx_prn];
};

struct UhdpAngleBinInfo071
{
    FLOAT azimuth;
    FLOAT elevation;
}; // [num_angle_groups][num_beamforming_angles];

struct UhdpAngleBinInfo /* UHDP_TYPE_ANGLE_BINS */
{
    FLOAT   azimuth;
    FLOAT   elevation;
    int32_t angle_noise_floorQ8;
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
    uint32_t   total_raw_bytes;
    uint32_t   cur_byte_offset;
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
    uint16_t   range_bin_start;
};

struct UhdpCoreDumpHeader /* UHDP_TYPE_REG_CORE_DUMP */
{
    uint32_t   scan_sequence_num;
    uint32_t   packet_counter;
    uint16_t   total_core_dump_size;
    uint16_t   cur_core_dump_offset;
    uint8_t    proc_or_scan_flag;
    uint8_t    reserved;
    uint16_t   unit_id;
};

struct UhdpLLDCoreDumpRequest  /* UHDP_TYPE_LLD_CORE_DUMP */
{
    // If hardware_unit_name[0] is "?", the response will be a comma separated
    // list of the names of available hardware units
    CHAR       hardware_unit_name[8];
    INT        preload;
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

struct UhdpADCDescriptor
{
    static const uint32_t magic_value = 0x41444343UL; // "ADCC"
    static const uint32_t format_version = 1U;

    enum
    {
        ADC_CAPTURE_MODE_DATA_SPRAY,
        ADC_CAPTURE_MODE_RAW,
        ADC_CAPTURE_MODE_ADIE,
        ADC_CAPTURE_MODE_REVA_RAW,
        ADC_CAPTURE_MODE_REVA_DATASPRAY,

        SAMPLE_WIDTH_8BIT = 8,
        SAMPLE_WIDTH_16BIT = 16,
    };

    uint32_t magic;
    uint32_t version;
    uint8_t  adc_capture_mode;
    uint8_t  sample_bit_width;
    uint8_t  tx_one_rx_zero;
    uint8_t  reserved_u8;
    uint16_t channel_bitmap;
    uint16_t reserved_u16;
    uint32_t samples_per_channel;
    uint32_t total_data_size_bytes;
    uint32_t initial_rx_lane_numbers; // nibble per receiver

    bool validation_check()
    {
        bool ok = true;

        if (magic != magic_value)
        {
            ok = false;
        }
        else if (adc_capture_mode > ADC_CAPTURE_MODE_REVA_DATASPRAY)
        {
            ok = false;
        }
        else if (sample_bit_width != SAMPLE_WIDTH_8BIT && sample_bit_width != SAMPLE_WIDTH_16BIT)
        {
            ok = false;
        }
        else if (tx_one_rx_zero > 1U)
        {
            ok = false;
        }
        else if (channel_bitmap == 0U)
        {
            ok = false;
        }
        else if (samples_per_channel == 0U)
        {
            ok = false;
        }
        else if (version < format_version)
        {
            // assign default values to new fields
        }
        // TODO: validate total size given adc_capture_mode, samples_per_channel,
        // and sample_bit_width

        return ok;
    }

    void set_defaults()
    {
        magic = magic_value;
        version = format_version;
        adc_capture_mode = ADC_CAPTURE_MODE_DATA_SPRAY;
        sample_bit_width = SAMPLE_WIDTH_8BIT;
        tx_one_rx_zero = 0U;
        reserved_u8 = 0U;
        reserved_u16 = 0U;
        channel_bitmap = 1U;
        samples_per_channel = 0U;
        total_data_size_bytes = 0U;
        initial_rx_lane_numbers = 0U;
    }
};

SRS_CLOSE_NAMESPACE()

#endif // SRS_HDR_UHDP_MSG_STRUCTS_H
