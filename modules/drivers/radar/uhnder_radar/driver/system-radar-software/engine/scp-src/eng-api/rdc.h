#ifndef SRS_HDR_RDC_H
#define SRS_HDR_RDC_H 1
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

#include "modules/drivers/radar/uhnder_radar/driver/system-radar-software/env-uhnder/coredefs/uhnder-common.h"
#include "modules/drivers/radar/uhnder_radar/driver/system-radar-software/engine/common/eng-api/uhtypes.h"
#include "modules/drivers/radar/uhnder_radar/driver/system-radar-software/engine/common/eng-api/devmm.h"
#include "modules/drivers/radar/uhnder_radar/driver/system-radar-software/engine/common/eng-api/event-enums.h"
#include "modules/drivers/radar/uhnder_radar/driver/system-radar-software/engine/scp-src/eng-api/rdc-errors.h"
#include "modules/drivers/radar/uhnder_radar/driver/system-radar-software/engine/common/eng-api/rdc-structs.h" // DetectionData, etc
#include "modules/drivers/radar/uhnder_radar/driver/system-radar-software/engine/common/eng-api/rhal_out.h"
#include "modules/drivers/radar/uhnder_radar/driver/system-radar-software/engine/scp-src/eng-api/tcs-cal-flashdata.h"
#include "modules/drivers/radar/uhnder_radar/driver/system-radar-software/engine/scp-src/eng-api/qilo-cal-flashdata.h"
#include "modules/drivers/radar/uhnder_radar/driver/system-radar-software/engine/scp-src/eng-api/ldo-cal-flashdata.h"
#include "modules/drivers/radar/uhnder_radar/driver/system-radar-software/engine/scp-src/eng-api/vtr-cal-flashdata.h"
#include "modules/drivers/radar/uhnder_radar/driver/system-radar-software/engine/scp-src/eng-api/default-presets.h"

SRS_DECLARE_NAMESPACE()

class RDC_ScanConfig;
class RDC_Layer;
class RDC_ScanInstance;
class RDC_DetectionIterator;
class RDC_StaticImage;
class RDC_Calibrate;

struct AngleCalKey;
struct IqCalKey;
struct RangeBinAlignKey;
struct DetectionData;
struct VrxAlignCalData;
struct UhdpThresholdControl;
struct RDC_ScanDescriptor;
struct RDC_ScanInfo;
struct RDC_AdaptiveThreshold;

//!< Class with methods for calibrating the radar
class RDC_Calibrate
{
public:

    RDC_Calibrate() {}

    virtual                ~RDC_Calibrate() {}

    //! Configure vrx alignment calibration parameters
    virtual void            calibrate_vrx_align_control(EventEnum completion_event_id, //!< Event to post when done with current step
                                                        FLOAT     target_range,        //!< Target physical distance in meters
                                                        uint16_t  rb_halfwidth,        //!< Range bin halfwidth to define range window size
                                                        uint16_t  flip_threshold,      //!< Flip threshod
                                                        uint16_t  rx_flip_time,        //!< Rx flip time
                                                        uint16_t  tx_extra_threshold,  //!< Tx extra threshold
                                                        uint16_t  tx_flip_time,        //!< Tx flip time
                                                        uint16_t  tx_nflip,            //!< tx num flips
                                                        uint16_t  max_iterations,      //!< Maximum iterations for the alignment
                                                        bool      log_enable,          //!< Verbose enable flag
                                                        bool      factory_mode,        //!< Factory mode or field mode
                                                        VrxAlignCalData *vrx_offset_data
                                                      ) = 0;

    //! step vrx calibration
    virtual void            calibrate_vrx_align_step() = 0;

    //! Write vrx calibrated date to flash
    virtual bool            calibrate_vrx_align_write(EventEnum completion) = 0;

    virtual void            calibrate_abort() = 0;

    //! Exit from calibration mode
    virtual void            release() = 0;
};


//! Scan parameters
//! These parameters do not change from scan to scan of the same configuration
//! This structure is returned by RDC_ScanInstance::get_parameters()
struct RDC_ScanParams
{
    uint32_t                sequence_number;                    //!< Sequence nubmer of scan
    FLOAT                   scan_time;                          //!< Duration of scan (seconds)
    FLOAT                   pulse_time;                         //!< Duration of one pulse (system PRI) (seconds)
    FLOAT                   chip_time;                          //!< Duration of one chip (Tc) (seconds)
    FLOAT                   doppler_bin_width;                  //!< Width of doppler bin (m/s)
    FLOAT                   range_bin_width;                    //!< Width of range bin (m)
    int32_t                 pulses_per_scan;                    //!< Number of pulses (N)
    int32_t                 chips_per_pulse;                    //!< Number of chips per pulse
    INT /* enum */          code_type;                          //!< PRBS31, APAS, Golay, M-sequence, ...
    // FOV
    // etc..
    int16_t                 range_bins;                         //!< Number of range bins (R)
    int16_t                 azimuth_bins;                       //!< Number of azimuth angle bins
    int16_t                 elevation_bins;                     //!< Number of elevation angle bins
};

//! Per scan-instance information
struct RDC_ScanInfo
{
    // Information about non-Static-Slice [Range,Doppler] bins ("hardware activations")
    uint32_t                num_RD_lower;                       //!< Number of non-Static-Slice RD bins in lower buffer
    uint32_t                num_RD_upper;                       //!< Number of non-Static-Slice RD bins in upper buffer
    uint32_t                num_RD_special;                     //!< Number of non-Static-Slice RD bins in special buffer
    uint32_t                num_RD_above_cutoff;                //!< Number of non-Static-Slice RD bins not discarded
    FLOAT                   cutoff_RD_snr;                      //!< Smallest peak SNR (dB) in a not discarded RD bin
    FLOAT                   min_RD_snr_lower;                   //!< Smallest peak SNR (dB) in a lower buffer RD bin
    FLOAT                   max_RD_snr_lower;                   //!< Largest  peak SNR (dB) in a lower buffer RD bin
    FLOAT                   min_RD_snr_upper;                   //!< Smallest peak SNR (dB) in a upper buffer RD bin
    FLOAT                   max_RD_snr_upper;                   //!< Largest  peak SNR (dB) in a upper buffer RD bin
};


//! Instance of a clutter image
class RDC_ClutterImage
{
public:

    RDC_ClutterImage() {}

    virtual                ~RDC_ClutterImage() {}

    virtual void           *get_magnitude_image() = 0;          //!< Get magnitude clutter image buffer
    virtual void           *get_height_doppler_image() = 0;     //!< Get height/doppler clutter image buffer
    virtual void            get_size(                           //!< Get clutter image buffer size and format
        INT &                    x,                             //!< Downrange size (pixels) (major dimension)
        INT &                    y,                             //!< Crossrange size (pixels) (minor dimension)
        INT &                    z,                             //!< Height size (voxels)
        RDC_clutter_image_format &format                         //!< Format of clutter image output (enum RDC_clutter_image_format)
        ) = 0;                                                  //!< Get the size of the image (in pixels, and bits-per-pixel)
    virtual void            get_height_doppler_image_size(      //!< Get the size of the height doppler image (in pixels, and bits-per-pixel)
        INT &x, INT &y) = 0;
    virtual FLOAT           get_pixel_size() = 0;               //!< Meters per image pixel
    virtual FLOAT           get_doppler_resolution() = 0;       //!< Doppler bin size (meters/second)
    virtual int16_t         get_image_exponent() = 0;           //!< Image exponent
    virtual uint16_t        get_num_doppler_bins() = 0;         //!< Number of Doppler bins
    virtual uint16_t       *get_zero_bins() = 0;                //!< Get Doppler indices of Static Slice zero bin for each angle bin
};

class RDC_EgoVelocity
{
public:

    RDC_EgoVelocity() {}

    virtual                 ~RDC_EgoVelocity() {}

    virtual void           *get_histogram() = 0;                //!< Get ego velocity histogram buffer
    virtual uint16_t        get_histogram_size() = 0;           //!< Get size of the histogram
    virtual void            get_next_ego_velocity_data(EgoVelocityData &d) = 0;                                //!< Get the ego velocity data from iterator
    virtual uint16_t        get_num_ego_velocity_data() = 0;    //!< Get the number of ego velocity data
    virtual vec3f_t         get_current_ego_velocity() = 0;     //!< Get ego velocity used for current scan
    virtual void            reset() = 0;                        //!< reset the ego velocity data iteration to the beginning
};


class RDC_DetectionIterator
{
public:

    RDC_DetectionIterator() {}

    virtual                ~RDC_DetectionIterator() {}

    virtual void            get_next_detection(DetectionData &d) = 0; //!< Get next detection
    virtual INT             get_num_detections() = 0;           //!< Get number of detections

    virtual void            reset() = 0;                        //!< Reset to first detection
};

//! Instance of a scan
class RDC_ScanInstance
{
public:

    RDC_ScanInstance() {}

    virtual                ~RDC_ScanInstance() {}

    virtual RDC_ScanConfig *get_scan_config() = 0;              //!< Get pointer to scan config
    virtual RDC_ScanParams *get_parameters() = 0;               //!< Get complete scan parameters for this scan configuration
    virtual RDC_ScanInfo   *get_information() = 0;              //!< Get complete scan information for this scan instance
    virtual uint32_t        get_sequence_number() = 0;          //!< Get monotonically incrementing scan sequence number
    virtual RDC_DetectionIterator *create_detection_iterator(RDC_detection_type    type, //!< Select moving only, static only, or both
                                                             RDC_detection_sort    sort, //!< Sort by range, or by magnitude
                                                             RDC_detection_region *region //!< Detection region
                                                             ) = 0; //!< Iterate through list of detections
    virtual RDC_ClutterImage *get_clutter_image() = 0;          //!< Get clutter image
    virtual RDC_EgoVelocity *get_ego_velocity_mgr() = 0;        //!< Get ego velocity manager class
    virtual void             set_estimated_ego_velocity(vec3f_t ego_vel, vec3f_t ego_accel) = 0;    //!< Ego velocity and acceleration estimated by object layer based on the info from scan corresponding to this rsi
    virtual void             request_music(                     //!< Request MUSIC processing
        RDC_music_request *req,                                 //!< Pointer to array of MUSIC requests
        INT                num_req                              //!< Number of MUSIC requests
        ) = 0;

    //! read extended detection from this scan and store into user-provided
    //! buffer. size_bytes is the size of the provided buffer. Returns the
    //! number of bytes written (size of the extended detection).
    virtual uint32_t        store_extended_detection(INT index, int8_t* buffer, uint32_t size_bytes) = 0;
    virtual uint32_t        get_extended_detection_size(INT index) = 0;
    virtual INT             get_num_extended_detections() = 0;

    //! returns true if there were any underflow or overflow conditions in this scan
    virtual bool            get_overflow_underflow_status() = 0;

    virtual cint64         *get_rdc2_zerod_buf() = 0;

    virtual int16_t         get_software_exponent() = 0;

    virtual FLOAT           get_noise_floor_max(uint16_t hw_rbin) = 0;

    virtual int32_t*        get_ridge_snr_vec() = 0;                //!< Return: Ridge SNR (dB) for all angle bins (vector)

    virtual int16_t         get_hw_rbin(uint16_t rb_distance) = 0;
    virtual const int8_t*   get_rev_map() = 0;
    virtual const int8_t*   get_vrx_map() = 0;

    virtual void            calc_steering_vector(cint16 *stv,
                                                 FLOAT   azimuth,
                                                 FLOAT   elevation,
                                                 FLOAT   range,
                                                 bool    y_major = false,
                                                 bool    active_only = false) = 0;

#if WITH_UHDP
    virtual uint32_t        get_env_datagram_buffer_max_length() const = 0;
    virtual void*           get_env_datagram_buffer() = 0;
    virtual bool            send_env_datagram_buffer(uint32_t msg_type, uint32_t msg_length) = 0;
#endif

    virtual uint32_t        get_scan_end_timestamp() const = 0;
    virtual FLOAT           get_scan_chip_temperature() const = 0;
    virtual FLOAT           get_scan_board_temperature() const = 0;

    virtual void            release() = 0;                      //!< Release this scan instance
};

//! Special request for dense RDC3 data from a specified region from future
//! scans
struct RDC_extra_data_descriptor
{
    FLOAT   range;                  //!< range (m) of region center
    FLOAT   doppler;                //!< doppler velocity (m/s) of region center
    FLOAT   azimuth;                //!< azimuth angle (radians) of region center
    FLOAT   elevation;              //!< elevation angle (radians) of region center
    INT     azimuth_half_width;     //!< number of azimuth bins around center to extract
    INT     elevation_half_width;   //!< number of elevation bins around center to extract
    INT     range_half_width;       //!< number of range bins around center to extract
    INT     doppler_half_width;     //!< number of doppler bins around center to extract
};

//! Configure radar operation and perform scans
class RDC_ScanConfig
{
public:

    RDC_ScanConfig() {}

    virtual                ~RDC_ScanConfig() {}

    //! Release this RDC_ScanConfig (automatically disables scanning)
    virtual void            release() = 0;

    //! Set format for clutter images
    virtual void            set_clutter_image_format(RDC_clutter_image_format az_only,
                                                     RDC_clutter_image_format az_el) = 0;

    //! Set interval of scan starts, in microseconds. When using deterministic
    //! loop, this specifies the interval of the entire scan loop.  The
    //! specified interval must be larger than the total dwell time of all
    //! scans (plus some slack time for intra-scan calibrations).
    virtual void            set_scan_interval(uint32_t interval_us) = 0;

    //! absolute base time (in 64bit hardware clock) of scan intervals
    //! user can specify a new scan interval timebase before or during scanning,
    //! but set_scan_interval() must be called before set_scan_interval_timebase()
    virtual void            set_scan_interval_timebase(uint64_t timebase) = 0;

    //! request dense RDC3 data from subsequent scans. The desc describes the
    //! region of interest and number_of_scans describes the duration. Returns
    //! a handle which may be canceled prior to the scheduled duration by
    //! calling cancel_extra_data_request()
    virtual uint32_t        request_extra_data(INT number_of_scans, const RDC_extra_data_descriptor &desc) = 0;

    //! Cancel an extra data request, providing the handle which was returned by
    //! request_extra_data()
    virtual void            cancel_extra_data_request(uint32_t request_handle) = 0;

    virtual void            request_rdc2_zero_doppler(int16_t rb_center_bin, int16_t rb_halfwidth) = 0;

    virtual void            request_adc_capture(RHAL_ADC_CaptureMode mode, uint8_t rx_num, uint32_t capture_size_bytes) = 0;

    //! Enable operation in digital playback mode
    virtual void            set_playback_mode(int8_t*            samples,           //!< ADC output samples for 8 Rx
                                              uint8_t*           codes,             //!< PRN code bits for 8 Tx
                                              uint32_t           samples_len,       //!< Number of bytes of sample data
                                              uint32_t           codes_len,         //!< Number of bytes of code data
                                              bool               repeat_pulse_data, //!< Whether to repeat first pulse data for all pulses
                                              RHAL_DummyScanMode synth_input) = 0;

    //! special purpose callback event (meant for calibration scans); triggered
    //! event handler is passed RDC_ScanInstance pointer, which must be released
    //! when processing is completed. Failure to release in a timely matter will
    //! cause delays in scanning
    virtual void            set_scan_complete_notification(EventEnum event) = 0;

    //! configure all the settings together in an RDC_ScanDescriptor. this is a
    //! convenience wrapper for setup_scan_loop(desc[0], 1)
    virtual void            setup_scan(const RDC_ScanDescriptor& desc) = 0;

    //! configure all the settings together in one or more RDC_ScanDescriptor
    virtual void            setup_scan_loop(const RDC_ScanDescriptor* desc, uint32_t scan_loop_count) = 0;

    //! Start performing scans using this configuration. Returns the number of
    //! scans scheduled, or num_scans is -1 if continuous
    virtual INT             enable_scanning(INT num_scans) = 0;

    //! Stop performing scans (required for changing modes)
    virtual void            disable_scanning() = 0;
};

class RDC_AntennaDesc
{
public:

    RDC_antenna_type        type;                               //!< Transmitter or receiver?
    uint16_t                index;                              //!< Hardware (HW-12) transmitter number (0..11) or receiver number (0..31)
    vec3f_t                 position;                           //!< position (meters) of each antenna array element (SAE coordinates)
    FLOAT                   gain;                               //!< boresight gain (dBi) of single antenna array element
    FLOAT                   az_angles[RDC_ANTENNA_GAIN_NUM];    //!< azimuth angle (deg) of -3dB, etc. points
    FLOAT                   el_angles[RDC_ANTENNA_GAIN_NUM];    //!< elevation angle (deg) of -3dB, etc. points
};

struct RDC_LayerDesc
{
public:

    RDC_LayerDesc()
    {
        sc_stack =              USER_SRAM;
        mgr_stack =             USER_SRAM_UNCACHED;
        detections_stack =      USER_SRAM_UNCACHED;
#if SRS_HAVE_DDR
        clutter_image_stack =   RDC_DRAM;
        rsi_stack =             RDC_DRAM;
        scratchpad_stack =      RDC_DRAM;
#else
        clutter_image_stack =   USER_SRAM_UNCACHED;
        rsi_stack =             USER_SRAM_UNCACHED;
        scratchpad_stack =      USER_SRAM_UNCACHED;
#endif
        ext_lo_enable_flags =   LO_Onboard;
        adc_sample_rate =       FSAMP_1G;
        carrier_frequency =     DEF_RF;
    }

    virtual                ~RDC_LayerDesc() {}

    StackEnum               sc_stack;
    StackEnum               mgr_stack;
    StackEnum               rsi_stack;
    StackEnum               detections_stack;
    StackEnum               clutter_image_stack;
    StackEnum               scratchpad_stack;

    RHAL_LOMode             ext_lo_enable_flags;
    RHAL_Sampling_rate      adc_sample_rate;
    FLOAT                   carrier_frequency;
    INT                     adc_ldo_code;

    TCSCalData              initial_tx_carrier_suppression;
    QiloCalData             initial_qilo_dcap;
    LDOCalData              initial_ldo_config;
    VTRCalData              initial_vtr_config;
};

//! Primary interface to the RDC API.
//
//! A global singleton pointer to this class (rdc_layer) provides the
//! initial entry point into the RDC API.
class RDC_Layer
{
public:

    RDC_Layer() {}

    static RDC_Layer& instance();

    virtual                ~RDC_Layer() {}

    virtual void            init(const RDC_LayerDesc& desc) = 0;
    virtual void            set_antenna_config(RDC_AntennaDesc &desc                                   //!< Antenna descriptor
                                               ) = 0;           //!< Configure an antenna group
    virtual RDC_ScanConfig *create_scan_config() = 0;           //!< Create new scan configuration
    virtual RDC_ScanConfig *get_current_scan_config() = 0;      //!< Get current active scan configuration
    virtual RDC_Calibrate  *calibrate() = 0;                    //!< Enter calibration mode
    virtual void            report_boot_calibrations() = 0;     //!< report boot time calibration errors
    virtual RDC_error       get_last_error() = 0;               //!< Get error status from last RDC API call
    virtual void            set_ego_velocity(vec3f_t linear,
                                             quatf_t angular,
                                             uint32_t age_us) = 0; //!< Estimate of radar linear & angular velocity

    //! side-lobe and ridge suppression thresholds
    virtual void            set_detection_thresholds(const UhdpThresholdControl& thresh_ctrl) = 0;

    //! original adaptive thresholding thresholds
    virtual void            set_adaptive_thresholds(const RDC_AdaptiveThreshold& static_thresh,
                                                    const RDC_AdaptiveThreshold& moving_thresh,
                                                    uint8_t twoD_flag //!< 0 - 1D, 1 - 2D, 2 - Both
                                                    ) = 0;

    virtual void            set_cfar_pid_constants(FLOAT Kp, FLOAT Ki, FLOAT Kd, uint32_t num_act) = 0;

    virtual void            enable_ego_velocity_detection(bool enable) = 0;
    virtual void            register_scan_complete_notification(EventEnum event) = 0;
    virtual void            unregister_scan_complete_notification(EventEnum event) = 0;
    virtual void            register_music_complete_notification(EventEnum event) = 0;
    virtual void            unregister_music_complete_notification(EventEnum event) = 0;
    virtual void            register_scan_config_release_notification(EventEnum event) = 0;
    virtual void            unregister_scan_config_release_notification(EventEnum event) = 0;
    virtual void            register_scanning_ready_notification(EventEnum event) = 0;
    virtual void            unregister_scanning_ready_notification(EventEnum event) = 0;

    //! get the chip temperature
    virtual FLOAT           get_temperature() = 0;
    //! get the last measured analog high voltage
    virtual FLOAT           get_analog_high_voltage() = 0;

    virtual bool            test_dc_bias_key(const struct DCBiasKey&) = 0;

    virtual RHAL_LOMode     get_local_oscillator_mode() const = 0;
    virtual FLOAT           get_carrier_frequency() const = 0;
};

SRS_CLOSE_NAMESPACE()

#endif // SRS_HDR_RDC_H
