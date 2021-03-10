// Copyright (C) Uhnder, Inc. All rights reserved. Confidential and Proprietary - under NDA.
// Refer to SOFTWARE_LICENSE file for details
#ifndef SRS_HDR_RDC_H
#define SRS_HDR_RDC_H 1
/*! \file */

#include "modules/drivers/radar/rocket_radar/driver/system-radar-software/env-reference/coredefs/uhnder-common.h"
#include "modules/drivers/radar/rocket_radar/driver/system-radar-software/engine/common/eng-api/uhtypes.h"
#include "modules/drivers/radar/rocket_radar/driver/system-radar-software/engine/common/eng-api/devmm.h"
#include "modules/drivers/radar/rocket_radar/driver/system-radar-software/engine/common/eng-api/event-enums.h"
#include "modules/drivers/radar/rocket_radar/driver/system-radar-software/engine/scp-src/eng-api/rdc-errors.h"
#include "modules/drivers/radar/rocket_radar/driver/system-radar-software/engine/common/eng-api/rdc-structs.h" // RDC_clutter_image_format, etc
#include "modules/drivers/radar/rocket_radar/driver/system-radar-software/engine/common/eng-api/rdc-common.h"
#include "modules/drivers/radar/rocket_radar/driver/system-radar-software/engine/common/eng-api/rhal_out.h"

SRS_DECLARE_NAMESPACE()

class RDC_FrameConfig;
class RDC_ScanConfig;
class RDC_Layer;
class RDC_ScanInstance;
class RDC_DetectionIterator;
class RDC_StaticImage;

struct AntennaSetParameters;
struct UhdpScanInformation;
struct UhdpADCDescriptor;
struct DetectionData;
struct RDC_ThresholdControl;
struct RDC_ScanDescriptor;
struct RDC_ScanInfo;
struct RDC_LayerDesc;
struct RDC_extra_data_descriptor;
struct RDC_AprobeRequest;

struct DCCalData;
struct DCCalBData;
struct IQCalData;
struct TCSCalData;
struct QiloCalData;
struct QiloCalBData;
struct VrxAlignCalBData;
struct CDataCalBData;
struct LDOCalBData;
struct TxGainCalData;
struct RxGainCalData;
struct TxBwCalData;
struct RxBwCalData;
struct RegulatorVoltageRecord;
struct LdoCalibrationParameters;
struct PeakDetCalData;

//! Scan information
//
//! These scan parameters do not change from scan to scan of the same configuration.
//! This structure is returned by RDC_ScanInstance::get_parameters().
struct RDC_ScanParams
{
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
    int16_t                 vrx_used;                           //!< Number of virtual receivers used
    FLOAT                  *az_angle_bins;                      //!< Vector of beamformed Azimuth angles (in Radians) (length is azimuth_bins*elevation_bins)
    FLOAT                  *el_angle_bins;                      //!< Vector of beamformed Elevation angles beamform (in Radians) (length is azimuth_bins*elevation_bins)
    int16_t                *az_angle_bins_Q14;                  //!< Vector of beamformed Azimuth angles, in Q.14 format. -PI/2 to PI/2 (length is azimuth_bins*elevation_bins)
    int16_t                *el_angle_bins_Q14;                  //!< Vector of beamformed Elevation angles, in Q.14 format. -PI/2 to PI/2 (length is azimuth_bins*elevation_bins)
    bool                    wrap_azimuth_angles;                //!< If true, azimuth angles wrap around because of ambiguity
    bool                    wrap_elevation_angles;              //!< If true, elevation angles wrap around because of ambiguity
    FLOAT                   scan_lambda;                        //!< Wavelength for scan (meters)
    uint16_t                first_range_bin;                    //!< Spatial bin number of the first range bin in the range bins map
};

//! Per scan-instance information
//
//! These scan parameters change from scan to scan of the same configuration.
//! This structure is returned by RDC_ScanInstance::get_information().
struct RDC_ScanInfo
{
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


//! Accessor for per-scan Clutter Image data
//
//! This class is returned by RDC_ScanInstance::get_clutter_image().
class RDC_ClutterImage
{
public:

                            RDC_ClutterImage() {}

    UHVIRTDESTRUCT(RDC_ClutterImage)

    //! Get magnitude clutter image buffer
    virtual void           *get_magnitude_image(
                                ) = 0;

    //! Get height/doppler clutter image buffer
    virtual void           *get_height_doppler_image(
                                ) = 0;

    //! Get the size and format of the Clutter Image magnitude plane
    virtual void            get_size(
                                INT &                     x,        //!< Downrange size (pixels) (major dimension)
                                INT &                     y,        //!< Crossrange size (pixels) (minor dimension)
                                INT &                     z,        //!< Elevation (altitude) size (voxels)
                                RDC_clutter_image_format &format    //!< Format of clutter image output
                                ) = 0;

    //! Get the size of the Clutter Image height/doppler planes (in pixels, and bits-per-pixel)
    virtual void            get_height_doppler_image_size(
                                INT &       x,          //!< Downrange size (pixels)
                                INT &       y           //!< Crossrange size (pixels)
                                ) = 0;

    //! Get Clutter Image pixel size (meters)
    virtual FLOAT           get_pixel_size(
                                ) = 0;

    //! Get Doppler bin size (meters/second)
    virtual FLOAT           get_doppler_resolution(
                                ) = 0;

    //! Get Clutter Image exponent
    virtual int16_t         get_image_exponent(
                                ) = 0;

    //! Get Number of Doppler bins
    virtual uint16_t        get_num_doppler_bins(
                                ) = 0;

    //! Get Doppler indices of Static Slice zero bin for each angle bin
    virtual uint16_t       *get_zero_bins(
                                ) = 0;
};


//! Accessor for per-scan Ego-Velocity data
//
//! This class is returned by RDC_ScanInstance::get_ego_velocity_mgr().
class RDC_EgoVelocity
{
public:

                            RDC_EgoVelocity() {}

    UHVIRTDESTRUCT(RDC_EgoVelocity)

    //! Get ego velocity histogram buffer
    virtual void           *get_histogram(
                                    ) = 0;

    //! Get size of the ego-velocity histogram
    virtual uint16_t        get_histogram_size(
                                    ) = 0;

    //! Get the ego velocity data from iterator - DEPRECATED
    virtual void            get_next_ego_velocity_data(
                                    EgoVelocityData &d
                                    ) = 0;

    //! Get the number of ego velocity data - DEPRECATED
    virtual uint16_t        get_num_ego_velocity_data(
                                    ) = 0;

    //! Get ego velocity used for current scan
    virtual vec3f_t         get_current_ego_velocity(
                                    ) = 0;

    //! Reset the ego velocity data iteration to the beginning - DEPRECATED
    virtual void            reset(
                                    ) = 0;
};


//! Accessor for per-scan Detections data
//
//! This class is returned by RDC_ScanInstance::create_detection_iterator().
class RDC_DetectionIterator
{
public:

                            RDC_DetectionIterator() {}

    UHVIRTDESTRUCT(RDC_DetectionIterator)

    //! Get next detection
    //
    //! If this method is called more than get_num_detections()
    //! times after the creation of the iterator or after a
    //! call to reset(), the output DetectionData is not valid,
    //! and RDC_Layer::get_last_error() will return RDC_ERR_no_more.
    virtual void            get_next_detection(
                                    DetectionData &d            //!< Output: next detection
                                    ) = 0;

    //! Get the total number of detections
    //
    //! This is the number of times that get_next_detection() can be
    //! called successfully after creation of the iterator or after
    //! a call to reset().
    virtual INT             get_num_detections(
                                    ) = 0;

    //! Reset the detection iterator to the first detection.
    //
    //! Following reset, the get_next_detection() method can be
    //! called successfully get_num_detections() times.
    virtual void            reset(
                                    ) = 0;

    //! Get all detections in raw format (fixed-point and in units of "bins" instead of SI units)
    //
    //! Note: The data structure returned by this API may change
    //! from version to version.  It is the responsability of any
    //! user of this data to ensure compliance with the documentation
    //! provided for the RDC_DetectionData structure.
    //! This method returns a pointer to a RDC_DetectionData structure,
    //! which is a structure of arrays.  It also returns the
    //! the number of entries in the arrays in the structure.
    //! This number can be different than the number of detections returned
    //! by get_num_detections().  This is because some detections
    //! may be flagged as invalid by their magnitude_raw member being
    //! set to zero.
    //! The arrays of raw detetions are not sorted in an particular order.
    virtual RDC_DetectionData *get_raw_detections(
                                    uint32_t &num_raw_detections    //!< Number of raw detections
                                    ) = 0;

};


//! Accessor for per-scan Point Cloud data
//
//! This class is returned by RDC_ScanInstance::get_point_cloud().
class RDC_PointCloud
{
public:
                            RDC_PointCloud() {}

    UHVIRTDESTRUCT(RDC_PointCloud)

    //! Get all points in the Point Cloud
    //
    //! Returns a pointer to an array of PointCloudData structures,
    //! as well as the number of entries in the array.  The array
    //! is not sorted in an particular order.
    virtual PointCloudData *get_all(
                                    uint32_t &num_points        //!< Number of PointCloudData entries in the returned array
                                    ) = 0;

    //! Get indices of all points in a specified region of RDC3
    //
    //! Returns a pointer to a vector of indices into the
    //! PointCLoudData array returned by get_all()
    //
    virtual uint32_t    *get_points_in_region(
                                  uint16_t    range_min,      //!< Input: Lowest range bin index of query region
                                  uint16_t    range_max,      //!< Input: Highest range bin index of query region
                                  uint16_t    azimuth_min,    //!< Input: Lowest azimuth angle bin index of query region
                                  uint16_t    azimuth_max,    //!< Input: Highest azimuth angle bin index of query region
                                  uint16_t    doppler_min,    //!< Input: Lowest Doppler bin index of query region
                                  uint16_t    doppler_max,    //!< Input: Highest Doppler bin index of query region
                                  uint32_t   &num_points      //!< Output: Number of PointCloudData in region>
                                  ) = 0;

    //! Convert a point cloud point from units of "bins" to SI units,
    //! and to floating-point format
    virtual void        convert_to_si(
                                PointCloudData     &in,         //!< Input: Point cloud point in fixed-point units of "bins"
                                PointCloudPoint    &out         //!< Output: Point cloud point converted to floating-point SI units
                                ) = 0;
};


//! Instance of a scan
//
//! This class contains accessors for all data output from the RDC Layer
//! for a scan.
class RDC_ScanInstance
{
public:

                            RDC_ScanInstance() {}

    UHVIRTDESTRUCT(RDC_ScanInstance)

    //! Get pointer to frame config
    virtual RDC_FrameConfig *get_frame_config(
                                    ) = 0;

    //! Get pointer to scan config
    virtual RDC_ScanConfig  *get_scan_config(
                                    ) = 0;

    //! Get complete scan parameters for this scan configuration
    virtual RDC_ScanParams  *get_parameters(
                                    ) = 0;

    //! Get complete scan information for this scan instance
    virtual RDC_ScanInfo    *get_information(
                                    ) = 0;

    //! Generate an IOVEC pair (address, offset) list which covers the sparsified
    //! RDC3 and all the related data required to replay the scan in the
    //! future; including noise floors and software exponents.  The user must
    //! provide storage for the IOVEC list and incidentals.
    //
    //! If the function does not set any errors, the working buffer will contain
    //! an IOVEC array terminated with a (0, X) pair. It is the user's
    //! responsibility to push this data out of the radar
    //
    //! The resulting blob can be loaded back onto the radar and replayed. The
    //! user is responsible for regenerating the same (or compatible) frame config
    virtual void            generate_scan_blob(
                                    uint8_t* working_buffer,             //!< user-provided working buffer storage
                                    uint32_t working_buffer_size_bytes,  //!< size in bytes of user-provided working buffer
                                    uint16_t maximum_datagram_size       //!< maximum number of bytes in each IOVEC
                                    ) = 0;

    //! Accessor used by UhDP protocol implementation for fetching scan information
    virtual void            fill_scan_information(
                                    UhdpScanInformation&        //!< Output: UhDP data structure to be filled
                                    ) = 0;

    //! Get monotonically incrementing scan sequence number
    virtual uint32_t        get_sequence_number() = 0;

    //! Iterate through list of detections
    virtual RDC_DetectionIterator *create_detection_iterator(
                                    RDC_detection_type    type, //!< Select moving only, static only, or both
                                    RDC_detection_sort    sort, //!< Sort by range, or by magnitude
                                    RDC_detection_region *region//!< Detection region
                                    ) = 0;

    //! Get clutter image
    virtual RDC_ClutterImage *get_clutter_image(
                                    ) = 0;

    //! Get ego velocity manager class
    virtual RDC_EgoVelocity *get_ego_velocity_mgr(
                                    ) = 0;

    //! Ego velocity and acceleration estimated by Object Layer based on
    //! the info from scan corresponding to this RSI
    virtual void             set_estimated_ego_velocity(
                                    vec3f_t     ego_vel,        //!< World velocity vector (X,Y,Z, in meters/second), in radar coordinate system
                                    vec3f_t     ego_accel       //!< World acceleration vector - CURRENTLY UNUSED
                                    ) = 0;

    //! Request MUSIC angle super-resolution post processing for
    //! selected Range-Doppler bins.
    virtual void             request_music(                     //!< Request MUSIC processing
                                    RDC_music_request *req,     //!< Pointer to array of MUSIC requests
                                    INT                num_req  //!< Number of MUSIC requests
                                    ) = 0;

    //! Get results from requested D-Probe measurements
    //
    //! The RDC_DProbe_Output pointer is only valid if DProbe requests
    //! were registered with the RDC_FrameConfig. The dprobe_completion_flags
    //! data member describes which dprobes completed successful
    virtual const RDC_DProbe_Output* get_requested_dprobes(
                                    ) = 0;

    //! Returns the number of A-Probe measurements that were completed during
    //! this scan
    virtual INT              get_num_completed_aprobes(
                                    ) const = 0;

    //! Get the results from a request for A-Probe measurement
    //
    //! Returns the measured value for one of the completed aprobes. If request
    //! is provided, it is filled with the A-Probe request info (bus, channel, probe, etc).
    //! Returns false if index is out of range
    virtual bool             get_completed_aprobe(
                                    INT                 index,      //!< Input:  A-Probe index, from 0 to get_num_completed_aprobes() - 1
                                    RDC_AprobeRequest&  request     //!< Output: A-Probe request struture, with result in measured_value
                                    ) const = 0;

    //! Get the results from a request for DC measurement
    //
    //! The RDC_adi_measured_dc pointer is only valid if DC measurements were
    //! requested with the RDC_FrameConfig
    virtual const RDC_adi_measured_dc* get_requested_dc_measurements(
                                    ) = 0;

    //! Get requested ADC data, returns NULL if no ADC data was captured
    virtual const void*     get_adc_descriptor(
                                    UhdpADCDescriptor& desc         //!< Input/Output: user-provided storage for outputs
                                    ) const = 0;

    //! Get data for an extended detection from this scan.
    //
    //! The returns value is the number of bytes written into the output buffer
    //! (the size of the extended detection).
    virtual uint32_t        store_extended_detection(
                                    INT             index,          //!< Extended detection index
                                    int8_t*         buffer,         //!< Buffer to store output into
                                    uint32_t        size_bytes      //!< Size of the supplied buffer
                                    ) = 0;

    //! Get the size of an extended detection
    virtual uint32_t        get_extended_detection_size(
                                    INT index                       //!< Extended detection index
                                    ) = 0;


    //! Get the number of extended detections from this scan
    virtual INT             get_num_extended_detections(
                                    ) = 0;

    //! Returns true if there were any underflow or overflow conditions in this scan
    virtual bool            get_overflow_underflow_status(
                                    ) = 0;

    //! Get class for accessing Point Cloud data
    virtual RDC_PointCloud *get_point_cloud(
                                    ) = 0;

    //! Get the requested RDC2 Zero-Doppler buffer for this scan
    //
    //! If request_rdc2_zero_doppler() was invoked when configuring
    //! the current scan, this method returns a pointer to a buffer
    //! that contains the computed RDC2 Zero-Doppler matrix.
    //! All VRxs, as indicated by get_num_vrx(), are included in the
    //! output. There is no option to choose a subset of VRx.
    //! VRx output is stored in Rx-major (hardware) order rather
    //! than in spatial (Y-major) order.
    virtual cint64         *get_rdc2_zerod_buf(
                                    ) = 0;

    //! Get a a raw cell from the RDC3 matrix (i.e.: from the Static Slice or from Activations)
    //
    //! If the selected RDC3 cell exists in the hardware output,
    //! in either the Static Slice or or in the thresholded Activation
    //! data, this method returns information about the cell. If the
    //! cell does not exist, it sets the RDC_Layer error code returned
    //! by get_last_error() to RDC_ERR_not_found.  If the cell exists,
    //! information is returned in a RDC_RawPoint structure.
    virtual void            get_raw_rdc3_point(
                                    uint16_t        range_bin,      //!< Input:  Range bin (0..R)  (spatial, not hardware mapped)
                                    uint16_t        azimuth_bin,    //!< Input:  Azimuth bin (0..A)
                                    uint16_t        elevation_bin,  //!< Input:  Elevation bin (0..E)
                                    uint16_t        doppler_bin,    //!< Input:  Doppler bin (0..D)
                                    RDC_RawPoint   &data            //!< Output: Structure to fill with point data
                                    ) = 0;

    //! Get the software exponent for the scan
    //
    //! The software exponent is an exponent that must be applied in addition
    //! to the per range-bin hardware exponent returned by get_rdc3_range_exponent().
    //! The software exponent is a function of the hardware fixed-point scaling
    //! factors that were calculated and configured for the scan.
    virtual int16_t         get_software_exponent(
                                    ) = 0;

    //! Return the noise floor magnitude (sigma_R) for a selected range bin.
    //
    //! Returns the noise floor magnitude for a selected range bin, with the
    //! hardware exponent for the range bin applied, but the scan's software
    //! exponent not applied.  The hardware exponent for the scan can be
    //! determined by calling RDC_ScanInstance:: get_rdc3_range_exponent(),
    //! whereas the software exponent can be determined by calling
    //! RDC_ScanInstance::get_software_exponent()
    virtual FLOAT           get_noise_floor_max(
                                    uint16_t range_bin              //!< Input:  Range bin (spatial, not hardware mapped)
                                    ) = 0;

    //! Return the hardware exponent for a selected range bin.
    //
    //! The returned exponent does not include the software exponent
    //! due to scale factors, and is applicable only to the raw
    //! Static Slice and Activations data.
    virtual int16_t         get_rdc3_range_exponent(
                                    uint16_t range_bin              //!< Input:  Range bin (spatial, not hardware mapped:  0..R-1)
                                    ) = 0;

    //! Return the SNR (dB) of any detected ridges, for all angle bins (vector)
    virtual int32_t*        get_ridge_snr_vec(
                                    ) = 0;

    //! Return the hardware mapped range bin index (or -1 if it doesn't exist)
    //! for a particular spatial range bin number.
    //
    //! Hardware data structures such as RDC1 and Static Slice store range
    //! bins in an order that is different than normal spatial order.
    //! In addition, some hardware range bins are not valid and do not map
    //! to any actual range.
    //! Additionally, spatial range bin index 0 is not always zero range:  the
    //! actual spatial range bin index always is relative to
    //! RDC_ScanParams::first_range_bin
    //! For example, if the range_bin_width = 0.5 meters, and first_range bin = 30,
    //! then get_hw_rbin(10) returns the hardware index for the bin at 20 meters.
    virtual int16_t         get_hw_rbin(
                                    uint16_t range_bin              //!< Input:  Range bin (spatial, not hardware mapped:  0..R-1)
                                    ) = 0;

    //! Get a pointer to a vector of integers that maps VRx indices from Y-major order to Rx-major order
    //! There are MAX_VRX entries in the vector.
    virtual const int8_t*   get_vrx_rev_map(
                                    ) = 0;

    //! Get a pointer to a vector of integers that maps VRx indices from Rx-major order to Y-major order
    //! There are MAX_VRX entries in the vector.
    virtual const int8_t*   get_vrx_map(
                                    ) = 0;

    //! Get the current number of VRx.
    //
    //! This number is the product of the number of active Transmitters and active Receivers.
    //! It includes the VRx that are marked as inactive in the beamforming.
    virtual uint16_t        get_num_vrx(
                                    ) = 0;

    //! Calculate the steering vector for a specified angle of arrival.
    //
    //! The stv parameter must point to a buffer able to store a vector of
    //! MAX_VRX cint16 elements.  The actual number of elements written
    //! into the buffer depends on the number of VRx for the current
    //! antenna config, as returned by get_num_vrx(), and whether
    //! active_only is true or false.
    virtual void            calc_steering_vector(
                                    cint16 *stv,                   //!< Output: Steering vector
                                    FLOAT   azimuth,               //!< Input:  Azimuth angle (radians)
                                    FLOAT   elevation,             //!< Input:  Elevation angle (radians)
                                    FLOAT   range,                 //!< Input:  Range (meters), or 0 for far-field STV
                                    bool    y_major = false,       //!< Input:  If true, store STV in Y-major VRx order instead of Rx-major
                                    bool    active_only = false    //!< Input:  If true, store STV for only active VRx
                                    ) = 0;

#if WITH_UHDP
    virtual uint32_t        get_env_datagram_buffer_max_length(
                                    ) const = 0;

    virtual void*           get_env_datagram_buffer(
                                    ) = 0;

    virtual bool            send_env_datagram_buffer(
                                    uint32_t    msg_type,
                                    uint32_t    msg_length
                                    ) = 0;
#endif

    //! Accessor for raw RDC1 buffer (pointer and length)
    //
    //! This method is intended to be used to capture RDC1 for subsequent playback.
    //! This buffer, together with the RDC1 Exponents buffer and the RDC1 Info buffer must
    //! be captured in order to later replay a scan from a saved RDC1
    virtual void            get_rdc1_raw_buffer(
                                    void       *&ptr,               //!< Pointer to RDC1 buffer
                                    uint32_t    &sizebytes          //!< Length of RDC1 buffer, in bytes
                                    ) = 0;

    //! Accessor for raw RDC1 Exponents buffer (pointer and length)
    //
    //! This method is intended to be used to capture RDC1 for subsequent playback.
    //! This buffer, together with the RDC1 buffer and the RDC1 Info buffer must
    //! be captured in order to later replay a scan from a saved RDC1
    virtual void            get_rdc1_exp_buffer(
                                    void       *&ptr,               //!< Pointer to RDC1 Exponents buffer
                                    uint32_t    &sizebytes          //!< Length of RDC1 Exponents buffer, in bytes
                                    ) = 0;

    //! Accessor for raw RDC1 Info buffer (pointer and length)
    //
    //! This method is intended to be used to capture RDC1 for subsequent playback.
    //! This buffer, together with the RDC1 buffer and the RDC1 Exponents buffer must
    //! be captured in order to later replay a scan from a saved RDC1
    //! This buffer contains hardware scale factors for the Doppler processing and
    //! Beamforming hardware (FEU & RAU) to use to process this RDC1.  It also
    //! contains Thresholds for the hardware Activations filter (NSU).
    //! This information is needed by the hardware to properly play back the
    //! captured RDC1.
    virtual void            get_rdc1_inf_buffer(
                                    void       *& ptr,              //!< Pointer to RDC1 Information buffer
                                    uint32_t    & sizebytes         //!< Length of RDC1 Information buffer, in bytes
                                    ) = 0;

    //! Get scan end timestamp
    //
    //! Returns value returned by uh_read_raw32_clock_ticks(), which wraps
    //! at MAX_UINT32
    virtual uint32_t        get_scan_end_timestamp(
                                    ) const = 0;

    //! Get chip temperature when scan started
    //
    //! On Sabine-B, the Digital-die temperature measured by the PVT hardware
    //! is returned.  On Sabine-A, the die temperature measured by A-Probe is
    //! returned.
    virtual FLOAT           get_scan_chip_temperature(
                                    ) const = 0;

    //! Get board temperature when scan started
    //
    //! The board temperature measured by the Environment method
    //! ModuleConfig::get_last_board_temperature()
    virtual FLOAT           get_scan_board_temperature(
                                    ) const = 0;

    virtual bool            fetch_adc_cdata_correction(
                                    CDataCalBData& cdata
                                    ) = 0;

    //! Read scan error status register (interrupt_status_register) in various blocks
    //! and report errors
    virtual bool            generate_rspss_error_logs() = 0;

    //! Release a reference to this RDC_ScanInstance.
    //
    //! When the last reference has been released, the RDC_ScanInstance is
    //! returned to the free list so that it can be reused for subsequent
    //! scans.
    virtual void            release() = 0;
};


//! Class for managing a Scan Configuration
//
//! After setting up a Frame Configuration with N types of scans using
//! RDC_FrameConfig::setup_scan_loop(), pointers to the N automatically
//! created RDC_Scan_Config instances can be gotten using
//! RDC_FrameConfig::sc_instance().  The methods of this class can be used
//! to further update the Scan Configuration, for example, to request various
//! additional measurements such as D-Probe, A-Probe, DC Measurement,
//! RDC1 capture, ADC Capture, RDC2 Zero Doppler capture.
class RDC_ScanConfig
{
public:

                            RDC_ScanConfig() {}

    UHVIRTDESTRUCT(RDC_ScanConfig)


    // ====== APIs that are only effective if called BEFORE RDC_FrameConfig::configure() ======


    // Each of these features must be enabled prior to configuring the engine
    // structures, because they affect how memory is allocated or how hardware
    // is configured


    //! Request capture of ADC data
    //
    //! Allocate storage and configure the scan to collect ADC data. All other
    //! scan outputs should be considered invalid; these are special measurement
    //! scans. Must be called before configure()
    virtual void            request_adc_capture(
                                    RDC_ADC_CaptureMode     mode,
                                    uint16_t                rx_bitmap,
                                    bool                    sel_rx_tx,
                                    uint8_t                 bit_width,
                                    uint32_t                samples_per_channel
                                    ) = 0;

    //! Request capture of RDC2 Zero Doppler data
    //
    //! Must be called before enable_scannnig(), it allocates storage and
    //! configures the software post-proc to calculate RDC2 zero-doppler for
    //! the specified range bins every scan, so long as the warmup status is disabled.
    //! Must be called before configure().  The number of range bins captured
    //! is (1+(2*rb_halfwidth)).
    virtual void            request_rdc2_zero_doppler(
                                    int16_t         rb_center_bin,      //!< Index of the center range bin to be captured
                                    int16_t         rb_halfwidth        //!< Number of range bins on either side of the center bin to be captured
                                    ) = 0;

    //! Request capture fo RDC1 data
    //
    //! Must be called before enable_scannnig(), it prevents each scan's RDC1
    //! buffer from being reused until the user releases the RDC_ScanInstance
    //! Must be called before configure()
    virtual void            request_rdc1_capture(
                                    bool            enable              //!< If true, enable RDC1 capture
                                    ) = 0;


    //! Enable operation in digital playback mode
    //
    //! Must be called before configure()
    virtual void            set_playback_mode(
                                    int8_t*            samples,           //!< ADC output samples for 8 Rx
                                    uint8_t*           codes,             //!< PRN code bits for 8 Tx
                                    uint32_t           samples_len,       //!< Number of bytes of sample data
                                    uint32_t           codes_len,         //!< Number of bytes of code data
                                    bool               repeat_pulse_data, //!< Whether to repeat first pulse data for all pulses
                                    RHAL_DummyScanMode synth_input        //!< Sabine-A only parameter
                                    ) = 0;

    //! Request a coredump of hardware registers for debugging
    //
    //! When hardware coredump is enabled, the configuration and status registers
    //! of the RSPSS hardware units involved with scan and/or proc are stored and
    //! made available from the RDC_ScanInstance. Requesting scan core dumps has
    //! the side effect of keeping RDC1 available until the RDC_ScanInstance is
    //! released. Must be called before configure()
    virtual void            request_hardware_coredump(
                                    bool                scan,           //!< If true, enable coredump for HW Front End
                                    bool                proc            //!< If true, enable coredump for HW Back End
                                    ) = 0;


    // ====== APIs that are only effective if called AFTER RDC_FrameConfig::configure() ======


    //
    //! Get the Antenna Set parameters
    //
    //! This function should only be called after RDC_FrameConfig::configure(),
    //! otherwise, the returned structure will be invalid.
    virtual AntennaSetParameters *get_antenna_set_parameters(
                                    ) = 0;

    //! Request capture of A-Probe measurement
    //
    //! Must be called AFTER RDC_FrameConfig::configure().
    //! There is a limit to the number of A-Probe requests which
    //! can be performed each scan, defined by the engine.  Returns
    //! false if it was unable to add an additional request.  The results
    //! from completed probes will be available from each RDC_ScanInstance.
    //! Has no effect if called before configure()
    virtual bool            request_aprobe(
                                    const RDC_AprobeRequest& request    //!< A-Probe request descriptor
                                    ) = 0;

    //! Cancel all registered A-Probe requests for this RDC_ScanConfig.
    //
    //! This method has no effect if called before configure()
    virtual void            cancel_requested_aprobes(
                                    ) = 0;

    //! Set or clear the warm-up status flag.
    //
    //! When in warmup mode, all software post processing is disabled.
    //! The full scan and proc are completed and scale factors are
    //! allowed to converge.  Has no effect if called before configure()
    virtual void            set_warmup_status(
                                    bool enable             //!< If true, enable warm-up mode, else disable
                                    ) = 0;


    // The following correction methods should generally be called by the
    // CorrectionManager in response to RDC_FrameConfig::configure(). They should
    // only be called manually when over-riding the standard corrections, or
    // when you do not wish for them to be calibrated on demand.

    //! Apply a mask of transmitters to be left powered off while scans are running (HW-12)
    //! Equivalent to setting tx_power_mask in initial RDC_ScanDescriptor
    virtual void            set_tx_power_off_mask(
                                    uint16_t tx_off_mask                    //!< Input: mask of transmitters to power down
                                    ) = 0;

    //! Set or update the QILO correction parameters
    virtual void            set_qilo_correction(
                                    const QiloCalBData&     qilo_cal_data   //!< Input:  Correction data structure
                                    ) = 0;

    //! Get the current QILO correction parameters
    virtual void            get_qilo_correction(
                                    QiloCalBData&           qilo_cal_data   //!< Output: Correction data structure
                                    ) = 0;

    //! Set or update the Receiver DC correction parameters
    virtual void            set_dc_correction(
                                    const DCCalBData&       dc_cal_data     //!< Input:  Correction data structure
                                    ) = 0;

    //! Get the current Receiver DC correction parameters
    virtual void            get_dc_correction(
                                    DCCalBData&             dc_cal_data     //!< Output: Correction data structure
                                    ) = 0;

    //! Set or update the virtual receiver planar correction parameters, returns
    //! the amount of uncorrected range error in picoseconds
    virtual FLOAT           set_vrx_correction(
                                    const VrxAlignCalBData& vrx_cal_data    //!< Input:  Correction data structure
                                    ) = 0;

    //! Get the current virtual receiver planar correction parameters, including
    //! the most recent computed delays in the ADI
    virtual void            get_vrx_correction(
                                    VrxAlignCalBData&       vrx_cal_data    //!< Output: Correction data structure
                                    ) = 0;

    //! Set or update the transmitter gain settings
    virtual void            set_tx_gains(
                                    const TxGainCalData& tx_gain_data       //!< Input:  TX gains data structure
                                    ) = 0;

    //! Set or update the receiver gain settings
    virtual void            set_rx_gains(
                                    const RxGainCalData& rx_gain_data       //!< Input:  RX gains data structure
                                    ) = 0;

    //! Set or update the transmitter bandwidth settings
    virtual void            set_tx_bandwidth(
                                    const TxBwCalData& tx_bw_data           //!< Input:  TX bandwidth data structure
                                    ) = 0;

    //! Set or update the receiver bandwidth settings
    virtual void            set_rx_bandwidth(
                                    const RxBwCalData& rx_bw_data           //!< Input:  TX bandwidth data structure
                                    ) = 0;

    //! must be called after configure() but before request_ldo_cal().  This
    //! function initializes the LDO calibration engine and specifies where the
    //! calibration outputs are to be stored as well as the status results.
    virtual void            register_ldo_cal(
                                    LDOCalBData& ldo_data,
                                    LDOCalBData& ldo_status,
                                    RegulatorVoltageRecord& input_v,
                                    RegulatorVoltageRecord& final_v,
                                    EventEnum    callback
                                    ) = 0;

    //! perform a calibration on a single LDO, optionally override the
    //! calibration params, must be called after register_ldo_cal(), must be
    //! called after configure(). This requested LDO calibration is triggered
    //! when the next scan is started, and it only runs once.
    virtual void            request_ldo_cal(
                                    uint16_t ldo_num,
                                    uint16_t channel,
                                    const LdoCalibrationParameters* override_params
                                    ) = 0;

    //! must be called after configure() but before request_ldo_cal().  This
    //! function initializes the LDO calibration engine and specifies where the
    //! calibration outputs are to be stored as well as the status results.
    virtual void            register_peak_det_cal(
                                    PeakDetCalData& peak_det_data,
                                    PeakDetCalData& peak_det_status,
                                    EventEnum    callback
                                    ) = 0;

    //! perform a calibration on a single LDO, optionally override the
    //! calibration params, must be called after register_ldo_cal(), must be
    //! called after configure(). This requested LDO calibration is triggered
    //! when the next scan is started, and it only runs once.
    virtual void            request_peak_det_cal(
                                    uint16_t channel,
                                    EventEnum    callback
                                    ) = 0;

    //! directly control the rotation applied by RSU1 (for calibration purposes only).
    //! Must be called after configure(). rotate_pattern_count = 0 disables RSU1 rotations
    virtual void            set_rsu1_rotations(
                                    uint8_t  rotate_pattern_count,           //!< 5bit count of symbols to use (up to 32)
                                    uint32_t rotate_prog0,                   //!< first 16 2-bit symbols
                                    uint32_t rotate_prog1                    //!< last 16 2-bit symbols
                                    ) = 0;

    //! override the IQ correction and directly write to the MIQ rotation table
    //! (for calibration purposes only). Must be called after configure()
    //! num_rotations = 0 places the MIQ in bypass mode (disabled)
    virtual void            set_miq_rotations(
                                    uint32_t num_rotations,                 //!< Input: number of rotations specified per RX
                                    IQCorrMatrix rotation_matrix[NUM_RX_PER_BANK] //!< Input: array [num_rotations][NUM_RX_PER_BANK]
                                    ) = 0;

    //! Stops transmitting signals on the transmitter, only applicable if the
    //! scan desc had the RDCSCF_SIGGEN_ENABLE flag set
    virtual void            siggen_stop() = 0;


    //! Request a peak detection measurement to be performed during each scan.
    //! The result is returned in UhdpScanInformation.  Only one TX bit in the
    //! channel_map can be set in each request. Specify channel_map of 0 to
    //! disable future peak detect measurements
    virtual void            request_peak_detect_measurement(
                                uint16_t channel_map,
                                PeakDetector measure_point
                                ) = 0;

    //! Set or update the TCS (Transmitter Carrier Suppression) correction parameters
    virtual void            set_tcs_correction(
                                    const TCSCalData&       tcs_cal_data    //!< Input:  Correction data structure
                                    ) = 0;

    //! Get the current TCS (Transmitter Carrier Suppression) correction parameters
    virtual void            get_tcs_correction(
                                    TCSCalData&             tcs_cal_data    //!< Output: Correction data structure
                                    ) = 0;

    //! Set or update the IQ correction parameters
    virtual void            set_iq_correction(
                                    const IQCalData&        iq_cal_data     //!< Input:  Correction data structure
                                    ) = 0;

    //! Get the current IQ correction parameters
    virtual void            get_iq_correction(
                                    IQCalData&              iq_cal_data     //!< Output: Correction data structure
                                    ) = 0;


    //! Get the sampling to RSU1 output ratio
    virtual RDC_error       get_sampling_rate_to_rsu1Out_ratio(
                                   uint32_t& val             //!< Output:  Fsamp to RSU1output ratio
                                   ) = 0;


    // ====== APIs that are safe to be called at any time ======

    //! Request capture of DC levels, measured shortly after the ADC
    //
    //! DC levels are measured separately for I and Q channels of all
    //! ADC lanes of all receivers during each scan. Can be called with NULL to
    //! cancel DC measurements.
    virtual void            request_dc_measure(
                                    const RDC_DCMeasure_config* dc_measure_config   //!< Input: Configuration for DC measurement
                                    ) = 0;

    //! Request capture of D-Probe measurement
    //
    //! Configure 0, 1, 2, or 3 digital probes to be performed next sequential
    //! scans. Can be called with NULL pointers or with dprobe_cnt = 0 to cancel
    virtual void            request_dprobes(
                                    const RDC_DProbe_corr_config*   list_dprobe_corr_config,
                                    const RDC_DProbe_rms_config*    list_dprobe_rms_config,
                                    const RDC_DProbe_iq_config*     list_dprobe_iq_config,
                                    const uint8_t*                  rx_select,
                                    const uint32_t                  dprobe_cnt
                                    ) = 0;

    //! Enable or disable analog (RF) loopback, select the transmitter and
    //! receiver to loopback, and specify the (slow) rotation to apply to the
    //! signal. Can be called at before configure() or any time between scans
    virtual void            configure_rf_loopback(
                                    bool                  enable,       //!< enable or disable RF loopback
                                    uint8_t               tx_select,    //!< select the source transmitter (HW12)
                                    uint8_t               rx_select,    //!< select the target receiver
                                    RDC_Loopback_Rotation rotation      //!< 250Mhz, 125Mhz, 62.5Mhz, or bypass
                                    ) = 0;

    //! Control the hardware Doppler shifting operation
    //
    //! The interpretations of the "value" parameter depends on the mode
    //! DSM_ABSOLUTE_OFFSET: "value" is the absolute velocity shift (m/s)
    //! DSM_EGO_FRACTION:    "value" is a number from 0..1, and indicates the fraction of ego-velocity to shift the Doppler velocity by
    //! DSM_EGO_OFFSET:      "value" is the velocity (m/s) offset from ego-velocity
    //
    virtual void           set_ego_velocity_shift(
                                    RDC_Doppler_Shift_Update_Mode  update_mode,     //!< Doppler-shift mode selector
                                    RDC_Doppler_Shift_RDC_Mode     rdc_shift_mode,  //!< RDC shift mode selector
                                    FLOAT                          value            //!< Parameter, interpretation depends on mode
                                    ) = 0;

    //! Phased array transmitter array steering azimuth angle (radians)
    virtual void            set_phased_array_aziumth(
                                    FLOAT                 pa_az         //!< Input:  Azimuth angle (radians) to steer the Tx beam to
                                    ) = 0;

    //! Request dense RDC3 data from subsequent scans. The desc describes the
    //! region of interest and number_of_scans describes the duration. Returns
    //! a handle which may be canceled prior to the scheduled duration by
    //! calling cancel_extra_data_request()
    virtual uint32_t        request_extra_data(
                                    INT                               number_of_scans,  //!< Input:  Number of scans for request to be active
                                    const RDC_extra_data_descriptor  &desc              //!< Input:  Descriptor for region of interest
                                    ) = 0;

    //! Cancel an extra data request, providing the handle which was returned by
    //! request_extra_data()
    virtual void            cancel_extra_data_request(
                                    uint32_t            request_handle  //!< Input:  Handle of request to cancel, as returned by request_extra_data()
                                    ) = 0;

    //! Request raw RDC3 data to be kept after scan post-processing, so static
    //! slice and raw activations are available to the environment. Keeping the
    //! RDC3 data available for a longer time can have a negative impact on scan
    //! rate performance. Can be called at any time, even while scanning
    virtual void            request_raw_rdc3(
                                    bool                enable          //!< Input:  If true, keep RDC3 raw data until RSI is released
                                    ) = 0;

    //! Request a point cloud to be generated from dynamic RDC3 (activations),
    //! this method can be called at any time to toggle the generation of point
    //! cloud data from future scans
    virtual void            request_point_cloud(
                                    bool                enable          //!< Input:  If true, enable point cloud generation
                                    ) = 0;

    //! Detection, clutter image, point cloud, side-lobe and ridge suppression thresholds.
    //! Can be called at any time, even while scanning
    virtual void            setup_detection_thresholds(
                                    const RDC_ThresholdControl      &thresh_ctrl     //!< Input:  New detection threshold values to use
                                    ) = 0;

    //! Special purpose callback event (meant for calibration scans); triggered
    //! event handler is passed RDC_ScanInstance pointer, which must be released
    //! when processing is completed. Failure to release in a timely matter will
    //! cause delays in scanning. When this event handler is set, it bypasses
    //! event notifications that normally get sent to the Object Layer
    virtual void            set_scan_complete_notification(
                                    EventEnum           event,          //!< Input:  Event ID to post upon completion
                                    bool                notify_uhdp     //!< Input:  If true, send completed calibration scans to UhDP
                                    ) = 0;

    // Replay a sparse RDC3 "blob" created by calling
    // RDC_ScanInstance::generate_scan_blob() and serializing the address/length
    // pairs. This will only operate correctly if this RDC_ScanConfig was
    // configured in the exact same scan configuration as the captured scan.
    // Scan post-processing is performed on the sparse RDC3 from the blob and,
    // generating detections and point cloud points and a clutter image.
    // Eventually an RDC_ScanInstance will be emitted to the object layer in the
    // normal fashion, just as if the scan has just completed.  This function
    // should not be called when scans are running on the radar hardware, and
    // only one scan "blob" at a time should be in the replay state.
    virtual void            replay_scan_blob(
                                    const void*         blob_buffer,
                                    uint32_t            blob_buffer_max_size
                                    ) = 0;
};


//! Class for managing a Frame Configuration
//
class RDC_FrameConfig
{
public:

                            RDC_FrameConfig() {}

    UHVIRTDESTRUCT(RDC_FrameConfig)

    //! Set default formats for clutter images
    virtual void            set_clutter_image_format(
                                    RDC_clutter_image_format    az_only,    //!< Input:  CI format for Azimuth-only scans
                                    RDC_clutter_image_format    az_el       //!< Input:  CI format for "2D" scans
                                    ) = 0;

    //! Set the mode of controlling analog power. Must be called before
    //! configure() to be effective
    virtual void            set_analog_power_mode(
                                    RDC_AnalogPowerMode mode                //!< Input: mode of controlling analog power
                                    ) = 0;

    //! Set interval between frame starts.
    //
    //! When using deterministic loop, this specifies the interval
    //! of the entire frame (scan loop). The specified interval must
    //! be larger than the total dwell time of all scans (plus some
    //! slack time for intra-scan calibrations).  Setting the interval
    //! to zero causes the radar to always start the next frame as soon
    //! as possible, for maximum frame throughput.
    virtual void            set_frame_interval(
                                    uint32_t    interval_us         //!< Input:  Interval between frame starts (microseconds)
                                    ) = 0;

    //! Absolute base time (in 64bit hardware clock) of frame intervals
    //! user can specify a new frame interval timebase before or during scanning,
    //! but set_frame_interval() must be called before set_frame_interval_timebase()
    virtual void            set_frame_interval_timebase(
                                    uint64_t    timebase            //!< Input:  Time base (64-bit HW clock units)
                                    ) = 0;

    //! Configure and create one or more RDC_ScanConfig instances
    virtual void            setup_scan_loop(
                                    const RDC_ScanDescriptor*   desc,               //!< Input:  Pointer to array of scan descriptors
                                    uint32_t                    scan_loop_count     //!< Input:  Total number of scan descriptors
                                    ) = 0;

    //! Return the number of configured scans.
    //
    //! This number reflects the scan_loop_count paramter passed to
    //! setup_scan_loop().  Zero is returned if setup_scan_loop() has
    //! not been called yet.
    virtual uint32_t        get_scan_loop(
                                    ) const = 0;

    //! Get a pointer to a RDC_ScanConfig instance.
    //
    //! This method can be used to apply configurations and corrections
    //! to a RDC_ScanConfig.  The parameter "scan_index" is the index of the
    //! corresponding RDC_ScanDescriptor in the call to setup_scan_loop(),
    //! which must have already been called, otherwise NULL is returned.
    virtual RDC_ScanConfig* sc_instance(
                                    uint32_t    scan_index      //!< Input:  Index of the RDC_ScanConfig descriptor
                                    ) = 0;

    //! Perform all internal configuration the RDC_FrameConfig.
    //
    //! This method can only be called after setup_scan_loop(). It validates
    //! the scan configurations, allocates all engine data structures required
    //! to run the scans, and collects corrections from the CorrectionManager
    //! Once this method returns, it is safe to call the "AFTER configure"
    //! methods of RDC_ScanConfig.
    virtual void            configure() = 0;

    //! Start performing scans using this configuration. Returns the number of
    //! scans scheduled, or num_scans is -1 if continuous. The start of the first
    //! scan may be deferred by the CorrectionManager.
    virtual INT             enable_scanning(
                                    INT         num_scans       //< Number of scans to run, or -1 to keep running scans until stopped
                                    ) = 0;

    //! If called with a non-NULL event handle, all scans run by this frame
    //! config will be RDC1 playback scans. The registered callback will be
    //! called each time an RDC1 buffer needs to be provided for replay. The
    //! trigger function argument will be an RHAL_RDC1Info structure describing
    //! the RDC1 addresses and sizes.  Once those RDC1 and exponent buffers have
    //! been filled, your function must call rdc1_playback_continue(). If called
    //! with event handle EV_NULL, RDC1 playback is disabled
    virtual void            set_rdc1_playback_enable(
                                    EventEnum rdc1_request_callback
                                    ) = 0;

    //! Must be called by RDC1 playback trigger event after the RDC1 buffer and
    //! the RDC1 exponent buffer has been filled.  If the user wants to override
    //! the proc gain management, it should write the RDC1 info previously
    //! returned by get_rdc1_inf_buffer() into a TFS file named "rdc1_replay_info"
    //! prior to calling this function
    virtual void            rdc1_playback_continue() = 0;

    //! Returns the address of the last RDC1 buffer used for replay
    virtual uint8_t*        get_current_replay_rdc1_buffer() = 0;

    //! Returns the address of the last RDC1 exponents buffer used for replay
    virtual uint8_t*        get_current_replay_rdc1_exponents_buffer() = 0;

    //! Stop performing scans (required for changing modes)
    virtual void            disable_scanning() = 0;

    //! Release this RDC_FrameConfig (automatically disables scanning)
    virtual void            release() = 0;
};


//! Antenna descriptor
//
//! Instances of this class is passed to RDC_Layer::set_antenna_config(),
//! prior to initializing of the RDC_Layer via a call to RDC_Layer::init().
//! Each RDC_AntennaDesc describes one physical antenna (Tx or Rx), and
//! provides information about its position, gain, and directivity.
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


//! Primary interface to the RDC API.
//
//! A singleton of this class  provides the initial entry point
//! into the RDC API.  The singleton is accessed via the
//! static RDC_Layer::instance() method.
class RDC_Layer
{
public:

                            RDC_Layer() {}

    UHVIRTDESTRUCT(RDC_Layer)


    //! Get reference to the RDC_Layer singleton
    static RDC_Layer &      instance();


    //! Get error status from last RDC API call
    //
    //! This method should be called after every method defined in this header file.
    //! This includes all methods of the following classes:  RDC_Layer, RDC_FrameConfig,
    //! RDC_ScanConfig, RDC_ScanInstance, RDC_DetectionIterator, RDC_StaticImage,
    //! RDC_ClutterImage, RDC_EgoVelocity, and RDC_PointCloud.
    virtual RDC_error       get_last_error() = 0;

    //! Configure an antenna group
    //
    //! This method must be called BEFORE RDC_Layer::init() for each physical
    //! antenna (both Tx and Rx), and is typically called from the init()
    //! method of the AntennaConfig implementation class being used.
    virtual void            set_antenna_config(
                                    RDC_AntennaDesc &desc       //!< Antenna descriptor
                                    ) = 0;

    //! Initialize the RDC_Layer
    //
    //! This method must be called after RDC_Layer::set_antenna_config() has
    //! been called for each physical Tx and Rx antenna.
    //! Initialization is an asynchronous process, and is not complete when
    //! this method returns.  Rather, an event will be posted, if notification
    //! has been requested by calling the register_scanning_ready_notification()
    //! method.
    virtual void            init(
                                    const RDC_LayerDesc& desc   //!< Descriptor containing initialization parameters for RDC_Layer
                                    ) = 0;

    //! Create new frame configuration
    virtual RDC_FrameConfig *create_frame_config(
                                    ) = 0;

    //! Get the currently active frame configuration (i.e.: the one currently running scans)
    virtual RDC_FrameConfig *get_current_frame_config(
                                    ) = 0;

    //! Provides an external estimate of radar linear & angular velocity (e.g.: from CAN bus)
    virtual void            set_ego_velocity(
                                    vec3f_t linear,             //!< Linear velocity vector of the world, in the radar's coordinate system
                                    vec3f_t angular,            //!< Angular velocity quaternion of the world, in the radar's coordinate system
                                    uint32_t age_us             //!< Age of the velocidy data, in microseconds, at the time of the API call
                                    ) = 0;

    //! Configure the parameters for the PID controller that manages the HW Activation thresholds
    virtual void            set_cfar_pid_constants(
                                    FLOAT Kp,                   //!< The Proportional parameter for the PID controller
                                    FLOAT Ki,                   //!< The Integral parameter for the PID controller
                                    FLOAT Kd,                   //!< The Differential parameter for the PID controller
                                    uint32_t num_act            //!< Target number of activations: the PID controller will try to achieve the specified number of activations
                                    ) = 0;

    //! Control whether ego-velocity histograms are computed and output for each scan
    virtual void            enable_ego_velocity_detection(
                                    RHAL_Ego_Velocity_Mode mode //!< Mode selector for internal ego-velocity estimation
                                    ) = 0;

    //! Register event ID that is posted whenever basic processing for a scan has completed
    virtual void            register_scan_complete_notification(
                                    EventEnum event
                                    ) = 0;

    //! Unegister event ID that is posted whenever basic processing for a scan has completed
    virtual void            unregister_scan_complete_notification(
                                    EventEnum event
                                    ) = 0;

    //! Register event ID that is posted whenever MUSIC processing has completed
    virtual void            register_music_complete_notification(
                                    EventEnum event
                                    ) = 0;

    //! Unregister event ID that is posted whenever MUSIC processing has completed
    virtual void            unregister_music_complete_notification(
                                    EventEnum event
                                    ) = 0;

    //! Register event ID that is posted whenever the (deferred) release
    //! of a RDC_FrameConfig has completed.
    virtual void            register_frame_config_release_notification(
                                    EventEnum event
                                    ) = 0;

    //! Unegister event ID that is posted whenever the (deferred) release
    //! of a RDC_FrameConfig has completed.
    virtual void            unregister_frame_config_release_notification(
                                    EventEnum event
                                    ) = 0;

    //! Register event ID that is posted at startup, after RDC_Layer::init()
    //! has been called, to indicate that the RDC Layer is ready for normal
    //! operations.  At this point, it is safe to create and configure
    //! RDC_FrameConfig instances. and run scans.
    virtual void            register_scanning_ready_notification(
                                    EventEnum event
                                    ) = 0;

    //! Unegister event ID that is posted to indicate that the RDC Layer
    //! is ready for normal operations.
    virtual void            unregister_scanning_ready_notification(
                                    EventEnum event
                                    ) = 0;


    //! Request an A-Probe measurement.
    //
    //! If the priority is AP_SCAN_IDLE, the measurement guaranteed
    //! to occur at a time when a scan is NOT running.
    //! Otherwise, it is not synchronized with running a scan.  It can occur
    //! anytime, regardless  of whether a scan is running or not.
    //! In order to request an A-Probe during a scan, instead use the
    //! RDC_ScanConfig::request_aprobe() method of the RDC_ScanConfig class.
    //! When the completion callback is asynchronously triggered, the trigger
    //! function will be passed a pointer to a struct MeasurementRequest, and its
    //! "value" field will be valid. If a scan is in progress, pending Aprobes
    //! are executed in strict priority order: high priority, scan-registered
    //! Aprobes, then best effort. If a scan is not in progress, pending Aprobes
    //! are executed in priority order: high priority, scan idle, then best
    //! effort. There are queues per bus, so multiple requests can be measuring
    //! simultaneously.
    virtual bool            request_aprobe(
                                    EventEnum                completion,    //!< Event ID to be posted when A-Probe measurement completes
                                    RDC_AprobePriorities     pri,           //!< Priority and scheduling information
                                    const RDC_AprobeRequest& request        //!< A-Probe request descriptor
                                    ) = 0;

    virtual bool            request_peakdet(
                                    EventEnum                completion,    //!< Event ID to be posted when Peak Detector measurement completes
                                    const RDC_PeakDetRequest& request       //!< Peak Detector request descriptor
                                    ) = 0;

    //! Measure chip temperature
    //
    //! Returns the result of the most recent measurement of
    //! internal chip temperature (celcius).
    virtual FLOAT           get_chip_temperature(
                                    ) = 0;

    //! Get the number of chip input voltages that can be measured
    //
    //! The valid values for the selector parameter to get_input_voltage()
    //! range from 0 to one less than the number returned by this function.
    virtual INT             get_num_input_voltages(
                                    ) = 0;

    //! Measure chip input voltages
    //
    //! Returns the result of the most recent measurement of the voltage (in Volts)
    //! along with the name of the voltage rail.
    virtual FLOAT           get_input_voltage(
                                    INT             selector,           //!< Input:  Select which voltage to measure
                                    const CHAR *   &name                //!< Output: Name of voltage rail
                                    ) = 0;

    //! Sets microcal enable state, returns previous state
    virtual bool            set_enable_microcal(
                                    bool enable
                                    ) = 0;

    //! Apply a new LDO correction
    virtual void            apply_ldo_correction(
                                    const LDOCalBData& ldo_cal
                                    ) = 0;

    //! Apply a new ADC CData correction
    virtual void            apply_cdata_correction(
                                    const CDataCalBData& cdata
                                    ) = 0;

    //! Return the current default carrier frequency (in GHz)
    virtual FLOAT           get_default_carrier_frequency(
                                    ) const = 0;

    virtual RDC_Sampling_rate get_fixed_adc_sample_rate(
                                    ) const = 0;

    virtual RDC_DAC_Rate    get_fixed_dac_rate(
                                    ) const = 0;

    //! Measure a-die temperature
    //
    //! Returns the result of the most recent measurement of
    //! internal analog die temperature (celcius).
    virtual FLOAT           get_adie_temperature(
                                    ) const = 0;

    //! Emit log messages detailing the current radar status.
    virtual void            report_status(
                                    ) const = 0;

    //! Plot ASCII chart of antenna positions into the radar log
    virtual void            plot_antenna_positions(
                                    ) = 0;
};

SRS_CLOSE_NAMESPACE()

#endif // SRS_HDR_RDC_H
