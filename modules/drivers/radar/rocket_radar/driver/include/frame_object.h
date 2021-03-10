// Copyright (C) Uhnder, Inc. All rights reserved. Confidential and Proprietary - under NDA.
// Refer to SOFTWARE_LICENSE file for details
#pragma once

#include "modules/drivers/radar/rocket_radar/driver/system-radar-software/env-reference/coredefs/uhnder-common.h"
#include "modules/drivers/radar/rocket_radar/driver/system-radar-software/engine/common/eng-api/uhtypes.h"
#include "modules/drivers/radar/rocket_radar/driver/system-radar-software/env-reference/coredefs/uhunistd.h"

class SensorGroup;
class ScanObject;
class FrameObject;
class ScanSerializer;
struct Track;

struct FrameDetection
{
    // global coordinates relative to platform in cartesian, meters
    vec3f_t     position_m;

    // polar coordinates as witnessed by a sensor
    float       range;         //!< in meters
    float       azimuth;       //!< bearing in radians
    float       elevation;     //!< in radians
    float       doppler_mps;   //!< doppler velocity as witnessed by source sensor

    float       snr_decibels;  //!< target return strength as signal to noise ratio in decibels
    float       magnitude;     //!< signal power (dBFS)
    float       rcs;           //!< radar-cross-section, dBsm

    uint16_t    flags;         //!< detection flags, plus 0x8000 set if originated from az-el scan
    uint8_t     sensor_idx;    //!< index of sensor that witnessed the detection (direction of doppler)
    uint8_t     scan_idx;      //!< 0..scans_in_frame-1
};


class FrameObject
{
public:

    virtual ~FrameObject() {}

    // returns true if the frame contains the full complement of scans from all
    // sensors
    virtual bool            frame_is_complete() = 0;

    virtual ScanObject*     get_scan(uint32_t sensor_idx, uint32_t scan_idx) = 0;

    virtual uint32_t        get_num_scans_per_sensor() const = 0;

    virtual uint32_t        get_frame_number() const = 0;

    //! Returns the start (timeval) and length (interval_us) of time covered by
    //! this frame.
    virtual const timeval&  get_frame_time(uint32_t& interval_us) const = 0;

    virtual const vec3f_t&  get_platform_linear_velocity() const = 0;

    virtual const vec3f_t&  get_platform_angular_velocity() const = 0;

    virtual uint32_t        get_detection_count() = 0;

    virtual uint32_t        get_track_count() = 0;

    virtual const FrameDetection& get_detection(uint32_t det_idx) = 0;

    virtual const Track&          get_track(uint32_t track_idx) = 0;

    //! serialize Tracks and FrameDetections (does not serialize underlying
    //! scans, use SensorGroup::serialize_frame() if you need the underlying
    //! scans to be serialized).
    virtual void            serialize_data(ScanSerializer& ser) const = 0;

    //! Releases all frame data including any ScanObjects that have not yet been
    //! independently released
    virtual void            release() = 0;

    //! Factory method which reads a frame object (detections, ego vel, tracks)
    //! from a serializer
    static FrameObject*     deserialize(ScanSerializer& ser, uint32_t frame_number);
};


class FrameP2C
{
public:

    virtual ~FrameP2C() {}

    //! to update the entire image, use start_x=0, start_y=0, extent_x=dim_x, extent_y=dim_y
    //
    //! if the current view of the clutter image is restricted, you can save
    //! compute resources by only updating the visible region
    //
    //! returns false if the FrameObject is not from the same SensorGroup
    //! that created this FrameP2C, or the scans have changed.
    virtual bool            update_region(
            const FrameObject&  frame,
            uint32_t            start_x,
            uint32_t            extent_x,
            uint32_t            start_y,
            uint32_t            extent_y,
            bool                update_doppler_plane = false,
            bool                update_height_plane = false,
            bool                update_occupancy_plane = false) = 0;

    //! if create_cartesian_mapping() was called with asynchronous=true, then
    //! the returned FrameP2C object is not ready for use until this function
    //! returns true.
    virtual bool            cooking_completed() const = 0;

    //! Each image plane has the same dimensions, as returned here. These
    //! dimensions are derived from the sensor group's positions and orientations,
    //! the field of view of the configured scans, and the chosen pixels_per_range_bin
    virtual void            get_image_dimensions(uint32_t& dim_x, uint32_t& dim_y) const = 0;

    //! return the offset of the platform center within the image, the location of (0, 0)
    //! in the platform reference frame
    virtual void            get_platform_center_pos(uint32_t& x, uint32_t& y) const = 0;

    //! return the maximum doppler value (mps) that might be returned
    //! defined by the maximum doppler velocity witnessed by any static slice
    //! in any azimuth-only scan. Assume min doppler is -max doppler
    virtual float           get_max_doppler_value() const = 0;

    //! return the maximum height value (m) that might be returned
    //! defined by the maximum elevation field of view and max range
    //! in any azimuth-elevation scan. Assume min height is -max height
    virtual float           get_max_height() const = 0;

    //! 0 - not covered by any sensor
    //! 1 - noise floor
    //! > 1 - SNR in decibels
    //! the returned pointer is always the same for the life of the FrameP2C
    virtual const float*    get_clutter_magnitude_image() const = 0;

    //! returns doppler frequency as meters per second
    //
    //! Only pixels that have SNR in the magnitude clutter image will have
    //! valid height values.
    //
    //! the returned pointer is always the same for the life of the FrameP2C
    virtual const float*    get_clutter_doppler_image() const = 0;

    //! returns height height in meters for each pixel
    //
    //! Only pixels that have SNR in the magnitude clutter image will have
    //! valid height values.
    //
    //! the returned pointer is always the same for the life of the FrameP2C
    virtual const float*    get_clutter_height_image() const = 0;

    //! count of estimated occupancy?
    //! the returned pointer is always the same for the life of the FrameP2C
    virtual const float*    get_occupancy_probability_image() const = 0;

    //! release all resources
    virtual void            release() = 0;

    //! Factory Method
    //
    //! To create a combined clutter image for all sensors in a group, you
    //! must use this API to "cook" the mapping of polar clutter images into
    //! an output cartesian image.  The returned FrameP2C object can be used
    //! for the life of the SensorGroup.  It owns the image buffer storage.
    //
    //! multiple FrameP2C objects can be created at various zoom factors
    //! (pixels_per_range_bin) or you can create them on demand as the view
    //! zoom factor is changed.
    //
    //! Can return NULL if the provided frame does not have the full complement
    //! of scans or other similar errors. Caller must check for NULL return
    //! value
    //
    //! If asynchronous is true, the caller must wait for cooking_completed() to
    //! return true before using the returned FrameP2C to update images. The
    //! SensorGroup and FrameObjects passed as inputs are not used after this
    //! function returns, it is ok if they are released before cooking is
    //! complete.
    static FrameP2C* create_cartesian_mapping(
            const SensorGroup& sg,
            const FrameObject& frame,
            float pixels_per_meter,
            bool  asynchronous);
};
