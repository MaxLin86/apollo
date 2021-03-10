// Copyright (C) Uhnder, Inc. All rights reserved. Confidential and Proprietary - under NDA.
// Refer to SOFTWARE_LICENSE file for details
#pragma once

#include "modules/drivers/radar/rocket_radar/driver/system-radar-software/engine/scp-src/eng-api/rdc-scanctrl.h"
#include "modules/drivers/radar/rocket_radar/driver/system-radar-software/engine/scp-src/eng-api/rdc-threshctrl.h"
#include "modules/drivers/radar/rocket_radar/driver/system-radar-software/engine/scp-src/eng-api/uhdp-msg-structs.h"
#include "modules/drivers/radar/rocket_radar/driver/system-radar-software/engine/common/eng-api/logging-enums.h"


#include "modules/drivers/radar/rocket_radar/driver/system-radar-software/env-reference/coredefs/uhstdlib.h"
#include "modules/drivers/radar/rocket_radar/driver/system-radar-software/env-reference/coredefs/uhnder-common.h"
#include "modules/drivers/radar/rocket_radar/driver/system-radar-software/engine/common/eng-api/uhtypes.h"

class Connection;
class ScanObject;
class Detections;
class FrameObject;
class ScanSerializer;

enum LogDestination
{
    LD_DEV_NULL,
    LD_SHARED_LOG_FILE,
    LD_PRIVATE_LOG_FILE,
};

typedef void (*log_handler)(uint32_t sensor_idx, LogLevelEnum level, const char* message);

struct ConnectionLogDesc
{
    ConnectionLogDesc()
    {
        print_destination     = LD_SHARED_LOG_FILE;
        log_destination       = LD_SHARED_LOG_FILE;
        filter_level          = LL_WARN;
        user_log_handler      = NULL;
        private_log_filename  = NULL;
        sensor_name           = NULL;
    }

    LogDestination  print_destination;  //!< Where do you want radar print messages to go?
    LogDestination  log_destination;    //!< Where do you want radar log messages to go?

    //! if print_destination or log_destination are LD_PRIVATE_LOG_FILE, this
    //! filename is used for the private logs
    const char*     private_log_filename;

    //! name to use in log files, if unspecified it will be the sensor index number
    const char*     sensor_name;

    //! all log messages with level greater than or equal to filter_level will be
    //! sent to the user_log_handler if it is not NULL (regardless of log_destination)
    LogLevelEnum    filter_level;
    log_handler     user_log_handler;
};


class SensorGroup
{
public:

    virtual ~SensorGroup() {}


    // NOTE: All linear position and velocity variables are in SAE Coordinates,
    // in which +X is forward, +Y is right, and +Z is down. Units are in meters
    // or meters per second.
    //
    // For angular positions and velocities, X is roll, Y is pitch, and Z is yaw
    // Positive angular velocities are clockwise. Units are in radians or
    // radians per second.
    //
    // This is true for all Uhnder Radar APIs.


    //! Initialize sensor group. INS distance from the user's selected platform
    //! reference position (the origin of the world coordinates).
    //! if use_poll_thread is false, user is responsible for calling
    //! poll_completed_frame frequently.  frame_interval_us defines the rate at
    //! which FrameObjects are returned by the SensorGroup when scanning is active
    virtual void        initialize(bool use_poll_thread, uint32_t frame_interval_us, vec3f_t ins_position) = 0;

    //! Add a sensor (Connection) to the sensor group
    //! returns false if sensor limit is exceeded or connection failed
    virtual bool        add_sensor(uint32_t sensor_ip, const ConnectionLogDesc& log_desc) = 0;

    //! access methods for owned connections; do not poll scans from these
    //! raw connection objects, you will interfere with the SensorGroup
    virtual uint32_t    get_sensor_count() const = 0;
    virtual Connection* get_sensor_connection(uint32_t sensor_idx) = 0;

    //! By default the sensor position is learned from the mounting geometry in the
    //! radar's "modulecfg" flash file. But this API can override those settings.
    //! position_m is distance from the platform reference position. The Euler
    //! angles are applied in yaw (z-azimuth), pitch (y-elevation), roll (x) order
    virtual void        set_sensor_position(uint32_t sensor_idx, vec3f_t position_m, float yaw_deg, float pitch_deg, float roll_deg) = 0;

    //! Alternative method to assign the sensor position and pose directly as quaternion
    virtual void        set_sensor_position(uint32_t sensor_idx, vec3f_t position_m, quatf_t pose) = 0;

    //! Update the telemetry witnessed by the INS, send telemetry updates to each
    //! sensor based on their mounting geometry relative to the sensor group
    virtual bool        update_ego_velocity(vec3f_t ins_lin_velocity, vec3f_t ins_ang_velocity, uint32_t age_microseconds) = 0;

    //! configure radar data agent in all sensors
    virtual bool        configure_data_capture(const UhdpCaptureControl& cap_ctrl) = 0;

    //! configure thresholds for all sensors, for each scan in the scan loop
    virtual void        configure_detection_thresholds(const RDC_ThresholdControl* thresh_ctrl, uint32_t count=1) = 0;

    //! Configures the scan loop (frame) on the given sensor, but scans are not
    //! started until start_scanning() is called.
    virtual bool        configure_radar_scans(uint32_t sensor_idx, uint32_t scans_in_frame, const RDC_ScanDescriptor* desc) = 0;

    //! Start all sensors - scans must be configured already
    virtual void        start_scanning() = 0;

    //! Stop all sensors.  To restart scans, configure_radar_scans() and
    //! start_scanning() must be called again.
    virtual void        stop_scanning() = 0;

    //! Move the frame interval's timebase forward backward in time.  You can
    //! use this method to give sensors slightly different timebases, or adjust
    //! the common timebase (if all sensors are updated). Adjustments are
    //! acummulative
    virtual void        adjust_timebase(uint32_t sensor_idx, int32_t timebase_delta_us) = 0;

    //! Make small adjustments to the common frame interval. Adjustments are
    //! accumulative
    virtual void        adjust_frame_interval(int32_t frame_interval_delta_us) = 0;

    //! Returns FrameObject containing merged sensor data from the most recent
    //! frame interval. Returns NULL if next FrameObject is still incomplete.
    //!
    //! If the SensorGroup was initialized with use_poll_thread=true,
    //! poll_time_us is ignored. Otherwise, the function will spend poll_time_us
    //! microseconds servicing the sensor sockets, plus some additional time in
    //! service overhead. So consider poll_time_us the minimum amount of time the
    //! function will take.
    //
    //! A frame is considered complete (ready to be returned) if it has the full
    //! complement of scans from each sensor, or if one of the sensors is more
    //! than one frame ahead of the current frame (consider a failure case where
    //! one sensor has stopped scanning).
    //
    //! It is expected to receive partially completed frames for the first
    //! handfull of frames as the frame cadence is established on all of the
    //! sensors, but if this happens after the first handfull of frames you
    //! probably have one or more broken sensors.
    virtual FrameObject* poll_completed_frame(uint32_t poll_time_us) = 0;

    //! Add another device handle to the select() poll function. If this device has
    //! any read or write events, the callback will be called and passed user_ptr
    //! Returns false if there is no room for another handle. This handle will be
    //! polled for the remaining life of the SensorGroup.
    virtual bool        add_extra_device_handle(int device_handle, void (*callback)(void*), void* user_ptr) = 0;

    //! Serialize constituent frame data (including scans) such that it can be
    //! deserialized again in the future and re-generate the combined detections
    //! and tracks
    virtual void        serialize_frame(const FrameObject& frame, ScanSerializer& ser, const char* sgroup_data_filewrite) const = 0;

    //! stop scanning, close all sensor connections, and release all resources
    virtual void        close_and_release() = 0;

    //! Initialize ITS library in preparation for tracking objects via sensor
    //! detections. When a tracker library is initialized and the sensor names
    //! are configured, frames can be sent to the tracker.
    //
    //! NB the tracker name and platform name are extracted from the config file
    //! (we expect only one of each)
    virtual bool        initialize_tracker(const char* config_file, const char* userdata_folder) = 0;

    //! Register the sensor names to use when interfacing with the tracker.  If
    //! unspecified the sensor's detections will not be sent to the tracker
    virtual void        set_sensor_names(uint32_t sensor_idx, const char* azonly_tracker, const char* azel_tracker) = 0;

    //! If initialize_tracker() has been successfully called on this SensorGroup
    //! and set_sensor_names() has been called correcty for each sensor_idx, then
    //! send_frame_to_tracker() can be called.  The frame's detections are sent
    //! to the tracker and in return the list of current tracks is copied into
    //! the frame.
    virtual void        send_frame_to_tracker(FrameObject& frame) = 0;

    //! Deserialize frame object - only applicable if SensorGroup was created by
    //! deserialize_sensor_group.
    virtual FrameObject* deserialize_frame(ScanSerializer& ser) = 0;

    //! an enumeration of the errors which might be returned by this class
    enum Err
    {
        SG_NO_ERROR,
        SG_INPUT_OUT_OF_RANGE,
        SG_REPLAY_GROUP_CANNOT_SCAN,
        SG_INCOMPLETE_CONFIGURATION,
        SG_INVALID_SCAN_LOOP_COUNT,
        SG_UNABLE_TO_OPEN_FILE,
        SG_UNABLE_TO_READ_FILE,
        SG_SENSOR_CONNECTION_LOST,
        SG_SOCKET_FAILURE,               // host socket read failure
        SG_SENSOR_SOFTWARE_UNSUPPORTED,  // sensor software is too old
        SG_SENSOR_FRAME_CADENCE_ERROR,   // redundant scans received in one frame
    };

    //! returns the last error encountered by this instance, errors are sticky
    //! and are not cleared until clear_last_error() is called
    virtual Err         get_last_error() const = 0;

    virtual const char* get_last_error_string() const = 0;

    virtual void        clear_last_error() = 0;

    //! Factory method for sensor groups.  If replay_mode is true none of the
    //! sensors will be connected and none of the scanning and configuration
    //! methods will work; the SensorGroup can only be configured with a sensor
    //! list and then send_frame_to_tracker() can be called
    static  SensorGroup* create_new_sensor_group(bool replay_mode);

    //! Factory method to create a SensorGroup instance that is capable of
    //! deserializing frame objects
    static  SensorGroup* deserialize_sensor_group(const char* sgroup_data_fileread);
};
