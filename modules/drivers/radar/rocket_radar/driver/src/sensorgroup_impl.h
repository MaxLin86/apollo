// Copyright (C) Uhnder, Inc. All rights reserved. Confidential and Proprietary - under NDA.
// Refer to SOFTWARE_LICENSE file for details
#pragma once

#include "modules/drivers/radar/rocket_radar/driver/system-radar-software/env-reference/coredefs/uhnder-common.h"
#include "modules/drivers/radar/rocket_radar/driver/include/sensorgroup.h"
#include "modules/drivers/radar/rocket_radar/driver/system-radar-software/engine/scp-src/eng-api/rdc-scanctrl.h"
#include "modules/drivers/radar/rocket_radar/driver/system-radar-software/engine/scp-src/eng-api/rdc-threshctrl.h"
#include "modules/drivers/radar/rocket_radar/driver/system-radar-software/engine/scp-src/eng-api/uhdp-msg-structs.h"
#include "modules/drivers/radar/rocket_radar/driver/src/threading.h"
#include "modules/drivers/radar/rocket_radar/driver/system-radar-software/engine/common/eng-api/uhtypes.h"
#include "modules/drivers/radar/rocket_radar/driver/include/rra.h"
class Connection_Impl;
class FrameObject_Impl;

typedef void (*socket_callback)(void*);

struct Sensor
{
    ConnectionLogDesc   con_log_desc;
    vec3f_t             sensor_position;
    quatf_t             sensor_orientation;
    transform_t         sensor_xform;
    vec3f_t             last_ego_velocity;
    int32_t             timebase_offset;
    bool                frame_configured;
    Connection_Impl*    con;
    FILE*               private_log_file;
    const char*         logname;
    const char*         azonly_tracker;
    const char*         azel_tracker;


    Sensor()
        : sensor_position(0, 0, 0)
        , sensor_orientation(0, 0, 0, 1)
        , last_ego_velocity(0, 0, 0)
        , timebase_offset(0)
        , frame_configured(false)
        , con(NULL)
        , private_log_file(NULL)
        , logname(NULL)
        , azonly_tracker(NULL)
        , azel_tracker(NULL)
    {
    }

    ~Sensor();
};

static const char* sg_err_names[] =
{
    "SG_NO_ERROR",
    "SG_INPUT_OUT_OF_RANGE",
    "SG_REPLAY_GROUP_CANNOT_SCAN",
    "SG_INCOMPLETE_CONFIGURATION",
    "SG_INVALID_SCAN_LOOP_COUNT",
    "SG_UNABLE_TO_OPEN_FILE",
    "SG_UNABLE_TO_READ_FILE",
    "SG_SENSOR_CONNECTION_LOST",
    "SG_SOCKET_FAILURE",
    "SG_SENSOR_SOFTWARE_UNSUPPORTED",
    "SG_SENSOR_FRAME_CADENCE_ERROR",
};

class SensorGroup_Impl: public SensorGroup, public Thread, public UserLogAgent
{
public:

    SensorGroup_Impl(bool replay_mode)
        : num_sensors(0)
        , num_extra_handles(0)
        , cur_polled_sensor(0)
        , scan_loop_count(0)
        , frame_count(0)
        , frame_interval_us(0)
        , position_ins(0, 0, 0)
        , imu_lin_vel(0, 0, 0)
        , imu_ang_vel(0, 0, 0)
        , cur_frame_time(0)
        , thread_created(false)
        , thread_stopped(false)
        , scans_running(false)
        , scanning_mode(!replay_mode)
        , shared_log_file(NULL)
        , frame_replay_file(NULL)
        , cur_frame(NULL)
        , first_completed(NULL)
        , last_completed(NULL)
        , last_err(SG_NO_ERROR)
    {
        for (int i = 0; i < MAX_EXTRA_HANDLES; i++)
        {
            extra_handles[i] = 0;
            eh_user_pointers[i] = NULL;
            eh_callbacks[i] = NULL;
        }
    }

    virtual ~SensorGroup_Impl() {}

    // Virtual Methods of SensorGroup

    virtual void        initialize(bool create_group_poll_thread, uint32_t frame_interval_us, vec3f_t ins_position);

    virtual bool        add_sensor(uint32_t sensor_ip, const ConnectionLogDesc& log_desc);

    virtual uint32_t    get_sensor_count() const { return num_sensors; }

    virtual Connection* get_sensor_connection(uint32_t con_idx);

    virtual void        set_sensor_position(uint32_t sensor_idx, vec3f_t position_m, float yaw_deg, float pitch_deg, float roll_deg);

    virtual void        set_sensor_position(uint32_t sensor_idx, vec3f_t position_m, quatf_t pose);

    virtual void        set_sensor_names(uint32_t sensor_idx, const char* azonly_tracker, const char* azel_tracker);

    virtual bool        update_ego_velocity(vec3f_t ins_lin_velocity, vec3f_t ins_ang_velocity, uint32_t age_microseconds);

    virtual bool        add_extra_device_handle(int device_handle, void (*callback)(void*), void* user_ptr);

    virtual bool        configure_data_capture(const UhdpCaptureControl& cap_ctrl);

    virtual void        configure_detection_thresholds(const RDC_ThresholdControl* thresh_ctrl, uint32_t count);

    virtual bool        configure_radar_scans(uint32_t sensor_idx, uint32_t scans_in_frame, const RDC_ScanDescriptor* desc);

    virtual void        adjust_timebase(uint32_t sensor_idx, int32_t frame_interval_us);

    virtual void        adjust_frame_interval(int32_t frame_interval_us);

    virtual void        start_scanning();

    virtual void        stop_scanning();

    virtual FrameObject* poll_completed_frame(uint32_t poll_time_us);

    virtual bool        initialize_tracker(const char* config_file, const char* userdata_folder);

    virtual void        serialize_frame(const FrameObject& frame, ScanSerializer& ser, const char* sgroup_data_filewrite) const;

    virtual void        send_frame_to_tracker(FrameObject& frame);

    virtual void        close_and_release();

    virtual FrameObject* deserialize_frame(ScanSerializer& ser);

    virtual Err         get_last_error() const { return last_err; }

    virtual const char* get_last_error_string() const { return sg_err_names[last_err]; }

    virtual void        clear_last_error() { last_err = SG_NO_ERROR; }

    // Virtual Methods of Thread

    virtual void        thread_main();

    // Virtual Methods of UserLogAgent

    virtual void        radar_print_message(ICCQTargetEnum cpu, const char* message);

    virtual void        radar_log_message(ICCQTargetEnum cpu, LogLevelEnum level, const char* message);

    void                poll_sensors(uint32_t poll_time_us);

    FrameObject_Impl*   poll_completed_scans();

    void                stop_sensor_scanning(uint32_t sensor_mask);

    enum { MAX_SENSORS = 8 };

    enum { MAX_EXTRA_HANDLES = 4 };

    enum { MAX_SCAN_LOOP = 3 };

    enum { MAGIC = 0xcf70f2c0, VERSION = 1 };

    uint32_t            num_sensors;

    uint32_t            num_extra_handles;

    uint32_t            cur_polled_sensor;

    uint32_t            scan_loop_count;

    uint32_t            frame_count;

    uint32_t            frame_interval_us;

    vec3f_t             position_ins;

    vec3f_t             imu_lin_vel;

    vec3f_t             imu_ang_vel;

    uint64_t            cur_frame_time;

    bool                thread_created;

    bool                thread_stopped;

    bool                scans_running;

    bool                scanning_mode;

    FILE*               shared_log_file;

    mutable FILE*       frame_replay_file;

    Sensor              sensor[MAX_SENSORS];

    int                 extra_handles[MAX_EXTRA_HANDLES];

    void*               eh_user_pointers[MAX_EXTRA_HANDLES];

    socket_callback     eh_callbacks[MAX_EXTRA_HANDLES];

    Mutex               poll_mutex;

    Mutex               frame_mutex;

    FrameObject_Impl*   cur_frame;

    FrameObject_Impl*   first_completed;

    FrameObject_Impl*   last_completed;

    void*               its;

    mutable Err         last_err;
};
