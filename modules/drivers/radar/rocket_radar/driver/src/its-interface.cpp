// Copyright (C) Uhnder, Inc. All rights reserved. Confidential and Proprietary - under NDA.
// Refer to SOFTWARE_LICENSE file for details

#include "modules/drivers/radar/rocket_radar/driver/system-radar-software/env-reference/coredefs/uhnder-common.h"
#include "modules/drivers/radar/rocket_radar/driver/system-radar-software/engine/common/eng-api/uhtypes.h"
#include "modules/drivers/radar/rocket_radar/driver/src/sensorgroup_impl.h"
#include "modules/drivers/radar/rocket_radar/driver/src/connection_impl.h"
#include "modules/drivers/radar/rocket_radar/driver/src/frameobject_impl.h"
#include "modules/drivers/radar/rocket_radar/driver/src/scanning_impl.h"

#if defined(_WIN32)
#elif defined(__APPLE__)

/* RTLD_LAZY  - Only resolve symbols as the code that references them is executed.
 * RTLD_LOCAL - Symbols defined in this shared object are not made available to
 *              resolve references in subsequently loaded shared objects */
static INT dlopen_flags = RTLD_LAZY | RTLD_LOCAL;
#else // assumed Linux

/* RTLD_DEEPBIND - Place the lookup scope of the symbols in this shared object
 *                 ahead of the global scope. This means that a self-contained object
 *                 will use its own symbols in preference to global symbols with the
 *                 same name contained in objects that have already been loaded */
static INT dlopen_flags = RTLD_LAZY | RTLD_LOCAL | RTLD_DEEPBIND;
#endif // if _WIN32

static void* libhandle;

bool SensorGroup_Impl::initialize_tracker(const char* config_file, const char* userdata_folder)
{
#if !defined(_WIN32)
    typedef void* (*init_func)(const char*, const char*);
    libhandle = dlopen("librra-its.so", dlopen_flags);
    if (libhandle)
    {
        init_func init = (init_func)dlsym(libhandle, "initialize_tracker");
        if (init)
        {
            its = init(config_file, userdata_folder);
        }
    }
    else
    {
        printf("SensorGroup::initialize_tracker() unable to dlopen librra-its.so\n");
        printf("run `make` in its360agx and set LD_LIBRARY_PATH\n");
    }
#endif

    return its != NULL;
}


void SensorGroup_Impl::send_frame_to_tracker(FrameObject& frame)
{
#if !defined(_WIN32)
    if (its && libhandle)
    {
        typedef void (*track_func)(SensorGroup_Impl& sg, FrameObject_Impl& frame);
        static track_func trackfn;
        if (!trackfn)
        {
            trackfn = (track_func)dlsym(libhandle, "send_frame_to_tracker");
        }
        if (trackfn)
        {
            trackfn(*this, static_cast<FrameObject_Impl&>(frame));
        }
    }
#endif
}


void SensorGroup_Impl::set_sensor_names(uint32_t sensor_idx, const char* azonly_tracker, const char* azel_tracker)
{
    if (sensor_idx < num_sensors)
    {
        Sensor& s = sensor[sensor_idx];
        if (azonly_tracker)
        {
            s.azonly_tracker = strdup(azonly_tracker);
        }
        if (azel_tracker)
        {
            s.azel_tracker = strdup(azel_tracker);
        }

#if !defined(_WIN32)
        if (its && libhandle)
        {
            typedef void (*sn_func)(SensorGroup_Impl& sg, uint32_t sensor_idx, const char* azonly_tracker, const char* azel_tracker);
            static sn_func ssn_fn;
            if (!ssn_fn)
            {
                ssn_fn = (sn_func)dlsym(libhandle, "set_sensor_names");
            }
            if (ssn_fn)
            {
                ssn_fn(*this, sensor_idx, azonly_tracker, azel_tracker);
            }
        }
#endif
    }
}


void SensorGroup_Impl::set_sensor_position(uint32_t sensor_idx, vec3f_t position_m, float yaw_deg, float pitch_deg, float roll_deg)
{
    if (sensor_idx < num_sensors)
    {
        Sensor& s = sensor[sensor_idx];
        s.sensor_position = position_m;
        s.sensor_orientation = quatf_t(deg2rad(roll_deg), deg2rad(yaw_deg), deg2rad(pitch_deg));
        s.sensor_xform.initialize(s.sensor_position, s.sensor_orientation);

#if !defined(_WIN32)
        if (its && libhandle)
        {
            typedef void (*sp_func)(SensorGroup_Impl& sg, uint32_t sensor_idx, vec3f_t position_m, float azimuth, float elevation);
            static sp_func ssp_fn;
            if (!ssp_fn)
            {
                ssp_fn = (sp_func)dlsym(libhandle, "set_sensor_position");
            }
            if (ssp_fn)
            {
                float azimuth_rad = deg2rad(yaw_deg);
                float elevation_rad = deg2rad(pitch_deg);
                ssp_fn(*this, sensor_idx, position_m, azimuth_rad, elevation_rad);
            }
        }
#endif
    }
}


void SensorGroup_Impl::set_sensor_position(uint32_t sensor_idx, vec3f_t position_m, quatf_t pose)
{
    if (sensor_idx < num_sensors)
    {
        Sensor& s = sensor[sensor_idx];
        s.sensor_position = position_m;
        s.sensor_orientation = pose;
        s.sensor_xform.initialize(s.sensor_position, s.sensor_orientation);

#if !defined(_WIN32)
        if (its && libhandle)
        {
            typedef void (*sp_func)(SensorGroup_Impl& sg, uint32_t sensor_idx, vec3f_t position_m, float azimuth, float elevation);
            static sp_func ssp_fn;
            if (!ssp_fn)
            {
                ssp_fn = (sp_func)dlsym(libhandle, "set_sensor_position");
            }
            if (ssp_fn)
            {
                vec3f_t euler = pose.toEuler();
                ssp_fn(*this, sensor_idx, position_m, euler.z, euler.y);
            }
        }
#endif
    }
}


void SensorGroup_Impl::adjust_frame_interval(int32_t frame_interval_delta)
{
    frame_interval_us += frame_interval_delta;

    if (scans_running)
    {
        RDC_ScanControl ctrl;
        ctrl.defaults();
        ctrl.persistent_frame_config = 1;
        ctrl.scan_count = -1;
        ctrl.scan_loop_count = 0;
        ctrl.frame_interval_us = frame_interval_us;

        for (uint32_t i = 0; i < num_sensors; i++)
        {
            sensor[i].con->send_scan_control(ctrl, NULL);
        }
    }

#if !defined(_WIN32)
    if (its && libhandle)
    {
        typedef void (*fi_func)(SensorGroup_Impl& sg);
        static fi_func afi_fn;
        if (!afi_fn)
        {
            afi_fn = (fi_func)dlsym(libhandle, "set_sensor_position");
        }
        if (afi_fn)
        {
            afi_fn(*this);
        }
    }
#endif
}
