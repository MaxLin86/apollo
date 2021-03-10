// Copyright (C) Uhnder, Inc. All rights reserved. Confidential and Proprietary - under NDA.
// Refer to SOFTWARE_LICENSE file for details
#pragma once


#include "modules/drivers/radar/rocket_radar/driver/system-radar-software/env-reference/coredefs/uhnder-common.h"
#include "modules/drivers/radar/rocket_radar/driver/include/frame_object.h"
#include "modules/drivers/radar/rocket_radar/driver/src/sensorgroup_impl.h"

class ScanObject_Impl;
class Tracks_Impl;
struct Sensor;


struct ScanGroup
{
    ScanGroup()
    {
        for (uint32_t i = 0; i < SensorGroup_Impl::MAX_SCAN_LOOP; i++)
        {
            scan[i] = NULL;
        }
    }

    ~ScanGroup();

    ScanObject_Impl* scan[SensorGroup_Impl::MAX_SCAN_LOOP];
};


class FrameObject_Impl : public FrameObject
{
public:

    FrameObject_Impl(const vec3f_t& platform_ego_vel, const vec3f_t& platform_ang_vel, uint32_t num_sensors, uint32_t scan_loop_count, uint64_t timestamp, uint32_t frnum, uint32_t interval)
        : sensor_count(num_sensors)
        , scans_per_frame(scan_loop_count)
        , frame_interval(interval)
        , frame_number(frnum)
        , scan_count(0)
        , detection_count(0)
        , track_count(0)
        , ego_lin_vel(platform_ego_vel)
        , ego_ang_vel(platform_ego_vel)
        , tracks(NULL)
        , detections(NULL)
        , next_frame(NULL)
    {
        start_time.tv_sec = timestamp / (1000 * 1000);
        start_time.tv_usec = timestamp - start_time.tv_sec * 1000 * 1000;
        scans = new ScanGroup[num_sensors];
    }

    virtual ~FrameObject_Impl();

    virtual ScanObject*     get_scan(uint32_t sensor_idx, uint32_t scan_idx);

    virtual bool            frame_is_complete();

    virtual uint32_t        get_num_scans_per_sensor() const { return scans_per_frame; }

    virtual uint32_t        get_frame_number() const { return frame_number; }

    virtual const timeval&  get_frame_time(uint32_t& interval_us) const;

    virtual const vec3f_t&  get_platform_linear_velocity() const { return ego_lin_vel; }

    virtual const vec3f_t&  get_platform_angular_velocity() const { return ego_ang_vel; }

    virtual uint32_t        get_detection_count();

    virtual uint32_t        get_track_count();

    virtual const FrameDetection& get_detection(uint32_t det_idx);

    virtual const Track&          get_track(uint32_t track_idx);

    virtual void            serialize_data(ScanSerializer& ser) const;

    virtual void            release();

    void                    finish_frame(const Sensor* sensors);

    void                    serialize_scans(ScanSerializer& ser, FILE* framedata_file) const;

    const uint32_t      sensor_count;

    const uint32_t      scans_per_frame;

    const uint32_t      frame_interval;

    const uint32_t      frame_number;

    uint32_t            scan_count;

    uint32_t            detection_count;

    uint32_t            track_count;

    const vec3f_t       ego_lin_vel;

    const vec3f_t       ego_ang_vel;

    timeval             start_time;

    ScanGroup*          scans;

    Track*              tracks;

    FrameDetection*     detections;

    FrameObject_Impl*   next_frame;
};
