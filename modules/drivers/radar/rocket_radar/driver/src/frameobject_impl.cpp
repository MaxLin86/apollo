// Copyright (C) Uhnder, Inc. All rights reserved. Confidential and Proprietary - under NDA.
// Refer to SOFTWARE_LICENSE file for details

#include "modules/drivers/radar/rocket_radar/driver/system-radar-software/env-reference/coredefs/uhnder-common.h"
#include "modules/drivers/radar/rocket_radar/driver/src/sensorgroup_impl.h"
#include "modules/drivers/radar/rocket_radar/driver/src/frameobject_impl.h"
#include "modules/drivers/radar/rocket_radar/driver/src/scanning_impl.h"
#include "modules/drivers/radar/rocket_radar/driver/src/tracks_impl.h"
#include "modules/drivers/radar/rocket_radar/driver/include/detections.h"
#include "modules/drivers/radar/rocket_radar/driver/include/serializer.h"

ScanGroup::~ScanGroup()
{
    for (int i = 0; i < SensorGroup_Impl::MAX_SCAN_LOOP; i++)
    {
        if (scan[i])
        {
            scan[i]->release();
            scan[i] = NULL;
        }
    }
}


FrameObject_Impl::~FrameObject_Impl()
{
    delete [] scans;
    delete [] detections;
    delete [] tracks;
}


void            FrameObject_Impl::release()
{
    delete this;
}


bool            FrameObject_Impl::frame_is_complete()
{
    for (uint32_t s = 0; s < sensor_count; s++)
    {
        for (uint32_t i = 0; i < scans_per_frame; i++)
        {
            if (!scans[s].scan[i])
            {
                return false;
            }
        }
    }

    return true;
}


ScanObject* FrameObject_Impl::get_scan(uint32_t sensor_idx, uint32_t scan_idx)
{
    if (sensor_idx < sensor_count && scan_idx < scans_per_frame)
    {
        return scans[sensor_idx].scan[scan_idx];
    }
    else
    {
        return NULL;
    }
}


const timeval& FrameObject_Impl::get_frame_time(uint32_t& interval_us) const
{
    interval_us = frame_interval;

    return start_time;
}


uint32_t              FrameObject_Impl::get_track_count()
{
    return track_count;
}


uint32_t        FrameObject_Impl::get_detection_count()
{
    return detection_count;
}


const FrameDetection& FrameObject_Impl::get_detection(uint32_t det_idx)
{
    static FrameDetection null;

    if (detections && det_idx < detection_count)
    {
        return detections[det_idx];
    }
    else
    {
        return null;
    }
}


const Track&          FrameObject_Impl::get_track(uint32_t track_idx)
{
    static Track null;

    if (tracks && track_idx < track_count)
    {
        return tracks[track_idx];
    }
    else
    {
        return null;
    }
}

// Only called by SensorGroup::serialize_frame()
void            FrameObject_Impl::serialize_scans(ScanSerializer& ser, FILE* framedata_file) const
{
    uint64_t timestamp = start_time.tv_sec;
    timestamp *= 1000 * 1000;
    timestamp += start_time.tv_usec;
    fwrite(&timestamp, sizeof(timestamp), 1, framedata_file);
    fwrite(&ego_lin_vel, sizeof(ego_lin_vel), 1, framedata_file);
    fwrite(&ego_ang_vel, sizeof(ego_ang_vel), 1, framedata_file);

    for (uint32_t s = 0; s < sensor_count; s++)
    {
        for (uint32_t i = 0; i < scans_per_frame; i++)
        {
            uint32_t scan_sequence_number;

            if (scans[s].scan[i])
            {
                scan_sequence_number = scans[s].scan[i]->scan_info.scan_sequence_number;
                scans[s].scan[i]->serialize(ser);
            }
            else
            {
                scan_sequence_number = uint32_t(-1);
            }

            fwrite(&scan_sequence_number, sizeof(scan_sequence_number), 1, framedata_file);
        }
    }
}


void FrameObject_Impl::finish_frame(const Sensor* sensors)
{
    detection_count = 0;
    delete [] detections;

    for (uint32_t s = 0; s < sensor_count; s++)
    {
        for (uint32_t i = 0; i < scans_per_frame; i++)
        {
            if (scans[s].scan[i])
            {
                Detections* det = scans[s].scan[i]->get_detections();
                if (det)
                {
                    detection_count += det->get_count();
                }
            }
        }
    }

    if (detection_count)
    {
        detections = new FrameDetection[detection_count];

        uint32_t done_count = 0;

        for (uint32_t s = 0; s < sensor_count; s++)
        {
            const Sensor& sensor = sensors[s];

            for (uint32_t i = 0; i < scans_per_frame; i++)
            {
                if (scans[s].scan[i])
                {
                    uint16_t flag_3d = scans[s].scan[i]->get_scan_info().num_elevation_angles > 1 ? 0x8000 : 0;

                    Detections* det = scans[s].scan[i]->get_detections();
                    if (det)
                    {
                        for (uint32_t d = 0; d < det->get_count(); d++)
                        {
                            const DetectionData& dd = det->get_detection(d);
                            FrameDetection& fdd = detections[done_count];
                            fdd.scan_idx = i;
                            fdd.sensor_idx = s;
                            fdd.snr_decibels = dd.snr;
                            fdd.rcs = dd.rcs;
                            fdd.magnitude = dd.magnitude;
                            fdd.flags = dd.flags | flag_3d;
                            fdd.doppler_mps = dd.doppler;
                            fdd.range = dd.range;
                            fdd.azimuth = dd.azimuth;
                            fdd.elevation = dd.elevation;

                            vec3f_t pos(dd.pos_x, dd.pos_y, dd.pos_z);
                            fdd.position_m = sensor.sensor_xform.transform(pos);

                            done_count++;
                        }
                    }
                }
            }
        }
    }
}


void            FrameObject_Impl::serialize_data(ScanSerializer& ser) const
{
    char fname[128];
    bool ok = ser.begin_write_scan(frame_number);
    if (!ok)
    {
        return;
    }

    sprintf(fname, "frame_%06d_info.bin", frame_number);
    ok = ser.begin_write_scan_data_type(fname);
    if (ok)
    {
        uint64_t timestamp = start_time.tv_sec;
        timestamp *= 1000 * 1000;
        timestamp += start_time.tv_usec;
        ok &= ser.write_scan_data_type(&timestamp, sizeof(timestamp), 1);
        ok &= ser.write_scan_data_type(&frame_interval, sizeof(frame_interval), 1);
        ok &= ser.write_scan_data_type(&ego_lin_vel, sizeof(ego_lin_vel), 1);
        ok &= ser.write_scan_data_type(&ego_ang_vel, sizeof(ego_ang_vel), 1);
        ser.end_write_scan_data_type(!ok);
    }

    if (detections)
    {
        sprintf(fname, "frame_%06d_detections.bin", frame_number);
        ok = ser.begin_write_scan_data_type(fname);
        if (ok)
        {
            ok &= ser.write_scan_data_type(detections, sizeof(FrameDetection), detection_count);
            ser.end_write_scan_data_type(!ok);
        }
    }

    if (tracks)
    {
        sprintf(fname, "frame_%06d_tracks.bin", frame_number);
        ok = ser.begin_write_scan_data_type(fname);
        if (ok)
        {
            ok &= ser.write_scan_data_type(tracks, sizeof(Track), track_count);
            ser.end_write_scan_data_type(!ok);
        }
    }

    ser.end_write_scan(!ok);
}


FrameObject*    FrameObject::deserialize(ScanSerializer& ser, uint32_t frame_number)
{
    FrameObject_Impl* frame = NULL;
    char fname[128];

    bool ok = ser.begin_read_scan(frame_number);
    if (ok)
    {
        sprintf(fname, "frame_%06d_info.bin", frame_number);
        size_t len = ser.begin_read_scan_data_type(fname);
        if (len)
        {
            uint64_t timestamp;
            uint32_t frame_interval;
            vec3f_t  ego_lin_vel;
            vec3f_t  ego_ang_vel;
            ok &= ser.read_scan_data_type(&timestamp, sizeof(timestamp), 1);
            ok &= ser.read_scan_data_type(&frame_interval, sizeof(frame_interval), 1);
            ok &= ser.read_scan_data_type(&ego_lin_vel, sizeof(ego_lin_vel), 1);
            ok &= ser.read_scan_data_type(&ego_ang_vel, sizeof(ego_ang_vel), 1);
            if (ok)
            {
                const uint32_t num_sensors = 0;
                const uint32_t scan_loop_size = 0;
                frame = new FrameObject_Impl(ego_lin_vel, ego_ang_vel, num_sensors, scan_loop_size, timestamp, frame_number, frame_interval);
            }
            ser.end_read_scan_data_type();
        }
    }

    if (frame && ok)
    {
        sprintf(fname, "frame_%06d_detections.bin", frame_number);
        size_t len = ser.begin_read_scan_data_type(fname);
        if (len)
        {
            frame->detection_count = len / sizeof(FrameDetection);
            frame->detections = new FrameDetection[frame->detection_count];
            ok &= ser.read_scan_data_type(frame->detections, sizeof(FrameDetection), frame->detection_count);
            ser.end_read_scan_data_type();

            if (!ok)
            {
                frame->detection_count = 0;
                delete [] frame->detections;
                frame->detections = NULL;
            }
        }
    }

    if (frame && ok)
    {
        sprintf(fname, "frame_%06d_tracks.bin", frame_number);
        size_t len = ser.begin_read_scan_data_type(fname);
        if (len)
        {
            frame->track_count = len / sizeof(Track);
            frame->tracks = new Track[frame->track_count];
            ok &= ser.read_scan_data_type(frame->tracks, sizeof(Track), frame->track_count);
            ser.end_read_scan_data_type();

            if (!ok)
            {
                frame->track_count = 0;
                delete [] frame->tracks;
                frame->tracks = NULL;
            }
        }
    }

    return frame;
}
