// Copyright (C) Uhnder, Inc. All rights reserved. Confidential and Proprietary - under NDA.
// Refer to SOFTWARE_LICENSE file for details

#include "modules/drivers/radar/rocket_radar/driver/system-radar-software/env-reference/coredefs/uhnder-common.h"
#include "modules/drivers/radar/rocket_radar/driver/system-radar-software/engine/scp-src/eng-api/rdc-scanctrl.h"
#include "modules/drivers/radar/rocket_radar/driver/system-radar-software/engine/scp-src/eng-api/uhdp-msg-structs.h"
#include "modules/drivers/radar/rocket_radar/driver/system-radar-software/engine/common/eng-api/uhtypes.h"
#include "modules/drivers/radar/rocket_radar/driver/src/sensorgroup_impl.h"
#include "modules/drivers/radar/rocket_radar/driver/src/connection_impl.h"
#include "modules/drivers/radar/rocket_radar/driver/src/frameobject_impl.h"
#include "modules/drivers/radar/rocket_radar/driver/src/scanobject_impl.h"
#include "modules/drivers/radar/rocket_radar/driver/system-radar-software/engine/scp-src/eng-api/state-manager.h"
#include "modules/drivers/radar/rocket_radar/driver/system-radar-software/env-reference/coredefs/uhunistd.h"
#include "modules/drivers/radar/rocket_radar/driver/system-radar-software/env-reference/coredefs/uhinet.h"

#if __linux__
// on linux, select() automatically updates timeout to reflect time remaining
#define AUTO_TIME_UPDATES 1
#else
#define AUTO_TIME_UPDATES 0
#endif

static const char* nowstr()
{
    static char timebuf[32];
    time_t ltime;
    time(&ltime);
#if _WIN32
    ctime_s(timebuf, sizeof(timebuf), &ltime);
    return timebuf;
#else
    return ctime_r(&ltime, timebuf);
#endif
}


Sensor::~Sensor()
{
    if (con)
    {
        con->close_and_release();
        con = NULL;
    }

    if (private_log_file)
    {
        fclose(private_log_file);
        private_log_file = NULL;
    }

    free((void*)logname);
    free((void*)azonly_tracker);
    free((void*)azel_tracker);
}


Connection* SensorGroup_Impl::get_sensor_connection(uint32_t sensor_idx)
{
    if (sensor_idx < num_sensors)
    {
        return sensor[sensor_idx].con;
    }
    else
    {
        last_err = SG_INPUT_OUT_OF_RANGE;
        return NULL;
    }
}


void SensorGroup_Impl::initialize(bool create_group_poll_thread, uint32_t frame_interval_us, vec3f_t ins_position)
{
    position_ins = ins_position;

    this->frame_interval_us = frame_interval_us;

    if (scanning_mode)
    {
        if (create_group_poll_thread)
        {
            thread_created = true;
            start();
        }

        shared_log_file = fopen("shared_radar_logs.txt", "w");
        if (shared_log_file)
        {
            fprintf(shared_log_file, "Sensor group initialized at %s\n", nowstr());
            fprintf(shared_log_file, "RRA version %s\n", get_rra_version_string());
            fprintf(shared_log_file, "SRS header version %s\n", get_compiled_srs_timestamp_string());
        }
    }
}


bool SensorGroup_Impl::add_sensor(uint32_t sensor_ip, const ConnectionLogDesc& log_desc)
{
    bool ret = false;
    char ipbuf[32];

    if (num_sensors < MAX_SENSORS)
    {
        if (shared_log_file)
        {
            fprintf(shared_log_file, "\nConnecting to radar %s at %s\n\n", inet_ntop(AF_INET, &sensor_ip, ipbuf, 32), nowstr());
        }

        Sensor& s = sensor[num_sensors];

        if (log_desc.private_log_filename &&
            (log_desc.log_destination == LD_PRIVATE_LOG_FILE || log_desc.print_destination == LD_PRIVATE_LOG_FILE))
        {
            s.private_log_file = fopen("shared_radar_logs.txt", "w");
            if (s.private_log_file)
            {
                fprintf(s.private_log_file, "Connecting to radar %s at %s\n", inet_ntop(AF_INET, &sensor_ip, ipbuf, 32), nowstr());
                fprintf(s.private_log_file, "RRA version %s\n", get_rra_version_string());
                fprintf(s.private_log_file, "SRS header version %s\n\n", get_compiled_srs_timestamp_string());
            }
        }

        s.con_log_desc = log_desc;
        if (log_desc.sensor_name)
        {
            s.logname = strdup(log_desc.sensor_name);
        }
        else
        {
            char num[16];
            sprintf(num, "%u", num_sensors);
            s.logname = strdup(num);
        }

        if (!scanning_mode)
        {
            num_sensors++;
            return true;
        }

        poll_mutex.acquire();            // halt background thread polling while we make the new connection
        cur_polled_sensor = num_sensors; // for benefit of log agent methods
        Connection* newcon = make_connection(sensor_ip, 2, *this, false);
        if (newcon)
        {
            s.con = static_cast<Connection_Impl*>(newcon);
            s.con->mysensorgroup = this;
            s.con->use_thread = thread_created;
            s.con->configure_socket_wait_time(1);

            poll_mutex.release();

            float rear_axle_dist, centerline_dist, height, azimuth_deg, elevation_deg;
            s.con->get_mounting_distances(rear_axle_dist, centerline_dist, height);
            s.con->get_mounting_angles(azimuth_deg, elevation_deg);

            s.sensor_position.x = rear_axle_dist;
            s.sensor_position.y = centerline_dist;
            s.sensor_position.z = height;
            s.sensor_orientation = quatf_t(0, deg2rad(azimuth_deg), deg2rad(elevation_deg));
            s.sensor_xform.initialize(s.sensor_position, s.sensor_orientation);

            num_sensors++;
            ret = true;
        }
        else if (s.private_log_file)
        {
            poll_mutex.release();
            fclose(s.private_log_file);
            s.private_log_file = NULL;
        }
    }

    return ret;
}


bool SensorGroup_Impl::update_ego_velocity(vec3f_t ins_lin_velocity, vec3f_t ins_ang_velocity, uint32_t age_microseconds)
{
    UhdpTelemetryData tel;
    memset(&tel, 0, sizeof(tel));

    imu_lin_vel = ins_lin_velocity;
    imu_ang_vel = ins_ang_velocity;

    tel.telemetry_age_us = age_microseconds;

    for (uint32_t i = 0; i < num_sensors; i++)
    {
        Sensor& s = sensor[i];

        // component of linear velocity caused by angular velocity
        vec3f_t normal_velocity = (s.sensor_position - position_ins).cross(imu_ang_vel);

        // total linear velocity at the sensor position is the linear velocity
        // measured at the INS plus the normal velocity caused by platform
        // angular velocity (presuming this is a rigid body)
        tel.ego_linear_velocity = ins_lin_velocity + normal_velocity;

        // rotate this platform coordinate system ego velocity (at this
        // position) into the sensor coordinate system (accounting for yaw,
        // pitch, and roll)
        tel.ego_linear_velocity = s.sensor_orientation.rotateInv(tel.ego_linear_velocity);

        // the sensor needs to know the velocity of the world relative to its
        // coordinate system, which is the negative of its own velocity
        tel.ego_linear_velocity *= -1;

        // this makes the function unit-testable, it also is possible we will
        // use this value in the future for data continuity
        s.last_ego_velocity = tel.ego_linear_velocity;

        tel.ego_angular_velocity = s.sensor_orientation.rotateInv(ins_ang_velocity);
        tel.ego_angular_velocity *= -1;

        tel.yaw_rate = tel.ego_angular_velocity.z;

        if (s.con && scanning_mode)
        {
            s.con->configure_radar_telemetry(tel);
            if (!s.con->is_connection_valid())
            {
                last_err = SG_SENSOR_CONNECTION_LOST;
            }
        }
        else
        {
            last_err = SG_REPLAY_GROUP_CANNOT_SCAN;
        }
    }
    return false;
}


bool SensorGroup_Impl::add_extra_device_handle(int device_handle, void (*callback)(void*), void* user_ptr)
{
    bool ret = false;

    if (num_extra_handles < MAX_EXTRA_HANDLES)
    {
        extra_handles[num_extra_handles] = device_handle;
        eh_callbacks[num_extra_handles] = callback;
        eh_user_pointers[num_extra_handles] = user_ptr;
        num_extra_handles++;
        ret = true;
    }
    else
    {
        last_err = SG_INPUT_OUT_OF_RANGE;
    }

    return ret;
}


void SensorGroup_Impl::poll_sensors(uint32_t poll_time_us)
{
    if (num_sensors == 0 && num_extra_handles == 0)
    {
        return;
    }
    if (!scanning_mode)
    {
        return;
    }
    if (poll_time_us > 100 * 1000)
    {
        // clamp timeout to 100ms
        poll_time_us = 100000;
    }

    fd_set socket_set;
    FD_ZERO(&socket_set);
    int max_sd = -1;

    for (uint32_t i = 0; i < num_sensors; i++)
    {
        Sensor& s = sensor[i];
        int sd = s.con->get_socket_descriptor();
        FD_SET(sd, &socket_set);
        if (sd > max_sd)
        {
            max_sd = sd;
        }
    }
    for (uint32_t i = 0; i < num_extra_handles; i++)
    {
        FD_SET(extra_handles[i], &socket_set);
        if (extra_handles[i] > max_sd)
        {
            max_sd = extra_handles[i];
        }
    }

    max_sd += 1;

    timeval timeout;
    timeout.tv_sec = 0;
    timeout.tv_usec = poll_time_us;

#if !AUTO_TIME_UPDATES
    timeval tv_start;
    gettimeofday(&tv_start, NULL);
    uint64_t ts_start = tv_start.tv_sec * 1000 * 1000 + tv_start.tv_usec;
#endif

    while (true)
    {
        fd_set read_set = socket_set;

        int ret = select(max_sd, &read_set, NULL, NULL, &timeout);
        if (ret > 0)
        {
            for (uint32_t i = 0; i < num_sensors; i++)
            {
                Sensor& s = sensor[i];
                int sd = s.con->get_socket_descriptor();
                if (FD_ISSET(sd, &read_set))
                {
                    poll_mutex.acquire();
                    cur_polled_sensor = i; // for benefit of log agent methods
                    s.con->poll_socket_internal();
                    poll_mutex.release();
                }
            }
            for (uint32_t i = 0; i < num_extra_handles; i++)
            {
                if (FD_ISSET(extra_handles[i], &read_set))
                {
                    eh_callbacks[i](eh_user_pointers[i]);
                }
            }

#if !AUTO_TIME_UPDATES
            // update timeout with time remaining
            timeval now;
            gettimeofday(&now, NULL);
            uint64_t ts_now = now.tv_sec * 1000 * 1000 + now.tv_usec;
            uint64_t elapsed = ts_now - ts_start;
            if (elapsed >= poll_time_us)
            {
                break;
            }
            else
            {
                timeout.tv_usec = poll_time_us - elapsed;
            }
#endif
        }
        else if (ret == 0)
        {
            // timeout, stop polling
            break;
        }
        else
        {
            last_err = SG_SOCKET_FAILURE;
            perror("select() failure");
            break;
        }
    }

    for (uint32_t i = 0; i < num_sensors; i++)
    {
        Sensor& s = sensor[i];
        s.con->poll_retransmit_timers();

        if (s.private_log_file)
        {
            fflush(s.private_log_file);
        }
    }

    frame_mutex.acquire();
    FrameObject_Impl* out_frame = SensorGroup_Impl::poll_completed_scans();
    while (out_frame)
    {
        out_frame->next_frame = NULL;

        if (!last_completed)
        {
            last_completed = out_frame;
            first_completed = last_completed;
        }
        else
        {
            last_completed->next_frame = out_frame;
            last_completed = out_frame;
        }

        out_frame = SensorGroup_Impl::poll_completed_scans();
    }
    frame_mutex.release();

    if (shared_log_file)
    {
        fflush(shared_log_file);
    }
}


FrameObject* SensorGroup_Impl::poll_completed_frame(uint32_t poll_time_us)
{
    FrameObject_Impl* out_frame = NULL;

    if (!thread_created)
    {
        poll_sensors(poll_time_us);
    }

    frame_mutex.acquire();
    if (first_completed)
    {
        out_frame = first_completed;
        first_completed = first_completed->next_frame;
        if (first_completed == NULL)
        {
            last_completed = NULL;
        }
    }
    frame_mutex.release();

    return out_frame;
}


FrameObject_Impl* SensorGroup_Impl::poll_completed_scans()
{
    FrameObject_Impl* out_frame = NULL;

    if (cur_frame && cur_frame->scan_count == num_sensors * scan_loop_count)
    {
        // return frame that was completed previously
        out_frame = cur_frame;
        cur_frame = cur_frame->next_frame;
        cur_frame_time += frame_interval_us;
        return out_frame;
    }

    // Poll for completed scans
    for (uint32_t i = 0; i < num_sensors; i++)
    {
        ScanObject_Impl* scan = static_cast<ScanObject_Impl*>(sensor[i].con->poll_completed_scan());
        while (scan)
        {
            const timeval& t = scan->get_scan_end_local_time();
            const UhdpScanInformation& info = scan->get_scan_info();
            const uint64_t scantime = (uint64_t)t.tv_sec * 1000 * 1000 + t.tv_usec;
            const int64_t  diff = scantime - cur_frame_time - sensor[i].timebase_offset;
            const double   delta = double(diff) / frame_interval_us;

            if (info.scan_loop_size != scan_loop_count)
            {
                //printf("Scan from sensor %d discarded, unexpected scan_loop_size %d\n", i, info.scan_loop_size);
                last_err = SG_SENSOR_SOFTWARE_UNSUPPORTED;
                scan->release();
                break;
            }

            if (delta < 0)
            {
                //printf("Scan from sensor %d discarded, we have moved on\n", i);
                scan->release();
                break;
            }

            if (!cur_frame)
            {
                cur_frame = new FrameObject_Impl(imu_lin_vel, imu_ang_vel, num_sensors, scan_loop_count, cur_frame_time, frame_count, frame_interval_us);
                frame_count++;
            }

            FrameObject_Impl* use_frame = cur_frame;
            uint64_t frtime = cur_frame_time;

            for (uint32_t d = 1; delta > double(d); d++)
            {
                frtime += frame_interval_us;

                if (use_frame->next_frame == NULL)
                {
                    use_frame->next_frame = new FrameObject_Impl(imu_lin_vel, imu_ang_vel, num_sensors, scan_loop_count, frtime, frame_count, frame_interval_us);
                    frame_count++;
                }

                use_frame = use_frame->next_frame;
            }

            if (use_frame->scans[i].scan[info.scan_loop_idx])
            {
                //printf("Scan %d from sensor %d discarded, the slot was already full\n", info.scan_loop_idx, i);
                last_err = SG_SENSOR_FRAME_CADENCE_ERROR;
                scan->release();
            }
            else
            {
                use_frame->scans[i].scan[info.scan_loop_idx] = scan;
                use_frame->scan_count++;
                //printf("Sensor %u - Scan %u - delta %f, frame now has %d scans\n", i, info.scan_loop_idx, delta, use_frame->scan_count);

                if ((use_frame->scan_count == (num_sensors * scan_loop_count)) || delta > 2)
                {
                    out_frame = cur_frame;
                    cur_frame = cur_frame->next_frame;
                    cur_frame_time += frame_interval_us;

                    if (out_frame->scan_count < (num_sensors * scan_loop_count))
                    {
                        // finish incomplete output frame
                        out_frame->finish_frame(sensor);
                    }

                    if (use_frame->scan_count == (num_sensors * scan_loop_count))
                    {
                        // finish completed frame
                        use_frame->finish_frame(sensor);
                    }
                }
            }

            if (out_frame)
            {
                scan = NULL;
            }
            else
            {
                scan = static_cast<ScanObject_Impl*>(sensor[i].con->poll_completed_scan());
            }
        }
    }

    return out_frame;
}


bool SensorGroup_Impl::configure_data_capture(const UhdpCaptureControl& cap_ctrl)
{
    bool ret = true;

    if (scanning_mode)
    {
        for (uint32_t i = 0; i < num_sensors; i++)
        {
            ret &= sensor[i].con->configure_data_capture(cap_ctrl);
            if (!sensor[i].con->is_connection_valid())
            {
                last_err = SG_SOCKET_FAILURE;
            }
        }
    }
    else
    {
        last_err = SG_REPLAY_GROUP_CANNOT_SCAN;
    }

    return ret;
}


void SensorGroup_Impl::configure_detection_thresholds(const RDC_ThresholdControl* thresh_ctrl, uint32_t count)
{
    if (scanning_mode)
    {
        for (uint32_t i = 0; i < num_sensors; i++)
        {
            sensor[i].con->configure_detection_thresholds(thresh_ctrl, count);
            if (!sensor[i].con->is_connection_valid())
            {
                last_err = SG_SOCKET_FAILURE;
            }
        }
    }
    else
    {
        last_err = SG_REPLAY_GROUP_CANNOT_SCAN;
    }
}


bool SensorGroup_Impl::configure_radar_scans(uint32_t sensor_idx, uint32_t scans_in_frame, const RDC_ScanDescriptor* desc)
{
    stop_sensor_scanning(1U << sensor_idx);

    bool ret = false;

    if (scans_in_frame == 0 || scans_in_frame > MAX_SCAN_LOOP)
    {
        last_err = SG_INVALID_SCAN_LOOP_COUNT;
    }
    else if (!desc)
    {
        last_err = SG_INPUT_OUT_OF_RANGE;
    }
    else if (scan_loop_count != 0 && scans_in_frame != scan_loop_count)
    {
        last_err = SG_INVALID_SCAN_LOOP_COUNT;
    }
    else
    {
        RDC_ScanControl ctrl;
        ctrl.defaults();
        ctrl.persistent_frame_config = 1;
        ctrl.scan_count = 0;
        ctrl.scan_loop_count = scans_in_frame;
        ctrl.frame_interval_us = frame_interval_us;

        // this takes a while; seconds per sensor
        ret = true;

        if (shared_log_file)
        {
            fprintf(shared_log_file, "\nConfiguring scans on sensor %d at %s\n\n", sensor_idx, nowstr());
        }
        // TODO: make this async so we can configure all sensors simultaneously
        ret &= (sensor[sensor_idx].con->send_scan_control(ctrl, desc) == 0);
        if (!sensor[sensor_idx].con->is_connection_valid())
        {
            last_err = SG_SENSOR_CONNECTION_LOST;
        }
    }

    if (ret == true)
    {
        scan_loop_count = scans_in_frame;
        sensor[sensor_idx].frame_configured = true;
    }

    return ret;
}


void SensorGroup_Impl::start_scanning()
{
    bool all_configured = true;
    for (uint32_t i = 0; i < num_sensors; i++)
    {
        all_configured &= sensor[i].frame_configured;
    }

    if (all_configured)
    {
        // --pcontinue
        RDC_ScanControl ctrl;
        ctrl.defaults();
        ctrl.persistent_frame_config = 1;
        ctrl.scan_count = -1;
        ctrl.scan_loop_count = 0;
        ctrl.frame_interval_us = frame_interval_us;

        timeval time_base;
        gettimeofday(&time_base, NULL);
        cur_frame_time = (uint64_t)time_base.tv_sec * 1000 * 1000 + time_base.tv_usec;

        if (shared_log_file)
        {
            fprintf(shared_log_file, "\nStarting scans on all sensors at %s\n\n", nowstr());
        }

        for (uint32_t i = 0; i < num_sensors; i++)
        {
            sensor[i].con->synchronize_frame_interval(time_base);
        }

        for (uint32_t i = 0; i < num_sensors; i++)
        {
            sensor[i].con->send_scan_control(ctrl, NULL);
        }

        scans_running = true;
    }
    else
    {
        last_err = SG_INCOMPLETE_CONFIGURATION;
    }
}


void SensorGroup_Impl::adjust_timebase(uint32_t sensor_idx, int32_t frame_interval_delta_us)
{
    sensor[sensor_idx].timebase_offset += frame_interval_delta_us;

    uint64_t sensor_frame_time = cur_frame_time + sensor[sensor_idx].timebase_offset;

    timeval time_base;
    time_base.tv_sec = sensor_frame_time / (1000 * 1000);
    time_base.tv_usec = sensor_frame_time - time_base.tv_sec * 1000 * 1000;

    sensor[sensor_idx].con->synchronize_frame_interval(time_base);
    if (!sensor[sensor_idx].con->is_connection_valid())
    {
        last_err = SG_SENSOR_CONNECTION_LOST;
    }
}


void SensorGroup_Impl::stop_scanning()
{
    stop_sensor_scanning((1U << num_sensors) - 1U);

    scans_running = false;
}


void SensorGroup_Impl::stop_sensor_scanning(uint32_t sensor_mask)
{
    for (uint32_t i = 0; i < num_sensors; i++)
    {
        if (((1U << i) & sensor_mask) == 0)
        {
            ; // skip this sensor
        }
        else if (sensor[i].con->query_radar_status_bitmask() & STATE_ATTR_SCANNING)
        {
            if (shared_log_file)
            {
                fprintf(shared_log_file, "\nStopping scans on sensor %d at %s\n\n", i, nowstr());
            }

            //printf("Stopping scans on sensor %d\n", i);

            // --pstop
            RDC_ScanControl ctrl;
            ctrl.defaults();
            ctrl.scan_count = 0;
            ctrl.scan_loop_count = 0;
            sensor[i].con->send_scan_control(ctrl, NULL);
            sensor[i].frame_configured = false;
        }
    }

    for (uint32_t i = 0; i < num_sensors; i++)
    {
        if (((1U << i) & sensor_mask) == 0)
        {
            continue;
        }
        while (sensor[i].con->query_radar_status_bitmask() & STATE_ATTR_SCANNING)
        {
            uh_usleep(1000);
        }

        // discard any enqueued scan objects
        ScanObject* scan = sensor[i].con->poll_completed_scan();
        while (scan)
        {
            scan->release();
            scan = sensor[i].con->poll_completed_scan();
            if (!sensor[i].con->is_connection_valid())
            {
                last_err = SG_SENSOR_CONNECTION_LOST;
            }
        }
    }
}


void SensorGroup_Impl::radar_print_message(ICCQTargetEnum cpu, const char* message)
{
    Sensor& s = sensor[cur_polled_sensor];

    switch (s.con_log_desc.print_destination)
    {
    case LD_DEV_NULL:
        break;

    case LD_SHARED_LOG_FILE:
        if (shared_log_file)
        {
            fprintf(shared_log_file, "%s: %s-UHP: %s\n", s.logname, cpu_names[cpu], message);
        }
        break;

    case LD_PRIVATE_LOG_FILE:
        if (s.private_log_file)
        {
            fprintf(s.private_log_file, "%s-UHP: %s\n", cpu_names[cpu], message);
        }
        break;
    }
}


void SensorGroup_Impl::radar_log_message(ICCQTargetEnum cpu, LogLevelEnum level, const char* message)
{
    Sensor& s = sensor[cur_polled_sensor];

    switch (s.con_log_desc.log_destination)
    {
    case LD_DEV_NULL:
        break;

    case LD_SHARED_LOG_FILE:
        if (shared_log_file)
        {
            fprintf(shared_log_file, "%s: %s-LOG: %s\n", s.logname, cpu_names[cpu], message);
        }
        break;

    case LD_PRIVATE_LOG_FILE:
        if (s.private_log_file)
        {
            fprintf(s.private_log_file, "%s-LOG: %s\n", cpu_names[cpu], message);
        }
        break;
    }

    if (level >= s.con_log_desc.filter_level)
    {
        const log_handler& lh = s.con_log_desc.user_log_handler;
        if (lh)
        {
            lh(cur_polled_sensor, level, message);
        }
    }
}


void SensorGroup_Impl::thread_main()
{
    while (!thread_stopped)
    {
        poll_sensors(1000); // blocking wait for 1ms
    }
}


void SensorGroup_Impl::close_and_release()
{
    thread_stopped = true;
    if (thread_created)
    {
        stop();
    }

    for (uint32_t i = 0; i < num_sensors; i++)
    {
        Sensor& s = sensor[i];

        if (s.con)
        {
            s.con->mysensorgroup = NULL;
            s.con->use_thread = false; // do not try to stop non-existant Connection thread
            s.con->close_and_release();
            s.con = NULL;
        }
        if (s.private_log_file)
        {
            fclose(s.private_log_file);
            s.private_log_file = NULL;
        }
    }
    num_sensors = 0;

    if (shared_log_file)
    {
        fclose(shared_log_file);
        shared_log_file = NULL;
    }

    if (frame_replay_file)
    {
        fclose(frame_replay_file);
        frame_replay_file = NULL;
    }

    FrameObject* compframe = poll_completed_frame(0);
    while (compframe)
    {
        compframe->release();
        compframe = poll_completed_frame(0);
    }

    while (cur_frame)
    {
        cur_frame->release();
        cur_frame = cur_frame->next_frame;
    }

    delete this;
}


void SensorGroup_Impl::serialize_frame(const FrameObject& frame, ScanSerializer& ser, const char* sgroup_data_filewrite) const
{
    if (!frame_replay_file)
    {
        frame_replay_file = fopen(sgroup_data_filewrite, "wb");

        if (frame_replay_file)
        {
            uint32_t magic = MAGIC;
            uint32_t version = VERSION;

            fwrite(&magic, sizeof(magic), 1, frame_replay_file);
            fwrite(&version, sizeof(version), 1, frame_replay_file);
            fwrite(&num_sensors, sizeof(num_sensors), 1, frame_replay_file);
            fwrite(&scan_loop_count, sizeof(scan_loop_count), 1, frame_replay_file);
            fwrite(&frame_interval_us, sizeof(frame_interval_us), 1, frame_replay_file);
            fwrite(&position_ins, sizeof(position_ins), 1, frame_replay_file);

            for (uint32_t i = 0; i < num_sensors; i++)
            {
                const Sensor& s = sensor[i];
                fwrite(&s.sensor_position, sizeof(s.sensor_position), 1, frame_replay_file);
                fwrite(&s.sensor_orientation, sizeof(s.sensor_orientation), 1, frame_replay_file);
            }
        }
    }

    if (frame_replay_file)
    {
        static_cast<const FrameObject_Impl&>(frame).serialize_scans(ser, frame_replay_file);
    }
    else
    {
        last_err = SG_UNABLE_TO_OPEN_FILE;
    }
}


SensorGroup* SensorGroup::create_new_sensor_group(bool replay_mode)
{
    return new SensorGroup_Impl(replay_mode);
}


// static factory method - create sensor group from sgroup_data_fileread file contents
SensorGroup* SensorGroup::deserialize_sensor_group(const char* sgroup_data_fileread)
{
    SensorGroup_Impl* sg = NULL;

    FILE* fp = fopen(sgroup_data_fileread, "rb");
    if (fp)
    {
        uint32_t magic = 0;
        uint32_t version = 0;
        uint32_t num_sensors;
        uint32_t scan_loop_count;
        uint32_t frame_interval_us;
        vec3f_t  position_ins;

        bool ok = true;
        ok &= fread(&magic, sizeof(magic), 1, fp) == 1;
        ok &= fread(&version, sizeof(version), 1, fp) == 1;
        ok &= fread(&num_sensors, sizeof(num_sensors), 1, fp) == 1;
        ok &= fread(&scan_loop_count, sizeof(scan_loop_count), 1, fp) == 1;
        ok &= fread(&frame_interval_us, sizeof(frame_interval_us), 1, fp) == 1;
        ok &= fread(&position_ins, sizeof(position_ins), 1, fp) == 1;

        if (ok && magic == SensorGroup_Impl::MAGIC && version == SensorGroup_Impl::VERSION)
        {
            sg = new SensorGroup_Impl(true);
            sg->initialize(false, frame_interval_us, position_ins);
            sg->frame_replay_file = fp;
            sg->scan_loop_count = scan_loop_count;

            ConnectionLogDesc log_desc;
            vec3f_t position;
            quatf_t pose;

            for (uint32_t i = 0; i < num_sensors; i++)
            {
                ok &= fread(&position, sizeof(position), 1, fp) == 1;
                ok &= fread(&pose, sizeof(pose), 1, fp) == 1;

                sg->add_sensor(0, log_desc);
                sg->set_sensor_position(i, position, pose);
            }

            if (!ok)
            {
                sg->close_and_release();
                sg = NULL;
                fclose(fp);
            }
        }
        else
        {
            fclose(fp);
        }
    }

    return sg;
}


FrameObject* SensorGroup_Impl::deserialize_frame(ScanSerializer& ser)
{
    FrameObject_Impl* frame = NULL;

    if (frame_replay_file)
    {
        vec3f_t ego_lin_vel;
        vec3f_t ego_ang_vel;

        bool ok = true;
        uint64_t timestamp;
        ok &= fread(&timestamp, sizeof(timestamp), 1, frame_replay_file) == 1;
        ok &= fread(&ego_lin_vel, sizeof(ego_lin_vel), 1, frame_replay_file) == 1;
        ok &= fread(&ego_ang_vel, sizeof(ego_ang_vel), 1, frame_replay_file) == 1;

        frame = new FrameObject_Impl(ego_lin_vel, ego_ang_vel, num_sensors, scan_loop_count, timestamp, frame_count++, frame_interval_us);

        for (uint32_t s = 0; ok && s < num_sensors; s++)
        {
            for (uint32_t i = 0; i < scan_loop_count; i++)
            {
                uint32_t scan_sequence_number;
                ok &= fread(&scan_sequence_number, sizeof(scan_sequence_number), 1, frame_replay_file) == 1;

                if (scan_sequence_number != uint32_t(-1))
                {
                    ScanObject* scan = ScanObject::deserialize(ser, scan_sequence_number);
                    if (scan)
                    {
                        frame->scans[s].scan[i] = static_cast<ScanObject_Impl*>(scan);
                    }
                    else
                    {
                        ok = false;
                    }
                }
            }
        }

        if (!ok && frame)
        {
            frame->release();
            frame = NULL;
        }
    }
    else
    {
        last_err = SG_UNABLE_TO_READ_FILE;
    }

    if (frame)
    {
        frame->finish_frame(sensor);
    }
    return frame;
}


// We put this Connection_Impl method here to prevent both files from including
// each other's IMPL headers
void Connection_Impl::poll_sensor_group()
{
    // poll sockets for 10ms, then check for retransmissions and frames
    mysensorgroup->poll_sensors(10000);
}
