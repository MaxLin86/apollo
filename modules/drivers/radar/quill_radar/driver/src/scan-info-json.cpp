#include "modules/drivers/radar/quill_radar/driver/system-radar-software/misc/rapidjson/document.h"
#include "modules/drivers/radar/quill_radar/driver/system-radar-software/misc/rapidjson/prettywriter.h"
#include "modules/drivers/radar/quill_radar/driver/system-radar-software/misc/rapidjson/stringbuffer.h"

#include "modules/drivers/radar/quill_radar/driver/system-radar-software/env-uhnder/coredefs/uhnder-common.h"
#include "modules/drivers/radar/quill_radar/driver/include/rra.h"
#include "modules/drivers/radar/quill_radar/driver/system-radar-software/engine/common/eng-api/uhtypes.h"
#include "modules/drivers/radar/quill_radar/driver/system-radar-software/engine/scp-src/eng-api/uhdp-msg-structs.h"
#include "modules/drivers/radar/quill_radar/driver/include/serializer.h"
#include "modules/drivers/radar/quill_radar/driver/src/scanobject_impl.h"
#include <math.h>

using namespace rapidjson;

bool ScanObject_Impl::deserialize_scan_info(ScanSerializer& s, size_t len)
{
    char* buf = new char[len + 2];

    if (!s.read_scan_data_type(buf, len, 1))
    {
        delete [] buf;
        return false;
    }

    if (buf[len - 1] != '\n')
        buf[len++] = '\n';
    buf[len] = 0;

    rapidjson::Document d;
    d.Parse(buf);

    delete [] buf;

    if (!d.IsObject())
    {
        return false;
    }

#define DO_FLOAT(var)  if (d.HasMember(#var) && d[#var].IsDouble()) scan_info.var = d[#var].GetDouble(); else scan_info.var = 0.0f
#define DO_INT(T, var) if (d.HasMember(#var) && d[#var].IsInt())    scan_info.var = (T)d[#var].GetInt(); else scan_info.var = 0
#define DO_INT64(T, var) if (d.HasMember(#var) && d[#var].IsInt64()) scan_info.var = (T)d[#var].GetInt64(); else scan_info.var = 0

    DO_INT64(uint64_t, rdc1_full_scale_value);
    DO_INT64(uint64_t, rdc2_full_scale_value);
    DO_INT64(uint64_t, rdc2ch_full_scale_value);
    DO_INT64(uint64_t, rdc3_full_scale_value);
    DO_INT(uint32_t, scan_sequence_number);
    DO_INT(uint32_t, scan_ID_number);
    DO_INT(uint32_t, scan_timestamp);
    DO_INT(uint32_t, current_time);
    DO_INT(uint32_t, device_id);
    DO_FLOAT(sample_rate);
    DO_INT(uint16_t, num_range_bins);
    DO_INT(uint16_t, num_pulses);
    DO_INT(uint16_t, total_vrx);
    DO_INT(uint16_t, num_beamforming_angles);
    DO_INT(uint16_t, num_channelizer_iters);
    DO_INT(uint16_t, num_channelizer_doppler_bins);
    DO_INT(uint16_t, num_detections);
    DO_INT(uint16_t, SS_size_R);
    DO_INT(uint16_t, SS_size_A);
    DO_INT(uint16_t, SS_size_D);
    DO_INT(uint16_t, CI_width);
    DO_INT(uint16_t, CI_height);
    DO_INT(uint16_t, CI_doppler_width);
    DO_INT(uint16_t, CI_doppler_height);
    DO_FLOAT(CI_pixel_size_in_meters);
    DO_INT(uint16_t, CI_bytes_per_pixel);
    DO_INT(uint16_t, CI_format);
    DO_INT(int16_t, rdc1_software_exponent);
    DO_INT(int16_t, rdc2_software_exponent);
    DO_INT(int16_t, rdc3_software_exponent);
    DO_INT(int16_t, clutter_image_exponent);
    DO_INT(int16_t, system_exponent);
    DO_INT(uint16_t, overflow_underflow_flags);
    DO_INT(uint16_t, num_music_instances);
    DO_INT(uint8_t, angle_wrap_flags);
    DO_INT(uint8_t, num_angle_groups);
    DO_FLOAT(scan_time);
    DO_FLOAT(pulse_time);
    DO_FLOAT(chip_time);
    DO_FLOAT(doppler_bin_width);
    DO_FLOAT(range_bin_width);
    DO_INT(int32_t, chips_per_pulse);
    DO_INT(int16_t, code_type);
    DO_INT(int16_t, vp_scan_mode);
    DO_FLOAT(ego_velocity_X);
    DO_FLOAT(ego_velocity_Y);
    DO_FLOAT(ego_velocity_Z);
    DO_FLOAT(estimated_ego_velocity_X);
    DO_FLOAT(estimated_ego_velocity_Y);
    DO_FLOAT(estimated_ego_velocity_Z);
    DO_INT(uint16_t, num_azimuth_angles);
    DO_INT(uint8_t, azimuth_nyquist_oversampling_factor);
    DO_INT(uint8_t, elevation_nyquist_oversampling_factor);
    DO_FLOAT(chip_temp_C);
    DO_FLOAT(analog_die_temp_C);
    DO_FLOAT(board_T3_temp_C);
    DO_INT(uint16_t, total_points);
    DO_INT(uint16_t, rdc2_zd_rb_center);
    DO_INT(uint16_t, rdc2_zd_rb_halfwidth);
    DO_FLOAT(activation_filter_snr);
    DO_INT(uint32_t, radar_status_bitmap);
    DO_INT(uint32_t, antenna_config_id);
    DO_INT(uint16_t, num_tx_prn);
    DO_INT(uint16_t, tx_power_map);
    DO_INT(uint16_t, tx_prn_map);
    DO_INT(uint8_t, rb_combine);
    DO_INT(uint8_t, ss_doppler_0_only);
    DO_INT(uint8_t, num_elevation_angles);
    DO_FLOAT(carrier_frequency);
    DO_FLOAT(vrx_position_offset_X);
    DO_FLOAT(vrx_position_offset_Y);
    DO_FLOAT(vrx_position_offset_Z);
    DO_INT(uint8_t, complex_rdc3);
    DO_FLOAT(peak_detector_output);

#undef DO_FLOAT
#undef DO_INT
#undef DO_INT64

    // fixup older scan info JSON files
    if (scan_info.num_elevation_angles == 0)
    {
        scan_info.num_elevation_angles = scan_info.num_beamforming_angles / scan_info.num_azimuth_angles;
    }

    if (d.HasMember("host_scan_end_sec"))  // new hotness
    {
        host_local_time.tv_sec = uint32_t(d["host_scan_end_sec"].GetInt());
        if (d.HasMember("host_scan_end_usec"))
        {
            host_local_time.tv_usec = uint32_t(d["host_scan_end_usec"].GetInt());
        }
        else
        {
            host_local_time.tv_usec = 0;
        }
    }
    else if (d.HasMember("host_datetime")) // old busted
    {
        // "host_datetime": "2017-11-20 14:47:40.990320"
        const char* timestr = d["host_datetime"].GetString();
        tm scan_host_time = {};
        char* res = strptime(timestr, "%Y-%m-%d %H:%M:%S", &scan_host_time);

        if (res && *res == '.' && strlen(res) > 6)
        {
            host_local_time.tv_usec = atof(res) * 1000 * 1000;
        }
        else
        {
            host_local_time.tv_usec = 0;
        }

#if __linux__
        // SJB - I am not at all happy about this, but it seems to be necessary
        // to keep the time from shifting by an hour when deserializing on Linux
        tzset();
        scan_host_time.tm_isdst = daylight;
#endif

        time_t time_here = mktime(&scan_host_time);
        host_local_time.tv_sec = time_here;
    }

    if (d.HasMember("dc_bias"))
    {
        const Value& arr = d["dc_bias"];

        if (arr.IsArray() && arr.Capacity() == 2 * 8 * 2) // cfloat, NUM_RX, 1D/2D
        {
            uint32_t v = 0;
            for (Value::ConstValueIterator itr = arr.Begin(); itr != arr.End(); ++itr)
            {
                scan_info.dc_bias[v/8][v%8].i = itr->GetDouble(); ++itr;
                scan_info.dc_bias[v/8][v%8].q = itr->GetDouble();
                v++;
            }
        }
    }
    else
    {
        memset(&scan_info.dc_bias[0][0], 0, sizeof(scan_info.dc_bias));
    }

    if (d.HasMember("srs_version"))
    {
        srs_version_str = strdup(d["srs_version"].GetString());
    }
    if (d.HasMember("module_name"))
    {
        module_name = strdup(d["module_name"].GetString());
    }
    if (d.HasMember("module_type_name"))
    {
        module_type_name = strdup(d["module_type_name"].GetString());
    }
    if (d.HasMember("motherboard_type_name"))
    {
        motherboard_type_name = strdup(d["motherboard_type_name"].GetString());
    }
    if (d.HasMember("antennaboard_type_name"))
    {
        antennaboard_type_name = strdup(d["antennaboard_type_name"].GetString());
    }
    if (d.HasMember("antenna_module_type_name"))
    {
        antenna_module_type_name = strdup(d["antenna_module_type_name"].GetString());
    }

    if (d.HasMember("uhdp_version") && d["uhdp_version"].IsInt())
    {
        uhdp_version = (uint8_t)d["uhdp_version"].GetInt();
    }
    else
    {
        uhdp_version = 0;
    }

    if (d.HasMember("thresholds"))
    {
        const Value& t = d["thresholds"];

#define DO_FLOAT(var)  if (t.HasMember(#var) && t[#var].IsDouble()) mythresh.var = t[#var].GetDouble(); else mythresh.var = 0.0f
        DO_FLOAT(detection_thresh_snr_dB);
        DO_FLOAT(clutter_image_thresh_snr_dB);
        DO_FLOAT(point_cloud_thresh_snr_dB);
        DO_FLOAT(scale_det_thresh_max_range);
        DO_FLOAT(scale_det_thresh_snr_adj);
        DO_FLOAT(ego_zero_stationary_threshold_mps);
        DO_FLOAT(ego_nonz_stationary_threshold_mps);
        DO_FLOAT(notch_width_radians);
        DO_FLOAT(notch_depth_dB);
        DO_FLOAT(outer_depth_dB);
        DO_FLOAT(ridge_extra_thresh_dB);
#undef DO_FLOAT
    }

    if (d.HasMember("rx_pos") && scan_info.num_tx_prn)
    {
        const Value& arr = d["rx_pos"];

        uint32_t total_rx = scan_info.total_vrx / scan_info.num_tx_prn;

        if (!arr.IsArray() || arr.Capacity() != total_rx * 3)
        {
            return false;
        }

        rx_pos = new vec3f_t[total_rx];
        uint32_t v = 0;
        for (Value::ConstValueIterator itr = arr.Begin(); itr != arr.End(); ++v)
        {
            rx_pos[v].x = itr->GetFloat(); ++itr;
            rx_pos[v].y = itr->GetFloat(); ++itr;
            rx_pos[v].z = itr->GetFloat(); ++itr;
        }
    }

    if (d.HasMember("tx_pos") && scan_info.num_tx_prn)
    {
        const Value& arr = d["tx_pos"];

        if (!arr.IsArray() || arr.Capacity() != scan_info.num_tx_prn * 3)
        {
            return false;
        }

        tx_pos = new vec3f_t[scan_info.num_tx_prn];
        uint32_t v = 0;
        for (Value::ConstValueIterator itr = arr.Begin(); itr != arr.End(); ++v)
        {
            tx_pos[v].x = itr->GetFloat(); ++itr;
            tx_pos[v].y = itr->GetFloat(); ++itr;
            tx_pos[v].z = itr->GetFloat(); ++itr;
        }
    }

    if (d.HasMember("mount_geometry"))
    {
        const Value& arr = d["mount_geometry"];

        if (!arr.IsArray() || arr.Capacity() != 5)
        {
            return false;
        }

        Value::ConstValueIterator itr = arr.Begin();
        rear_axle_distance = itr->GetFloat(); ++itr;
        centerline_distance = itr->GetFloat(); ++itr;
        mount_height = itr->GetFloat(); ++itr;
        mount_azimuth = itr->GetFloat(); ++itr;
        mount_elevation = itr->GetFloat(); ++itr;
    }

    // Not loading "rra_version": "scc-v0.4.18-Au+242-3e2e73f6cfe7"

    if (d.HasMember("vrx_map"))
    {
        const Value& arr = d["vrx_map"];

        if (!arr.IsArray() || arr.Capacity() != scan_info.total_vrx)
        {
            return false;
        }

        uint32_t v = 0;
        vrx_mapping = new uint8_t[scan_info.total_vrx];
        for (Value::ConstValueIterator itr = arr.Begin(); itr != arr.End(); ++itr, ++v)
            vrx_mapping[v] = itr->GetInt();
        return true;
    }
    else
    {
        return false;
    }
}


bool ScanObject_Impl::serialize_scan_info(ScanSerializer& s) const
{
    Document d;
    d.SetObject();

#define DO_INT(T, var) d.AddMember(#var, scan_info.var, d.GetAllocator())
#define DO_INT64(T, var) d.AddMember(#var, scan_info.var, d.GetAllocator())
#define DO_FLOAT(var)  d.AddMember(#var, scan_info.var, d.GetAllocator())

    DO_INT64(uint64_t, rdc1_full_scale_value);
    DO_INT64(uint64_t, rdc2_full_scale_value);
    DO_INT64(uint64_t, rdc2ch_full_scale_value);
    DO_INT64(uint64_t, rdc3_full_scale_value);
    DO_INT(uint32_t, scan_sequence_number);
    DO_INT(uint32_t, scan_ID_number);
    DO_INT(uint32_t, scan_timestamp);
    DO_INT(uint32_t, current_time);
    DO_INT(uint32_t, device_id);
    DO_FLOAT(sample_rate);
    DO_INT(uint16_t, num_range_bins);
    DO_INT(uint16_t, num_pulses);
    DO_INT(uint16_t, total_vrx);
    DO_INT(uint16_t, num_beamforming_angles);
    DO_INT(uint16_t, num_channelizer_iters);
    DO_INT(uint16_t, num_channelizer_doppler_bins);
    DO_INT(uint16_t, num_detections);
    DO_INT(uint16_t, SS_size_R);
    DO_INT(uint16_t, SS_size_A);
    DO_INT(uint16_t, SS_size_D);
    DO_INT(uint16_t, CI_width);
    DO_INT(uint16_t, CI_height);
    DO_INT(uint16_t, CI_doppler_width);
    DO_INT(uint16_t, CI_doppler_height);
    DO_FLOAT(CI_pixel_size_in_meters);
    DO_INT(uint16_t, CI_bytes_per_pixel);
    DO_INT(uint16_t, CI_format);
    DO_INT(int16_t, rdc1_software_exponent);
    DO_INT(int16_t, rdc2_software_exponent);
    DO_INT(int16_t, rdc3_software_exponent);
    DO_INT(int16_t, clutter_image_exponent);
    DO_INT(int16_t, system_exponent);
    DO_INT(uint16_t, overflow_underflow_flags);
    DO_INT(uint16_t, num_music_instances);
    DO_INT(uint8_t, angle_wrap_flags);
    DO_INT(uint8_t, num_angle_groups);
    DO_FLOAT(scan_time);
    DO_FLOAT(pulse_time);
    DO_FLOAT(chip_time);
    DO_FLOAT(doppler_bin_width);
    DO_FLOAT(range_bin_width);
    DO_INT(int32_t, chips_per_pulse);
    DO_INT(int16_t, code_type);
    DO_INT(int16_t, vp_scan_mode);
    DO_FLOAT(ego_velocity_X);
    DO_FLOAT(ego_velocity_Y);
    DO_FLOAT(ego_velocity_Z);
    DO_FLOAT(estimated_ego_velocity_X);
    DO_FLOAT(estimated_ego_velocity_Y);
    DO_FLOAT(estimated_ego_velocity_Z);
    DO_INT(uint16_t, num_azimuth_angles);
    DO_INT(uint8_t, azimuth_nyquist_oversampling_factor);
    DO_INT(uint8_t, elevation_nyquist_oversampling_factor);
    DO_FLOAT(chip_temp_C);
    DO_FLOAT(analog_die_temp_C);
    DO_FLOAT(board_T3_temp_C);
    DO_INT(uint16_t, total_points);
    DO_INT(uint16_t, rdc2_zd_rb_center);
    DO_INT(uint16_t, rdc2_zd_rb_halfwidth);
    DO_FLOAT(activation_filter_snr);
    DO_INT(uint32_t, radar_status_bitmap);
    DO_INT(uint32_t, antenna_config_id);
    DO_INT(uint16_t, num_tx_prn);
    DO_INT(uint16_t, tx_power_map);
    DO_INT(uint16_t, tx_prn_map);
    DO_INT(uint8_t, rb_combine);
    DO_INT(uint8_t, ss_doppler_0_only);
    DO_INT(uint8_t, num_elevation_angles);
    DO_FLOAT(carrier_frequency);
    DO_FLOAT(vrx_position_offset_X);
    DO_FLOAT(vrx_position_offset_Y);
    DO_FLOAT(vrx_position_offset_Z);
    DO_INT(uint8_t, complex_rdc3);
    DO_FLOAT(peak_detector_output);

#undef DO_FLOAT
#undef DO_INT
#undef DO_INT64

    Value a(kArrayType);
    Document::AllocatorType& allocator = d.GetAllocator();
    for (uint32_t i = 0; i < scan_info.total_vrx; i++)
    {
        a.PushBack(vrx_mapping[i], allocator);
    }
    d.AddMember("vrx_map", a, allocator);

    char timebuf[128];
    time_t local_time = host_local_time.tv_sec;
    tm* scan_host_time = localtime(&local_time);
    size_t bytes = strftime(timebuf, sizeof(timebuf), "%Y-%m-%d %H:%M:%S", scan_host_time);
    sprintf(timebuf + bytes, ".%06u", (uint32_t)host_local_time.tv_usec);
    d.AddMember("host_datetime", StringRef((const char*)&timebuf[0]), d.GetAllocator());

    d.AddMember("host_scan_end_sec",  uint32_t(host_local_time.tv_sec),  d.GetAllocator());
    d.AddMember("host_scan_end_usec", uint32_t(host_local_time.tv_usec), d.GetAllocator());

    Value dc(kArrayType);
    for (uint32_t d12 = 0; d12 < 2; d12++)
    {
        for (uint32_t rx = 0; rx < 8; rx++)
        {
            float i = scan_info.dc_bias[d12][rx].i;
            float q = scan_info.dc_bias[d12][rx].q;
            if (!std::isnormal(i))
            {
                i = 0.0f;
            }
            if (!std::isnormal(q))
            {
                q = 0.0f;
            }
            dc.PushBack(i, allocator);
            dc.PushBack(q, allocator);
        }
    }
    d.AddMember("dc_bias", dc, allocator);

    if (srs_version_str)
    {
        d.AddMember("srs_version", StringRef(srs_version_str), d.GetAllocator());
    }
    if (module_name)
    {
        d.AddMember("module_name", StringRef(module_name), d.GetAllocator());
    }
    if (module_type_name)
    {
        d.AddMember("module_type_name", StringRef(module_type_name), d.GetAllocator());
    }
    if (motherboard_type_name)
    {
        d.AddMember("motherboard_type_name", StringRef(motherboard_type_name), d.GetAllocator());
    }
    if (antennaboard_type_name)
    {
        d.AddMember("antennaboard_type_name", StringRef(antennaboard_type_name), d.GetAllocator());
    }
    if (antenna_module_type_name)
    {
        d.AddMember("antenna_module_type_name", StringRef(antenna_module_type_name), d.GetAllocator());
    }

    d.AddMember("uhdp_version", uhdp_version, d.GetAllocator());

    {
        Value t(kObjectType);

#define DO_FLOAT(var)  t.AddMember(#var, mythresh.var, d.GetAllocator())
        DO_FLOAT(detection_thresh_snr_dB);
        DO_FLOAT(clutter_image_thresh_snr_dB);
        DO_FLOAT(point_cloud_thresh_snr_dB);
        DO_FLOAT(scale_det_thresh_max_range);
        DO_FLOAT(scale_det_thresh_snr_adj);
        DO_FLOAT(ego_zero_stationary_threshold_mps);
        DO_FLOAT(ego_nonz_stationary_threshold_mps);
        DO_FLOAT(notch_width_radians);
        DO_FLOAT(notch_depth_dB);
        DO_FLOAT(outer_depth_dB);
        DO_FLOAT(ridge_extra_thresh_dB);
#undef DO_FLOAT

        d.AddMember("thresholds", t, allocator);
    }

    if (scan_info.num_tx_prn && tx_pos)
    {
        Value a(kArrayType);
        Document::AllocatorType& allocator = d.GetAllocator();
        for (uint32_t i = 0; i < scan_info.num_tx_prn; i++)
        {
            a.PushBack(tx_pos[i].x, allocator);
            a.PushBack(tx_pos[i].y, allocator);
            a.PushBack(tx_pos[i].z, allocator);
        }
        d.AddMember("tx_pos", a, allocator);
    }

    if (scan_info.num_tx_prn && rx_pos)
    {
        Value a(kArrayType);
        Document::AllocatorType& allocator = d.GetAllocator();
        for (uint32_t i = 0; i < (uint32_t)(scan_info.total_vrx / scan_info.num_tx_prn); i++)
        {
            a.PushBack(rx_pos[i].x, allocator);
            a.PushBack(rx_pos[i].y, allocator);
            a.PushBack(rx_pos[i].z, allocator);
        }
        d.AddMember("rx_pos", a, allocator);
    }

    {
        Value a(kArrayType);
        Document::AllocatorType& allocator = d.GetAllocator();
        a.PushBack(rear_axle_distance, allocator);
        a.PushBack(centerline_distance, allocator);
        a.PushBack(mount_height, allocator);
        a.PushBack(mount_azimuth, allocator);
        a.PushBack(mount_elevation, allocator);
        d.AddMember("mount_geometry", a, allocator);
    }

    d.AddMember("rra_version", StringRef(get_rra_version_string()), d.GetAllocator());

    StringBuffer buffer;
    PrettyWriter<StringBuffer> writer(buffer);
    writer.SetFormatOptions(kFormatSingleLineArray);
    d.Accept(writer);
    const char* output = buffer.GetString();
    return s.write_scan_data_type(output, strlen(output), 1);
}
