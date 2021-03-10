#include "modules/drivers/radar/rocket_radar/driver/system-radar-software/misc/rapidjson/document.h"
#include "modules/drivers/radar/rocket_radar/driver/system-radar-software/misc/rapidjson/prettywriter.h"
#include "modules/drivers/radar/rocket_radar/driver/system-radar-software/misc/rapidjson/stringbuffer.h"

#include "modules/drivers/radar/rocket_radar/driver/system-radar-software/env-reference/coredefs/uhnder-common.h"
#include "modules/drivers/radar/rocket_radar/driver/include/rra.h"
#include "modules/drivers/radar/rocket_radar/driver/system-radar-software/engine/common/eng-api/uhtypes.h"
#include "modules/drivers/radar/rocket_radar/driver/system-radar-software/engine/scp-src/eng-api/uhdp-msg-structs.h"
#include "modules/drivers/radar/rocket_radar/driver/include/serializer.h"
#include "modules/drivers/radar/rocket_radar/driver/src/scanobject_impl.h"
#include <math.h>

using namespace rapidjson;

struct UhdpScanInformation_075_UhDP_33
{
    uint64_t   rdc1_full_scale_value;
    uint64_t   rdc2_full_scale_value;
    uint64_t   rdc2ch_full_scale_value;
    uint64_t   rdc3_full_scale_value;

    uint32_t   scan_sequence_number;    // UHDP sequence number
    uint32_t   scan_ID_number;          // RHAL sequence number
    uint32_t   scan_timestamp;          // radar time at scan end
    uint32_t   current_time;            // radar time at info message send

    FLOAT      estimated_ego_velocity_X;
    FLOAT      estimated_ego_velocity_Y;
    FLOAT      estimated_ego_velocity_Z;
    FLOAT      ego_velocity_X;
    FLOAT      ego_velocity_Y;
    FLOAT      ego_velocity_Z;

    uint32_t   device_id;
    FLOAT      scan_time;               // dwell time
    FLOAT      pulse_time;              // PRI time
    FLOAT      chip_time;               // seconds
    FLOAT      sample_rate;             // Hz
    FLOAT      doppler_bin_width;       // mps
    FLOAT      range_bin_width;         // meters
    int32_t    chips_per_pulse;

    uint16_t   num_tx_prn;              // number of tx codes that were active in this scan
    int16_t    vp_scan_mode;            // enum RHAL_ScanModes VP_INVALID(-1) if not variable power
    int16_t    code_type;
    uint16_t   total_vrx;
    uint16_t   num_range_bins;
    uint16_t   num_pulses;
    uint16_t   num_channelizer_iters;
    uint16_t   num_channelizer_doppler_bins;
    uint16_t   num_beamforming_angles;  // Total number of angle-bins including padding (ss2_num_angles)
    uint16_t   num_azimuth_angles;      // In-use azimuth angles
                                        // Total in-use angles (num_angles) = num_azimuth_angles * num_elevation_angles
    uint8_t    num_angle_groups;        // number of RAU passes on RDC2
    uint8_t    azimuth_nyquist_oversampling_factor;
    uint8_t    elevation_nyquist_oversampling_factor;
    uint8_t    angle_wrap_flags;

    int16_t    rdc1_software_exponent;
    int16_t    rdc2_software_exponent;
    int16_t    rdc3_software_exponent;
    int16_t    system_exponent;

    uint16_t   overflow_underflow_flags;
    uint16_t   num_detections;          // number of output detections
    uint16_t   total_points;            // number of output point cloud points TBD
    uint16_t   num_music_instances;     // number of SVD and COV outputs

    uint16_t   SS_size_R;
    uint16_t   SS_size_A;
    uint16_t   SS_size_D;
    uint16_t   CI_width;
    uint16_t   CI_height;
    uint16_t   CI_format;
    uint16_t   CI_doppler_width;
    uint16_t   CI_doppler_height;
    uint16_t   CI_bytes_per_pixel;
    int16_t    clutter_image_exponent;
    FLOAT      CI_pixel_size_in_meters;

    FLOAT      chip_temp_C;            // PVT reported D-Die temperature
    FLOAT      analog_die_temp_C;      // analog probe measured A-Die temperature
    FLOAT      board_T3_temp_C;
    uint16_t   rdc2_zd_rb_center;
    uint16_t   rdc2_zd_rb_halfwidth;

    cfloat     dc_bias[2][8];          // 1D, 2D. For all 8 RX

    FLOAT      activation_filter_snr;  // magnitude of smallest saved activation (dB)
    uint32_t   radar_status_bitmap;
    uint32_t   antenna_config_id;
    uint8_t    rb_combine;
    uint8_t    complex_rdc3;           // RDC_rdc3_complex bits
    uint8_t    ss_doppler_0_only;
    uint8_t    num_elevation_angles;

    FLOAT      vrx_position_offset_X;
    FLOAT      vrx_position_offset_Y;
    FLOAT      vrx_position_offset_Z;
    FLOAT      carrier_frequency;

    uint16_t   tx_power_map;           // bitmap of TX powered on (HW12); 1 bit means ON
    uint16_t   tx_prn_map;             // bitmap of TX transmiting (HW12); 1 bit means ON

    FLOAT      peak_detector_output;
    FLOAT      dop_rotator_shift;

    uint32_t   reserved_u32[13];
};


bool ScanObject_Impl::deserialize_scan_info_bin(ScanSerializer& s, size_t bytes, bool& format_075)
{
    bool ok = true;

    if (format_075)
    {
        memset(&scan_info, 0, sizeof(scan_info));
        ok &= s.read_scan_data_type(&scan_info, sizeof(UhdpScanInformation_075_UhDP_33), 1);
        // when this scan info is re-serialized with the 0.7.6 scan info size,
        // we need a non-zero scan_loop_size to prevent re-expanding the struct
        scan_info.scan_loop_size = 1;
        scan_info.connection_uhdp_version = 0;
    }
    else
    {
        ok &= s.read_scan_data_type(&scan_info, sizeof(scan_info), 1);
        if (ok == true && scan_info.scan_loop_size == 0)
        {
            // this is a 0.7.5 info.bin file
            format_075 = true;
            ok = false;
            return ok;
        }
    }

    ok &= s.read_scan_data_type(&mythresh, sizeof(mythresh), 1);
    uint64_t sec, usec;
    ok &= s.read_scan_data_type(&sec, sizeof(sec), 1);
    ok &= s.read_scan_data_type(&usec, sizeof(usec), 1);
    host_local_time.tv_sec = time_t(sec);
    host_local_time.tv_usec = usec;

    if (ok)
    {
        uint32_t total_rx = scan_info.total_vrx / scan_info.num_tx_prn;
        vrx_mapping = new uint8_t[scan_info.total_vrx];
        tx_pos = new vec3f_t[scan_info.num_tx_prn];
        rx_pos = new vec3f_t[total_rx];
    }
    else
    {
        return ok;
    }
    ok &= s.read_scan_data_type(&vrx_mapping[0], sizeof(vrx_mapping[0]), scan_info.total_vrx);
    ok &= s.read_scan_data_type(&tx_pos[0], sizeof(tx_pos[0]), scan_info.num_tx_prn);
    ok &= s.read_scan_data_type(&rx_pos[0], sizeof(rx_pos[0]), scan_info.total_vrx / scan_info.num_tx_prn);
    ok &= s.read_scan_data_type(&rear_axle_distance, sizeof(rear_axle_distance), 1);
    ok &= s.read_scan_data_type(&centerline_distance, sizeof(centerline_distance), 1);
    ok &= s.read_scan_data_type(&mount_height, sizeof(mount_height), 1);
    ok &= s.read_scan_data_type(&mount_azimuth, sizeof(mount_azimuth), 1);
    ok &= s.read_scan_data_type(&mount_elevation, sizeof(mount_elevation), 1);
    ok &= s.read_scan_data_type(&uhdp_version, sizeof(uhdp_version), 1);
    if (!ok) return ok;

    if (!scan_info.connection_uhdp_version)
    {
        scan_info.connection_uhdp_version = uhdp_version;
    }

#define DE_SERIALIZE_STRING(S) \
        ok &= s.read_scan_data_type(&len, sizeof(len), 1); \
        if (ok && len) { \
            char* temp = (char*)malloc(len + 1); \
            if (temp) { \
                ok &= s.read_scan_data_type(temp, len, 1); \
                temp[len] = 0; \
                S = temp; \
            } else { return false; } \
        }

    uint16_t len;
    DE_SERIALIZE_STRING(srs_version_str)
    DE_SERIALIZE_STRING(module_name)
    DE_SERIALIZE_STRING(module_type_name)
    DE_SERIALIZE_STRING(motherboard_type_name)
    DE_SERIALIZE_STRING(antennaboard_type_name)
    DE_SERIALIZE_STRING(antenna_module_type_name)
#undef DE_SERIALIZE_STRING
    return ok;
}


bool ScanObject_Impl::serialize_scan_info_bin(ScanSerializer& s) const
{
    bool ok = true;
    uint64_t sec, usec;
    sec = uint64_t(host_local_time.tv_sec);
    usec = uint64_t(host_local_time.tv_usec);
    ok &= s.write_scan_data_type(&scan_info, sizeof(scan_info), 1);
    ok &= s.write_scan_data_type(&mythresh, sizeof(mythresh), 1);
    ok &= s.write_scan_data_type(&sec, sizeof(sec), 1);
    ok &= s.write_scan_data_type(&usec, sizeof(usec), 1);
    ok &= s.write_scan_data_type(&vrx_mapping[0], sizeof(vrx_mapping[0]), scan_info.total_vrx);
    ok &= s.write_scan_data_type(&tx_pos[0], sizeof(tx_pos[0]), scan_info.num_tx_prn);
    ok &= s.write_scan_data_type(&rx_pos[0], sizeof(rx_pos[0]), scan_info.total_vrx / scan_info.num_tx_prn);
    ok &= s.write_scan_data_type(&rear_axle_distance, sizeof(rear_axle_distance), 1);
    ok &= s.write_scan_data_type(&centerline_distance, sizeof(centerline_distance), 1);
    ok &= s.write_scan_data_type(&mount_height, sizeof(mount_height), 1);
    ok &= s.write_scan_data_type(&mount_azimuth, sizeof(mount_azimuth), 1);
    ok &= s.write_scan_data_type(&mount_elevation, sizeof(mount_elevation), 1);
    ok &= s.write_scan_data_type(&uhdp_version, sizeof(uhdp_version), 1);
    if (!ok) return ok;

#define SERIALIZE_STRING(S) \
    if (S) len = uint16_t(strlen(S)); else len = 0; \
    ok &= s.write_scan_data_type(&len, sizeof(len), 1); \
    if (len) ok &= s.write_scan_data_type(S, len, 1);

    uint16_t len;
    SERIALIZE_STRING(srs_version_str)
    SERIALIZE_STRING(module_name)
    SERIALIZE_STRING(module_type_name)
    SERIALIZE_STRING(motherboard_type_name)
    SERIALIZE_STRING(antennaboard_type_name)
    SERIALIZE_STRING(antenna_module_type_name)
#undef SERIALIZE_STRING
    return ok;
}


void ScanObject_Impl::handle_scan_info(const UhdpScanInformation* msg, uint32_t total_size, const timeval& ht, uint32_t uhdp_ver)
{
    host_local_time = ht;
    uhdp_version = uhdp_ver;
    memcpy(&scan_info, msg, sizeof(UhdpScanInformation));
    const char* payload = (const char*)(msg + 1);
    total_size -= sizeof(UhdpScanInformation);

    // fixup num_elevation_angles from older radars
    if (scan_info.num_elevation_angles == 0)
    {
        scan_info.num_elevation_angles = scan_info.num_beamforming_angles / scan_info.num_azimuth_angles;
    }

    assert(uhdp_ver >= 25);

    memcpy(&mythresh, payload, sizeof(RDC_ThresholdControl));
    total_size -= sizeof(RDC_ThresholdControl);
    payload    += sizeof(RDC_ThresholdControl);

    if (total_size >= scan_info.total_vrx)
    {
        vrx_mapping = new uint8_t[scan_info.total_vrx];
        memcpy(vrx_mapping, payload, scan_info.total_vrx * sizeof(uint8_t));

        total_size -= scan_info.total_vrx;
        payload    += scan_info.total_vrx;
    }

    if (scan_info.num_tx_prn)
    {
        uint32_t num_rx = scan_info.total_vrx / scan_info.num_tx_prn;
        uint32_t rxsize = num_rx * sizeof(vec3f_t);
        uint32_t txsize = scan_info.num_tx_prn * sizeof(vec3f_t);

        if (total_size >= rxsize)
        {
            rx_pos = new vec3f_t[num_rx];
            memcpy(rx_pos, payload, rxsize);

            total_size -= rxsize;
            payload    += rxsize;
        }

        if (total_size >= txsize)
        {
            tx_pos = new vec3f_t[scan_info.num_tx_prn];
            memcpy(tx_pos, payload, txsize);

            total_size -= txsize;
            payload    += txsize;
        }
    }
}


void                       ScanObject_Impl::serialize_scan_info_json(ScanSerializer& s) const
{
    bool ok = s.begin_write_scan(scan_info.scan_sequence_number);
    if (!ok)
    {
        return;
    }

    char fname[128];
    sprintf(fname, "scan_%06d_info.json", scan_info.scan_sequence_number);
    ok = s.begin_write_scan_data_type(fname);
    if (ok)
    {
        ok &= serialize_scan_info(s);
        s.end_write_scan_data_type(!ok);
    }

    s.end_write_scan(!ok);
}


const char*                ScanObject_Impl::get_scan_info_as_json() const
{
    static char buffer[1024 * 16];
    char fname[128];
    char* buf = NULL;

    MemorySerializer memser(buffer, sizeof(buffer));

    // serialize scan info into static buffer
    memser.begin_write_scan(scan_info.scan_sequence_number);
    sprintf(fname, "scan_%06d_info.json", scan_info.scan_sequence_number);
    memser.begin_write_scan_data_type(fname);
    serialize_scan_info(memser);
    memser.end_write_scan_data_type(false);
    memser.end_write_scan(false);

    // deserialize to heap buffer
    memser.begin_read_scan(scan_info.scan_sequence_number);
    size_t len = memser.begin_read_scan_data_type(fname);
    if (len)
    {
        buf = new char[len + 1];
        memser.read_scan_data_type(buf, len, 1);
        memser.end_read_scan_data_type();
        buf[len] = 0;
    }
    memser.end_read_scan();
    return buf;
}


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
    DO_FLOAT(ego_linear_velocity_X);
    DO_FLOAT(ego_linear_velocity_Y);
    DO_FLOAT(ego_linear_velocity_Z);
    DO_INT(uint8_t, estimated_ego_flag);
    DO_FLOAT(ego_angular_velocity_X);
    DO_FLOAT(ego_angular_velocity_Y);
    DO_FLOAT(ego_angular_velocity_Z);
    DO_FLOAT(estimated_ego_velocity_X);
    DO_FLOAT(estimated_ego_velocity_Y);
    DO_FLOAT(estimated_ego_velocity_Z);
    DO_FLOAT(extrapolated_ego_velocity_X);
    DO_FLOAT(extrapolated_ego_velocity_Y);
    DO_FLOAT(extrapolated_ego_velocity_Z);
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
    DO_INT(uint8_t, preset_applied);
    DO_INT(uint8_t, preset_diff_flags);
    DO_INT(uint8_t, rb_combine);
    DO_INT(uint8_t, ss_doppler_0_only);
    DO_INT(uint8_t, num_elevation_angles);
    DO_FLOAT(carrier_frequency);
    DO_FLOAT(vrx_position_offset_X);
    DO_FLOAT(vrx_position_offset_Y);
    DO_FLOAT(vrx_position_offset_Z);
    DO_INT(uint8_t, complex_rdc3);
    DO_FLOAT(peak_detector_output);
    DO_FLOAT(dop_rotator_shift);
    DO_INT(uint8_t, scan_loop_size);
    DO_INT(uint8_t, scan_loop_idx);
    DO_INT(uint8_t, connection_uhdp_version);
    DO_INT(uint16_t, clock_tick_numerator);
    DO_INT(uint16_t, clock_tick_denominator);
    DO_INT(uint16_t, num_RD_upper);
    DO_INT(uint16_t, num_RD_lower);
    DO_INT(uint16_t, num_RD_above_cutoff);
    DO_FLOAT(az_vrx_spacing_lambda);
    DO_FLOAT(el_vrx_spacing_lambda);
    DO_INT(uint16_t, az_uniform_vrx);
    DO_INT(uint16_t, el_uniform_vrx);
    DO_INT(uint32_t, user_data);

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

        // Copy version in case the scan info is re-serialized
        if (!scan_info.connection_uhdp_version)
        {
            scan_info.connection_uhdp_version = uhdp_version;
        }
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
    DO_FLOAT(ego_linear_velocity_X);
    DO_FLOAT(ego_linear_velocity_Y);
    DO_FLOAT(ego_linear_velocity_Z);
    DO_INT(uint8_t, estimated_ego_flag);
    DO_FLOAT(ego_angular_velocity_X);
    DO_FLOAT(ego_angular_velocity_Y);
    DO_FLOAT(ego_angular_velocity_Z);
    DO_FLOAT(estimated_ego_velocity_X);
    DO_FLOAT(estimated_ego_velocity_Y);
    DO_FLOAT(estimated_ego_velocity_Z);
    DO_FLOAT(extrapolated_ego_velocity_X);
    DO_FLOAT(extrapolated_ego_velocity_Y);
    DO_FLOAT(extrapolated_ego_velocity_Z);
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
    DO_INT(uint8_t, preset_applied);
    DO_INT(uint8_t, preset_diff_flags);
    DO_INT(uint8_t, rb_combine);
    DO_INT(uint8_t, ss_doppler_0_only);
    DO_INT(uint8_t, num_elevation_angles);
    DO_FLOAT(carrier_frequency);
    DO_FLOAT(vrx_position_offset_X);
    DO_FLOAT(vrx_position_offset_Y);
    DO_FLOAT(vrx_position_offset_Z);
    DO_INT(uint8_t, complex_rdc3);
    DO_FLOAT(peak_detector_output);
    DO_FLOAT(dop_rotator_shift);
    DO_INT(uint8_t, scan_loop_size);
    DO_INT(uint8_t, scan_loop_idx);
    DO_INT(uint8_t, connection_uhdp_version);
    DO_INT(uint16_t, clock_tick_numerator);
    DO_INT(uint16_t, clock_tick_denominator);
    DO_INT(uint16_t, num_RD_upper);
    DO_INT(uint16_t, num_RD_lower);
    DO_INT(uint16_t, num_RD_above_cutoff);
    DO_FLOAT(az_vrx_spacing_lambda);
    DO_FLOAT(el_vrx_spacing_lambda);
    DO_INT(uint16_t, az_uniform_vrx);
    DO_INT(uint16_t, el_uniform_vrx);
    DO_INT(uint32_t, user_data);

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
