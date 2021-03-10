#include "modules/drivers/radar/rocket_radar/driver/include/rra.h"
#include "modules/drivers/radar/rocket_radar/driver/include/scanning.h"
#include "modules/drivers/radar/rocket_radar/driver/include/scanobject.h"
#include "modules/drivers/radar/rocket_radar/driver/include/serializer.h"
#include "modules/drivers/radar/rocket_radar/driver/src/scanobject_impl.h"
#include "modules/drivers/radar/rocket_radar/driver/src/pointcloud_impl.h"
#include "modules/drivers/radar/rocket_radar/driver/src/clutterimage_impl.h"
#include "modules/drivers/radar/rocket_radar/driver/system-radar-software/engine/scp-src/eng-api/uhmath.h"
#include "modules/drivers/radar/rocket_radar/driver/system-radar-software/engine/common/eng-api/rdc-structs.h" //PointCloudData


PointCloud_Impl::~PointCloud_Impl()
{
    delete [] points;
}


void  PointCloud_Impl::get_points(Point* output)
{
    if (!output)
    {
        return;
    }

    float azimuths_rad[MAX_MAX_ROUGH_ANGLES];
    float elevations_rad[MAX_MAX_ROUGH_ANGLES];

    const UhdpScanInformation& info = myscan.scan_info;
    const RDC_ThresholdControl& thresh = myscan.mythresh;

    // ego velocity derived from sensors connected to CAN bus
    vec3f_t can  = vec3f_t(info.ego_linear_velocity_X, info.ego_linear_velocity_Y, info.ego_linear_velocity_Z);

    // ego velocity derived from static slice histogram analysis
    vec3f_t hist = vec3f_t(info.estimated_ego_velocity_X, info.estimated_ego_velocity_Y, info.estimated_ego_velocity_Z);
    if (myscan.uhdp_version > 35 && (info.estimated_ego_flag == 0))
    {
        hist = 0;
    }

    bool ego_vel_is_zero = can.iszero() && (hist.abs() <= 0.1 * myscan.scan_info.doppler_bin_width);
    float stationary_thresh = ego_vel_is_zero ? thresh.ego_zero_stationary_threshold_mps : thresh.ego_nonz_stationary_threshold_mps;

    uint32_t AZ = info.num_azimuth_angles;
    uint32_t EL = info.num_elevation_angles;
    uint32_t zero_doppler_bin = info.num_pulses / 2;

    for (uint32_t i = 0; i < info.num_beamforming_angles; i++)
    {
        myscan.get_angle(i, azimuths_rad[i], elevations_rad[i]);
    }
    // safety check that angle-bins were received for this scan
    if (myscan.get_last_error() != ScanObject::SCAN_NO_ERROR)
    {
        return;
    }

    for (uint32_t i = 0; i < num_points; i++)
    {
        PointCloudData& p = points[i];
        Point& o = output[i];

        o.range = p.range * info.range_bin_width;
        o.doppler = info.doppler_bin_width * (p.doppler_bin - (info.num_pulses / 2));
        o.mag_snr = (float)p.snr_dB / (1 << PointCloudData::PC_SNR_FRAC_BITS);
        o.flags = p.flags;
        if (p.mag_i || p.mag_q)
        {
            float exp_mag = db2mag(TWO_IN_DB * (FLOAT)p.exponent);
            o.mag_i = float(p.mag_i) * exp_mag;
            o.mag_q = float(p.mag_q) * exp_mag;
        }
        else
        {
            o.mag_i = 0.0f;
            o.mag_q = 0.0f;
        }

        float az = (float)p.azimuth_fbin / (1 << PointCloudData::PC_AZIMUTH_FRAC_BITS);
        int az_bin_lo = (int)az;
        float az_fract = az - az_bin_lo;
        float az_lo = azimuths_rad[az_bin_lo];
        float az_delta;
        if (az_bin_lo == (int)AZ - 1)
        {
            // assume a wrap situation, pretend there is an azimuth angle at -azimuth_angles[0]
            az_delta = azimuths_rad[1] - azimuths_rad[0];
        }
        else
        {
            az_delta = azimuths_rad[az_bin_lo + 1] - az_lo;
        }
        o.azimuth = az_lo + (az_delta * az_fract);

        float el = (float)p.elevation_fbin / (1 << PointCloudData::PC_ELEVATION_FRAC_BITS);
        int el_bin_lo = (int)el;
        float el_fract = el - el_bin_lo;
        float el_lo = elevations_rad[el_bin_lo * AZ];
        float el_delta;
        if (el_bin_lo == (int)EL - 1)
        {
            // assume a wrap situation, pretend there is an elevation angle at -elevations_rad[0]
            el_delta = elevations_rad[AZ * 1] - elevations_rad[AZ * 0];
        }
        else
        {
            el_delta = elevations_rad[(el_bin_lo + 1) * AZ] - el_lo;
        }
        o.elevation = el_lo + el_delta * el_fract;

        // NOTE: This code is adapted from RDC layer detection iterator
        if (stationary_thresh == 0.0f)
        {
            if (o.flags & PF_CLUTTER)
            {
                o.flags |= PF_STATIONARY;
            }
        }
        else
        {
            FLOAT zero_doppler_at_angle;

            // The new behavior is for RDC_DET_FLAG_STATIC to indicate that the detection
            // is within a threshold Doppler velocity of being stationary
            if (ego_vel_is_zero)
            {
                zero_doppler_at_angle = 0.0f;
                // TODO:  zero_doppler_at_angle += pi->pi_dop_rotator_shift;              // Account for HW Doppler rotator effect
            }
            else if (!hist.iszero())
            {
                FLOAT cos_elevation;
                FLOAT sin_elevation;
                if (o.elevation == 0.0F)
                {
                    // compute optimization because elevation is always zero for 1D azimuth-only scans
                    cos_elevation = 1.0F;
                    sin_elevation = 0.0F;
                }
                else
                {
                    cos_elevation = cosf(o.elevation);
                    sin_elevation = sinf(o.elevation);
                }

                vec3f_t polars(
                    cosf(o.azimuth) * cos_elevation,    // X component of (azimuth, elevation) polar angles
                    sinf(o.azimuth) * cos_elevation,    // Y component of (azimuth, elevation) polar angles
                    sin_elevation);                     // Z component of (azimuth, elevation) polar angles

                zero_doppler_at_angle = hist.dot(polars);
                // TODO:  zero_doppler_at_angle += pi->pi_dop_rotator_shift;              // Account for HW Doppler rotator effect
            }
            else if (myscan.zero_d_bins)   // Assume that the center of Static Slice is correct
            {
                INT angle_bin = az_bin_lo + (el_bin_lo * info.num_azimuth_angles);

                int16_t sszero = (int16_t)myscan.zero_d_bins[angle_bin] - zero_doppler_bin;   // Doppler bin index of the 0-velocity bin

                zero_doppler_at_angle = info.doppler_bin_width * (FLOAT)sszero;   // Doppler of SS center (m/s) for angle
                // NODO: pi->pi_dop_rotator_shift should be accounted for in SS curve
            }
            else
            {
                zero_doppler_at_angle = 0.0f;
            }

            // TODO: handle wrap-around properly

            if (uh_fabsf(o.doppler - zero_doppler_at_angle) <= stationary_thresh)
            {
                o.flags |= PF_STATIONARY;
            }
        }
    }
}


void  PointCloud_Impl::apply_threshold(float point_cloud_thresh)
{
    if (points)
    {
        uint16_t mag_snr_min = (uint16_t)(point_cloud_thresh * (1 << PointCloudData::PC_SNR_FRAC_BITS));
        uint32_t out_idx = 0;
        for (uint32_t in_idx = 0; in_idx < num_points; in_idx++)
        {
            if (out_idx != in_idx)
            {
                points[out_idx] = points[in_idx];
            }

            if (points[in_idx].snr_dB >= mag_snr_min)
            {
                out_idx++;
            }
        }

        myscan.scan_info.total_points = out_idx;
    }
}


void  PointCloud_Impl::release()
{
    myscan.release_pointcloud(*this);
}

struct PointCloudData075
{
    uint16_t range;
    uint16_t azimuth_fbin;
    uint16_t elevation_fbin;
    uint16_t doppler_bin;
    uint16_t snr_dB;
    uint8_t  flags;
    uint8_t  future_use;
};


bool  PointCloud_Impl::deserialize(ScanSerializer& s)
{
    char fname[128];
    bool ok = true;

    UhdpScanInformation& info = myscan.scan_info;
    if (0 == info.total_points)
    {
        ok = false;
    }
    else
    {
        sprintf(fname, "scan_%06d_pointcloud.bin", info.scan_sequence_number);

        size_t len = s.begin_read_scan_data_type(fname);

        if (len == sizeof(PointCloudData) * info.total_points)
        {
            points = new PointCloudData[info.total_points];
            ok &= s.read_scan_data_type(points, sizeof(points[0]), info.total_points);
            s.end_read_scan_data_type();
            num_points = info.total_points;
        }
        else if (len == sizeof(PointCloudData075) * info.total_points)
        {
            PointCloudData075* temp = new PointCloudData075[info.total_points];
            points = new PointCloudData[info.total_points];

            ok &= s.read_scan_data_type(temp, sizeof(temp[0]), info.total_points);
            s.end_read_scan_data_type();

            num_points = info.total_points;

            for (uint32_t i = 0; i < info.total_points; i++)
            {
                const PointCloudData075& old = temp[i];
                PointCloudData& p = points[i];

                p.range = old.range;
                p.azimuth_fbin = old.azimuth_fbin;
                p.elevation_fbin = old.elevation_fbin;
                p.doppler_bin = old.doppler_bin;
                p.snr_dB = old.snr_dB;
                p.flags = old.flags;
                p.mag_i = 0;
                p.mag_q = 0;
                p.exponent = 0;
            }

            delete [] temp;
        }
        else
        {
            ok = false;
        }
    }

    return ok;
}


bool  PointCloud_Impl::serialize(ScanSerializer& s)
{
    bool ok = true;

    if (points)
    {
        char fname[128];
        const UhdpScanInformation& info = myscan.scan_info;

        sprintf(fname, "scan_%06d_pointcloud.bin", info.scan_sequence_number);
        ok &= s.begin_write_scan_data_type(fname);
        if (ok)
        {
            ok &= s.write_scan_data_type(points, sizeof(points[0]), info.total_points);
            s.end_write_scan_data_type(!ok);
        }
    }

    return ok;
}


void  PointCloud_Impl::handle_uhdp(PointCloudData* p, uint32_t total_size)
{
    if (aborted)
    {
        return;
    }

    uint32_t count = total_size / sizeof(points[0]);
    if (count * sizeof(points[0]) != total_size)
    {
        aborted = true;
        return;
    }

    if (count + num_points > myscan.scan_info.total_points)
    {
        aborted = true;
        return;
    }

    if (!points)
    {
        points = new PointCloudData[myscan.scan_info.total_points];
    }

    memcpy(points + num_points, p, total_size);
    num_points += count;
}


// Called when UHDP packets for point cloud are complete for this scan
void  PointCloud_Impl::setup()
{
    if (num_points != myscan.scan_info.total_points)
    {
        aborted = true;
    }

    const UhdpScanInformation& info = myscan.scan_info;
    uint32_t AZ = info.num_azimuth_angles;
    uint32_t EL = info.num_elevation_angles;
    uint32_t negative_range = 0;
    bool valid = true;
    for (uint32_t i = 0; i < num_points && valid; i++)
    {
        const PointCloudData& p = points[i];
        if (p.range == 65535)
        {
            negative_range++;
        }
        else if (p.range >= info.num_range_bins)
        {
            printf("p.range = %d\n", p.range);
            valid = false;
        }

        float az = (float)p.azimuth_fbin / (1 << PointCloudData::PC_AZIMUTH_FRAC_BITS);
        uint32_t az_bin_lo = (uint32_t)az;
        if (az_bin_lo > AZ)
        {
            printf("p.az_bin = %d\n", az_bin_lo);
            valid = false;
        }

        float el = (float)p.elevation_fbin / (1 << PointCloudData::PC_ELEVATION_FRAC_BITS);
        uint32_t el_bin_lo = (uint32_t)el;
        if (el_bin_lo > EL)
        {
            printf("p.el_bin = %d\n", el_bin_lo);
            valid = false;
        }
    }

    if (negative_range)
    {
        printf("There were %d points with negative range\n", negative_range);
    }
    if (!valid)
    {
        printf("Invalid points received from the radar, discarding point cloud data\n");
        aborted = true;
    }

    if (aborted)
    {
        myscan.release_pointcloud(*this);
    }
}


void  PointCloud_Impl::extract_static_points()
{
    if (!clutter_points_collected)
    {
        ClutterImage_Impl *ci = static_cast<ClutterImage_Impl*>(myscan.get_clutter_image());
        if (ci)
        {
            const UhdpScanInformation& info = myscan.scan_info;
            uint32_t max_num_points = info.num_range_bins * info.num_azimuth_angles * info.num_elevation_angles;
            PointCloudData* cipts = new PointCloudData[max_num_points];

            uint32_t actual_cipt_count = ci->get_point_cloud_points(cipts, max_num_points);
            if (actual_cipt_count)
            {
                if (points)
                {
                    // append clutter image points to dynamic points
                    PointCloudData* newpts = new PointCloudData[actual_cipt_count + num_points];
                    memcpy(newpts, points, sizeof(points[0]) * num_points);
                    memcpy(newpts + num_points, cipts, sizeof(points[0]) * actual_cipt_count);
                    num_points += actual_cipt_count;
                    delete [] points;
                    delete [] cipts;
                    points = newpts;
                }
                else
                {
                    points = cipts;
                    num_points = actual_cipt_count;
                }
            }

            myscan.scan_info.total_points = num_points;
            clutter_points_collected = true;
        }
    }
}
