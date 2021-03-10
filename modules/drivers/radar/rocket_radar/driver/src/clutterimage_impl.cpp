#include "modules/drivers/radar/rocket_radar/driver/system-radar-software/env-reference/coredefs/uhnder-common.h"
#include "modules/drivers/radar/rocket_radar/driver/src/clutterimage_impl.h"
#include "modules/drivers/radar/rocket_radar/driver/src/scanobject_impl.h"
#include "modules/drivers/radar/rocket_radar/driver/include/serializer.h"
#include "modules/drivers/radar/rocket_radar/driver/system-radar-software/engine/scp-src/eng-api/uhmath.h"
#include "modules/drivers/radar/rocket_radar/driver/system-radar-software/engine/common/eng-api/rdc-structs.h"
#include "modules/drivers/radar/rocket_radar/driver/system-radar-software/engine/common/eng-api/rdc-internals.h"
#include "modules/drivers/radar/rocket_radar/driver/include/pointcloud.h"

void ClutterImage_Impl::handle_uhdp(const UhdpClutterImageHeader* msg, uint32_t len)
{
    if (aborted)
    {
        return;
    }

    len -= sizeof(UhdpClutterImageHeader);

    const char* payload = (char*)(msg + 1);
    uint32_t samples = len / myscan.scan_info.CI_bytes_per_pixel;

    uint32_t AZ = myscan.scan_info.num_azimuth_angles;
    uint32_t EL = get_elevation_dimension();
    uint32_t R  = myscan.scan_info.num_range_bins;
    uint32_t total_size = AZ * EL * R;

    if (mag_offset < total_size)
    {
        if (msg->sample_offset != mag_offset)
        {
            printf("Clutter image %d dropped mag packet detected (msg offset %d, received %d), aborting\n",
                    msg->scan_sequence_num, msg->sample_offset, mag_offset);
            aborted = true;
            return;
        }
        else if (mag_offset + samples <= total_size)
        {
            memcpy(mag_plane + mag_offset, payload, len);
            mag_offset += samples;
        }
        else
        {
            printf("Clutter image buffer overrun (%d > %d)\n", mag_offset + len, total_size);
            aborted = true;
            return;
        }
    }
    else if (hd_plane && hd_offset < total_size)
    {
        if (msg->sample_offset != hd_offset)
        {
            printf("Clutter image %d dropped HD packet detected (msg offset %d, received %d), aborting\n",
                   msg->scan_sequence_num, msg->sample_offset, hd_offset);
            aborted = true;
            return;
        }
        else if (hd_offset + samples <= total_size)
        {
            memcpy(hd_plane + hd_offset, payload, len);
            hd_offset += samples;
        }
        else
        {
            printf("Clutter image buffer overrun (%d > %d)\n", hd_offset + len, total_size);
            aborted = true;
            return;
        }
    }
    else
    {
        printf("Extra clutter image data received\n");
        aborted = true;
    }
}

uint32_t ClutterImage_Impl::get_azimuth_dimension() const
{
    return myscan.scan_info.num_azimuth_angles;
}

uint32_t ClutterImage_Impl::get_elevation_dimension() const
{
    return (myscan.scan_info.CI_format == CIMG_M16PP_D7PP) ? myscan.scan_info.num_elevation_angles : 1;
}

uint32_t ClutterImage_Impl::get_range_dimension() const
{
    return myscan.scan_info.CI_height;
}


bool     ClutterImage_Impl::height_data_supported() const
{
    return cifmt_has_height((RDC_clutter_image_format)myscan.scan_info.CI_format);
}


bool     ClutterImage_Impl::doppler_data_supported() const
{
    return cifmt_has_doppler((RDC_clutter_image_format)myscan.scan_info.CI_format);
}


ClutterImage::PeakCountEnum ClutterImage_Impl::get_sample(
    uint32_t az_idx,
    uint32_t el_idx,
    uint32_t range_idx,
    float& dbFS,
    float& snr,
    float& range,
    float& doppler,
    float& azimuth,
    float& elevation) const
{
    if (!mag_plane)
    {
        last_err = CI_INVALID_DATA_PLANE;
        return NO_PEAK;
    }

    const UhdpScanInformation& info = myscan.scan_info;

    uint32_t AZ = info.num_azimuth_angles;
    uint32_t EL = get_elevation_dimension();
    uint32_t R  = info.num_range_bins;

    if (az_idx >= AZ || el_idx >= EL || range_idx >= R)
    {
        last_err = CI_INDEX_OUT_OF_RANGE;
        return NO_PEAK;
    }

    int16_t d = ci_row_distance[range_idx];          // map from CI row (range_idx) to distance
    if (d < 0)
    {
        // this row of the clutter image has no real data in it
        dbFS = 0.0f;
        snr = 0.0f;
        range = 0.0f;
        doppler = 0.0f;
        azimuth = 0.0f;
        elevation = 0.0f;
        return NO_PEAK;
    }

    uint16_t mag = mag_plane[range_idx * AZ * EL + el_idx * AZ + az_idx];

    int clutter_image_exponent_nosw = info.clutter_image_exponent - info.rdc3_software_exponent;
    if (clutter_image_exponent_nosw >= 0)
    {
        snr  = mag2db((float)mag  * (1 << clutter_image_exponent_nosw));
    }
    else
    {
        snr  = mag2db((float)mag  / (1 << -clutter_image_exponent_nosw));
    }

    int16_t rbin = myscan.range_bins[d].reverse_map; // lookup range bin ID from distance
    snr -= myscan.range_bins[rbin].noise_floor_max_peak_dB;
    dbFS = mag2db(mag) + 6 * info.clutter_image_exponent - mag2db(info.rdc3_full_scale_value);

    azimuth = myscan.get_azimuth_rad(az_idx);
    range   = d * info.CI_pixel_size_in_meters;

    last_err = CI_NO_ERROR;
    if (mag > ci_row_noise_floor[range_idx] && hd_plane)
    {
        uint16_t val = hd_plane[range_idx * AZ * EL + el_idx * AZ + az_idx];

        if (doppler_data_supported() && myscan.zero_d_bins)
        {
            int d = val & 0x7f;
            int dabs = (int)myscan.zero_d_bins[el_idx * AZ + az_idx] - (info.num_pulses / 2);
            d -= ((info.SS_size_D - 1) / 2) << MAP_DEC_DOP_BITS;
            d += dabs << MAP_DEC_DOP_BITS;
            doppler = (d * info.doppler_bin_width) / (1 << MAP_DEC_DOP_BITS);
        }

        if (height_data_supported())
        {
            int h = (val >> 7) & 0x7f;
            int hint = h >> MAP_DEC_EL_BITS;
            int frac = h - (hint << MAP_DEC_EL_BITS);

            elevation = myscan.get_elevation_rad(hint);
            elevation += ((float)frac / (1 << MAP_DEC_EL_BITS)) *
                         (myscan.get_elevation_rad(1) - myscan.get_elevation_rad(0));
        }
        else if (info.CI_format == CIMG_M16PP_D7PP)
        {
            elevation = myscan.get_elevation_rad(el_idx);
        }

        static PeakCountEnum mapping[] = {
            MORE_THAN_THREE_PEAKS,
            SINGLE_PEAK,
            TWO_PEAKS,
            THREE_PEAKS,
        };

        return mapping[(val >> 14) & 0x3];
    }
    else
    {
        doppler = 0.0f;
        elevation = 0.0f;
        return mag > ci_row_noise_floor[range_idx] ? SINGLE_PEAK : NO_PEAK;
    }
}


uint32_t ClutterImage_Impl::get_point_cloud_points(PointCloudData* buffer, uint32_t buffer_size_points)
{
    if (!mag_plane || !buffer || !buffer_size_points)
    {
        last_err = CI_INVALID_DATA_PLANE;
        return 0;
    }

    const UhdpScanInformation& info = myscan.scan_info;
    const uint32_t AZ = info.num_azimuth_angles;
    const uint32_t EL = get_elevation_dimension();
    const uint32_t R  = info.num_range_bins;
    const int clutter_image_exponent_nosw = info.clutter_image_exponent - info.rdc3_software_exponent;
    uint32_t num_points = 0U;

    for (uint32_t range_idx = 0; range_idx < R; range_idx++)
    {
        int16_t d = ci_row_distance[range_idx];          // map from CI row (range_idx) to distance
        if (d < 0)
        {
            continue;
        }

        int16_t rbin = myscan.range_bins[d].reverse_map; // lookup range bin ID from distance
        const int ss_half_width = (info.SS_size_D - 1) / 2;

        for (uint32_t el_idx = 0; el_idx < EL; el_idx++)
        {
            for (uint32_t az_idx = 0; az_idx < AZ; az_idx++)
            {
                PointCloudData& p = buffer[num_points];

                const uint16_t mag = mag_plane[range_idx * AZ * EL + el_idx * AZ + az_idx];
                const float mag_snr = mag2db(mag) + TWO_IN_DB * clutter_image_exponent_nosw - myscan.range_bins[rbin].noise_floor_max_peak_dB;

                if (mag_snr >= myscan.mythresh.point_cloud_thresh_snr_dB)
                {
                    p.range          = d;
                    p.doppler_bin    = info.num_pulses / 2;
                    p.snr_dB         = uint16_t(mag_snr * (1 << PointCloudData::PC_SNR_FRAC_BITS));
                    p.azimuth_fbin   = az_idx * (1 << PointCloudData::PC_AZIMUTH_FRAC_BITS);
                    p.elevation_fbin = el_idx * (1 << PointCloudData::PC_ELEVATION_FRAC_BITS);

                    if (hd_plane)
                    {
                        uint16_t val = hd_plane[range_idx * AZ * EL + el_idx * AZ + az_idx];

                        if (doppler_data_supported() && myscan.zero_d_bins)
                        {
                            int d = ((val & 0x7f) + (1 << (MAP_DEC_DOP_BITS - 1))) >> MAP_DEC_DOP_BITS;
                            int dabs = (int)myscan.zero_d_bins[el_idx * AZ + az_idx];
                            p.doppler_bin = d + dabs - ss_half_width;
                        }

                        if (height_data_supported())
                        {
                            int h = (val >> 7) & 0x7f;
                            int hint = h >> MAP_DEC_EL_BITS;
                            //int frac = h - (hint << MAP_DEC_EL_BITS);
                            p.elevation_fbin = hint;
                        }

                        // mapping from SRS clutter image flags to point cloud flags
                        static INT flag_mapping[] = {
                            PointCloud::PF_MORE_THAN_THREE_PEAKS, // MORE_THAN_THREE_PEAKS,
                            0,                                    // SINGLE_PEAK,
                            PointCloud::PF_TWO_PEAKS,             // TWO_PEAKS,
                            PointCloud::PF_THREE_PEAKS,           // THREE_PEAKS,
                        };

                        p.flags = PointCloud::PF_CLUTTER | flag_mapping[(val >> 14) & 0x3];
                    }

                    if (++num_points >= buffer_size_points)
                    {
                        return num_points;
                    }
                }
            }
        }
    }

    return num_points;
}


void     ClutterImage_Impl::get_raw_sample(uint32_t az_idx,
                        uint32_t el_idx,
                        uint32_t range_idx,
                        uint16_t& magnitude,
                        uint16_t& count_height_doppler) const
{
    if (!mag_plane)
    {
        last_err = CI_INVALID_DATA_PLANE;
        return;
    }

    uint32_t AZ = myscan.scan_info.num_azimuth_angles;
    uint32_t EL = get_elevation_dimension();
    uint32_t R  = myscan.scan_info.num_range_bins;

    if (az_idx >= AZ || el_idx >= EL || range_idx >= R)
    {
        last_err = CI_INDEX_OUT_OF_RANGE;
        return;
    }

    magnitude = mag_plane[range_idx * AZ * EL + el_idx * AZ + az_idx];

    if (hd_plane)
    {
        count_height_doppler = hd_plane[range_idx * AZ * EL + el_idx * AZ + az_idx];
    }

    last_err = CI_NO_ERROR;
}


void     ClutterImage_Impl::release()
{
    myscan.release_clutter_image(*this);
}


void     ClutterImage_Impl::allocate()
{
    uint32_t R = myscan.scan_info.num_range_bins;
    uint32_t AZ = myscan.scan_info.num_azimuth_angles;
    uint32_t EL = get_elevation_dimension();
    RDC_clutter_image_format fmt = (RDC_clutter_image_format)myscan.scan_info.CI_format;

    mag_plane = new uint16_t[R * AZ * EL + 2]; /* two extra for polar2cartesian zero use */
    mag_plane[R * AZ * EL + 0] = mag_plane[R * AZ * EL + 1] = 0;

    ci_row_noise_floor = new uint16_t[myscan.scan_info.CI_height];
    ci_row_distance    = new uint16_t[myscan.scan_info.CI_height];

    if (cifmt_has_height_or_doppler(fmt))
    {
        hd_plane = new uint16_t[R * AZ * EL];
    }
}


void     ClutterImage_Impl::setup()
{
    if (!post_process())
    {
        release();
        return;
    }
}


bool     ClutterImage_Impl::post_process()
{
    if (aborted)
    {
        return false;
    }

    uint32_t AZ = myscan.scan_info.num_azimuth_angles;
    uint32_t EL = get_elevation_dimension();
    uint32_t R  = myscan.scan_info.num_range_bins;
    uint32_t total_size = AZ * EL * R;

    if (mag_offset != total_size)
    {
        printf("Clutter magnitude image is incomplete, discarding\n");
        return false;
    }
    if (!myscan.range_bins || !myscan.angle_bins)
    {
        printf("Clutter image is missing range or angle data, discarding\n");
        return false;
    }
    if (hd_plane && hd_offset != total_size)
    {
        printf("Clutter height/doppler image is incomplete, discarding\n");
        return false;
    }
    if (myscan.scan_info.num_range_bins != myscan.scan_info.CI_height)
    {
        printf("Unsupported clutter image height (does not match R)\n");
        return false;
    }

    // Determine the noise floor of each range bin by finding min

    uint32_t first_az = 0;
    uint32_t last_az  = AZ * EL - 1;

    if (myscan.scan_info.azimuth_nyquist_oversampling_factor &&
        !(myscan.scan_info.angle_wrap_flags & 1))
    {
        first_az += myscan.scan_info.azimuth_nyquist_oversampling_factor;
        last_az  -= myscan.scan_info.azimuth_nyquist_oversampling_factor;
    }

    uint32_t realrows = 0;
    for (uint32_t d = 0; d < R; d++)
    {
        int16_t rbin = myscan.range_bins[d].reverse_map;
        if (rbin >= 0)
        {
            ci_row_distance[realrows++] = d;
        }
    }

    uint32_t r;
    for (r = 0; r < realrows; r++)
    {
        uint16_t floor = 0xFFFF;

        // in two-d without wrap this is not ignoring the middle elevation bin
        // azimith extents.. but hopefully we never do such a thing
        for (uint32_t az = first_az; az <= last_az; az++)
        {
            floor = uh_uintmin(floor, mag_plane[r * AZ * EL + az]);
        }

        ci_row_noise_floor[r] = uh_uintmax(floor, 1); // do not allow 0

        // overwrite discarded bins with noise floor, to be safe
        for (uint32_t az = 0; az < first_az; az++)
        {
            mag_plane[r * AZ * EL + az] = floor;
        }
        for (uint32_t az = last_az; az < AZ * EL; az++)
        {
            mag_plane[r * AZ * EL + az] = floor;
        }
    }

    // force-zero the unused rows of the clutter image
    for (; r < R; r++)
    {
        memset(mag_plane + r * AZ * EL, 0, AZ * EL * sizeof(mag_plane[0]));
        ci_row_noise_floor[r] = 1;
        ci_row_distance[r]    = -1;
    }

    return true;
}


struct ClutterImageFileHeader
{
    uint16_t slice_num;

    uint16_t x_size;
    uint16_t y_size;
    uint16_t depth_bytes;
    int16_t  image_exponent;
    uint16_t fmt;
    float    pixel_size_in_meters;
};


// called by ScanObject::load_scan() and presumes the ScanInfo JSON has already
// been parsed and this object has been completely allocated
bool     ClutterImage_Impl::deserialize(ScanSerializer& s)
{
    char fname[128];

    const UhdpScanInformation& info = myscan.scan_info;

    ClutterImageFileHeader hdr;

#define CHECK(expr) if (!(expr)) { s.end_read_scan_data_type(); return false; }

    sprintf(fname, "scan_%06d_clutterimage.bin", info.scan_sequence_number);
    size_t len = s.begin_read_scan_data_type(fname);
    if (len == sizeof(hdr) + sizeof(uint16_t) * info.CI_height * info.CI_width)
    {
        s.read_scan_data_type(&hdr, sizeof(hdr), 1);

        CHECK(hdr.slice_num == 0);
        CHECK(hdr.x_size == info.CI_height);
        CHECK(hdr.y_size == info.CI_width);
        CHECK(hdr.depth_bytes == info.CI_bytes_per_pixel);
        CHECK(hdr.image_exponent == info.clutter_image_exponent);
        CHECK(hdr.fmt == info.CI_format);
        CHECK(hdr.pixel_size_in_meters == info.CI_pixel_size_in_meters);
        mag_offset = info.CI_height * info.CI_width;
        s.read_scan_data_type(mag_plane, sizeof(uint16_t), hdr.x_size * hdr.y_size);
        s.end_read_scan_data_type();
    }
    else
    {
        s.end_read_scan_data_type();
        return false;
    }

    if (hd_plane)
    {
        memset(hd_plane, 0, sizeof(uint16_t) * info.CI_doppler_height *
                                               info.CI_doppler_width);
    }

    if (height_data_supported())
    {
        sprintf(fname, "scan_%06d_ci_height.bin", info.scan_sequence_number);
        size_t len = s.begin_read_scan_data_type(fname);
        if (len == sizeof(hdr) + info.CI_doppler_height * info.CI_doppler_width)
        {
            s.read_scan_data_type(&hdr, sizeof(hdr), 1);

            CHECK(hdr.slice_num == 0);
            CHECK(hdr.x_size == info.CI_doppler_height);
            CHECK(hdr.y_size == info.CI_doppler_width);
            CHECK(hdr.depth_bytes == 1);
            CHECK(hdr.image_exponent == 0);
            CHECK(hdr.fmt == info.CI_format);
            CHECK(hdr.pixel_size_in_meters == info.CI_pixel_size_in_meters);

            uint8_t val;
            for (int i = 0; i < hdr.x_size * hdr.y_size; i++)
            {
                s.read_scan_data_type(&val, 1, 1);
                hd_plane[i] |= (val & 0x7f) << 7;
            }
            s.end_read_scan_data_type();
        }
        else
        {
            s.end_read_scan_data_type();
            return false;
        }
    }

    if (doppler_data_supported())
    {
        sprintf(fname, "scan_%06d_ci_doppler.bin", info.scan_sequence_number);
        len = s.begin_read_scan_data_type(fname);
        if (len == sizeof(hdr) + info.CI_doppler_height * info.CI_doppler_width)
        {
            s.read_scan_data_type(&hdr, sizeof(hdr), 1);

            CHECK(hdr.slice_num == 0);
            CHECK(hdr.x_size == info.CI_doppler_height);
            CHECK(hdr.y_size == info.CI_doppler_width);
            CHECK(hdr.depth_bytes == 1);
            CHECK(hdr.image_exponent == 0);
            CHECK(hdr.fmt == info.CI_format);
            CHECK(hdr.pixel_size_in_meters == info.CI_pixel_size_in_meters);

            uint8_t val;
            for (int i = 0; i < hdr.x_size * hdr.y_size; i++)
            {
                s.read_scan_data_type(&val, 1, 1);
                hd_plane[i] |= val & 0x7f;
                hd_offset++;
            }
            s.end_read_scan_data_type();
        }
        else
        {
            s.end_read_scan_data_type();
            return false;
        }

        static uint8_t revcountmap[] = { 0xFF, 1, 2, 3, 0, 0xFF, 0xFF, 0xFF };

        sprintf(fname, "scan_%06d_ci_hdpeaks.bin", info.scan_sequence_number);
        len = s.begin_read_scan_data_type(fname);
        if (len == sizeof(hdr) + info.CI_doppler_height * info.CI_doppler_width)
        {
            s.read_scan_data_type(&hdr, sizeof(hdr), 1);

            CHECK(hdr.slice_num == 0);
            CHECK(hdr.x_size == info.CI_doppler_height);
            CHECK(hdr.y_size == info.CI_doppler_width);
            CHECK(hdr.depth_bytes == 1);
            CHECK(hdr.image_exponent == 0);
            CHECK(hdr.fmt == info.CI_format);
            CHECK(hdr.pixel_size_in_meters == info.CI_pixel_size_in_meters);

            uint8_t val;
            for (int i = 0; i < hdr.x_size * hdr.y_size; i++)
            {
                s.read_scan_data_type(&val, 1, 1);
                hd_plane[i] |= (revcountmap[val & 0x7]) << 14;
            }
            s.end_read_scan_data_type();
        }
        else
        {
            s.end_read_scan_data_type();
            return false;
        }
    }

    return post_process();
}


// called by ScanObject::serialize()
bool     ClutterImage_Impl::serialize(ScanSerializer& s) const
{
    if (aborted)
    {
        // CI cannot be written, but should not abort writing the remaining
        // parts of the scan
        return true;
    }

    char fname[128];
    const UhdpScanInformation& info = myscan.scan_info;

    ClutterImageFileHeader hdr;

    hdr.slice_num = 0;
    hdr.x_size = info.CI_height;
    hdr.y_size = info.CI_width;
    hdr.depth_bytes = info.CI_bytes_per_pixel;
    hdr.image_exponent = info.clutter_image_exponent;
    hdr.fmt = info.CI_format;
    hdr.pixel_size_in_meters = info.CI_pixel_size_in_meters;

    sprintf(fname, "scan_%06d_clutterimage.bin", info.scan_sequence_number);
    bool ok = s.begin_write_scan_data_type(fname);
    if (ok)
    {
        ok &= s.write_scan_data_type(&hdr, sizeof(hdr), 1);
        ok &= s.write_scan_data_type(mag_plane, sizeof(uint16_t), hdr.x_size * hdr.y_size);
        s.end_write_scan_data_type(!ok);
    }

    hdr.x_size = info.CI_doppler_height;
    hdr.y_size = info.CI_doppler_width;
    hdr.depth_bytes = 1;
    hdr.image_exponent = 0;

    if (height_data_supported() && ok)
    {
        sprintf(fname, "scan_%06d_ci_height.bin", info.scan_sequence_number);
        ok &= s.begin_write_scan_data_type(fname);
        if (ok)
        {
            ok &= s.write_scan_data_type(&hdr, sizeof(hdr), 1);
            for (int i = 0; i < hdr.x_size * hdr.y_size; i++)
            {
                uint8_t val = (hd_plane[i] >> 7) & 0x7f;
                ok &= s.write_scan_data_type(&val, 1, 1);
            }
            s.end_write_scan_data_type(!ok);
        }
    }

    if (doppler_data_supported() && ok)
    {
        sprintf(fname, "scan_%06d_ci_doppler.bin", info.scan_sequence_number);
        ok &= s.begin_write_scan_data_type(fname);
        if (ok)
        {
            ok &= s.write_scan_data_type(&hdr, sizeof(hdr), 1);
            for (int i = 0; i < hdr.x_size * hdr.y_size; i++)
            {
                uint8_t val = hd_plane[i] & 0x7f;
                ok &= s.write_scan_data_type(&val, 1, 1);
            }
            s.end_write_scan_data_type(!ok);
        }
        else
        {
            return ok;
        }

        static uint8_t countmap[] = {4, 1, 2, 3};

        sprintf(fname, "scan_%06d_ci_hdpeaks.bin", info.scan_sequence_number);
        ok &= s.begin_write_scan_data_type(fname);
        if (ok)
        {
            ok &= s.write_scan_data_type(&hdr, sizeof(hdr), 1);
            for (int i = 0; i < hdr.x_size * hdr.y_size; i++)
            {
                uint8_t val = countmap[hd_plane[i] >> 14];
                ok &= s.write_scan_data_type(&val, 1, 1);
            }
            s.end_write_scan_data_type(!ok);
        }
    }

    return ok;
}

static const float INVALID_SAMPLE = 1e-5f;

float ClutterImage_Impl::doppler_sample(uint32_t range_bin, uint32_t azimuth_bin)
{
    const UhdpScanInformation& info = myscan.scan_info;
    const uint32_t A = get_azimuth_dimension();
    const uint32_t R = get_range_dimension();

    if (azimuth_bin >= A || range_bin >= R)
    {
        return INVALID_SAMPLE;
    }

    // document scan_desc.get_doppler_bin_width_mps() and FRAC_BITS
    uint16_t mag = 0, count_height_doppler = 0;
    get_raw_sample(azimuth_bin, 0, range_bin, mag, count_height_doppler);

    if (mag <= ci_row_noise_floor[range_bin])
    {
        return INVALID_SAMPLE;
    }

    int d = count_height_doppler & 0x7f;
    int dabs = (int)myscan.zero_d_bins[0 * A + azimuth_bin] - (info.num_pulses / 2);
    d -= ((info.SS_size_D - 1) / 2) << MAP_DEC_DOP_BITS;
    d += dabs << MAP_DEC_DOP_BITS;
    float mps = (d * info.doppler_bin_width) / (1 << MAP_DEC_DOP_BITS);
    return mps;
}


float ClutterImage_Impl::doppler_interpolate(float range, float azimuth)
{
    if (!hd_plane)
    {
        return 0.0f;
    }

    uint32_t A = get_azimuth_dimension();
    uint16_t az_idx = 0;
    // find native az bin in raw polar clutter image
    for (az_idx = 0; az_idx < A - 1; az_idx++)
    {
        if (myscan.get_azimuth_rad(az_idx + 1) >= azimuth)
        {
            break;
        }
    }

    uint32_t r0 = uint32_t(range);
    uint32_t r1 = r0 + 1;
    float range_frac = range - r0;

    float az_frac = (azimuth - myscan.get_azimuth_rad(az_idx)) /
                    (myscan.get_azimuth_rad(az_idx + 1) - myscan.get_azimuth_rad(az_idx));

    float d00 = doppler_sample(r0, az_idx);
    float d01 = doppler_sample(r0, az_idx + 1);
    float d10 = doppler_sample(r1, az_idx);
    float d11 = doppler_sample(r1, az_idx + 1);

    float d0;
    if (d00 == INVALID_SAMPLE)
    {
        d0 = d01;
    }
    else if (d01 == INVALID_SAMPLE)
    {
        d0 = d00;
    }
    else
    {
        d0 = d00 * (1 - az_frac) + d01 * az_frac;
    }

    float d1;
    if (d10 == INVALID_SAMPLE)
    {
        d1 = d11;
    }
    else if (d11 == INVALID_SAMPLE)
    {
        d1 = d10;
    }
    else
    {
        d1 = d10 * (1 - az_frac) + d11 * az_frac;
    }

    float ret = 0;
    if (d0 == INVALID_SAMPLE)
    {
        ret = d1;
    }
    else if (d1 == INVALID_SAMPLE)
    {
        ret = d0;
    }
    else
    {
        ret = d0 * (1 - range_frac) + d1 * range_frac;
    }
    if (ret == INVALID_SAMPLE)
    {
        ret = 0.0f;
    }

    return ret;
}


// returns interpolated bin index
static float quadratic_interp(float yl, float yc, float yu)
{
    float d1 = (yu - yl) / 2;
    float d2 = yu - yc + yl - yc;

    if (d2)
    {
        float dd = d1 / d2;
        // interpmag = yc - d1 * dd / 2.0f;
        return dd;
    }
    else
    {
        // interpmag = yc;
        return 0;
    }
}


float ClutterImage_Impl::elevation_sample(uint32_t range_bin, uint32_t azimuth_bin)
{
    const uint32_t A = get_azimuth_dimension();
    const uint32_t E = get_elevation_dimension();
    const uint32_t R = get_range_dimension();

    if (azimuth_bin >= A || range_bin >= R)
    {
        return INVALID_SAMPLE;
    }

    uint16_t magnitudes[128];
    uint16_t max_mag = 0;
    uint16_t min_mag = 0xFFFF;
    uint32_t max_idx = 0;
    for (uint32_t e = 0; e < E; e++)
    {
        magnitudes[e] = mag_plane[range_bin * A * E + e * A + azimuth_bin];
        if (magnitudes[e] > max_mag)
        {
            max_mag = magnitudes[e];
            max_idx = e;
        }
        if (magnitudes[e] < min_mag)
        {
            min_mag = magnitudes[e];
        }
    }

    if (min_mag == max_mag)
    {
        // elevation is all noise
        return 0.0f;
    }
    else if ((max_idx == 0) || (max_idx == E - 1))
    {
        // edge conditions
        return myscan.get_elevation_rad(max_idx);
    }
    else
    {
        float interpbin = quadratic_interp(magnitudes[max_idx - 1], magnitudes[max_idx], magnitudes[max_idx + 1]);
        if (interpbin == 0)
        {
            return myscan.get_elevation_rad(max_idx);
        }
        else if (interpbin < 0)
        {
            float low = myscan.get_elevation_rad(max_idx - 1);
            float high = myscan.get_elevation_rad(max_idx);
            return high + interpbin * (high - low);
        }
        else
        {
            float low = myscan.get_elevation_rad(max_idx);
            float high = myscan.get_elevation_rad(max_idx + 1);
            return low + interpbin * (high - low);
        }
    }
}


float ClutterImage_Impl::elevation_interpolate(float range_bins, float azimuth_fbin)
{
    uint32_t r0 = uint32_t(range_bins);
    uint32_t r1 = r0 + 1;
    float range_frac = range_bins - r0;

    uint32_t az_idx = uint32_t(azimuth_fbin);
    float az_frac = azimuth_fbin - az_idx;

    float d00 = elevation_sample(r0, az_idx);
    float d01 = elevation_sample(r0, az_idx + 1);
    float d10 = elevation_sample(r1, az_idx);
    float d11 = elevation_sample(r1, az_idx + 1);

    float d0;
    if (d00 == INVALID_SAMPLE)
    {
        d0 = d01;
    }
    else if (d01 == INVALID_SAMPLE)
    {
        d0 = d00;
    }
    else
    {
        d0 = d00 * (1 - az_frac) + d01 * az_frac;
    }

    float d1;
    if (d10 == INVALID_SAMPLE)
    {
        d1 = d11;
    }
    else if (d11 == INVALID_SAMPLE)
    {
        d1 = d10;
    }
    else
    {
        d1 = d10 * (1 - az_frac) + d11 * az_frac;
    }

    float ret = 0;
    if (d0 == INVALID_SAMPLE)
    {
        ret = d1;
    }
    else if (d1 == INVALID_SAMPLE)
    {
        ret = d0;
    }
    else
    {
        ret = d0 * (1 - range_frac) + d1 * range_frac;
    }
    if (ret == INVALID_SAMPLE)
    {
        ret = 0.0f;
    }

    return ret;
}

