// Copyright (C) Uhnder, Inc. All rights reserved. Confidential and Proprietary - under NDA.
// Refer to SOFTWARE_LICENSE file for details

#include "modules/drivers/radar/rocket_radar/driver/system-radar-software/env-reference/coredefs/uhnder-common.h"
#include "modules/drivers/radar/rocket_radar/driver/include/polar2cartesian.h"
#include "modules/drivers/radar/rocket_radar/driver/src/clutterimage_impl.h"
#include "modules/drivers/radar/rocket_radar/driver/src/sensorgroup_impl.h"
#include "modules/drivers/radar/rocket_radar/driver/src/scanobject_impl.h"
#include "modules/drivers/radar/rocket_radar/driver/src/frameobject_impl.h"
#include "modules/drivers/radar/rocket_radar/driver/src/framep2c_impl.h"
#include "modules/drivers/radar/rocket_radar/driver/system-radar-software/engine/common/eng-api/rdc-internals.h"// doppler fraction bins

#include <assert.h>

FrameP2C_Impl::~FrameP2C_Impl()
{
    delete [] mag_image;
    delete [] dop_image;
    delete [] ht_image;
    delete [] prob_image;
    delete [] comb_controls;
}


void            FrameP2C_Impl::release()
{
    aborted = true;
    stop();
    delete this;
}


bool            FrameP2C_Impl::update_region(
    const FrameObject&  frame,
    uint32_t            start_x,
    uint32_t            extent_x,
    uint32_t            start_y,
    uint32_t            extent_y,
    bool                update_doppler_plane,
    bool                update_height_plane,
    bool                update_occupancy_plane)
{
    const FrameObject_Impl& fr  = static_cast<const FrameObject_Impl&>(frame);
    bool ok = true;

    const uint32_t endX = uh_uintmin(start_x + extent_x, dimX);
    const uint32_t endY = uh_uintmin(start_y + extent_y, dimY);

    for (uint32_t s = 0; s < fr.sensor_count; s++)
    {
        for (uint32_t i = 0; i < fr.scans_per_frame; i++)
        {
            ScanObject_Impl* scan = fr.scans[s].scan[i];
            if (scan)
            {
                ClutterImage_Impl* ci = static_cast<ClutterImage_Impl*>(scan->get_clutter_image());
                if (ci)
                {
                    comb_controls[s * fr.scans_per_frame + i].combine_angle_bins(*ci, 0xFFFF);
                }
                // TODO: else
            }
        }
    }

    for (uint32_t x = start_x; x < endX; x++)
    {
        for (uint32_t y = start_y; y < endY; y++)
        {
            float value = 0; // most pixels will be 0, not in FOV
            uint32_t strongest = 0;

            for (uint32_t m = 0; m < MAX_MERGE_POINTS; m++)
            {
                const FFrac2D& f2d = frac[m][x * dimY + y];
                if (f2d.sensor_idx < 0)
                {
                    break;
                }

                const uint32_t f2d_r1 = f2d.r0 + 1;
                const uint32_t s = f2d.sensor_idx * fr.scans_per_frame + f2d.scan_idx;
                const CombinationControl& cc = comb_controls[s];

                // bilinear interpolation
                uint16_t q00 = cc.combined[f2d.r0 * cc.stride + f2d.r0_a0];
                uint16_t q01 = cc.combined[f2d.r0 * cc.stride + f2d.r0_a1];
                uint16_t q10 = cc.combined[f2d_r1 * cc.stride + f2d.r1_a0];
                uint16_t q11 = cc.combined[f2d_r1 * cc.stride + f2d.r1_a1];

                float q0 = (q00 * (1 - f2d.r0_azimuth_frac) + q01 * f2d.r0_azimuth_frac) / cc.noise_floor_linear[f2d.r0];
                float q1 = (q10 * (1 - f2d.r1_azimuth_frac) + q11 * f2d.r1_azimuth_frac) / cc.noise_floor_linear[f2d_r1];

                float linear_snr = q0 * (1 - f2d.range_frac) + q1 * f2d.range_frac;
                if (linear_snr <= 2.0f)
                {
                    if (!value)
                    {
                        strongest = m;
                        value = 1; // many pixels will be 1, noise floor
                    }
                }
                else
                {
                    float db = mag2db(linear_snr);
                    if (db > value)
                    {
                        strongest = m;
                        value = db; // the rest will be signal on decibels
                    }
                }
            }

            mag_image[x * dimY + y] = value;
            ht_image[x * dimY + y] = value;
            dop_image[x * dimY + y] = value;

            if (value > 2.0f)
            {
                if (update_doppler_plane)
                {
                    dop_image[x * dimY + y] = get_doppler_sample(fr, frac[strongest][x * dimY + y]);
                }

                if (update_height_plane)
                {
                    ht_image[x * dimY + y] = get_height_sample(fr, frac[strongest][x * dimY + y]);
                }
            }
        }
    }

    if (update_occupancy_plane)
    {
    }

    return ok;
}


float FrameP2C_Impl::get_height_sample(const FrameObject_Impl& frame, const FFrac2D& f2d)
{
    const uint32_t s = f2d.sensor_idx * frame.scans_per_frame + f2d.scan_idx;
    const CombinationControl& cc = comb_controls[s];
    const float az0 = cc.combined_angles[f2d.r0][f2d.r0_a0];
    const float az1 = cc.combined_angles[f2d.r0][f2d.r0_a1];
    const float azimuth1D = az0 * (1 - f2d.r0_azimuth_frac) + az1 * f2d.r0_azimuth_frac;

    ScanObject_Impl* scan1D = frame.scans[f2d.sensor_idx].scan[f2d.scan_idx];
    assert(scan1D);
    const UhdpScanInformation& info1D = scan1D->get_scan_info();
    const float range1D = (f2d.r0 + f2d.range_frac) * info1D.range_bin_width; // meters
    float xc = range1D * cosf(azimuth1D); // meters
    float yc = range1D * sinf(azimuth1D); // meters

    for (uint32_t scan_idx = 0; scan_idx < frame.scans_per_frame; scan_idx++)
    {
        ScanObject_Impl* scan2D = frame.scans[f2d.sensor_idx].scan[scan_idx];
        if (scan2D && scan2D->scan_info.num_elevation_angles > 1)
        {
            ClutterImage* ci = scan2D->get_clutter_image();
            if (ci)
            {
                const UhdpScanInformation& info2D = scan2D->get_scan_info();
                const uint32_t s = f2d.sensor_idx * frame.scans_per_frame + scan_idx;
                const CombinationControl& cc = comb_controls[s];

                float diff_sec = 0;
                if (info2D.connection_uhdp_version >= 34)
                {
                    float diff_tics = float(int32_t(info2D.scan_timestamp - info1D.scan_timestamp));
                    float diff_us = (diff_tics * info2D.clock_tick_numerator) / info2D.clock_tick_denominator;
                    diff_sec = diff_us / 1e6;
                }

                // D2_range = sqrt((xc - dx) ^ 2 + (yc - dy) ^ 2);
                // D2_azimuth = atan2d(xc - dx, yc - dy) - da;
                float dx = xc - info2D.estimated_ego_velocity_X * diff_sec; // meters
                float dy = yc - info2D.estimated_ego_velocity_Y * diff_sec; // meters
                float range2D = sqrtf(dx * dx + dy * dy); // meters
                float azimuth2D = atan2f(dy, dx) - info2D.ego_angular_velocity_Z * diff_sec;

                if (info2D.angle_wrap_flags & 1)
                {
                    // use unwrapped full field of view
                    uint32_t az;
                    for (az = 0; az < cc.num_expanded_2D_azimuth_angles - 1; az++)
                    {
                        if (azimuth2D < cc.expanded_2D_azimuth_angles[az + 1])
                        {
                            break;
                        }
                    }
                    float az_frac = (azimuth2D - cc.expanded_2D_azimuth_angles[az]) /
                            (cc.expanded_2D_azimuth_angles[az + 1] - cc.expanded_2D_azimuth_angles[az]);
                    uint32_t num_extras = (cc.num_expanded_2D_azimuth_angles - 1 - info2D.num_azimuth_angles) / 2;
                    int32_t rel_bin = int32_t(az) - num_extras;
                    while (rel_bin < 0)
                    {
                        rel_bin += info2D.num_azimuth_angles;
                    }
                    while (rel_bin >= info2D.num_azimuth_angles)
                    {
                        rel_bin -= info2D.num_azimuth_angles;
                    }
                    azimuth2D = rel_bin + az_frac;
                }
                else if (azimuth2D < cc.az_fov_start || azimuth2D > cc.az_fov_end)
                {
                    // outside of the 2D FOV
                    return 0;
                }

                float elevation = static_cast<ClutterImage_Impl*>(ci)->elevation_interpolate(range2D / info2D.range_bin_width, azimuth2D);
                return range1D * sinf(-elevation);
            }
        }
    }

    return 0;
}


float FrameP2C_Impl::get_doppler_sample(const FrameObject_Impl& frame, const FFrac2D& f2d)
{
    const uint32_t s = f2d.sensor_idx * frame.scans_per_frame + f2d.scan_idx;
    ScanObject_Impl* scan = frame.scans[f2d.sensor_idx].scan[f2d.scan_idx];
    if (scan && scan->zero_d_bins)
    {
        ClutterImage* ci = scan->get_clutter_image();
        if (ci)
        {
            const CombinationControl& cc = comb_controls[s];
            float az0 = cc.combined_angles[f2d.r0][f2d.r0_a0];
            float az1 = cc.combined_angles[f2d.r0][f2d.r0_a1];
            float azimuth = az0 * (1 - f2d.r0_azimuth_frac) + az1 * f2d.r0_azimuth_frac;
            float range = f2d.r0 + f2d.range_frac;

            return static_cast<ClutterImage_Impl*>(ci)->doppler_interpolate(range, azimuth);
        }
    }

    return 0;
}


static void add_to_bounding_area(float& maxX, float& minX, float& maxY, float& minY, float x, float y)
{
    maxX = fmaxf(maxX, x);
    minX = fminf(minX, x);
    maxY = fmaxf(maxY, y);
    minY = fminf(minY, y);
}


FrameP2C* FrameP2C::create_cartesian_mapping(
        const SensorGroup& sg,          // sensor positions
        const FrameObject& frame,       // scan-info
        float pixels_per_meter,         // zoom level
        bool asynchronous)
{
    const SensorGroup_Impl& sgi = static_cast<const SensorGroup_Impl&>(sg);
    const FrameObject_Impl& fr  = static_cast<const FrameObject_Impl&>(frame);

    float maxXmeters = 0;
    float maxYmeters = 0;
    float minXmeters = 0;
    float minYmeters = 0;

    // origin is always visible, for.. reasons
    add_to_bounding_area(maxXmeters, minXmeters, maxYmeters, minYmeters, 0, 0);

    // determine the required extents for this collection of scans at this pixel
    // density
    for (uint32_t s = 0; s < fr.sensor_count; s++)
    {
        float max_range = 0;
        const vec3f_t pos = sgi.sensor[s].sensor_position;

        // the sensor position must be visible
        add_to_bounding_area(maxXmeters, minXmeters, maxYmeters, minYmeters, pos.x, pos.y);

        for (uint32_t i = 0; i < fr.scans_per_frame; i++)
        {
            ScanObject_Impl* scan = fr.scans[s].scan[i];

            if (scan == NULL)
            {
                // We need a completely full frame to pre-cook the image
                printf("Frame is missing a scan, unable to create a p2c\n");
                return NULL;
            }
            else
            {
                ClutterImage* ci = scan->get_clutter_image();
                if (!ci)
                {
                    printf("Clutter image not captured, cannot create p2c\n");
                    return NULL;
                }
                if (ci->get_elevation_dimension() == 1)
                {
                    const UhdpScanInformation& info = scan->get_scan_info();
                    float range = ci->get_range_dimension() * info.range_bin_width;
                    const float az_fov = scan->get_azimuth_rad(0);

                    // the scan's right most last range bin must be visible
                    vec3f_t rel_pos;
                    const vec3f_t right = vec3f_t(range * cosf(az_fov), range * sin(az_fov), 0);
                    rel_pos = sgi.sensor[s].sensor_xform.transform(right);
                    add_to_bounding_area(maxXmeters, minXmeters, maxYmeters, minYmeters, rel_pos.x, rel_pos.y);

                    // the scan's left most last range bin must be visible
                    const vec3f_t left = vec3f_t(range * cosf(-az_fov), range * sin(-az_fov), 0);
                    rel_pos = sgi.sensor[s].sensor_xform.transform(left);
                    add_to_bounding_area(maxXmeters, minXmeters, maxYmeters, minYmeters, rel_pos.x, rel_pos.y);

                    // the scan's center most last range bin must be visible
                    const vec3f_t center = vec3f_t(range, 0, 0);
                    rel_pos = sgi.sensor[s].sensor_xform.transform(center);
                    add_to_bounding_area(maxXmeters, minXmeters, maxYmeters, minYmeters, rel_pos.x, rel_pos.y);

                    max_range = fmaxf(max_range, range);
                    if (pixels_per_meter * info.range_bin_width < 1)
                    {
                        printf("This pixel density is too low for this range bin size\n");
                    }
                }
            }
        }

        if (max_range == 0)
        {
            printf("The frame config must include at least one azimuth-only scan\n");
            return NULL;
        }
    }

    const uint32_t dimX = uint32_t(ceilf((maxXmeters - minXmeters) * pixels_per_meter));
    const uint32_t dimY = uint32_t(ceilf((maxYmeters - minYmeters) * pixels_per_meter));
    const uint32_t centerX = uint32_t(-minXmeters * pixels_per_meter);
    const uint32_t centerY = uint32_t(-minYmeters * pixels_per_meter);

    FrameP2C_Impl* p2c = new FrameP2C_Impl(dimX, dimY, centerX, centerY, fr.sensor_count * fr.scans_per_frame, pixels_per_meter);

    for (uint32_t s = 0; s < fr.sensor_count; s++)
    {
        const vec3f_t sensor_pos = sgi.sensor[s].sensor_position;
        const quatf_t sensor_orientation = sgi.sensor[s].sensor_orientation;

        for (uint32_t i = 0; i < fr.scans_per_frame; i++)
        {
            ScanObject_Impl* scan = fr.scans[s].scan[i];
            ClutterImage_Impl* ci = static_cast<ClutterImage_Impl*>(scan->get_clutter_image());
            const UhdpScanInformation& info = scan->get_scan_info();
            const float pixels_per_range_bin = pixels_per_meter * info.range_bin_width;
            const uint32_t scanidx = s * fr.scans_per_frame + i;

            // pre-compute Anti-Aliased angle bins for each scan at this pixel density
            CombinationControl& cc = p2c->comb_controls[scanidx];
            cc.compute_angle_bins(*ci, pixels_per_range_bin);
            cc.sensor_pos          = sensor_pos;
            cc.sensor_orientation  = sensor_orientation;
            cc.sensor_idx          = s;
            cc.scan_idx            = i;

            if (cc.az_el_scan)
            {
                float max_el = fmax(fabsf(cc.el_fov_end), fabsf(cc.el_fov_start));
                float max_height = cc.max_range_m * sin(max_el);
                p2c->max_height_m = fmax(p2c->max_height_m, max_height);
            }
            else
            {
                p2c->max_doppler_mps = fmax(p2c->max_doppler_mps, cc.max_doppler_mps);
            }
        }
    }

    if (asynchronous)
    {
        p2c->start();
    }
    else
    {
        p2c->cook();
    }

    return p2c;
}


void FrameP2C_Impl::thread_main()
{
    cook();
}


void FrameP2C_Impl::cook()
{
    const float epsilon = 1e-4f;
    const float minX = -float(platformCenterX);
    const float minY = -float(platformCenterY);
    uint32_t discarded_merge_pixels = 0;

    vec3f_t pixel_global_pos; // pixel position as distance from platform center (m)
    pixel_global_pos.z = 0;

    for (uint32_t x = 0; !aborted && (x < dimX); x++)
    {
        pixel_global_pos.x = (minX + x) / pixels_per_meter;

        for (uint32_t y = 0; !aborted && (y < dimY); y++)
        {
            pixel_global_pos.y = (minY + y) / pixels_per_meter;

            for (uint32_t i = 0; i < num_scans; i++)
            {
                const CombinationControl& cc = comb_controls[i];

                if (cc.az_el_scan)
                {
                    continue;
                }

                // calculate relative position, in this sensor's reference frame
                const vec3f_t rel_pos = cc.sensor_orientation.rotateInv(pixel_global_pos - cc.sensor_pos);

                float range = rel_pos.abs();
                if (range + epsilon >= cc.max_range_m)
                {
                    continue;
                }

                const float az = atan2f(rel_pos.y, rel_pos.x);
                if ((az < cc.az_fov_start) || (az >= cc.az_fov_end))
                {
                    continue;
                }

                FrameP2C_Impl::FFrac2D* f2d = NULL;
                for (uint32_t m = 0; m < FrameP2C_Impl::MAX_MERGE_POINTS; m++)
                {
                    if (frac[m][x * dimY + y].sensor_idx < 0)
                    {
                        f2d = &frac[m][x * dimY + y];
                        break;
                    }
                }
                if (!f2d)
                {
                    discarded_merge_pixels++;
                    continue;
                }

                // Range LERP
                const float range_bins = range / cc.range_bin_width;
                f2d->sensor_idx = cc.sensor_idx;
                f2d->scan_idx   = cc.scan_idx;
                f2d->r0         = uh_floorf(range_bins);
                f2d->range_frac = range_bins - f2d->r0;

                // azimuth LERP in near range bin
                uint32_t num_az = cc.num_combined_angles[f2d->r0];
                const float* az_angles = &cc.combined_angles[f2d->r0][0];
                float r0_azimuth = num_az - 1 - epsilon; // if no greater az is found, it must be last

                for (uint32_t abin = 1; abin < num_az; abin++)
                {
                    if (az_angles[abin] > az)
                    {
                        float a0 = az_angles[abin - 1];
                        float a1 = az_angles[abin];
                        // calculate fractional azimuth bin ID
                        r0_azimuth = abin - (a1 - az) / (a1 - a0);
                        break;
                    }
                }

                f2d->r0_a0 = uh_floorf(r0_azimuth);
                f2d->r0_a1 = f2d->r0_a0 + 1;
                f2d->r0_azimuth_frac = r0_azimuth - f2d->r0_a0;
                f2d->r0_a0 = f2d->r0_a0 % num_az;
                f2d->r0_a1 = f2d->r0_a1 % num_az;

                if ((f2d->r0 + 1U) < cc.num_valid_range_bins)
                {
                    // azimuth LERP in far range bin
                    num_az = cc.num_combined_angles[f2d->r0 + 1];
                    az_angles = &cc.combined_angles[f2d->r0 + 1][0];

                    float r1_azimuth = num_az - 1 - epsilon; // if no greater az is found, it must be last
                    for (uint32_t abin = 1; abin < num_az; abin++)
                    {
                        if (az_angles[abin] > az)
                        {
                            float a0 = az_angles[abin - 1];
                            float a1 = az_angles[abin];
                            // calculate fractional azimuth bin ID
                            r1_azimuth = abin - (a1 - az) / (a1 - a0);
                            break;
                        }
                    }

                    f2d->r1_a0 = uh_floorf(r1_azimuth);
                    f2d->r1_a1 = f2d->r1_a0 + 1;
                    f2d->r1_azimuth_frac = r1_azimuth - f2d->r1_a0;
                    f2d->r1_a0 = f2d->r1_a0 % num_az;
                    f2d->r1_a1 = f2d->r1_a1 % num_az;
                }
                else
                {
                    // do not look past the last range bin
                    f2d->range_frac = 0;
                    f2d->r1_a0 = 0;
                    f2d->r1_a1 = 0;
                    f2d->r1_azimuth_frac = 0;
                }
            }
        }
    }

    if (discarded_merge_pixels)
    {
        printf("%u active pixels were dropped for FOV overlap density\n", discarded_merge_pixels);
    }

    cooking_complete = true;
}
