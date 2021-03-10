// Copyright (C) Uhnder, Inc. All rights reserved. Confidential and Proprietary - under NDA.
// Refer to SOFTWARE_LICENSE file for details

#include "modules/drivers/radar/rocket_radar/driver/system-radar-software/env-reference/coredefs/uhnder-common.h"
#include "modules/drivers/radar/rocket_radar/driver/src/polar2cartesian_impl.h"
#include "modules/drivers/radar/rocket_radar/driver/src/clutterimage_impl.h"
#include "modules/drivers/radar/rocket_radar/driver/src/scanobject_impl.h"
#include "modules/drivers/radar/rocket_radar/driver/system-radar-software/engine/scp-src/eng-api/uhmath.h"
#include <assert.h>

/* special small floating point number used to avoid indexing into the very last
 * array index, since we always interpolate between floor(val) .. floor(val)+1.
 * When val is the last possible index, we subtract epsilon from it so
 * floor(val) is val-1 and the fract portion is 1-epsilon */
static const float epsilon = 1e-4f;

const uint32_t* Polar2Cartesian_Impl::process(const ClutterImage& ci, SubSpace* subs, float alpha)
{
    const ClutterImage_Impl& cii = static_cast<const ClutterImage_Impl&>(ci);

    uint32_t R = cii.get_range_dimension();
    uint32_t A = cii.get_azimuth_dimension();
    uint32_t E = cii.get_elevation_dimension();

    //uint32_t max_range_bins = subs->max_range_bins + 10;
    //uint32_t min_range_bins = subs->min_range_bins < 10  ? 0 : subs->min_range_bins - 10;
    if (range_bins != R)
    {
        last_err = P2C_INVALID_CLUTTER_IMAGE;
        return NULL;
    }
    else if ((azimuth_bins != A) && (azimuth_bins != 3 * A + 1))
    {
        last_err = P2C_INVALID_CLUTTER_IMAGE;
        return NULL;
    }
    else if ((elevation_bins != E) && (elevation_bins != 3 * E + 1))
    {
        last_err = P2C_INVALID_CLUTTER_IMAGE;
        return NULL;
    }

    SubSpace defaults;
    if (!subs)
    {
        memset(&defaults, 0, sizeof(defaults));
        if (frac2dfront)
        {
            // default to all range bins
            defaults.extent_X = R;
            defaults.extent_Y = 2;
            defaults.extent_Z = 2;
        }
        else
        {
            defaults.extent_X = dim_X;
            defaults.extent_Y = dim_Y;
            defaults.extent_Z = dim_Z;
            // default to all elevation bins
            defaults.elevation_mask = (1 << E) - 1;
        }
        subs = &defaults;
    }

    if ((alpha < 0.0f) || (alpha > 1.0f) ||
        memcmp(subs, &buffered_subspace, sizeof(SubSpace)))
    {
        alpha = 0.0f;
    }
    memcpy(&buffered_subspace, subs, sizeof(SubSpace));
    float one_sub_alpha = 1 - alpha;

    int32_t first_Y = subs->center_Y - subs->extent_Y / 2 + dim_Y / 2;
    int32_t first_Z = subs->center_Z - subs->extent_Z / 2 + dim_Z / 2;

    if (frac2dfront)
    {
        if (subs->first_X + subs->extent_X > R || !subs->extent_X)
        {
            last_err = P2C_SUBSPACE_OUT_OF_RANGE;
            return NULL;
        }
    }
    else if ((subs->first_X + subs->extent_X > dim_X) ||
             (first_Y < 0) || (first_Z < 0) ||
            ((first_Y + subs->extent_Y) > dim_Y) ||
            ((first_Z + subs->extent_Z) > dim_Z))
    {
        last_err = P2C_SUBSPACE_OUT_OF_RANGE;
        return NULL;
    }

    // output values are SNR linear, so will be somewhat small. We have 32bits
    // for output, use 8 bits to represent fractional values. aka: Q24.8
    enum { FRAC_BITS = (1 << 8) };

    int32_t value;
    if (frac2d)
    {
        comb_control.combine_angle_bins(cii, subs->elevation_mask);
        const uint16_t* plane = &comb_control.combined[0];
        const uint32_t stride = comb_control.stride;

        for (uint32_t x = 0; x < subs->extent_X; x++)
        {
            uint32_t read_x  = subs->first_X + x;
            uint32_t write_x = subs->flip_X ? dim_X - read_x - 1 : read_x;

            for (uint32_t y = 0; y < subs->extent_Y; y++)
            {
                uint32_t read_y = first_Y + y;

                const Frac2D& f2d = frac2d[read_x * dim_Y + read_y];
                if (f2d.r0 == R)
                {
                    // NB: if this function is SIMD optimized, this early out
                    // can be completely ignored, the combined buffer has a
                    // reserved section that can safely interpolate these pixels
                    value = 0;
                }
                else
                {
                    // bilinear interpolation
                    uint16_t q00 = plane[f2d.r0 * stride + f2d.r0_a0];
                    uint16_t q01 = plane[f2d.r0 * stride + f2d.r0_a1];
                    uint16_t q10 = plane[f2d.r1 * stride + f2d.r1_a0];
                    uint16_t q11 = plane[f2d.r1 * stride + f2d.r1_a1];

                    float q0 = (q00 * (1 - f2d.r0_azimuth_frac) + q01 * f2d.r0_azimuth_frac) / cii.ci_row_noise_floor[f2d.r0];
                    float q1 = (q10 * (1 - f2d.r1_azimuth_frac) + q11 * f2d.r1_azimuth_frac) / cii.ci_row_noise_floor[f2d.r1];

                    value = (int32_t)((q0 * (1 - f2d.range_frac) + q1 * f2d.range_frac) * FRAC_BITS) + 1;
                }

                uint32_t write_y = subs->flip_Y ? dim_Y - read_y - 1 : read_y;
                uint32_t &out = obuf[write_x * dim_Y + write_y];
                out = one_sub_alpha * value + alpha * out;
            }
        }
    }
    else if (frac2dfront)
    {
        float* plane = new float[A * E];
        memset(plane, 0, sizeof(plane[0]) * (A * E));

        /* MAX across all selected range bins */
        for (uint32_t r = subs->first_X; r < subs->first_X + subs->extent_X; r++)
        {
            uint16_t* rb = cii.mag_plane + r * A * E;

            for (uint32_t a = 0; a < A * E; a++)
            {
                plane[a] = uh_fmaxf(plane[a], rb[a] / cii.ci_row_noise_floor[r]);
            }
        }

        for (uint32_t x = 0; x < dim_X; x++)
        {
            uint32_t read_x  = x;
            uint32_t write_x = subs->flip_X ? dim_X - x - 1 : x;

            for (uint32_t y = 0; y < dim_Y; y++)
            {
                uint32_t read_y = y;
                uint32_t write_y = subs->flip_Y ? dim_Y - y - 1 : y;

                const Frac2DFrontView& f2d = frac2dfront[read_x * dim_Y + read_y];

                uint32_t e0 = uh_floorf(f2d.elevation);
                float efrac = f2d.elevation - e0;

                uint32_t a0 = uh_floorf(f2d.azimuth);
                float afrac = f2d.azimuth - a0;

                uint32_t a1 = a0 + 1;
                uint32_t e1 = e0 + 1;

                // bilinear interpolation
                float q00 = plane[e0 * A + a0];
                float q01 = plane[e0 * A + a1];
                float q10 = plane[e1 * A + a0];
                float q11 = plane[e1 * A + a1];

                float q0 = (q00 * (1 - afrac) + q01 * afrac);
                float q1 = (q10 * (1 - afrac) + q11 * afrac);

                value = (int32_t)((q0 * (1 - efrac) + q1 * efrac) * FRAC_BITS) + 1;

                uint32_t &out = obuf[write_x * dim_Y + write_y];
                out = one_sub_alpha * uh_fmaxf(0, value) + alpha * out;
            }
        }

        delete [] plane;
    }
    else
    {
        for (uint32_t x = 0; x < subs->extent_X; x++)
        {
            uint32_t read_x  = subs->first_X + x;
            uint32_t write_x = subs->flip_X ? dim_X - read_x - 1 : read_x;

            for (uint32_t z = 0; z < subs->extent_Z; z++)
            {
                uint32_t read_z = first_Z + z;
                uint32_t write_z = subs->flip_Z ? dim_Z - read_z - 1 : read_z;

                for (uint32_t y = 0; y < subs->extent_Y; y++)
                {
                    uint32_t read_y = first_Y + y;
                    uint32_t write_y = subs->flip_Y ? dim_Y - read_y - 1 : read_y;
                    uint32_t &out =  obuf[(write_x * dim_Y * dim_Z) + (write_z * dim_Y) + write_y];

                    const Frac3D& f3d = frac3d[(read_x * dim_Y * dim_Z) + (read_z * dim_Y) + read_y];
                    if (f3d.range < 0)
                    {
                        value = 0;
                    }
                    else
                    {
                        uint32_t r0 = uh_floorf(f3d.range);
                        float rfrac = f3d.range - r0;

                        uint32_t a0 = uh_floorf(f3d.azimuth);
                        float afrac = f3d.azimuth - a0;

                        uint32_t e0 = uh_floorf(f3d.elevation);
                        float efrac = f3d.elevation - e0;

                        uint32_t r1 = r0 + 1;
                        uint32_t a1 = a0 + 1;
                        uint32_t e1 = e0 + 1;

                        // trilinear interpolation
                        uint16_t q000 = cii.mag_plane[r0 * A * E + e0 * A + a0];
                        uint16_t q001 = cii.mag_plane[r0 * A * E + e0 * A + a1];
                        uint16_t q010 = cii.mag_plane[r0 * A * E + e1 * A + a0];
                        uint16_t q011 = cii.mag_plane[r0 * A * E + e1 * A + a1];
                        uint16_t q100 = cii.mag_plane[r1 * A * E + e0 * A + a0];
                        uint16_t q101 = cii.mag_plane[r1 * A * E + e0 * A + a1];
                        uint16_t q110 = cii.mag_plane[r1 * A * E + e1 * A + a0];
                        uint16_t q111 = cii.mag_plane[r1 * A * E + e1 * A + a1];

                        float q00 = q000 * (1 - afrac) + q001 * afrac;
                        float q01 = q010 * (1 - afrac) + q011 * afrac;
                        float q10 = q100 * (1 - afrac) + q101 * afrac;
                        float q11 = q110 * (1 - afrac) + q111 * afrac;

                        float q0 = (q00 * (1 - efrac) + q01 * efrac) / cii.ci_row_noise_floor[r0];
                        float q1 = (q10 * (1 - efrac) + q11 * efrac) / cii.ci_row_noise_floor[r1];

                        value = (int32_t)((q0 * (1 - rfrac) + q1 * rfrac) * FRAC_BITS) + 1;
                    }

                    out = one_sub_alpha * uh_fmaxf(0, value) + alpha * out;
                }
            }
        }
    }

    last_err = P2C_NO_ERROR;
    return obuf;
}


Polar2Cartesian* Polar2Cartesian::create2DMapping(const ClutterImage& ci, const PixelSpace2D& psp, bool asynchronous)
{
    const ClutterImage_Impl& cii = static_cast<const ClutterImage_Impl&>(ci);

    uint32_t R = cii.get_range_dimension();
    uint32_t A = cii.get_azimuth_dimension();
    uint32_t E = cii.get_elevation_dimension();

    Polar2Cartesian_Impl* p2c = new Polar2Cartesian_Impl;

    p2c->dim_X = psp.dim_X;
    p2c->dim_Y = psp.dim_Y;
    p2c->dim_Z = 1;
    p2c->range_bins = R;
    p2c->azimuth_bins = A;
    p2c->elevation_bins = E;
    p2c->frac2d = new Polar2Cartesian_Impl::Frac2D[psp.dim_X * psp.dim_Y];
    p2c->obuf = new uint32_t[psp.dim_X * psp.dim_Y];

    p2c->comb_control.compute_angle_bins(cii, psp.pixels_per_range_bin);
    p2c->psp2d = psp;

    if (asynchronous)
    {
        p2c->cooking_complete = false;
        p2c->start();
    }
    else
    {
        p2c->cook();
    }

    return p2c;
}


Polar2Cartesian* Polar2Cartesian::create3DMapping(const ClutterImage& ci, const VoxelSpace3D& vsp)
{
    const ClutterImage_Impl& cii = static_cast<const ClutterImage_Impl&>(ci);

    uint32_t R = cii.get_range_dimension();
    uint32_t A = cii.get_azimuth_dimension();
    uint32_t E = cii.get_elevation_dimension();

    Polar2Cartesian_Impl* p2c = new Polar2Cartesian_Impl;

    p2c->dim_X = vsp.dim_X;
    p2c->dim_Y = vsp.dim_Y;
    p2c->dim_Z = vsp.dim_Z;
    p2c->range_bins = R;
    p2c->azimuth_bins = A;
    p2c->elevation_bins = E;
    p2c->frac3d = new Polar2Cartesian_Impl::Frac3D[vsp.dim_X * vsp.dim_Y * vsp.dim_Z];
    p2c->obuf   = new uint32_t[vsp.dim_X * vsp.dim_Y * vsp.dim_Z];

    for (uint32_t i = 0; i < vsp.dim_X * vsp.dim_Y * vsp.dim_Z; i++)
    {
        p2c->frac3d[i].range = -1;
        p2c->frac3d[i].azimuth = -1;
        p2c->frac3d[i].elevation = -1;
    }

    BoundsInfo b;
    estimate_bounds(b, ci, vsp.wrap_azimuth_ambiguity, vsp.wrap_elevation_ambiguity);
    A = b.num_azimuth_bins;
    E = b.num_elevation_bins;

    float first_Y = vsp.center_Y - (vsp.dim_Y * 0.5f / vsp.y_voxels_per_range_bin);
    float first_Z = vsp.center_Z - (vsp.dim_Z * 0.5f / vsp.z_voxels_per_range_bin);

    float elevation_limit = uh_fmaxf(E - 1 - epsilon, 0);
    float azimuth_limit = uh_fmaxf(A - 1 - epsilon, 0);

    for (uint32_t x = 0; x < vsp.dim_X; x++)
    {
        float xc = vsp.first_X + x / vsp.x_voxels_per_range_bin;

        if (xc >= b.num_valid_range_bins - 1) // no angle can be closer than this range
        {
            break;
        }

        for (uint32_t y = 0; y < vsp.dim_Y; y++)
        {
            float yc = first_Y + y / vsp.y_voxels_per_range_bin;
            float az = atan(yc / xc);

            if ((az < b.az_angles[0]) || (az >= b.az_angles[A - 1]))
            {
                continue;
            }

            float azimuth = azimuth_limit; // if no greater az is found, it must be last
            for (uint32_t abin = 1; abin < A; abin++)
            {
                if (b.az_angles[abin] > az)
                {
                    float a0 = b.az_angles[abin - 1];
                    float a1 = b.az_angles[abin];
                    // calculate fractional azimuth bin ID
                    azimuth = abin - (a1 - az) / (a1 - a0);

                    // modulo azimuth bin within original angle set (ambiguity)
                    float fract = azimuth - int(azimuth);
                    azimuth = (int(azimuth) % cii.get_azimuth_dimension()) + fract;
                    break;
                }
            }

            for (uint32_t z = 0; z < vsp.dim_Z; z++)
            {
                float zc = first_Z + z / vsp.z_voxels_per_range_bin;

                float range = sqrt(xc * xc + yc * yc + zc * zc);
                if (range >= b.num_valid_range_bins - 1)
                {
                    continue;
                }

                float el = asin(zc / range);

                if ((el < b.el_angles[0]) || (el >= b.el_angles[E - 1]))
                {
                    continue;
                }

                float elevation = elevation_limit; // if no greater az is found, it must be last
                for (uint32_t ebin = 1; ebin < E; ebin++)
                {
                    if (b.el_angles[ebin] > el)
                    {
                        float e0 = b.el_angles[ebin - 1];
                        float e1 = b.el_angles[ebin];
                        // calculate fractional elevation bin ID
                        elevation = ebin - (e1 - el) / (e1 - e0);

                        // modulo elevation bin within original angle set (ambiguity)
                        float fract = elevation - int(elevation);
                        elevation = (int(elevation) % cii.get_elevation_dimension()) + fract;
                        break;
                    }
                }

                p2c->frac3d[(x * vsp.dim_Y * vsp.dim_Z) + (z * vsp.dim_Y) + y].range     = range;
                p2c->frac3d[(x * vsp.dim_Y * vsp.dim_Z) + (z * vsp.dim_Y) + y].azimuth   = azimuth;
                p2c->frac3d[(x * vsp.dim_Y * vsp.dim_Z) + (z * vsp.dim_Y) + y].elevation = elevation;
            }
        }
    }

    return p2c;
}


Polar2Cartesian* Polar2Cartesian::createFrontView(const ClutterImage& ci, const FrontViewDesc& fv)
{
    const ClutterImage_Impl& cii = static_cast<const ClutterImage_Impl&>(ci);

    if (cii.myscan.scan_info.num_elevation_angles == 1)
    {
        return NULL;
    }

    Polar2Cartesian_Impl* p2c = new Polar2Cartesian_Impl;

    BoundsInfo b;
    estimate_bounds(b, ci, fv.wrap_azimuth_ambiguity, fv.wrap_elevation_ambiguity);
    const uint32_t R = cii.get_range_dimension();
    const uint32_t A = b.num_azimuth_bins;
    const uint32_t E = b.num_elevation_bins;

    p2c->dim_X = E * fv.pixels_per_angle_bin;
    p2c->dim_Y = A * fv.pixels_per_angle_bin;
    p2c->dim_Z = 1;
    p2c->range_bins = R;
    p2c->azimuth_bins = A;
    p2c->elevation_bins = E;
    p2c->frac2dfront = new Polar2Cartesian_Impl::Frac2DFrontView[p2c->dim_X * p2c->dim_Y];
    p2c->obuf = new uint32_t[p2c->dim_X * p2c->dim_Y];

    float first_el = b.el_angles[0];
    float last_el = b.el_angles[E - 1];
    float first_az = b.az_angles[0];
    float last_az = b.az_angles[A - 1];

    float elevation_limit = uh_fmaxf(E - 1 - epsilon, 0);
    float azimuth_limit = uh_fmaxf(A - 1 - epsilon, 0);

    for (uint32_t x = 0; x < p2c->dim_X; x++)
    {
        float el = first_el + x * (last_el - first_el) / p2c->dim_X;

        float elevation = elevation_limit; // if no greater el is found, it must be last
        for (uint32_t ebin = 1; ebin < E; ebin++)
        {
            if (b.el_angles[ebin] > el)
            {
                float e0 = b.el_angles[ebin - 1];
                float e1 = b.el_angles[ebin];
                // calculate fractional azimuth bin ID
                elevation = ebin - (e1 - el) / (e1 - e0);

                float fract = elevation - int(elevation);
                elevation = (int(elevation) % cii.get_elevation_dimension()) + fract;
                break;
            }
        }

        for (uint32_t y = 0; y < p2c->dim_Y; y++)
        {
            float az = first_az + y * (last_az - first_az) / p2c->dim_Y;

            float azimuth = azimuth_limit; // if no greater az is found, it must be last
            for (uint32_t abin = 1; abin < A; abin++)
            {
                if (b.az_angles[abin] > az)
                {
                    float a0 = b.az_angles[abin - 1];
                    float a1 = b.az_angles[abin];
                    // calculate fractional azimuth bin ID
                    azimuth = abin - (a1 - az) / (a1 - a0);

                    float fract = azimuth - int(azimuth);
                    azimuth = (int(azimuth) % cii.get_azimuth_dimension()) + fract;
                    break;
                }
            }

            p2c->frac2dfront[x * p2c->dim_Y + y].elevation = elevation;
            p2c->frac2dfront[x * p2c->dim_Y + y].azimuth = azimuth;
        }
    }

    return p2c;
}



bool Polar2Cartesian::estimate_bounds(
        BoundsInfo& b,
        const ClutterImage& ci,
        bool  wrap_azimuth_ambiguity,
        bool  wrap_elevation_ambiguity)
{
    const ClutterImage_Impl& cii = static_cast<const ClutterImage_Impl&>(ci);
    return estimate_bounds_scan_object(b, cii.myscan, wrap_azimuth_ambiguity, wrap_elevation_ambiguity);
}


bool Polar2Cartesian::estimate_bounds_scan_object(
        BoundsInfo& b,
        const ScanObject& scan_obj,
        bool  wrap_azimuth_ambiguity,
        bool  wrap_elevation_ambiguity)
{
    const ScanObject_Impl& scan = static_cast<const ScanObject_Impl&>(scan_obj);
    if (!scan.range_bins || !scan.angle_bins)
    {
        return false;
    }

    const UhdpScanInformation& scan_info = scan.scan_info;
    const uint32_t R = scan_info.num_range_bins;
    const uint32_t A = scan_info.num_azimuth_angles;
    const uint32_t E = (scan_info.CI_format == CIMG_M16PP_D7PP) ? scan_info.num_elevation_angles : 1;
    const uint32_t mid_el = (E > 1) ? A * (E / 2) : 0;
    b.num_valid_range_bins = 0;

    for (uint32_t r = 0; r < R; r++)
    {
        if (scan.range_bins[r].distance_in_bins >= 0)
        {
            b.num_valid_range_bins++;
        }
    }

    b.az_angles = NULL;
    if (wrap_azimuth_ambiguity && (scan_info.angle_wrap_flags & 1))
    {
        b.num_azimuth_bins = 3 * A + 1;
        b.az_angles = new float[b.num_azimuth_bins];

        // middle third is a copy of original azimuth angles
        for (uint32_t i = 0; i < A; i++)
        {
            b.az_angles[A + i] = scan.angle_bins[mid_el + i].azimuth;
        }

        // extend counter clock-wise
        float first_sin = sin(scan.angle_bins[mid_el + 0].azimuth);
        float diff_sin_angle_rad = sin(scan.angle_bins[mid_el + 1].azimuth) - first_sin;
        for (uint32_t i = 0; i < A; i++)
        {
            float res = first_sin - (i + 1) * diff_sin_angle_rad;
            if (res > -1.0f)
                b.az_angles[A - (i + 1)] = asin(res);
            else
                b.az_angles[A - (i + 1)] = -(M_PI / 2);
        }

        // extend clock-wise, one extra
        float last_sin = sin(scan.angle_bins[mid_el + A - 1].azimuth);
        for (uint32_t i = 0; i <= A; i++)
        {
            float res = last_sin + (i + 1) * diff_sin_angle_rad;
            if (res < 1.0f)
                b.az_angles[2 * A + i] = asin(res);
            else
                b.az_angles[2 * A + i] = M_PI / 2;
        }
    }
    else
    {
        b.num_azimuth_bins = A;
        b.az_angles = new float[b.num_azimuth_bins];
        for (uint32_t ii = 0; ii < A; ii++)
        {
            b.az_angles[ii] = scan.angle_bins[mid_el + ii].azimuth;
        }
    }

    b.el_angles = NULL;
    if (wrap_elevation_ambiguity && (scan_info.angle_wrap_flags & 2))
    {
        b.num_elevation_bins = 3 * E + 1;
        b.el_angles = new float[b.num_elevation_bins];

        // middle third is a copy of original elevation angles
        for (uint32_t i = 0; i < E; i++)
        {
            b.el_angles[E + i] = scan.angle_bins[i * A].elevation;
        }

        // extend counter clock-wise
        float first_sin = sin(scan.angle_bins[0].elevation);
        float diff_sin_angle_rad = sin(scan.angle_bins[1 * A].elevation) - first_sin;
        for (uint32_t i = 0; i < E; i++)
        {
            float res = first_sin - (i + 1) * diff_sin_angle_rad;
            if (res > -1.0f)
                b.el_angles[E - (i + 1)] = asin(res);
            else
                b.el_angles[E - (i + 1)] = -(M_PI / 2);
        }

        // extend clock-wise, one extra
        float last_sin = sin(scan.angle_bins[(E - 1) * A].elevation);
        for (uint32_t i = 0; i <= E; i++)
        {
            float res = last_sin + (i + 1) * diff_sin_angle_rad;
            if (res < 1.0f)
                b.el_angles[2 * E + i] = asin(res);
            else
                b.el_angles[2 * E + i] = M_PI / 2;
        }
    }
    else
    {
        b.num_elevation_bins = E;
        b.el_angles = new float[b.num_elevation_bins];
        for (uint32_t e = 0; e < E; e++)
        {
            b.el_angles[e] = scan.angle_bins[e * A].elevation;
        }
    }

    b.range_to_azimuth_ratio = sin(-b.az_angles[0]) * 2;
    b.range_to_elevation_ratio = sin(-b.el_angles[0]) * 2;

    return true;
}


Polar2Cartesian_Impl::~Polar2Cartesian_Impl()
{
    aborted = true;
    stop();

    delete [] frac2d;
    delete [] frac3d;
    delete [] frac2dfront;
    delete [] obuf;
}


void            Polar2Cartesian_Impl::thread_main()
{
    cook();
}

void            Polar2Cartesian_Impl::cook()
{
    const CombinationControl& cc = comb_control;

    Polar2Cartesian_Impl::Frac2D not_in_fov;
    not_in_fov.r0 = range_bins;
    not_in_fov.r1 = range_bins;
    not_in_fov.range_frac = 0;
    not_in_fov.r0_a0 = 0;
    not_in_fov.r0_a1 = 0;
    not_in_fov.r0_azimuth_frac = 0;
    not_in_fov.r1_a0 = 0;
    not_in_fov.r1_a1 = 0;
    not_in_fov.r1_azimuth_frac = 0;

    for (uint32_t i = 0; i < dim_X * dim_Y; i++)
    {
        frac2d[i] = not_in_fov;
    }

    float first_Y = psp2d.center_Y - (dim_Y * 0.5f / psp2d.pixels_per_range_bin);
    for (uint32_t x = 0; !aborted && (x < dim_X); x++)
    {
        float xc = psp2d.first_X + x / psp2d.pixels_per_range_bin;

        if (xc >= cc.num_valid_range_bins - 1) // no angle at this X bin can be closer than this range
        {
            break;                            // nor can any subsequent X bins
        }
        for (uint32_t y = 0; !aborted && (y < dim_Y); y++)
        {
            float yc = first_Y + y / psp2d.pixels_per_range_bin;

            float range = sqrt(xc * xc + yc * yc);
            if (range >= cc.num_valid_range_bins - 1)
            {
                continue;
            }

            float az = atan(yc / xc);
            if ((az < cc.az_fov_start) || (az >= cc.az_fov_end))
            {
                continue;
            }

            // Range LERP
            Polar2Cartesian_Impl::Frac2D& f2d = frac2d[x * dim_Y + y];
            f2d.r0         = uh_floorf(range);
            f2d.r1         = f2d.r0 + 1;
            f2d.range_frac = range - f2d.r0;

            // azimuth LERP in near range bin
            uint32_t num_az = cc.num_combined_angles[f2d.r0];
            const float* az_angles = &cc.combined_angles[f2d.r0][0];
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

            f2d.r0_a0 = uh_floorf(r0_azimuth);
            f2d.r0_a1 = f2d.r0_a0 + 1;
            f2d.r0_azimuth_frac = r0_azimuth - f2d.r0_a0;
            f2d.r0_a0 = f2d.r0_a0 % num_az;
            f2d.r0_a1 = f2d.r0_a1 % num_az;

            // azimuth LERP in far range bin
            num_az = cc.num_combined_angles[f2d.r1];
            az_angles = &cc.combined_angles[f2d.r1][0];
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

            f2d.r1_a0 = uh_floorf(r1_azimuth);
            f2d.r1_a1 = f2d.r1_a0 + 1;
            f2d.r1_azimuth_frac = r1_azimuth - f2d.r1_a0;
            f2d.r1_a0 = f2d.r1_a0 % num_az;
            f2d.r1_a1 = f2d.r1_a1 % num_az;
        }
    }

    cooking_complete = true;
}
