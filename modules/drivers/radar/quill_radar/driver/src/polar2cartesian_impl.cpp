// START_SOFTWARE_LICENSE_NOTICE
// -------------------------------------------------------------------------------------------------------------------
// Copyright (C) 2016-2019 Uhnder, Inc. All rights reserved.
// This Software is the property of Uhnder, Inc. (Uhnder) and is Proprietary and Confidential.  It has been provided
// under license for solely use in evaluating and/or developing code for Uhnder products.  Any use of the Software to
// develop code for a product not manufactured by or for Uhnder is prohibited.  Unauthorized use of this Software is
// strictly prohibited.
// Restricted Rights Legend:  Use, Duplication, or Disclosure by the Government is Subject to Restrictions as Set
// Forth in Paragraph (c)(1)(ii) of the Rights in Technical Data and Computer Software Clause at DFARS 252.227-7013.
// THIS PROGRAM IS PROVIDED UNDER THE TERMS OF THE UHNDER END-USER LICENSE AGREEMENT (EULA). THE PROGRAM MAY ONLY
// BE USED IN A MANNER EXPLICITLY SPECIFIED IN THE EULA, WHICH INCLUDES LIMITATIONS ON COPYING, MODIFYING,
// REDISTRIBUTION AND WARRANTIES. PROVIDING AFFIRMATIVE CLICK-THROUGH CONSENT TO THE EULA IS A REQUIRED PRECONDITION
// TO YOUR USE OF THE PROGRAM. YOU MAY OBTAIN A COPY OF THE EULA FROM WWW.UHNDER.COM. UNAUTHORIZED USE OF THIS
// PROGRAM IS STRICTLY PROHIBITED.
// THIS SOFTWARE IS PROVIDED "AS IS".  NO WARRANTIES ARE GIVEN, WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING
// WARRANTIES OR MERCHANTABILITY OR FITNESS FOR A PARTICULAR PURPOSE, NONINFRINGEMENT AND TITLE.  RECIPIENT SHALL HAVE
// THE SOLE RESPONSIBILITY FOR THE ADEQUATE PROTECTION AND BACK-UP OF ITS DATA USED IN CONNECTION WITH THIS SOFTWARE.
// IN NO EVENT WILL UHNDER BE LIABLE FOR ANY CONSEQUENTIAL DAMAGES WHATSOEVER, INCLUDING LOSS OF DATA OR USE, LOST
// PROFITS OR ANY INCIDENTAL OR SPECIAL DAMAGES, ARISING OUT OF OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS
// SOFTWARE, WHETHER IN ACTION OF CONTRACT OR TORT, INCLUDING NEGLIGENCE.  UHNDER FURTHER DISCLAIMS ANY LIABILITY
// WHATSOEVER FOR INFRINGEMENT OF ANY INTELLECTUAL PROPERTY RIGHTS OF ANY THIRD PARTY.
// -------------------------------------------------------------------------------------------------------------------
// END_SOFTWARE_LICENSE_NOTICE

#include "modules/drivers/radar/quill_radar/driver/system-radar-software/env-uhnder/coredefs/uhnder-common.h"
#include "modules/drivers/radar/quill_radar/driver/src/polar2cartesian_impl.h"
#include "modules/drivers/radar/quill_radar/driver/src/clutterimage_impl.h"
#include "modules/drivers/radar/quill_radar/driver/src/scanobject_impl.h"
#include "modules/drivers/radar/quill_radar/driver/system-radar-software/engine/scp-src/eng-api/uhmath.h"
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
        uint16_t* elcombined = NULL;
        uint16_t* plane = cii.mag_plane;
        uint32_t  range_stride = A * E;

        if (E > 1)
        {
            uint32_t ecount = 0;
            for (uint32_t e = 0; e < E; e++)
            {
                if (subs->elevation_mask & (1U << e))
                {
                    plane = cii.mag_plane + e * A;
                    ecount++;
                }
            }

            if (!ecount)
            {
                last_err = P2C_SUBSPACE_OUT_OF_RANGE;
                return NULL;
            }
            else if (ecount > 1)
            {
                elcombined = new uint16_t[R * A + 16];
                memset(elcombined, 0, sizeof(elcombined[0]) * (R * A + 16));

                /* MAX across all selected planes */
                for (uint32_t e = 0; e < E; e++)
                {
                    if (subs->elevation_mask & (1U << e))
                    {
                        uint16_t* el = cii.mag_plane + e * A;
                        // HACK HACK HACK - ignore the first two range bins in
                        // two-D scans; the noise floor tends to be non-sensical
                        // and once we go to cartesian they occupy very little
                        // space on-screen anyway.  If the CI bug(s) are
                        // resolved this 2 can be made back into 0
                        for (uint32_t r = 2; r < R; r++)
                        {
                            for (uint32_t a = 0; a < A; a++)
                            {
                                elcombined[r * A + a] = uh_uintmax(elcombined[r * A + a], el[r * range_stride + a]);
                            }
                        }
                    }
                }

                plane = elcombined;
                range_stride = A;
            }
        }

        for (uint32_t x = 0; x < subs->extent_X; x++)
        {
            uint32_t read_x  = subs->first_X + x;
            uint32_t write_x = subs->flip_X ? dim_X - read_x - 1 : read_x;

            for (uint32_t y = 0; y < subs->extent_Y; y++)
            {
                uint32_t read_y = first_Y + y;
                uint32_t write_y = subs->flip_Y ? dim_Y - read_y - 1 : read_y;
                uint32_t &out = obuf[write_x * dim_Y + write_y];

                const Frac2D& f2d = frac2d[read_x * dim_Y + read_y];
                if (f2d.range < 0)
                {
                    value = 0;
                }
                else
                {
                    uint32_t r0 = uh_floorf(f2d.range);
                    float rfrac = f2d.range - r0;

                    uint32_t a0 = uh_floorf(f2d.azimuth);
                    float afrac = f2d.azimuth - a0;

                    uint32_t a1 = (a0 + 1) % cii.get_azimuth_dimension();
                    uint32_t r1 = r0 + 1;

                    // bilinear interpolation
                    uint16_t q00 = plane[r0 * range_stride + a0];
                    uint16_t q01 = plane[r0 * range_stride + a1];
                    uint16_t q10 = plane[r1 * range_stride + a0];
                    uint16_t q11 = plane[r1 * range_stride + a1];

                    float q0 = (q00 * (1 - afrac) + q01 * afrac) / cii.ci_row_noise_floor[r0];
                    float q1 = (q10 * (1 - afrac) + q11 * afrac) / cii.ci_row_noise_floor[r1];

                    value = (int32_t)((q0 * (1 - rfrac) + q1 * rfrac) * FRAC_BITS) + 1;
                }

                out = one_sub_alpha * uh_uintmax(0, value) + alpha * out;
            }
        }

        delete [] elcombined;
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


Polar2Cartesian* Polar2Cartesian::create2DMapping(const ClutterImage& ci, const PixelSpace2D& psp)
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

    BoundsInfo b;
    estimate_bounds(b, ci, psp.wrap_ambiguity, false);
    A = b.num_azimuth_bins;
    E = b.num_elevation_bins;

    for (uint32_t i = 0; i < psp.dim_X * psp.dim_Y; i++)
    {
        p2c->frac2d[i].range = -1;
        p2c->frac2d[i].azimuth = -1;
    }

    float first_Y = psp.center_Y - (psp.dim_Y * 0.5f / psp.pixels_per_range_bin);
    for (uint32_t x = 0; x < psp.dim_X; x++)
    {
        float xc = psp.first_X + x / psp.pixels_per_range_bin;

        if (xc >= b.num_valid_range_bins - 1)// no angle at this X bin can be closer than this range
        {
            break;                     // nor can any subsequent X bins
        }
        for (uint32_t y = 0; y < psp.dim_Y; y++)
        {
            float yc = first_Y + y / psp.pixels_per_range_bin;

            float range = sqrt(xc * xc + yc * yc);
            if (range >= b.num_valid_range_bins - 1)
            {
                continue;
            }

            float az = atan(yc / xc);
            if ((az < b.az_angles[0]) || (az >= b.az_angles[A - 1]))
            {
                continue;
            }

            float azimuth = cii.get_azimuth_dimension() - 1 - epsilon; // if no greater az is found, it must be last
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

            p2c->frac2d[x * psp.dim_Y + y].range   = range;
            p2c->frac2d[x * psp.dim_Y + y].azimuth = azimuth;
        }
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


void Polar2Cartesian::estimate_bounds(
        BoundsInfo& b,
        const ClutterImage& ci,
        bool  wrap_azimuth_ambiguity,
        bool  wrap_elevation_ambiguity)
{
    const ClutterImage_Impl& cii = static_cast<const ClutterImage_Impl&>(ci);
    Polar2Cartesian::estimate_bounds_process(b,
                                             cii.myscan.scan_info,
                                             cii.myscan.range_bins,
                                             cii.myscan.angle_bins,
                                             wrap_azimuth_ambiguity, 
                                             wrap_elevation_ambiguity);
 
}


void Polar2Cartesian::estimate_bounds_scan_object(
        BoundsInfo& b,
        const ScanObject& obj,
        bool  wrap_azimuth_ambiguity,
        bool  wrap_elevation_ambiguity)
{
    const ScanObject_Impl& scan_obj = static_cast<const ScanObject_Impl&>(obj);
    Polar2Cartesian::estimate_bounds_process(b,
                                             scan_obj.get_scan_info(),
                                             scan_obj.range_bins,
                                             scan_obj.angle_bins,
                                             wrap_azimuth_ambiguity, 
                                             wrap_elevation_ambiguity);
}


void  Polar2Cartesian::estimate_bounds_process(BoundsInfo& b,
                                               const UhdpScanInformation& scan_info,
                                               const UhdpRangeBinInfo* range_bins,
                                               const UhdpAngleBinInfo* angle_bins,
                                               bool  wrap_azimuth_ambiguity,
                                               bool  wrap_elevation_ambiguity)
{
    
    const uint32_t R = scan_info.CI_height;
    const uint32_t A = scan_info.num_azimuth_angles;
    const uint32_t E = (scan_info.CI_format == CIMG_M16PP_D7PP) ? scan_info.num_elevation_angles : 1;
    b.num_valid_range_bins = 0;

    for (uint32_t r = 0; r < R; r++)
    {
        if (range_bins[r].distance_in_bins >= 0)
        {
            b.num_valid_range_bins++;
        }
    }

    //const UhdpAngleBinInfo* angle_bins = angle_bins;

    b.az_angles = NULL;
    if (wrap_azimuth_ambiguity && (scan_info.angle_wrap_flags & 1))
    {
        b.num_azimuth_bins = 3 * A + 1;
        b.az_angles = new float[b.num_azimuth_bins];

        // middle third is a copy of original azimuth angles
        for (uint32_t i = 0; i < A; i++)
        {
            b.az_angles[A + i] = angle_bins[i].azimuth;
        }

        // extend counter clock-wise
        float first_sin = sin(angle_bins[0].azimuth);
        float diff_sin_angle_rad = sin(angle_bins[1].azimuth) - first_sin;
        for (uint32_t i = 0; i < A; i++)
        {
            float res = first_sin - (i + 1) * diff_sin_angle_rad;
            if (res > -1.0f)
                b.az_angles[A - (i + 1)] = asin(res);
            else
                b.az_angles[A - (i + 1)] = -(M_PI / 2);
        }

        // extend clock-wise, one extra
        float last_sin = sin(angle_bins[A - 1].azimuth);
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
            b.az_angles[ii] = angle_bins[ii].azimuth;
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
            b.el_angles[E + i] = angle_bins[i * A].elevation;
        }

        // extend counter clock-wise
        float first_sin = sin(angle_bins[0].elevation);
        float diff_sin_angle_rad = sin(angle_bins[1 * A].elevation) - first_sin;
        for (uint32_t i = 0; i < E; i++)
        {
            float res = first_sin - (i + 1) * diff_sin_angle_rad;
            if (res > -1.0f)
                b.el_angles[E - (i + 1)] = asin(res);
            else
                b.el_angles[E - (i + 1)] = -(M_PI / 2);
        }

        // extend clock-wise, one extra
        float last_sin = sin(angle_bins[(E - 1) * A].elevation);
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
            b.el_angles[e] = angle_bins[e * A].elevation;
        }
    }

    b.range_to_azimuth_ratio = sin(-b.az_angles[0]) * 2;
    b.range_to_elevation_ratio = sin(-b.el_angles[0]) * 2;

}
