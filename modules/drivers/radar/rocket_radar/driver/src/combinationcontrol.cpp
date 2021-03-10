// Copyright (C) Uhnder, Inc. All rights reserved. Confidential and Proprietary - under NDA.
// Refer to SOFTWARE_LICENSE file for details
#include "modules/drivers/radar/rocket_radar/driver/system-radar-software/env-reference/coredefs/uhnder-common.h"
#include "modules/drivers/radar/rocket_radar/driver/src/clutterimage_impl.h"
#include "modules/drivers/radar/rocket_radar/driver/src/combinationcontrol.h"
#include "modules/drivers/radar/rocket_radar/driver/include/polar2cartesian.h"
#include "modules/drivers/radar/rocket_radar/driver/src/scanobject_impl.h"
#include "modules/drivers/radar/rocket_radar/driver/system-radar-software/engine/scp-src/eng-api/uhmath.h"

#include <assert.h>

// Precompute which angle bins need to be combined for this pixel density
// output combined angle data per range bin
void CombinationControl::compute_angle_bins(const ClutterImage_Impl& ci, float pixels_per_range_bin)
{
    memset(this, 0, sizeof(*this));

    const uint32_t R = ci.get_range_dimension();
    const uint32_t A = ci.get_azimuth_dimension();
    const uint32_t E = ci.get_elevation_dimension();

    Polar2Cartesian::BoundsInfo b;
    Polar2Cartesian::estimate_bounds(b, ci, false, false);
    const UhdpScanInformation& info = ci.myscan.scan_info;

    range_bin_width      = info.range_bin_width;
    num_valid_range_bins = b.num_valid_range_bins;
    max_doppler_mps      = (info.doppler_bin_width / 2) * info.SS_size_D;
    max_range_m          = b.num_valid_range_bins * range_bin_width;
    el_fov_start         = b.el_angles[0];
    el_fov_end           = b.el_angles[b.num_elevation_bins - 1];
    az_fov_start         = b.az_angles[0];
    az_fov_end           = b.az_angles[b.num_azimuth_bins - 1];
    stride               = b.num_azimuth_bins;
    az_el_scan           = E > 1;

    // if azimuth wrap is enabled, we will copy the first bin to the last
    // to make the FOV symmetrical
    if (info.angle_wrap_flags & 1)
    {
        az_fov_end = -az_fov_start;
        stride += 1;
    }

    for (uint32_t r = 0; r < R; r++)
    {
        noise_floor_linear[r] = ci.ci_row_noise_floor[r];
    }

    if (az_el_scan)
    {
        for (uint32_t r = 0; r < R; r++)
        {
            num_combined_angles[r] = b.num_azimuth_bins;
            for (uint32_t a = 0; a < b.num_azimuth_bins; a++)
            {
                combined_angles[r][a] = b.az_angles[a];
            }
            if (info.angle_wrap_flags & 1)
            {
                uint32_t ang = num_combined_angles[r]++;
                combined_angles[r][ang] = b.az_angles[0];
            }
        }

        float width_sin_rad = sinf(b.az_angles[1]) - sinf(b.az_angles[0]);
        num_expanded_2D_azimuth_angles = ceilf(2.0 / width_sin_rad) + 1;
        expanded_2D_azimuth_angles = new float[num_expanded_2D_azimuth_angles];

        for (uint32_t i = 0; i < num_expanded_2D_azimuth_angles; i++)
        {
            expanded_2D_azimuth_angles[i] = asinf(-1 + width_sin_rad * i);
        }
        return;
    }

    // === Calculate the width of each angle bin in radians ===
    // do the calculation in sin of angle
    float bin_width_radians[MAX_ROUGH_ANGLES];

    // first bin starts at the bin angle itself
    float sbin_start = sinf(ci.myscan.angle_bins[0].azimuth);
    float sbin_end;
    for (uint32_t a = 0; a < A - 1; a++)
    {
        // midway point between this bin and next bin
        sbin_end = (sinf(ci.myscan.angle_bins[a].azimuth) +
                    sinf(ci.myscan.angle_bins[a + 1].azimuth)) / 2.0f;
        bin_width_radians[a] = asinf(sbin_end) - asinf(sbin_start);
        sbin_start = sbin_end;
    }
    // last bin ends at the bin angle itself
    sbin_end = sinf(ci.myscan.angle_bins[A - 1].azimuth);
    bin_width_radians[A - 1] = asinf(sbin_end) - asinf(sbin_start);

    // Magic number which must be the diagonal of one pixel to prevent undersampling in angle
    const float combine_up_to_pixels = sqrtf(2.0f);

    // === Combine angle bins that are smaller than one pixel cross-wise ===
    // for each range bin
    for (uint32_t r = 0; r < R; r++)
    {
        // First bin is immutable
        combined_angles[r][0] = ci.myscan.angle_bins[0].azimuth;
        comb_bitmap[r][0] |= 1;
        num_combined_angles[r] = 1;

        uint32_t a = 1;

        while (a < A)
        {
            float    total_angle = 0;
            uint32_t first_bin   = a;
            uint32_t last_bin    = a;

            while ((a < A) && (r * sinf(total_angle / 2.0f)) < (combine_up_to_pixels / (2.0f * pixels_per_range_bin)))
            {
                total_angle += bin_width_radians[a];
                last_bin = a;
                a++;
            }

            // last bin of each combined angle bin gets marked with a 1 bit
            comb_bitmap[r][last_bin >> 5] |= (1U << (last_bin & 31));

            const float sfirst_angle = sinf(ci.myscan.angle_bins[first_bin].azimuth);
            const float slast_angle  = sinf(ci.myscan.angle_bins[last_bin].azimuth);
            const uint32_t comb_ang  = num_combined_angles[r]++;
            combined_angles[r][comb_ang] = asinf((sfirst_angle + slast_angle) * 0.5f);
        }

        if (info.angle_wrap_flags & 1)
        {
            const uint32_t comb_ang = num_combined_angles[r]++;
            combined_angles[r][comb_ang] = -combined_angles[r][0];
        }
    }
}


// Combine angle bins in each range bin to achieve final output bins that
// are big enough to avoid aliasing (using pre-calculated control data).
// it also combines elevation if necessary
void CombinationControl::combine_angle_bins(const ClutterImage_Impl& ci, uint32_t elevation_mask)
{
    uint32_t R = ci.get_range_dimension();
    uint32_t A = ci.get_azimuth_dimension();
    uint32_t E = ci.get_elevation_dimension();

    memset(&combined[0], 0, sizeof(combined));

    // If this is an az-el scan, MAX across all selected elevation planes to
    // make the data azimuth only
    if (az_el_scan)
    {
        for (uint32_t e = 0; e < E; e++)
        {
            if (elevation_mask & 1U)
            {
                elevation_mask >>= 1;

                const uint16_t* el = ci.mag_plane + e * A;
                const uint32_t input_stride = A * E;

                for (uint32_t r = 2; r < R; r++)
                {
                    for (uint32_t a = 0; a < A; a++)
                    {
                        combined[r * stride + a] = uh_uintmax(combined[r * stride + a], el[r * input_stride + a]);
                    }
                }
            }
        }
        // if azimuth wrap is enabled, copy the first bin to the last
        if (ci.myscan.scan_info.angle_wrap_flags & 1)
        {
            for (uint32_t r = 0; r < R; r++)
            {
                combined[r * stride + A] = combined[r * stride];
            }
        }
        return;
    }

    // input_buf points to the input data and the stride is A
    const uint16_t* input_buf = ci.mag_plane;
    const uint32_t input_stride = A;

    // now we perform the pseudo-MipMap and combine angle bins that are too
    // small for the cooked pixel density
    uint32_t num_azimuth_bitmap_words = (A + 31) / 32;
    for (uint32_t r = 0; r < R; r++)
    {
        uint32_t azin = 0;
        uint32_t azout = 0;
        bool new_bin = true;

        for (uint32_t aw = 0; aw < num_azimuth_bitmap_words; aw++)
        {
            uint32_t word = comb_bitmap[r][aw];

            for (uint32_t bit = 0; bit < 32; bit++)
            {
                const uint16_t inval = input_buf[r * input_stride + azin];
                uint16_t& outval     = combined[r * stride + azout];

                if (new_bin)
                {
                    outval = inval;
                }
                else
                {
                    outval = uh_uintmax(inval, outval);
                }

                bool final_input = !!(word & 1U); word >>= 1;
                if (final_input)
                {
                    azout++;
                    new_bin = true;
                }
                else
                {
                    new_bin = false;
                }

                if (++azin == A)
                {
                    break;
                }
            }
        }

        if (ci.myscan.scan_info.angle_wrap_flags & 1)
        {
            combined[r * stride + azout] = input_buf[r * input_stride];
        }
    }
}
