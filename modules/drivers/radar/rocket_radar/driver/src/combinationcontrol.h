// Copyright (C) Uhnder, Inc. All rights reserved. Confidential and Proprietary - under NDA.
// Refer to SOFTWARE_LICENSE file for details
#pragma once

#include "modules/drivers/radar/rocket_radar/driver/system-radar-software/env-reference/coredefs/uhnder-common.h"
#include "modules/drivers/radar/rocket_radar/driver/system-radar-software/engine/common/eng-api/rhal_out.h"

class ClutterImage_Impl;

struct CombinationControl
{
    // a 0 bit indicates that an angle bin has been consumed. A 1
    // bit indicates the current angle bin is complete
    uint32_t comb_bitmap[MAX_RANGE_BINS][(MAX_ROUGH_ANGLES + 31)/ 32];

    float    combined_angles[MAX_RANGE_BINS][MAX_ROUGH_ANGLES];

    uint32_t num_combined_angles[MAX_RANGE_BINS];

    uint16_t combined[MAX_RANGE_BINS * MAX_ROUGH_ANGLES + 4];

    uint16_t noise_floor_linear[MAX_RANGE_BINS];

    uint32_t stride; // stride of combined[] buffer

    uint32_t num_valid_range_bins; // occupancy is stride * num_valid_range_bins

    // start of sensorgroup only fields
    vec3f_t  sensor_pos;

    quatf_t  sensor_orientation;

    uint32_t sensor_idx;

    uint32_t scan_idx;

    float*   expanded_2D_azimuth_angles;

    uint32_t num_expanded_2D_azimuth_angles;

    float    az_fov_start;
    float    az_fov_end;

    float    el_fov_start;
    float    el_fov_end;

    float    max_doppler_mps;

    float    max_range_m;

    float    range_bin_width;

    bool     az_el_scan;
    // end of sensorgroup only fields

    CombinationControl()
    {
        expanded_2D_azimuth_angles = NULL;
    }

    ~CombinationControl()
    {
        delete [] expanded_2D_azimuth_angles;
    }

    // Cook Anti-Alias
    void compute_angle_bins(const ClutterImage_Impl& ci, float pixels_per_range_bin);

    // Apply Anti-Alias
    void combine_angle_bins(const ClutterImage_Impl& ci, uint32_t elevation_mask);
};
