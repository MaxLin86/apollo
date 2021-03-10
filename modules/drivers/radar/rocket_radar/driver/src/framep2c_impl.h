// Copyright (C) Uhnder, Inc. All rights reserved. Confidential and Proprietary - under NDA.
// Refer to SOFTWARE_LICENSE file for details
#pragma once


#include "modules/drivers/radar/rocket_radar/driver/system-radar-software/env-reference/coredefs/uhnder-common.h"
#include "modules/drivers/radar/rocket_radar/driver/include/polar2cartesian.h"
#include "modules/drivers/radar/rocket_radar/driver/include/frame_object.h"
#include "modules/drivers/radar/rocket_radar/driver/src/combinationcontrol.h"
#include "modules/drivers/radar/rocket_radar/driver/src/threading.h"

class FrameP2C_Impl : public FrameP2C, public Thread
{
public:

    enum { MAX_MERGE_POINTS = 2 };

    struct FFrac2D
    {
        FFrac2D() : sensor_idx(-1), scan_idx(-1) {}

        int8_t   sensor_idx;
        int8_t   scan_idx;
        float    range_frac;
        float    r0_azimuth_frac;
        float    r1_azimuth_frac;
        uint16_t r0;
        uint16_t r0_a0;
        uint16_t r0_a1;
        uint16_t r1_a0;
        uint16_t r1_a1;
    };

    FrameP2C_Impl(uint32_t X, uint32_t Y, uint32_t cX, uint32_t cY, uint32_t total_scans, float ppm)
        : dimX(X)
        , dimY(Y)
        , platformCenterX(cX)
        , platformCenterY(cY)
        , num_scans(total_scans)
        , pixels_per_meter(ppm)
        , cooking_complete(false)
        , aborted(false)
        , comb_controls(NULL)
        , max_doppler_mps(0.0f)
        , max_height_m(0.0f)
    {
        mag_image  = new float[X * Y];
        dop_image  = new float[X * Y];
        ht_image   = new float[X * Y];
        prob_image = new float[X * Y];

        comb_controls = new CombinationControl[total_scans];
        for (uint32_t i = 0; i < FrameP2C_Impl::MAX_MERGE_POINTS; i++)
        {
            frac[i]   = new FrameP2C_Impl::FFrac2D[dimX * dimY];
        }
    }

    virtual ~FrameP2C_Impl();

    virtual bool            update_region(
            const FrameObject& frame,
            uint32_t start_x, uint32_t extent_x,
            uint32_t start_y, uint32_t extent_y,
            bool update_doppler_plane = false,
            bool update_height_plane = false,
            bool update_occupancy_plane = false);

    virtual bool            cooking_completed() const                                    { return cooking_complete; }
    virtual void            get_image_dimensions(uint32_t& dim_x, uint32_t& dim_y) const { dim_x = dimX; dim_y = dimY; }
    virtual void            get_platform_center_pos(uint32_t& x, uint32_t& y) const      { x = platformCenterX, y = platformCenterY; }
    virtual const float*    get_clutter_magnitude_image() const                          { return mag_image; }
    virtual const float*    get_clutter_doppler_image() const                            { return dop_image; }
    virtual const float*    get_clutter_height_image() const                             { return ht_image; }
    virtual const float*    get_occupancy_probability_image() const                      { return prob_image; }
    virtual float           get_max_doppler_value() const                                { return max_doppler_mps; }
    virtual float           get_max_height() const                                       { return max_height_m; }
    virtual void            release();
    virtual void            thread_main();

            void            cook();
            float           get_doppler_sample(const FrameObject_Impl& frame, const FFrac2D& f2d);
            float           get_height_sample(const FrameObject_Impl& frame, const FFrac2D& f2d);

    const uint32_t  dimX;
    const uint32_t  dimY;
    const uint32_t  platformCenterX;
    const uint32_t  platformCenterY;
    const uint32_t  num_scans;
    const float     pixels_per_meter;

    bool            cooking_complete;
    bool            aborted;

    float*          mag_image;
    float*          dop_image;
    float*          ht_image;
    float*          prob_image;

    CombinationControl* comb_controls;

    float           max_doppler_mps;
    float           max_height_m;

    FFrac2D*        frac[MAX_MERGE_POINTS];
};
