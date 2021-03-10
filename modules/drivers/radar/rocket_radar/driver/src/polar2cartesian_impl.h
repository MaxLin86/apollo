// Copyright (C) Uhnder, Inc. All rights reserved. Confidential and Proprietary - under NDA.
// Refer to SOFTWARE_LICENSE file for details
#pragma once

#include "modules/drivers/radar/rocket_radar/driver/system-radar-software/env-reference/coredefs/uhnder-common.h"
#include "modules/drivers/radar/rocket_radar/driver/include/polar2cartesian.h"
#include "modules/drivers/radar/rocket_radar/driver/src/combinationcontrol.h"
#include "modules/drivers/radar/rocket_radar/driver/src/threading.h"

class Polar2Cartesian_Impl : public Polar2Cartesian, public Thread
{
public:

    struct Frac2D
    {
        // fractional bin indices
        float    range_frac;
        float    r0_azimuth_frac;
        float    r1_azimuth_frac;
        uint16_t r0;
        uint16_t r1;
        uint16_t r0_a0;
        uint16_t r0_a1;
        uint16_t r1_a0;
        uint16_t r1_a1;
    };

    struct Frac2DFrontView
    {
        // fractional bin indices
        float azimuth;
        float elevation;
    };

    struct Frac3D
    {
        // fractional bin indices
        float range;
        float azimuth;
        float elevation;
    };

    Polar2Cartesian_Impl()
        : last_err(P2C_NO_ERROR)
        , cooking_complete(true)
        , aborted(false)
        , frac2d(NULL)
        , frac3d(NULL)
        , frac2dfront(NULL)
        , obuf(NULL)
    {
        // invalid subspace extent, so first process() does not blend
        buffered_subspace.extent_X = (uint32_t)-1;
    }


    virtual ~Polar2Cartesian_Impl();

    virtual const uint32_t* process(const ClutterImage&, SubSpace*, float alpha);

    virtual bool            cooking_completed() const { return cooking_complete; }

    virtual Err             get_last_error() const { return last_err; }

    virtual void            release()              { delete this; }

    virtual void            thread_main();

            void            cook();

    Err                     last_err;

    bool                    cooking_complete;

    bool                    aborted;

    uint32_t                dim_X;

    uint32_t                dim_Y;

    uint32_t                dim_Z;

    uint32_t                range_bins;

    uint32_t                azimuth_bins;

    uint32_t                elevation_bins;

    Frac2D*                 frac2d;

    Frac3D*                 frac3d;

    Frac2DFrontView*        frac2dfront;

    uint32_t*               obuf;

    SubSpace                buffered_subspace;

    PixelSpace2D            psp2d;

    CombinationControl      comb_control;
};
