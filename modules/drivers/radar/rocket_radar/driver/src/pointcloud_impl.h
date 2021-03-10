// Copyright (C) Uhnder, Inc. All rights reserved. Confidential and Proprietary - under NDA.
// Refer to SOFTWARE_LICENSE file for details
#pragma once

#include "modules/drivers/radar/rocket_radar/driver/system-radar-software/env-reference/coredefs/uhnder-common.h"
#include "modules/drivers/radar/rocket_radar/driver/include/pointcloud.h"

SRS_DECLARE_NAMESPACE()
struct PointCloudData;
SRS_CLOSE_NAMESPACE()

class  ScanObjectImpl;

class PointCloud_Impl : public PointCloud
{
public:

    PointCloud_Impl(ScanObject_Impl& scan)
        : myscan(scan)
        , points(NULL)
        , num_points(0)
        , aborted(false)
        , clutter_points_collected(false)
        , last_err(PC_NO_ERROR)
    {
    }

    virtual ~PointCloud_Impl();

    virtual uint32_t              get_count() { return num_points; }

    virtual void                  get_points(Point* output);

    virtual const PointCloudData* get_raw_points() { return points; }

    virtual void                  extract_static_points();

    virtual void                  apply_threshold(float point_cloud_thresh);

    virtual void                  release();

    virtual Err                   get_last_error() const { return last_err; }

            bool                  deserialize(ScanSerializer&);

            bool                  serialize(ScanSerializer&);

            void                  handle_uhdp(PointCloudData* p, uint32_t total_size);

            void                  setup();

    ScanObject_Impl&    myscan;

    PointCloudData*     points;

    uint32_t            num_points;

    bool                aborted;

    bool                clutter_points_collected;

    mutable Err         last_err;
};
