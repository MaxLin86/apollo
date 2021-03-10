// Copyright (C) Uhnder, Inc. All rights reserved. Confidential and Proprietary - under NDA.
// Refer to SOFTWARE_LICENSE file for details
#pragma once

#include "modules/drivers/radar/rocket_radar/driver/system-radar-software/env-reference/coredefs/uhnder-common.h"
#include "modules/drivers/radar/rocket_radar/driver/system-radar-software/engine/common/eng-api/rdc-structs.h"

//! Point cloud detections. Users must capture DL_POINT_CLOUD to get dynamic points
//! from the radar. Users must capture DL_CI to get static points from the radar
class PointCloud
{
public:

    virtual ~PointCloud() {}

    //! returns the number of points
    virtual uint32_t              get_count() = 0;

    enum PointFlags
    {
        PF_CLUTTER                = 1 << 0,  //!< from clutter image/static slice (not activations)
        PF_TWO_PEAKS              = 1 << 1,  //!< two peaks were at this location (different dopplers) (PF_CLUTTER only)
        PF_THREE_PEAKS            = 1 << 2,  //!< three peaks were at this location (different dopplers) (PF_CLUTTER only)
        PF_MORE_THAN_THREE_PEAKS  = 1 << 3,  //!< 4+ peaks were at this location (different dopplers) (PF_CLUTTER only)
        PF_STATIONARY             = 1 << 4,
    };

    //! output point structure
    struct Point
    {
        float    range;        //!< meters
        float    doppler;      //!< mps
        float    azimuth;      //!< radians from boresight
        float    elevation;    //!< radians from boresight
        float    mag_snr;      //!< decibels
        float    mag_i;        //!< full scale magnitude, 0 if unsupported
        float    mag_q;        //!< full scale magnitude, 0 if unsupported
        uint32_t flags;        //!< PointFlags
    };

    //! user must provide output storage for get_count() Point instances, this
    //! method writes them all
    virtual void                  get_points(Point* output) = 0;

    //! extract points from the static environment as represented by the clutter
    //! image.
    virtual void                  extract_static_points() = 0;

    //! returns raw fixed-point (uninterpolated) point cloud points - the user
    //! is responsible for properly interpreting the fixed-point data
    virtual const PointCloudData* get_raw_points() = 0;

    //! discard all points below the specified SNR threshold. This function can
    //! obviously reduce the number returned by get_count().  If the scan is
    //! serialized after this function is called, only the surviving points are
    //! serialized.
    virtual void                  apply_threshold(float point_cloud_snr_dB) = 0;

    //! release all storage of the point cloud
    virtual void                  release() = 0;

    //! an enumeration of the errors potentially returned by this class
    enum Err
    {
        PC_NO_ERROR,
        PC_INVALID_INPUT,
        PC_FILE_WRITE_FAILURE,
    };

    //! returns the last error encountered by this instance
    virtual Err                   get_last_error() const = 0;
};
