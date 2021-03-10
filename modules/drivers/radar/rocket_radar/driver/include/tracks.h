// Copyright (C) Uhnder, Inc. All rights reserved. Confidential and Proprietary - under NDA.
// Refer to SOFTWARE_LICENSE file for details
#pragma once

#include "modules/drivers/radar/rocket_radar/driver/system-radar-software/env-reference/coredefs/uhnder-common.h"
#include "modules/drivers/radar/rocket_radar/driver/system-radar-software/engine/common/eng-api/uhtypes.h"

struct Track
{
    uint32_t  id;
    uint32_t  last_update_sec;
    uint32_t  last_update_usec;
    vec3f_t   posRelHost;
    vec3f_t   velRelHost;
    vec3f_t   posVariance;
    vec3f_t   velVariance;
    float     covXY, covXZ, covYZ;
    float     covXvelYvel, covXvelZvel, covYvelZvel;
    vec3f_t   covXvel;
    vec3f_t   covYvel;
    vec3f_t   covZvel;
    int       status;
};

class Tracks
{
public:

    virtual ~Tracks() {}

    //! returns the number of tracks reported in this scan
    virtual uint32_t             get_count() const = 0;

    //! returns a specific track instance
    virtual const Track&         get_track(uint32_t idx) const = 0;

    //! release all of the memory associated with this Tracks instance
    virtual void                 release() = 0;

    //! an enumeration of the errors which might be returned by this class
    enum Err
    {
        TRACK_NO_ERROR,
        TRACK_INDEX_OUT_OF_RANGE,
    };

    //! returns the last error encountered by this instance
    virtual Err                  get_last_error() const = 0;
};
