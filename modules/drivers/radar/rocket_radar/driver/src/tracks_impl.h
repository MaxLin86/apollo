// Copyright (C) Uhnder, Inc. All rights reserved. Confidential and Proprietary - under NDA.
// Refer to SOFTWARE_LICENSE file for details
#pragma once

#include "modules/drivers/radar/rocket_radar/driver/system-radar-software/env-reference/coredefs/uhnder-common.h"
#include "modules/drivers/radar/rocket_radar/driver/include/tracks.h"

class ScanObject_Impl;

class Tracks_Impl : public Tracks
{
public:

    Tracks_Impl(ScanObject_Impl& impl)
        : myscan(impl)
        , tracks(NULL)
        , track_count(0)
        , last_err(TRACK_NO_ERROR)
    {
    }

    virtual ~Tracks_Impl()
    {
        delete [] tracks;
    }

    virtual uint32_t             get_count() const      { return track_count; }
    virtual const Track&         get_track(uint32_t idx) const;
    virtual void                 release();
    virtual Err                  get_last_error() const { return last_err; }

            bool                 serialize(ScanSerializer& s) const;
            bool                 deserialize(ScanSerializer& s);

    ScanObject_Impl&    myscan;
    Track*              tracks;
    uint32_t            track_count;

    mutable Err         last_err;
};
