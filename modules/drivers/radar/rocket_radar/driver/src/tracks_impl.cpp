// Copyright (C) Uhnder, Inc. All rights reserved. Confidential and Proprietary - under NDA.
// Refer to SOFTWARE_LICENSE file for details


#include "modules/drivers/radar/rocket_radar/driver/system-radar-software/env-reference/coredefs/uhnder-common.h"
#include "modules/drivers/radar/rocket_radar/driver/src/scanning_impl.h"
#include "modules/drivers/radar/rocket_radar/driver/src/tracks_impl.h"
#include "modules/drivers/radar/rocket_radar/driver/include/serializer.h"

#include "modules/drivers/radar/rocket_radar/driver/include/detections.h"
#include "modules/drivers/radar/rocket_radar/driver/system-radar-software/engine/scp-src/eng-api/uhdp-msg-structs.h"
#include "modules/drivers/radar/rocket_radar/driver/system-radar-software/engine/scp-src/eng-api/uhmath.h"
#include "modules/drivers/radar/rocket_radar/driver/src/threading.h"

namespace
{
    Track DeadTrack;
}

const Track& Tracks_Impl::get_track(uint32_t idx) const
{
    if (tracks && idx < track_count)
    {
        last_err = TRACK_NO_ERROR;
        return tracks[idx];
    }
    else
    {
        last_err = TRACK_INDEX_OUT_OF_RANGE;
        return DeadTrack;
    }
}


void         Tracks_Impl::release()
{
    myscan.release_tracks(*this);
}


bool         Tracks_Impl::serialize(ScanSerializer& s) const
{
    bool ok = true;

    if (tracks && track_count > 0)
    {
        char fname[128];
        sprintf(fname, "scan_%06d_tracks.bin", myscan.scan_info.scan_sequence_number);
        ok = s.begin_write_scan_data_type(fname);
        if (ok)
        {
            ok &= s.write_scan_data_type(tracks, sizeof(Track), track_count);
            s.end_write_scan_data_type(!ok);
        }
    }

    return ok;
}


bool         Tracks_Impl::deserialize(ScanSerializer& s)
{
    char fname[128];
    sprintf(fname, "scan_%06d_tracks.bin", myscan.scan_info.scan_sequence_number);

    bool ok = true;
    size_t len = s.begin_read_scan_data_type(fname);
    if (len)
    {
        track_count = len / sizeof(Track);
        tracks = new Track[track_count];
        ok &= s.read_scan_data_type(tracks, sizeof(Track), track_count);
    }

    return ok;
}
