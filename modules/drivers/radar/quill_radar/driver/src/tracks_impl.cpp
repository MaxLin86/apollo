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
#include "modules/drivers/radar/quill_radar/driver/src/scanobject_impl.h"
#include "modules/drivers/radar/quill_radar/driver/src/tracks_impl.h"
#include "modules/drivers/radar/quill_radar/driver/include/serializer.h"


#if defined(EXTERNAL_TRACKER_ENABLED)
//! User-implemented: Step an external tracker with provded ScanObject instance
//
//! If radar-remote-api is compiled with the BUILD_TRACKER CMake flag enabled,
//! this function will get called each time a scan arrives from the radar. This
//! user-implemented function must pass the detections and/or points to the
//! external tracker and step the tracker state machine. All output tracks should
//! be written back to the ScanObject using the Tracks::set_tracks() method
extern "C" void step_external_tracker(class ScanObject& scan);
#endif

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


void         Tracks_Impl::set_tracks(uint32_t new_count, const Track* new_tracks)
{
    if (tracks)
    {
        delete [] tracks;
        tracks = NULL;
        track_count = 0;
    }

    if (new_count)
    {
        tracks = new Track[new_count];
        memcpy(tracks, new_tracks, sizeof(tracks[0]) * new_count);
        track_count = new_count;
    }

    last_err = TRACK_NO_ERROR;
}


void         Tracks_Impl::release()
{
    myscan.release_tracks(*this);
}


bool         Tracks_Impl::serialize(ScanSerializer& s) const
{
    bool ok = true;

    if (track_count > 0)
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


void         Tracks_Impl::step_tracker()
{
#if defined(EXTERNAL_TRACKER_ENABLED)
    step_external_tracker(myscan);
#endif
}
